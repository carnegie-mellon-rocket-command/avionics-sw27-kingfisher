/* imu.c -- see imu.h */
#include "imu.h"
#include "lsm6dsox_reg.h"
#include <math.h>
#include <stdio.h>

#define I2C_TIMEOUT_MS 10
#define PI_F           3.14159265f
#define MG_TO_FTPS2    (0.001f * GRAVITY)          /* mg -> ft/s^2 */
#define MDPS_TO_RADPS  (0.001f * PI_F / 180.0f)    /* mdps -> rad/s */

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t addr8;          /* 7-bit address shifted left, as HAL wants */
} imu_bus_t;

static imu_bus_t   bus;
static stmdev_ctx_t ctx;
static bool         ready = false;

#define IMU_ADDR_LOW          0x6Au
#define IMU_ADDR_HIGH         0x6Bu
#define IMU_PROBE_ROUNDS      10
#define IMU_VERIFY_READS      3
#define IMU_STARTUP_SAMPLES   10

/* ---- platform glue: ST's driver calls these ---- */
static int32_t plat_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    imu_bus_t *b = (imu_bus_t *)handle;
    return HAL_I2C_Mem_Write(b->hi2c, b->addr8, reg, I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)buf, len, I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

static int32_t plat_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    imu_bus_t *b = (imu_bus_t *)handle;
    return HAL_I2C_Mem_Read(b->hi2c, b->addr8, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, I2C_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

static void plat_delay(uint32_t ms) { HAL_Delay(ms); }

static bool select_device(int rounds)
{
    static const uint8_t candidates[2] = {IMU_ADDR_LOW, IMU_ADDR_HIGH};
    uint8_t id;

    for (int round = 0; round < rounds; round++) {
        for (int i = 0; i < 2; i++) {
            bus.addr8 = (uint16_t)(candidates[i] << 1);
            id = 0;
            if (lsm6dsox_device_id_get(&ctx, &id) == 0 && id == LSM6DSOX_ID) return true;
        }
        HAL_Delay(1);
    }
    return false;
}

static bool verify_fixed_address(void)
{
    uint8_t id;
    bus.addr8 = (uint16_t)(IMU_ADDR_HIGH << 1);
    for (int i = 0; i < IMU_VERIFY_READS; i++) {
        id = 0;
        if (lsm6dsox_device_id_get(&ctx, &id) != 0 || id != LSM6DSOX_ID) return false;
        HAL_Delay(1);
    }
    return true;
}

static bool configure_and_verify(void)
{
    lsm6dsox_fs_xl_t xl_fs;
    lsm6dsox_fs_g_t gy_fs;
    lsm6dsox_odr_xl_t xl_odr;
    lsm6dsox_odr_g_t gy_odr;
    lsm6dsox_xl_fs_mode_t xl_fs_mode;
    lsm6dsox_sdo_pu_en_t pullup;
    uint8_t enabled;

    if (lsm6dsox_sdo_sa0_mode_set(&ctx, LSM6DSOX_PULL_UP_CONNECT) != 0) return false;

    /* Enabling the pull-up can change the address immediately from 0x6A to 0x6B. */
    if (!verify_fixed_address()) return false;

    if (lsm6dsox_auto_increment_set(&ctx, PROPERTY_ENABLE) != 0 ||
        lsm6dsox_block_data_update_set(&ctx, PROPERTY_ENABLE) != 0 ||
        lsm6dsox_aux_xl_fs_mode_set(&ctx, LSM6DSOX_USE_SAME_XL_FS) != 0 ||
        lsm6dsox_xl_full_scale_set(&ctx, LSM6DSOX_16g) != 0 ||
        lsm6dsox_gy_full_scale_set(&ctx, LSM6DSOX_2000dps) != 0 ||
        lsm6dsox_xl_data_rate_set(&ctx, LSM6DSOX_XL_ODR_208Hz) != 0 ||
        lsm6dsox_gy_data_rate_set(&ctx, LSM6DSOX_GY_ODR_208Hz) != 0) return false;

    if (lsm6dsox_sdo_sa0_mode_get(&ctx, &pullup) != 0 ||
        lsm6dsox_auto_increment_get(&ctx, &enabled) != 0 || !enabled ||
        lsm6dsox_block_data_update_get(&ctx, &enabled) != 0 || !enabled ||
        lsm6dsox_aux_xl_fs_mode_get(&ctx, &xl_fs_mode) != 0 ||
        xl_fs_mode != LSM6DSOX_USE_SAME_XL_FS ||
        lsm6dsox_xl_full_scale_get(&ctx, &xl_fs) != 0 || xl_fs != LSM6DSOX_16g ||
        lsm6dsox_gy_full_scale_get(&ctx, &gy_fs) != 0 || gy_fs != LSM6DSOX_2000dps ||
        lsm6dsox_xl_data_rate_get(&ctx, &xl_odr) != 0 || xl_odr != LSM6DSOX_XL_ODR_208Hz ||
        lsm6dsox_gy_data_rate_get(&ctx, &gy_odr) != 0 || gy_odr != LSM6DSOX_GY_ODR_208Hz ||
        pullup != LSM6DSOX_PULL_UP_CONNECT) return false;

    return true;
}

bool imu_init(I2C_HandleTypeDef *hi2c)
{
    uint8_t rst = 1;
    uint8_t data_ready = 0;
    bool reset_complete = false;
    imu_data_t sample;
    float magnitude_sum = 0.0f;
    uint8_t ctrl[3] = {0};

    ready = false;
    ctx.write_reg = plat_write;
    ctx.read_reg  = plat_read;
    ctx.mdelay    = plat_delay;
    ctx.handle    = &bus;
    bus.hi2c = hi2c;

    if (!select_device(IMU_PROBE_ROUNDS)) {
        printf("[FAULT] IMU: WHO_AM_I 0x6C not found at 0x6A or 0x6B\r\n");
        return false;
    }
    printf("[BOOT] IMU found at 7-bit address 0x%02lX\r\n", (unsigned long)(bus.addr8 >> 1));

    /* Software reset, wait for it to finish */
    if (lsm6dsox_reset_set(&ctx, PROPERTY_ENABLE) != 0) {
        printf("[FAULT] IMU: software reset write failed\r\n");
        return false;
    }
    for (int i = 0; i < 100; i++) {
        if (lsm6dsox_reset_get(&ctx, &rst) == 0 && !rst) {
            reset_complete = true;
            break;
        }
        /* A floating SA0 can move while the reset is taking place. */
        select_device(1);
        HAL_Delay(1);
    }
    if (!reset_complete) {
        printf("[FAULT] IMU: software reset did not complete\r\n");
        return false;
    }

    if (!select_device(IMU_PROBE_ROUNDS) || !configure_and_verify()) {
        printf("[FAULT] IMU: address stabilization/configuration readback failed\r\n");
        return false;
    }

    if (plat_read(&bus, LSM6DSOX_CTRL1_XL, ctrl, sizeof(ctrl)) != 0) {
        printf("[FAULT] IMU: control-register readback failed\r\n");
        return false;
    }
    printf("[BOOT] IMU stable at 0x6B; CTRL1_XL=0x%02X CTRL2_G=0x%02X CTRL3_C=0x%02X\r\n",
           ctrl[0], ctrl[1], ctrl[2]);

    for (int i = 0; i < 100 && !data_ready; i++) {
        if (lsm6dsox_xl_flag_data_ready_get(&ctx, &data_ready) != 0) {
            printf("[FAULT] IMU: data-ready read failed\r\n");
            return false;
        }
        if (!data_ready) HAL_Delay(1);
    }
    if (!data_ready) {
        printf("[FAULT] IMU: no accelerometer data-ready within 100 ms\r\n");
        return false;
    }

    ready = true;

    /* Detect the dangerous case where the chip stayed at its reset +/-2 g range
     * while the software would otherwise convert samples as +/-16 g. */
    for (int i = 0; i < IMU_STARTUP_SAMPLES; i++) {
        if (!imu_read(&sample)) {
            ready = false;
            printf("[FAULT] IMU: startup sample %d failed\r\n", i);
            return false;
        }
        magnitude_sum += sample.accel_mag;
        HAL_Delay(5);
    }
    float avg_g = magnitude_sum / (IMU_STARTUP_SAMPLES * GRAVITY);
    if (!isfinite(avg_g) || avg_g < 0.5f || avg_g > 1.5f) {
        ready = false;
        printf("[FAULT] IMU: startup acceleration implausible (%ld milli-g average)\r\n",
               (long)(avg_g * 1000.0f));
        return false;
    }
    printf("[BOOT] IMU startup samples valid: average magnitude=%ld milli-g\r\n",
           (long)(avg_g * 1000.0f));
    return true;
}

bool imu_read(imu_data_t *out)
{
    int16_t a[3], g[3], t;
    if (!ready) return false;

    if (lsm6dsox_acceleration_raw_get(&ctx, a) != 0) return false;
    if (lsm6dsox_angular_rate_raw_get(&ctx, g) != 0) return false;
    if (lsm6dsox_temperature_raw_get(&ctx, &t) != 0) return false;

    out->accel_x = lsm6dsox_from_fs16_to_mg(a[0]) * MG_TO_FTPS2;
    out->accel_y = lsm6dsox_from_fs16_to_mg(a[1]) * MG_TO_FTPS2;
    out->accel_z = lsm6dsox_from_fs16_to_mg(a[2]) * MG_TO_FTPS2;
    out->gyro_x  = lsm6dsox_from_fs2000_to_mdps(g[0]) * MDPS_TO_RADPS;
    out->gyro_y  = lsm6dsox_from_fs2000_to_mdps(g[1]) * MDPS_TO_RADPS;
    out->gyro_z  = lsm6dsox_from_fs2000_to_mdps(g[2]) * MDPS_TO_RADPS;
    out->temp_c  = lsm6dsox_from_lsb_to_celsius(t);
    out->accel_mag = sqrtf(out->accel_x * out->accel_x +
                           out->accel_y * out->accel_y +
                           out->accel_z * out->accel_z);
    return isfinite(out->accel_mag) && isfinite(out->temp_c);
}
