/* ats.c -- the ATS application. See ats.h for the project overview.
 *
 * FLIGHT FLOW (one pass of ats_run_once() every LOOP_TARGET_MS):
 *   1. runTimer()      wait until 30 ms have passed since the last pass
 *   2. read sensors    barometer -> altitude, IMU -> acceleration
 *   3. filterData()    Kalman filter -> filtered altitude / velocity / acceleration
 *   4. PRE-FLIGHT      acceleration above ACCEL_THRESHOLD => launched (flaps armed, logging starts)
 *   5. DURING FLIGHT   adjustATS() moves the flaps; detectLanding() watches for the rocket to stop
 *   6. log             one 64-byte record to flash (after launch)
 *   7. END OF FLIGHT   flush the log, detach servo, blink forever
 *
 * Differences from last year's cantaloupe-ats-ref.cpp (on purpose):
 *   - no Arduino/SD/Serial; data goes to the on-board flash in binary (see flashlog.c)
 *   - simulation mode and assert macros removed
 *   - flap command clamped to 0..1 (the PID output was unbounded, which could drive the flaps past their stop)
 *   - landing detection fixed (old countdown was an unsigned variable that never reached its test)
 *   - predicted-altitude formula fixed (macro precedence bug), no blocking delay() in the flight loop
 */
#include "ats.h"
#include "ats_config.h"
#include "imu.h"
#include "ms5611.h"
#include "kalman.h"
#include "servo.h"
#include "flashlog.h"
#include "piezo.h"
#include "stlink_export.h"
#include <math.h>
#include <stdio.h>

#define PI_F 3.14159265f

/* Peripheral handles created by CubeMX in main.c */
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

/* ***************** STATE ***************** */
static ms5611_t   baro;
static imu_data_t imu;
static kalman_t   kf;

static uint32_t gStartTime, gLaunchTime, gPrevLoopTime, gTimeDelta;   /* ms */
static float    gAltFiltered, gVelocityFiltered, gAccelFiltered;      /* from the Kalman filter */
static float    gPredictedAltitude;                                   /* where the rocket would peak at the current flap setting (ft) */
static float    gATSPosition = ATS_IN;                                /* commanded flap position, 0 (in) .. 1 (out) */
static float    gATSCommand = ATS_IN;
static bool     gLaunched = false, gLanded = false;
static bool     gFlightFault = false;
static float    absolute_alt_target = ALT_TARGET;                     /* ALT_TARGET is above the pad; this one is above sea level */

/* ***************** STATUS LED ***************** */
static void led_set(bool on)  { HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void led_toggle(void)  { HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN); }

/** @brief Persistent fast blink means fault/not armed; the exact reason is printed over SWO. */
static void fatal_fault(const char *reason)
{
    printf("[FAULT] %s; system NOT ARMED\r\n", reason);
    piezo_set_fault();
    while (1) {
        stlink_export_poll();
        piezo_update();
        led_toggle();
        HAL_Delay(100);
    }
}

/** @brief Delay while maintaining the slow startup/not-armed heartbeat. */
static void boot_delay(uint32_t delay_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < delay_ms) {
        stlink_export_poll();
        led_set(((HAL_GetTick() / 500u) & 1u) == 0u);
        piezo_update();
        HAL_Delay(20);
    }
}

/* ***************** ATS (FLAP) METHODS ***************** */
/** @brief Start the servo (flaps hold position). */
static bool attachATS(void) { return SKIP_ATS || servo_attach(); }

/** @brief Stop the servo (saves power). */
static void detachATS(void) { if (!SKIP_ATS) servo_detach(); }

/** @brief Set flap position.
  * @arg percent_rot: 0 (fully in) .. 1 (fully out). Mapped to the servo angle range ATS_MAX..ATS_MIN. */
static void setATSPosition(float percent_rot)
{
    gATSCommand = percent_rot;
    if (SKIP_ATS) return;
    float pos = ATS_MIN - (ATS_MIN - ATS_MAX) * (1.0f - percent_rot);
    servo_write_degrees(pos);
}

/** @brief Self-test: fully extend and retract the flaps, then detach. Takes ~6 s at startup. */
static void testATS(void)
{
    if (SKIP_ATS) {
        printf("[BOOT] Servo test skipped (SKIP_ATS=1)\r\n");
        return;
    }
    printf("[BOOT] Servo command test starting (no position feedback is fitted)\r\n");
    if (!attachATS()) fatal_fault("servo PWM start failed");
    if (!(htim2.Instance->CR1 & TIM_CR1_CEN) || !(htim2.Instance->CCER & TIM_CCER_CC3E))
        fatal_fault("TIM2 channel 3 did not enable");
    printf("[SERVO] PWM enabled: PSC=%lu ARR=%lu CCR3=%lu\r\n",
           (unsigned long)htim2.Instance->PSC,
           (unsigned long)htim2.Instance->ARR,
           (unsigned long)htim2.Instance->CCR3);
    setATSPosition(ATS_IN);
    printf("[SERVO] retract command: CCR3=%lu\r\n", (unsigned long)htim2.Instance->CCR3);
    boot_delay(2000);
    setATSPosition(ATS_OUT);
    printf("[SERVO] extend command: CCR3=%lu\r\n", (unsigned long)htim2.Instance->CCR3);
    boot_delay(2000);
    setATSPosition(ATS_IN);
    printf("[SERVO] retract command: CCR3=%lu\r\n", (unsigned long)htim2.Instance->CCR3);
    boot_delay(2000);
    printf("[BOOT] Servo PWM remains active at retracted command; physical motion is not electronically verified\r\n");
}
static void print_imu_debug(const imu_data_t *d)
{
#if DEBUG
    static uint32_t last_print = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_print < IMU_DEBUG_PERIOD_MS) return;
    last_print = now;

    long ax_mg = (long)(d->accel_x * 1000.0f / GRAVITY);
    long ay_mg = (long)(d->accel_y * 1000.0f / GRAVITY);
    long az_mg = (long)(d->accel_z * 1000.0f / GRAVITY);
    long mag_mg = (long)(d->accel_mag * 1000.0f / GRAVITY);
    long gx_mdps = (long)(d->gyro_x * 180000.0f / PI_F);
    long gy_mdps = (long)(d->gyro_y * 180000.0f / PI_F);
    long gz_mdps = (long)(d->gyro_z * 180000.0f / PI_F);
    long temp_centi_c = (long)(d->temp_c * 100.0f);

    printf("[IMU %lu ms] accel_mg=(%ld,%ld,%ld) mag=%ld gyro_mdps=(%ld,%ld,%ld) temp_centiC=%ld\r\n",
           (unsigned long)now, ax_mg, ay_mg, az_mg, mag_mg,
           gx_mdps, gy_mdps, gz_mdps, temp_centi_c);
#else
    (void)d;
#endif
}

/** @brief PD controller.
  * @arg error: acceleration error (always passed as a positive magnitude here)
  * @arg Kp: proportional gain (higher = stronger immediate response)
  * @arg Kd: derivative gain (reacts to how fast the error changes)
  * @return unclamped flap command; the caller clamps it to 0..1 */
static float PIDFactor(float error, float Kp, float Kd)
{
    static float old_error = 0.0f;
    float proportional = error * Kp;
    float derivative = (error - old_error) * Kd;
    old_error = error;
    return proportional + derivative;
}

/** @brief Decide where the flaps should be this cycle, and move them.
  * Timeline after launch: 0-4.5 s nothing; 4.5-6 s wiggle self-test; 6-18 s controller; after 18 s retract.
  * Controller: compare the deceleration we have with the deceleration needed to just reach the target
  * apogee (v^2 / (2 * distance_remaining)). Too fast => open the flaps by PD(error). */
static void adjustATS(void)
{
    uint32_t t = HAL_GetTick() - gLaunchTime;

    /* Retract ATS fully after 18 seconds */
    if (t > ATS_RETRACT_AFTER_MS) {
        setATSPosition(ATS_IN);
        gATSPosition = ATS_IN;
        return;
    }

    if (gAltFiltered >= absolute_alt_target) {
        /* Fully deploy ATS if the altitude target has been reached */
        gATSPosition = ATS_OUT;
    } else {
        float remaining = absolute_alt_target - gAltFiltered;
        float targetAcceleration = fabsf(gVelocityFiltered * gVelocityFiltered / (2.0f * remaining));

        /* Positive error means we're decelerating at least as much as needed (too slow = no flaps);
         * negative means too fast and we need to slow down. */
        float error = gAccelFiltered - targetAcceleration;
        float adjustment = (error > 0.0f) ? 0.0f : PIDFactor(fabsf(error), ATS_KP, ATS_KD);
        if (adjustment > 1.0f) adjustment = 1.0f;     /* never command past the flaps' mechanical stop */
        if (adjustment < 0.0f) adjustment = 0.0f;
        gATSPosition = adjustment;

        /* Predicted apogee rise if the flaps stay at this setting */
        gPredictedAltitude = (0.5f * ROCKET_MASS * gVelocityFiltered * gVelocityFiltered) /
                             (ROCKET_MASS * GRAVITY + 0.5f * ATMOSPHERE_FLUID_DENSITY * ROCKET_DRAG_COEFFICIENT *
                              gVelocityFiltered * gVelocityFiltered * ATS_MAX_SURFACE_AREA * adjustment);
    }

    /* ATS window */
    if (t > ATS_ACTIVE_AFTER_MS) {
        if (t < ATS_WIGGLE_END_MS) {
            if (t > ATS_WIGGLE_BACK_MS)      setATSPosition(ATS_IN);   /* reset position */
            else if (t > ATS_WIGGLE_OUT_MS)  setATSPosition(ATS_OUT);  /* extend */
            else                             setATSPosition(ATS_IN);
        } else {
            setATSPosition(gATSPosition);
        }
    }
}

/* ***************** TIMING ***************** */
/** @brief Wait until LOOP_TARGET_MS has passed since the last pass so data is sampled at a constant rate.
  * Also records gTimeDelta (the real elapsed ms), which the Kalman filter uses as its time step. */
static void runTimer(void)
{
    uint32_t now;
    while ((now = HAL_GetTick()) - gPrevLoopTime < LOOP_TARGET_MS) { stlink_export_poll(); }
    gTimeDelta = now - gPrevLoopTime;
    gPrevLoopTime = now;
}

/* ***************** FILTER + LANDING ***************** */
/** @brief Feed raw altitude (ft) and acceleration (ft/s^2) to the Kalman filter; update the g*Filtered globals. */
static void filterData(float alt, float acc)
{
    kalman_update(&kf, (float)gTimeDelta / 1000.0f, alt, acc);
    gAltFiltered      = kf.x[0];
    gVelocityFiltered = kf.x[1];
    gAccelFiltered    = kf.x[2];
}

/** @brief True once the rocket has been still for LANDING_HOLD_MS (acceleration ~ 1 g and velocity ~ 0),
  * or LANDING_CUTOFF_MS after launch regardless. */
static bool detectLanding(void)
{
    static bool still = false;
    static uint32_t still_since = 0;
    uint32_t now = HAL_GetTick();

    if (now - gLaunchTime > LANDING_CUTOFF_MS) return true;

    bool is_still = fabsf(gAccelFiltered - GRAVITY) < LANDING_ACCEL_TOL &&
                    fabsf(gVelocityFiltered) < LANDING_VEL_TOL;
    if (!is_still) { still = false; return false; }
    if (!still)    { still = true; still_since = now; }
    return (now - still_since) >= LANDING_HOLD_MS;
}

static void confirmLanding(void)
{
    if (gLanded) return;

    gLanded = true;
    piezo_set_landed();
    uint32_t flight_time_ms = HAL_GetTick() - gLaunchTime;
    const char *reason = (flight_time_ms > LANDING_CUTOFF_MS)
                       ? "five-minute cutoff" : "stillness criteria";
    printf("[FLIGHT] Landing detected by %s at %lu ms after launch "
           "(filtered accel=%.2f ft/s^2, velocity=%.2f ft/s)\r\n",
           reason, (unsigned long)flight_time_ms,
           (double)gAccelFiltered, (double)gVelocityFiltered);
}

/* ***************** STARTUP ***************** */
void ats_init(void)
{
    stlink_export_init();
    piezo_init();
    piezo_set_ready(false);
    led_set(false);
    printf("\r\n[BOOT] Kingfisher ATS startup; status LED solid is reserved for verified READY\r\n");

    /* Park every chip-select high so no device on the shared SPI bus listens to the others' traffic */
    HAL_GPIO_WritePin(FLASH_SS_GPIO_Port, FLASH_SS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ALT_SS_GPIO_Port,   ALT_SS_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_SS_GPIO_Port,  LORA_SS_Pin,  GPIO_PIN_SET);

    /* Flash (data storage) */
    if (!flashlog_init(&hspi1, FLASH_SS_GPIO_Port, FLASH_SS_Pin)) fatal_fault(flashlog_last_error());
    piezo_play_startup_chime();
    boot_delay(1300u);
    piezo_set_ready(false);

#if ERASE_ON_BOOT
    if (!flashlog_erase_all(led_toggle)) fatal_fault("flash erase failed");
    led_set(true);
#endif

#if DUMP_MODE
    flashlog_dump_csv();
    printf("[DUMP] Complete; system NOT ARMED\r\n");
    while (1) HAL_Delay(100);
#endif

    /* Sensors */
    if (!imu_init(&hi2c1)) fatal_fault("IMU initialization/verification failed");
    if (!ms5611_init(&baro, &hspi1, ALT_SS_GPIO_Port, ALT_SS_Pin)) fatal_fault("barometer initialization/CRC check failed");
    HAL_Delay(10);                                          /* let the first pressure conversion finish */
    if (!ms5611_update(&baro)) fatal_fault("barometer did not produce a plausible first sample");
    if (!imu_read(&imu)) fatal_fault("IMU did not produce a valid first sample");
    printf("[BOOT] Barometer first sample: pressure_centi_hPa=%ld temp_centiC=%ld\r\n",
           (long)(baro.pressure_hpa * 100.0f), (long)(baro.temperature_c * 100.0f));

    /* Flaps */
    servo_init(&htim2, TIM_CHANNEL_3);
    testATS();

    /* Filter starts from the first real readings */
    kalman_init(&kf, ms5611_altitude_ft(baro.pressure_hpa), imu.accel_mag);
    gAltFiltered = kf.x[0];
    gVelocityFiltered = kf.x[1];
    gAccelFiltered = kf.x[2];

    gStartTime = HAL_GetTick();
    gLaunchTime = gStartTime;
    gPrevLoopTime = gStartTime;
    led_set(true);
    piezo_set_ready(true);
    printf("[READY] Flash, IMU and barometer checks verified; system armed. "
           "Servo has no feedback. Status LED is solid.\r\n");
}

/* ***************** MAIN LOOP BODY ***************** */
void ats_run_once(void)
{
    runTimer();                                             /* keeps the loop at a consistent rate */

    /* Get measurements */
    bool baro_ok = ms5611_update(&baro);
    bool imu_ok = imu_read(&imu);
    float alt_raw = ms5611_altitude_ft(baro.pressure_hpa);
    bool sensors_ok = baro_ok && imu_ok;
    if (sensors_ok) {
        filterData(alt_raw, imu.accel_mag);
        print_imu_debug(&imu);
    } else if (!gLaunched) {
        fatal_fault(!imu_ok ? "IMU sample failed before launch" : "barometer sample failed before launch");
    } else {
        if (!gFlightFault) {
            printf("[FAULT] Sensor sample failed in flight; retracting ATS\r\n");
            gFlightFault = true;
            piezo_set_fault();
        }
        gATSPosition = ATS_IN;
        setATSPosition(ATS_IN);
    }

    /* **** (PRE-FLIGHT) **** detect launch from the acceleration threshold */
    static uint8_t launch_confirm_count = 0;
    if (!gLaunched && sensors_ok && gAccelFiltered > ACCEL_THRESHOLD) {
        if (launch_confirm_count < LAUNCH_CONFIRM_SAMPLES) launch_confirm_count++;
    } else if (!gLaunched) {
        launch_confirm_count = 0;
    }
    if (launch_confirm_count >= LAUNCH_CONFIRM_SAMPLES && !gLaunched) {
        if (!flashlog_mark_launch()) fatal_fault("could not persist launch marker");
        gLaunched = true;
        gLaunchTime = HAL_GetTick();
        if (!attachATS()) fatal_fault("servo PWM start failed at launch");
        setATSPosition(ATS_IN);
        led_set(true);
        piezo_play_event(PIEZO_EVENT_LAUNCH);
        printf("[FLIGHT] Launch confirmed by %u consecutive valid acceleration samples\r\n",
               (unsigned)LAUNCH_CONFIRM_SAMPLES);
    }

    /* These are timer cues; this board has no motor burnout or apogee sensor. */
    static bool burnout_timer_announced = false;
    static bool retraction_timer_announced = false;
    if (gLaunched) {
        uint32_t flight_time_ms = HAL_GetTick() - gLaunchTime;
        if (!burnout_timer_announced && flight_time_ms >= ATS_ACTIVE_AFTER_MS) {
            burnout_timer_announced = true;
            printf("[FLIGHT] Burnout timer elapsed at %lu ms after launch\r\n", (unsigned long)flight_time_ms);
            piezo_play_event(PIEZO_EVENT_BURNOUT_TIMER);
        }
        if (!retraction_timer_announced && flight_time_ms >= ATS_RETRACT_AFTER_MS) {
            retraction_timer_announced = true;
            printf("[FLIGHT] Final retraction timer elapsed at %lu ms after launch\r\n", (unsigned long)flight_time_ms);
            piezo_play_event(PIEZO_EVENT_RETRACTION_TIMER);
        }
    }

    /* **** (DURING FLIGHT) **** adjust flaps, watch for landing */
    if (gLaunched && !gFlightFault) {
        adjustATS();
        if (detectLanding()) confirmLanding();
    } else if (gLaunched) {
        if (detectLanding()) confirmLanding();
        led_set(((HAL_GetTick() / 100u) & 1u) != 0u);       /* persistent fast fault blink */
    } else {
        /* Still on the pad: keep measuring the pad altitude so the target is "ALT_TARGET above the pad" */
        absolute_alt_target = ALT_TARGET + gAltFiltered;
    }

    /* **** LOGGING **** (shows what was commanded this cycle) */
    if (gLaunched || LOG_BEFORE_LAUNCH) {
        log_record_t rec = {
            .time_ms         = HAL_GetTick() - gStartTime,
            .pressure_hpa    = baro.pressure_hpa,
            .alt_raw_ft      = alt_raw,
            .accel_x         = imu.accel_x, .accel_y = imu.accel_y, .accel_z = imu.accel_z,
            .gyro_x          = imu.gyro_x,  .gyro_y  = imu.gyro_y,  .gyro_z  = imu.gyro_z,
            .alt_filt_ft     = gAltFiltered,
            .vel_filt_fps    = gVelocityFiltered,
            .accel_filt_fps2 = gAccelFiltered,
            .temp_c          = imu.temp_c,
            .ats_position    = gATSCommand,
            .alt_pred_ft     = gPredictedAltitude,
            .flags           = (gLaunched ? LOG_FLAG_LAUNCHED : 0u) |
                               (gFlightFault ? LOG_FLAG_FAULT : 0u),
        };
        if (!flashlog_append(&rec) && !gFlightFault) {
            if (!gLaunched) fatal_fault("flash logging failed before launch");
            printf("[FAULT] Flash logging failed in flight; retracting ATS\r\n");
            gFlightFault = true;
            piezo_set_fault();
            gATSPosition = ATS_IN;
            setATSPosition(ATS_IN);
        }
    }

    /* **** (END OF FLIGHT) **** */
    if (gLanded) {
        if (!flashlog_flush()) {
            printf("[FAULT] Flash: landing flush or verification failed\r\n");
            gFlightFault = true;
            piezo_set_fault();
        }
        if (!gFlightFault && !flashlog_mark_landed()) {
            gFlightFault = true;
            piezo_set_fault();
        }
        detachATS();
        while (1) {
            stlink_export_poll();
            piezo_update();
            led_set(((HAL_GetTick() / 500u) & 1u) != 0u);
            HAL_Delay(20);
        }
    }
    piezo_update();
    stlink_export_poll();
}
