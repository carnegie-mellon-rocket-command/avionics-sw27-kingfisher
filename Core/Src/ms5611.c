/* ms5611.c -- see ms5611.h. Compensation follows the MS5611-01BA03 datasheet. */
#include "ms5611.h"
#include <math.h>
#include <stdio.h>

#define CMD_RESET      0x1E
#define CMD_CONV_D1    0x48   /* pressure,    OSR 4096 (max conversion time ~9.04 ms) */
#define CMD_CONV_D2    0x58   /* temperature, OSR 4096 */
#define CMD_ADC_READ   0x00
#define CMD_PROM_READ  0xA0   /* + 2 * address (0..7) */
#define SPI_TIMEOUT_MS 10
#define TEMP_EVERY_N   16

static inline void cs_low(ms5611_t *d)  { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET); }
static inline void cs_high(ms5611_t *d) { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET); }

static bool write_cmd(ms5611_t *d, uint8_t cmd)
{
    cs_low(d);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(d->hspi, &cmd, 1, SPI_TIMEOUT_MS);
    cs_high(d);
    return status == HAL_OK;
}

static bool read_prom(ms5611_t *d, uint8_t addr, uint16_t *value)
{
    uint8_t cmd = CMD_PROM_READ + 2 * addr;
    uint8_t rx[2] = {0, 0};
    cs_low(d);
    HAL_StatusTypeDef tx = HAL_SPI_Transmit(d->hspi, &cmd, 1, SPI_TIMEOUT_MS);
    HAL_StatusTypeDef read = HAL_SPI_Receive(d->hspi, rx, 2, SPI_TIMEOUT_MS);
    cs_high(d);
    if (tx != HAL_OK || read != HAL_OK) return false;
    *value = (uint16_t)((rx[0] << 8) | rx[1]);
    return true;
}

static bool read_adc(ms5611_t *d, uint32_t *value)
{
    uint8_t cmd = CMD_ADC_READ;
    uint8_t rx[3] = {0, 0, 0};
    cs_low(d);
    HAL_StatusTypeDef tx = HAL_SPI_Transmit(d->hspi, &cmd, 1, SPI_TIMEOUT_MS);
    HAL_StatusTypeDef read = HAL_SPI_Receive(d->hspi, rx, 3, SPI_TIMEOUT_MS);
    cs_high(d);
    if (tx != HAL_OK || read != HAL_OK) return false;
    *value = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
    return true;
}

/* CRC-4 from Application Note AN520 */
static uint8_t prom_crc4(const uint16_t prom_in[8])
{
    uint16_t prom[8];
    uint16_t rem = 0;
    for (int i = 0; i < 8; i++) prom[i] = prom_in[i];
    prom[7] &= 0xFF00;                       /* CRC byte excluded from calculation */
    for (int cnt = 0; cnt < 16; cnt++) {
        if (cnt & 1) rem ^= (prom[cnt >> 1] & 0x00FF);
        else         rem ^= (prom[cnt >> 1] >> 8);
        for (int bit = 8; bit > 0; bit--) {
            if (rem & 0x8000) rem = (rem << 1) ^ 0x3000;
            else              rem = (rem << 1);
        }
    }
    return (uint8_t)((rem >> 12) & 0x0F);
}

static void compute(ms5611_t *d)
{
    int32_t dT   = (int32_t)d->d2 - ((int32_t)d->c[5] << 8);
    int32_t temp = 2000 + (int32_t)(((int64_t)dT * d->c[6]) >> 23);   /* 0.01 deg C */
    int64_t off  = ((int64_t)d->c[2] << 16) + (((int64_t)d->c[4] * dT) >> 7);
    int64_t sens = ((int64_t)d->c[1] << 15) + (((int64_t)d->c[3] * dT) >> 8);

    /* Second-order temperature compensation (below 20 C) */
    if (temp < 2000) {
        int64_t t2    = ((int64_t)dT * dT) >> 31;
        int64_t dt    = temp - 2000;
        int64_t off2  = (5 * dt * dt) >> 1;
        int64_t sens2 = (5 * dt * dt) >> 2;
        if (temp < -1500) {
            int64_t dl = temp + 1500;
            off2  += 7 * dl * dl;
            sens2 += (11 * dl * dl) >> 1;
        }
        temp -= (int32_t)t2;
        off  -= off2;
        sens -= sens2;
    }

    int32_t p = (int32_t)((((int64_t)d->d1 * sens >> 21) - off) >> 15);  /* 0.01 mbar */
    d->temperature_c = temp / 100.0f;
    d->pressure_hpa  = p / 100.0f;
}

bool ms5611_init(ms5611_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->d1 = dev->d2 = 0;
    dev->conv_pending = dev->pending_is_temp = false;
    dev->cycles_since_temp = 0;
    dev->valid = false;
    cs_high(dev);

    if (!write_cmd(dev, CMD_RESET)) return false;
    HAL_Delay(5);                            /* datasheet: reload takes 2.8 ms */

    for (int i = 0; i < 8; i++) {
        if (!read_prom(dev, (uint8_t)i, &dev->c[i])) return false;
    }

    /* All-zero or all-one PROM means the chip isn't answering (wiring / CS / SPI mode) */
    if ((dev->c[1] == 0 && dev->c[2] == 0) || (dev->c[1] == 0xFFFF && dev->c[2] == 0xFFFF)) return false;
    if (prom_crc4(dev->c) != (dev->c[7] & 0x0F)) return false;

    /* First temperature reading (blocking) so the first pressure result is valid */
    if (!write_cmd(dev, CMD_CONV_D2)) return false;
    HAL_Delay(10);
    if (!read_adc(dev, &dev->d2) || dev->d2 == 0) return false;

    /* Kick off the first pressure conversion for ms5611_update() to collect */
    if (!write_cmd(dev, CMD_CONV_D1)) return false;
    dev->conv_pending = true;
    dev->pending_is_temp = false;
    printf("[BOOT] Barometer PROM CRC and initial temperature conversion verified\r\n");
    return true;
}

bool ms5611_update(ms5611_t *dev)
{
    if (dev->conv_pending) {
        uint32_t v;
        if (!read_adc(dev, &v)) return false;
        if (v == 0) return false;
        if (dev->pending_is_temp) dev->d2 = v;
        else                      dev->d1 = v;
    }

    /* Start the next conversion: temperature every TEMP_EVERY_N cycles, otherwise pressure */
    if (++dev->cycles_since_temp >= TEMP_EVERY_N) {
        dev->cycles_since_temp = 0;
        if (!write_cmd(dev, CMD_CONV_D2)) return false;
        dev->pending_is_temp = true;
    } else {
        if (!write_cmd(dev, CMD_CONV_D1)) return false;
        dev->pending_is_temp = false;
    }
    dev->conv_pending = true;

    if (dev->d1 != 0 && dev->d2 != 0) {
        compute(dev);
        dev->valid = isfinite(dev->pressure_hpa) && isfinite(dev->temperature_c) &&
                     dev->pressure_hpa >= 100.0f && dev->pressure_hpa <= 1200.0f &&
                     dev->temperature_c >= -50.0f && dev->temperature_c <= 85.0f;
    }
    return dev->valid;
}

float ms5611_altitude_ft(float pressure_hpa)
{
    float alt_m = 44330.0f * (1.0f - powf(pressure_hpa / SEA_LEVEL_PRESSURE_HPA, 0.1903f));
    return alt_m * METERS_TO_FEET;
}
