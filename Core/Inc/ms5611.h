/* ms5611.h -- SPI driver for the MS5611 barometer (Kingfisher board)
 *
 * Board wiring: SPI1 (shared with flash + LoRa), CS = PA4 (ALT_SS).
 * PS pin is tied to GND on the PCB, so the chip is in SPI mode.
 *
 * Usage:
 *   ms5611_t baro;
 *   ms5611_init(&baro, &hspi1, GPIOA, GPIO_PIN_4);   // once, at startup
 *   ...
 *   ms5611_update(&baro);   // call once per loop (>= 10 ms apart; your loop is 30 ms)
 *   baro.pressure_hpa, baro.temperature_c, ms5611_altitude_ft(baro.pressure_hpa)
 */
#ifndef MS5611_H
#define MS5611_H

#include "main.h"      /* pulls in HAL */
#include <stdbool.h>
#include <stdint.h>

#define SEA_LEVEL_PRESSURE_HPA 1013.25f
#define METERS_TO_FEET         3.28084f

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;

    uint16_t c[8];            /* PROM: c[1..6] are calibration constants */
    uint32_t d1;              /* last raw pressure ADC value */
    uint32_t d2;              /* last raw temperature ADC value */
    bool     conv_pending;    /* a conversion has been started */
    bool     pending_is_temp; /* ...and it is a temperature conversion */
    uint8_t  cycles_since_temp;

    float    pressure_hpa;    /* compensated pressure, hPa (mbar) */
    float    temperature_c;   /* compensated temperature, deg C */
    bool     valid;           /* true once both d1 and d2 have been read */
} ms5611_t;

/* Resets the chip, reads + CRC-checks the PROM, takes a first temperature reading.
 * Blocks ~20 ms. Returns false if the chip isn't responding or the CRC fails. */
bool  ms5611_init(ms5611_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Non-blocking. Reads the finished conversion, starts the next, recomputes outputs.
 * Pressure is converted every call; temperature every 16th call. */
bool  ms5611_update(ms5611_t *dev);

/* Standard barometric formula, same as Adafruit's readAltitude(). Returns feet MSL
 * (relative to SEA_LEVEL_PRESSURE_HPA; we only care about differences on the pad). */
float ms5611_altitude_ft(float pressure_hpa);

#endif
