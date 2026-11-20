/* flashlog.h -- flight-data logger for the W25Q128 SPI flash (Kingfisher board)
 *
 * Wiring: SPI1 (shared bus), CS = PA3 (FLASH_SS). 16 MB, 256-byte pages, 4 KB sectors.
 *
 * Design:
 *  - Fixed 64-byte binary records, 4 per page. No filesystem (erases stall; they would break the 30 ms loop).
 *  - Records are buffered in RAM and written one full page at a time. The page-program is started
 *    but NOT waited for; the next flash call waits for it (~1 ms), so the loop is never blocked long.
 *  - Data is always appended after existing data. The write pointer is found at boot by binary
 *    search for the first erased page, so a reset mid-flight just continues where it left off.
 *  - Erase is a separate, slow (~40 s, up to 200 s) step done on the pad -- NEVER in flight.
 *
 * Capacity at 33 Hz x 64 B = ~2.1 KB/s -> ~2 hours.
 */
#ifndef FLASHLOG_H
#define FLASHLOG_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define LOG_FLAG_LAUNCHED  (1u << 0)
#define LOG_FLAG_FAULT     (1u << 1)

typedef struct __attribute__((packed)) {
    uint32_t time_ms;          /* ms since boot */
    float pressure_hpa;
    float alt_raw_ft;
    float accel_x, accel_y, accel_z;   /* raw, ft/s^2 */
    float gyro_x, gyro_y, gyro_z;      /* rad/s */
    float alt_filt_ft;
    float vel_filt_fps;
    float accel_filt_fps2;
    float temp_c;              /* from IMU */
    float ats_position;        /* 0 = flaps in, 1 = flaps out */
    float alt_pred_ft;
    uint32_t flags;            /* LOG_FLAG_* */
} log_record_t;

_Static_assert(sizeof(log_record_t) == 64, "log_record_t must be 64 bytes (4 per 256-byte page)");

/* Checks the exact W25Q128 JEDEC ID, verifies the write-enable latch without
 * modifying data, and locates the write pointer. */
bool     flashlog_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* Appends one record. Returns false if the flash is full or a write timed out. */
bool     flashlog_append(const log_record_t *rec);

/* Writes out any partly filled page (unused bytes stay 0xFF). Call once after landing. */
void     flashlog_flush(void);

/* Erases the entire chip. Blocking, ~40 s. 'tick' (may be NULL) is called about every 100 ms
 * so you can blink an LED. After this the log is empty. */
bool     flashlog_erase_all(void (*tick)(void));

/* Number of records currently stored (including any buffered in RAM). */
uint32_t flashlog_record_count(void);

/* Prints every stored record as CSV through printf (ITM/SWO -- see flashlog.c).
 * Header matches last year's data format. */
void     flashlog_dump_csv(void);

#endif
