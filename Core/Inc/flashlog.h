/* flashlog.h -- flight-data logger for the W25Q128 SPI flash (Kingfisher board)
 *
 * Wiring: SPI1 (shared bus), CS = PA3 (FLASH_SS). 16 MB, 256-byte pages, 4 KB sectors.
 *
 * Each boot that logs data creates F000001.CSV, F000002.CSV, etc. on littlefs.
 * Rows are synchronized before append reports success. Program and erase operations
 * are verified. The last 4 KB sector is excluded from littlefs and holds the flight latch.
 * Filesystem commits can exceed the flight loop period; measure on the actual board.
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

/* Checks W25Q128 identity and mounts littlefs. Never formats on mount failure. */
bool     flashlog_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
const char *flashlog_last_error(void);

/* Appends one record. Returns false if the flash is full or a write timed out. */
bool     flashlog_append(const log_record_t *rec);

/* Synchronizes the current file. False means data durability could not be verified. */
bool     flashlog_flush(void);
bool     flashlog_interrupted_flight(void);
bool     flashlog_mark_launch(void);
bool     flashlog_mark_landed(void);
/* Call only after recovery, with the vehicle disarmed. Clears the reset fault latch. */
bool     flashlog_clear_flight_latch(void);
bool     flashlog_list(unsigned index, char name[16], uint32_t *size);
/* 1=file, 0=end, -1=directory/read error. */
int      flashlog_file_info(unsigned index, char name[16], uint32_t *size);
bool     flashlog_readout_ok(void);
bool     flashlog_scan(bool (*visit)(const char *, uint32_t, void *), void *context);
bool     flashlog_read_file(const char *name, uint32_t offset, void *data, uint16_t size);

/* Explicitly formats littlefs. Destroys files, preserves the independent flight latch.
 * Refuses while a file is open or the interrupted-flight latch is active. */
bool     flashlog_erase_all(void (*tick)(void));

/* Number of rows committed by this boot. */
uint32_t flashlog_record_count(void);

/* Dumps file contents through SWO when DEBUG=1. */
void     flashlog_dump_csv(void);

#endif
