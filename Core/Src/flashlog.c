/* flashlog.c -- see flashlog.h */
#include "flashlog.h"
#include <stdio.h>
#include <string.h>

#define CMD_WRITE_ENABLE  0x06
#define CMD_WRITE_DISABLE 0x04
#define CMD_READ_STATUS1  0x05
#define CMD_PAGE_PROGRAM  0x02
#define CMD_READ_DATA     0x03
#define CMD_CHIP_ERASE    0xC7
#define CMD_JEDEC_ID      0x9F

#define PAGE_SIZE         256u
#define TOTAL_PAGES       65536u                 /* 16 MB / 256 B */
#define REC_PER_PAGE      (PAGE_SIZE / sizeof(log_record_t))
#define SPI_TIMEOUT_MS    20

static SPI_HandleTypeDef *hspi;
static GPIO_TypeDef      *cs_port;
static uint16_t           cs_pin;

static uint32_t next_page   = 0;                 /* next page to program */
static uint8_t  page_buf[PAGE_SIZE];
static uint32_t buf_len     = 0;                 /* bytes currently in page_buf */
static bool     full        = false;

static inline void cs_low(void)  { HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET); }

/* ---------- low-level flash ops ---------- */

static bool wait_ready(uint32_t timeout_ms, void (*tick)(void))
{
    uint32_t start = HAL_GetTick(), last_tick = start;
    uint8_t cmd = CMD_READ_STATUS1, st = 0x01;
    while (st & 0x01) {                          /* bit 0 = BUSY */
        cs_low();
        HAL_StatusTypeDef tx = HAL_SPI_Transmit(hspi, &cmd, 1, SPI_TIMEOUT_MS);
        HAL_StatusTypeDef rx = HAL_SPI_Receive(hspi, &st, 1, SPI_TIMEOUT_MS);
        cs_high();
        if (tx != HAL_OK || rx != HAL_OK) return false;
        if (!(st & 0x01)) break;
        uint32_t now = HAL_GetTick();
        if (now - start > timeout_ms) return false;
        if (tick && now - last_tick >= 100) { tick(); last_tick = now; }
    }
    return true;
}

static bool write_enable(void)
{
    uint8_t cmd = CMD_WRITE_ENABLE;
    cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &cmd, 1, SPI_TIMEOUT_MS);
    cs_high();
    return status == HAL_OK;
}

static bool read_status1(uint8_t *status)
{
    uint8_t cmd = CMD_READ_STATUS1;
    cs_low();
    HAL_StatusTypeDef tx = HAL_SPI_Transmit(hspi, &cmd, 1, SPI_TIMEOUT_MS);
    HAL_StatusTypeDef rx = HAL_SPI_Receive(hspi, status, 1, SPI_TIMEOUT_MS);
    cs_high();
    return tx == HAL_OK && rx == HAL_OK;
}

static bool write_disable(void)
{
    uint8_t cmd = CMD_WRITE_DISABLE;
    cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &cmd, 1, SPI_TIMEOUT_MS);
    cs_high();
    return status == HAL_OK;
}

/* Starts a page program and returns immediately; the next flash call waits for it. */
static bool page_program(uint32_t page, const uint8_t *data)
{
    uint32_t addr = page * PAGE_SIZE;
    uint8_t hdr[4] = { CMD_PAGE_PROGRAM, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    if (!wait_ready(100, NULL)) return false;
    if (!write_enable()) return false;
    cs_low();
    HAL_StatusTypeDef header_status = HAL_SPI_Transmit(hspi, hdr, 4, SPI_TIMEOUT_MS);
    HAL_StatusTypeDef data_status = HAL_SPI_Transmit(hspi, (uint8_t *)data, PAGE_SIZE, SPI_TIMEOUT_MS);
    cs_high();
    return header_status == HAL_OK && data_status == HAL_OK;
}

static bool read_bytes(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t hdr[4] = { CMD_READ_DATA, (uint8_t)(addr >> 16), (uint8_t)(addr >> 8), (uint8_t)addr };
    if (!wait_ready(100, NULL)) return false;
    cs_low();
    HAL_StatusTypeDef header_status = HAL_SPI_Transmit(hspi, hdr, 4, SPI_TIMEOUT_MS);
    HAL_StatusTypeDef data_status = HAL_SPI_Receive(hspi, buf, len, SPI_TIMEOUT_MS);
    cs_high();
    return header_status == HAL_OK && data_status == HAL_OK;
}

/* A page counts as erased if its first 4 bytes are all 0xFF (a record's time_ms is never 0xFFFFFFFF). */
static bool page_is_erased(uint32_t page, bool *erased)
{
    uint8_t b[4];
    if (!read_bytes(page * PAGE_SIZE, b, 4)) return false;
    *erased = b[0] == 0xFF && b[1] == 0xFF && b[2] == 0xFF && b[3] == 0xFF;
    return true;
}

/* ---------- public API ---------- */

bool flashlog_init(SPI_HandleTypeDef *spi, GPIO_TypeDef *port, uint16_t pin)
{
    hspi = spi; cs_port = port; cs_pin = pin;
    cs_high();
    buf_len = 0;
    full = false;

    uint8_t cmd = CMD_JEDEC_ID, id[3] = {0, 0, 0};
    cs_low();
    HAL_StatusTypeDef tx = HAL_SPI_Transmit(hspi, &cmd, 1, SPI_TIMEOUT_MS);
    HAL_StatusTypeDef rx = HAL_SPI_Receive(hspi, id, 3, SPI_TIMEOUT_MS);
    cs_high();
    if (tx != HAL_OK || rx != HAL_OK) return false;
    if (id[0] != 0xEF || id[1] != 0x40 || id[2] != 0x18) return false; /* W25Q128 */
    printf("[BOOT] Flash JEDEC ID %02X %02X %02X verified\r\n", id[0], id[1], id[2]);

    /* Non-destructively verify that the flash accepts write commands. */
    uint8_t status;
    if (!write_enable() || !read_status1(&status) || !(status & 0x02u) ||
        !write_disable() || !read_status1(&status) || (status & 0x02u)) return false;
    printf("[BOOT] Flash write-enable latch verified (no data modified)\r\n");

    /* Binary search for the first erased page (data is contiguous from page 0) */
    uint32_t lo = 0, hi = TOTAL_PAGES;
    while (lo < hi) {
        uint32_t mid = (lo + hi) / 2;
        bool erased;
        if (!page_is_erased(mid, &erased)) return false;
        if (erased) hi = mid;
        else        lo = mid + 1;
    }
    next_page = lo;
    full = (next_page >= TOTAL_PAGES);
    return true;
}

bool flashlog_append(const log_record_t *rec)
{
    if (full) return false;
    memcpy(&page_buf[buf_len], rec, sizeof(log_record_t));
    buf_len += sizeof(log_record_t);

    if (buf_len >= PAGE_SIZE) {
        bool ok = page_program(next_page, page_buf);
        if (!ok) { full = true; return false; }
        buf_len = 0;
        next_page++;
        if (next_page >= TOTAL_PAGES) full = true;
        return true;
    }
    return true;
}

void flashlog_flush(void)
{
    if (full || buf_len == 0) return;
    memset(&page_buf[buf_len], 0xFF, PAGE_SIZE - buf_len);
    if (!page_program(next_page, page_buf)) {
        printf("[FAULT] Flash: failed to flush final page\r\n");
        return;
    }
    next_page++;
    buf_len = 0;
    if (next_page >= TOTAL_PAGES) full = true;
    if (!wait_ready(100, NULL)) printf("[FAULT] Flash: final page did not become ready\r\n");
}

bool flashlog_erase_all(void (*tick)(void))
{
    uint8_t cmd = CMD_CHIP_ERASE;
    if (!wait_ready(100, NULL)) return false;
    if (!write_enable()) return false;
    cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &cmd, 1, SPI_TIMEOUT_MS);
    cs_high();
    if (status != HAL_OK) return false;
    if (!wait_ready(250000, tick)) return false;        /* datasheet max ~200 s */
    next_page = 0;
    buf_len = 0;
    full = false;
    return true;
}

uint32_t flashlog_record_count(void)
{
    return next_page * REC_PER_PAGE + buf_len / sizeof(log_record_t);
}

/* printf -> SWO/ITM. Needs: CubeMX SYS Debug = "Trace Asynchronous Sw", SWV enabled in the
 * debug configuration, and "Use float with printf from newlib-nano" ticked in the linker settings. */
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) ITM_SendChar((uint32_t)ptr[i]);
    return len;
}

void flashlog_dump_csv(void)
{
    uint8_t page[PAGE_SIZE];
    printf("time (ms), pressure (hPa), altitude_raw (ft), acceleration_raw_x (ft/s^2), "
           "acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, "
           "gAltFiltered (ft), gVelocityFiltered (ft/s), gAccelFiltered (ft/s^2), "
           "temperature (from IMU; degrees C), gATSPosition (0=in 1=out), gAltPredicted (ft)\n");

    for (uint32_t p = 0; p < next_page; p++) {
        if (!read_bytes(p * PAGE_SIZE, page, PAGE_SIZE)) { printf("# read error page %lu\n", (unsigned long)p); continue; }
        for (uint32_t r = 0; r < REC_PER_PAGE; r++) {
            log_record_t rec;
            memcpy(&rec, &page[r * sizeof(log_record_t)], sizeof(rec));
            if (rec.time_ms == 0xFFFFFFFFu) continue;           /* unused slot in a flushed partial page */
            printf("%lu,%.2f,%.2f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%.2f,%.3f,%.2f\n",
                   (unsigned long)rec.time_ms, rec.pressure_hpa, rec.alt_raw_ft,
                   rec.accel_x, rec.accel_y, rec.accel_z,
                   rec.gyro_x, rec.gyro_y, rec.gyro_z,
                   rec.alt_filt_ft, rec.vel_filt_fps, rec.accel_filt_fps2,
                   rec.temp_c, rec.ats_position, rec.alt_pred_ft);
            HAL_Delay(1);                                       /* keep the SWO link from overflowing */
        }
    }
    printf("# end of data\n");
}
