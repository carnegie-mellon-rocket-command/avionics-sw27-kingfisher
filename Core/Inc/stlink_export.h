#ifndef STLINK_EXPORT_H
#define STLINK_EXPORT_H
#include <stdint.h>
#define STLINK_EXPORT_MAGIC 0x4b464c47u
#define STLINK_EXPORT_BYTES 4096u
/* Host writes request fields, then command last. Firmware publishes status last. */
typedef struct {
    uint32_t magic, version, command, status, index, offset, length, file_size;
    char name[16];
    uint8_t data[STLINK_EXPORT_BYTES];
} stlink_mailbox_t;
extern volatile stlink_mailbox_t stlink_mailbox;
void stlink_export_init(void);
void stlink_export_poll(void);
#endif
