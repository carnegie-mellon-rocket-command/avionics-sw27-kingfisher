#include "flashlog.h"
#include <assert.h>
#include <string.h>
#include "stlink_export.h"
#include <stdio.h>

static unsigned char memory[16 * 1024 * 1024];
static unsigned address, command, tick;
static bool enabled, corrupt;
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, int state) {
    (void)port; (void)pin;
    if (!state) command = 0;
}
uint32_t HAL_GetTick(void) { return tick++; }
void HAL_Delay(uint32_t ms) { tick += ms; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t len, unsigned timeout) {
    (void)spi; (void)timeout;
    if (!command) {
        command = data[0];
        if (command == 6) enabled = true;
        if (command == 4) enabled = false;
        if (len >= 4) address = (data[1] << 16) | (data[2] << 8) | data[3];
        if (command == 0x20 && enabled) memset(memory + (address & ~4095u), 255, 4096);
        if (command == 2 && len == 5 && enabled) memory[address] &= data[4];
    } else if (command == 2 && enabled) {
        for (unsigned i = 0; i < len; ++i) memory[address+i] &= data[i];
        if (corrupt) memory[address] ^= 1;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t len, unsigned timeout) {
    (void)spi; (void)timeout;
    if (command == 0x9f) { data[0]=0xef; data[1]=0x40; data[2]=0x18; }
    else if (command == 5) data[0] = enabled ? 2 : 0;
    else if (command == 3) memcpy(data, memory + address, len);
    else return 1;
    return HAL_OK;
}
int main(int argc,char **argv) {
    SPI_HandleTypeDef spi;
    GPIO_TypeDef gpio;
    memset(memory, 255, sizeof(memory));
    assert(!flashlog_init(&spi, &gpio, 0));
    assert(flashlog_erase_all(NULL));
    assert(!flashlog_interrupted_flight());
    assert(flashlog_mark_launch());
    assert(flashlog_init(&spi, &gpio, 0));
    assert(flashlog_interrupted_flight());
    assert(flashlog_clear_flight_latch());
    assert(flashlog_mark_launch());
    assert(flashlog_mark_landed());
    assert(flashlog_init(&spi, &gpio, 0));
    assert(!flashlog_interrupted_flight());
    log_record_t record = {.time_ms=42,.pressure_hpa=1000,.ats_position=0.5f};
    for(unsigned i=0;i<100;i++) assert(flashlog_append(&record));
    assert(flashlog_flush());
    stlink_export_init();
    stlink_mailbox.index=0;stlink_mailbox.command=1;
    stlink_export_poll();
    assert(stlink_mailbox.status==1 && !strcmp((const char *)stlink_mailbox.name,"F000001.CSV"));
    uint32_t original_size=stlink_mailbox.file_size;
    stlink_mailbox.offset=0;stlink_mailbox.length=128;stlink_mailbox.command=2;
    stlink_export_poll();
    assert(stlink_mailbox.status==1 && !memcmp((const void *)stlink_mailbox.data,"time_ms,",8));
    for(unsigned i=0;i<50;i++) assert(flashlog_append(&record));
    stlink_mailbox.command=1;stlink_export_poll();
    assert(stlink_mailbox.status==1 && stlink_mailbox.file_size>original_size);
    stlink_mailbox.length=STLINK_EXPORT_BYTES+1;stlink_mailbox.command=2;stlink_export_poll();
    assert(stlink_mailbox.status==3);
    stlink_mailbox.index=999;stlink_mailbox.command=1;stlink_export_poll();
    assert(stlink_mailbox.status==2);
    assert(flashlog_init(&spi,&gpio,0));
    char name[16], contents[256];uint32_t size;
    assert(flashlog_list(0,name,&size));
    assert(!strcmp(name,"F000001.CSV") && size>1024);
    assert(flashlog_read_file(name,0,contents,sizeof(contents)));
    assert(!memcmp(contents,"time_ms,",8));
    if(argc>1) {
        FILE *image=fopen(argv[1],"wb");assert(image);
        for(uint32_t offset=0;offset<size;) {
            unsigned n=size-offset;if(n>sizeof(contents))n=sizeof(contents);
            assert(flashlog_read_file(name,offset,contents,n));
            assert(fwrite(contents,1,n,image)==n);offset+=n;
        }
        assert(fclose(image)==0);
    }
    corrupt = true;
    assert(!flashlog_mark_launch());
    return 0;
}
