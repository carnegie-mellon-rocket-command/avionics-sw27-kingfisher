#include "stlink_export.h"
#include "flashlog.h"
#include <string.h>
#include <stddef.h>
volatile stlink_mailbox_t stlink_mailbox;
_Static_assert(offsetof(stlink_mailbox_t,data)==48,"Host mailbox layout changed");
void stlink_export_init(void) {
    memset((void *)&stlink_mailbox,0,sizeof(stlink_mailbox));
    stlink_mailbox.version=1;
    stlink_mailbox.magic=STLINK_EXPORT_MAGIC;
}
void stlink_export_poll(void) {
    uint32_t command=stlink_mailbox.command;
    if(!command)return;
    uint32_t status=3,size=0;
    char name[16]={0};
    if(command==1 || command==2) {
        if(!flashlog_readout_ok()) (void)flashlog_scan(NULL,NULL);
        int info=flashlog_file_info(stlink_mailbox.index,name,&size);
        if(info==1) {
            memcpy((void *)stlink_mailbox.name,name,sizeof(name));
            stlink_mailbox.file_size=size;
            status=1;
            if(command==2) {
                uint32_t off=stlink_mailbox.offset,n=stlink_mailbox.length;
                if(off>size || n>STLINK_EXPORT_BYTES || n>size-off)status=3;
                else if(!flashlog_read_file(name,off,(void *)stlink_mailbox.data,n))status=3;
            }
        } else if(info==0)status=2;
    }
    stlink_mailbox.command=0;
    __DMB();
    stlink_mailbox.status=status;
}
