/* W25Q128 littlefs logger; final sector holds the independent flight latch. */
#include "flashlog.h"
#include "ats_config.h"
#include "lfs.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#define LATCH_ADDRESS 0xFFF000u
static SPI_HandleTypeDef *spi;
static GPIO_TypeDef *cs_port;
static uint16_t cs_pin;
static lfs_t fs;
static lfs_file_t file;
static uint8_t read_cache[256],prog_cache[256],lookahead[64],file_cache[256];
static const struct lfs_file_config file_cfg={.buffer=file_cache};
static bool mounted,opened,failed,interrupted;
static uint32_t records,next_id;
static const char *last_error="flash not initialized";
const char *flashlog_last_error(void){return last_error;}
static bool transfer(uint8_t *cmd,unsigned n,void *data,unsigned size,bool write) {
    HAL_GPIO_WritePin(cs_port,cs_pin,GPIO_PIN_RESET);
    bool ok=HAL_SPI_Transmit(spi,cmd,n,20)==HAL_OK;
    if(ok && size) ok=(write?HAL_SPI_Transmit(spi,data,size,20):HAL_SPI_Receive(spi,data,size,20))==HAL_OK;
    HAL_GPIO_WritePin(cs_port,cs_pin,GPIO_PIN_SET); return ok;
}
static bool ready(void) {
    uint32_t start=HAL_GetTick();
    do {uint8_t cmd=5,status; if(!transfer(&cmd,1,&status,1,false))return false; if(!(status&1))return true;} while(HAL_GetTick()-start<500);
    return false;
}
static bool enable(void) {
    uint8_t cmd=6,status;
    if(!ready() || !transfer(&cmd,1,NULL,0,true))return false;
    cmd=5; return transfer(&cmd,1,&status,1,false) && (status&2);
}
static bool read_at(uint32_t addr,void *data,unsigned size) {
    uint8_t cmd[]={3,addr>>16,addr>>8,addr};
    return addr<=0x1000000u && size<=0x1000000u-addr && ready() && transfer(cmd,4,data,size,false);
}
static bool program(uint32_t addr,const void *data,unsigned size) {
    uint8_t cmd[]={2,addr>>16,addr>>8,addr},check[256];
    if(!size || size>256 || addr%256+size>256 || !enable())return false;
    return transfer(cmd,4,(void *)data,size,true) && read_at(addr,check,size) && !memcmp(check,data,size);
}
static bool erase(uint32_t addr) {
    uint8_t cmd[]={0x20,addr>>16,addr>>8,addr},check[256];
    if((addr&4095) || !enable() || !transfer(cmd,4,NULL,0,true) || !ready())return false;
    for(unsigned off=0;off<4096;off+=256) {
        if(!read_at(addr+off,check,256))return false;
        for(unsigned i=0;i<256;i++)if(check[i]!=255)return false;
    } return true;
}
static int bd_read(const struct lfs_config *c,lfs_block_t b,lfs_off_t off,void *data,lfs_size_t n) {
    if(b>=c->block_count || off>4096 || n>4096-off)return LFS_ERR_INVAL;
    return read_at(b*4096+off,data,n)?0:LFS_ERR_IO;
}
static int bd_prog(const struct lfs_config *c,lfs_block_t b,lfs_off_t off,const void *data,lfs_size_t n) {
    if(b>=c->block_count || off>4096 || n>4096-off)return LFS_ERR_INVAL;
    const uint8_t *p=data;
    while(n) {unsigned size=256-off%256; if(size>n)size=n;
        if(!program(b*4096+off,p,size))return LFS_ERR_IO;
        off+=size;p+=size;n-=size;
    }return 0;
}
static int bd_erase(const struct lfs_config *c,lfs_block_t b){return b<c->block_count && erase(b*4096)?0:LFS_ERR_IO;}
static int bd_sync(const struct lfs_config *c){(void)c;return ready()?0:LFS_ERR_IO;}
static const struct lfs_config cfg={.read=bd_read,.prog=bd_prog,.erase=bd_erase,.sync=bd_sync,
    .read_size=1,.prog_size=256,.block_size=4096,.block_count=4095,.block_cycles=500,
    .cache_size=256,.lookahead_size=64,.read_buffer=read_cache,.prog_buffer=prog_cache,.lookahead_buffer=lookahead};
bool flashlog_interrupted_flight(void){return interrupted;}
bool flashlog_clear_flight_latch(void){if(!erase(LATCH_ADDRESS))return false; interrupted=false;return true;}
static bool mark(unsigned off){const uint8_t zero=0;return program(LATCH_ADDRESS+off,&zero,1);}
bool flashlog_mark_launch(void){return mark(0);}
bool flashlog_mark_landed(void){return mark(1);}
static bool mount_fs(void) {
    last_error="littlefs mount failed (unformatted, old raw logs, or corrupt filesystem)";
    if(lfs_mount(&fs,&cfg)<0)return false;
    mounted=true;next_id=1;
    lfs_dir_t dir;struct lfs_info info;
    if(lfs_dir_open(&fs,&dir,"/")<0)return false;
    int rc;
    while((rc=lfs_dir_read(&fs,&dir,&info))>0) {
        unsigned id;char extra;
        if(sscanf(info.name,"F%6u.CSV%c",&id,&extra)==1 && id>=next_id)next_id=id+1;
    }
    int close_rc=lfs_dir_close(&fs,&dir);
    return rc==0 && close_rc==0 && next_id<=999999;
}
bool flashlog_init(SPI_HandleTypeDef *hspi,GPIO_TypeDef *port,uint16_t pin) {
    spi=hspi;cs_port=port;cs_pin=pin;opened=mounted=failed=false;records=0;
    last_error="flash JEDEC ID read failed or chip identity mismatched";
    HAL_GPIO_WritePin(port,pin,GPIO_PIN_SET);
    uint8_t cmd=0x9f,id[3],latch[2];
    if(!transfer(&cmd,1,id,3,false) || memcmp(id,"\xef\x40\x18",3))return false;
    last_error="flash flight-latch read failed";
    if(!read_at(LATCH_ADDRESS,latch,2))return false;
    interrupted=latch[0]!=255 && latch[1]!=0;
    if(!interrupted && latch[1]!=255 && !flashlog_clear_flight_latch())return false;
    /* Retraction remains available even if the filesystem was damaged. */
    if(interrupted)return true;
    {
        last_error="flash write-enable/write-disable verification failed";
        uint8_t status;
        if(!enable())return false;
        cmd=4;
        if(!transfer(&cmd,1,NULL,0,true))return false;
        cmd=5;
        if(!transfer(&cmd,1,&status,1,false) || (status&2))return false;
    }
#if ERASE_ON_BOOT
    return true;
#else
    return mount_fs();
#endif
}
bool flashlog_erase_all(void (*tick)(void)) {
    if(opened || interrupted)return false;
    if(mounted){if(lfs_unmount(&fs)<0)return false;mounted=false;}
    if(tick)tick();
    return lfs_format(&fs,&cfg)==0 && mount_fs();
}
static const char header[]="time_ms,pressure_milli_hPa,alt_milli_ft,ax_milli_ft_s2,ay_milli_ft_s2,az_milli_ft_s2,gx_milli_rad_s,gy_milli_rad_s,gz_milli_rad_s,alt_filt_milli_ft,velocity_milli_ft_s,accel_filt_milli_ft_s2,temp_milli_C,flap_milli,apogee_milli_ft,flags\r\n";
static long milli(float value) {
    if(!isfinite(value))return 0;
    double x=(double)value*1000;
    if(x>2147483647.0)return 2147483647L;
    if(x< -2147483647.0)return -2147483647L;
    return (long)x;
}
bool flashlog_append(const log_record_t *r) {
    if(!mounted || failed)return false;
    if(!opened) {
        char name[16];snprintf(name,sizeof(name),"F%06lu.CSV",(unsigned long)next_id);
        if(lfs_file_opencfg(&fs,&file,name,LFS_O_WRONLY|LFS_O_CREAT|LFS_O_EXCL,&file_cfg)<0){failed=true;return false;}
        opened=true;
        if(lfs_file_write(&fs,&file,header,sizeof(header)-1)!=(int)sizeof(header)-1){failed=true;return false;}
    }
    char line[256];
    int n=snprintf(line,sizeof(line),"%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%lu\r\n",
        (unsigned long)r->time_ms,milli(r->pressure_hpa),milli(r->alt_raw_ft),milli(r->accel_x),milli(r->accel_y),milli(r->accel_z),
        milli(r->gyro_x),milli(r->gyro_y),milli(r->gyro_z),milli(r->alt_filt_ft),milli(r->vel_filt_fps),milli(r->accel_filt_fps2),
        milli(r->temp_c),milli(r->ats_position),milli(r->alt_pred_ft),(unsigned long)r->flags);
    /* Commit every accepted row. Worst-case erase latency needs board measurement. */
    if(n<0 || n>=(int)sizeof(line) || lfs_file_write(&fs,&file,line,n)!=n || lfs_file_sync(&fs,&file)<0){failed=true;return false;}
    records++;return true;
}
bool flashlog_flush(void){return !failed && (!opened || lfs_file_sync(&fs,&file)==0);}
uint32_t flashlog_record_count(void){return records;}
int flashlog_file_info(unsigned index,char name[16],uint32_t *size) {
    if(!mounted)return -1;
    lfs_dir_t dir;struct lfs_info info;bool found=false;
    if(lfs_dir_open(&fs,&dir,"/")<0){failed=true;return -1;}
    int rc;
    while((rc=lfs_dir_read(&fs,&dir,&info))>0)if(info.type==LFS_TYPE_REG && strlen(info.name)<16 && index--==0){strcpy(name,info.name);*size=info.size;found=true;break;}
    if(lfs_dir_close(&fs,&dir)<0 || rc<0){failed=true;return -1;}
    return found?1:0;
}
bool flashlog_list(unsigned index,char name[16],uint32_t *size){return flashlog_file_info(index,name,size)==1;}
bool flashlog_readout_ok(void){return mounted;}
bool flashlog_scan(bool (*visit)(const char *,uint32_t,void *),void *context) {
    /* Interrupted-flight startup retracts first; USB may mount files afterward. */
    if(!mounted && interrupted && !mount_fs())return false;
    if(!flashlog_readout_ok())return false;
    lfs_dir_t dir;struct lfs_info info;bool ok=true;int rc;
    if(lfs_dir_open(&fs,&dir,"/")<0)return false;
    while((rc=lfs_dir_read(&fs,&dir,&info))>0) {
        if(info.type==LFS_TYPE_REG && visit && !visit(info.name,info.size,context)){ok=false;break;}
    }
    return lfs_dir_close(&fs,&dir)==0 && rc>=0 && ok;
}
bool flashlog_read_file(const char *name,uint32_t offset,void *data,uint16_t size) {
    if(!mounted)return false;
    lfs_file_t f;uint8_t cache[256];struct lfs_file_config fc={.buffer=cache};
    if(lfs_file_opencfg(&fs,&f,name,LFS_O_RDONLY,&fc)<0)return false;
    bool ok=lfs_file_seek(&fs,&f,offset,LFS_SEEK_SET)==(lfs_soff_t)offset && lfs_file_read(&fs,&f,data,size)==size;
    return lfs_file_close(&fs,&f)==0 && ok;
}
int _write(int fd,char *ptr,int len) {
    (void)fd;
#if DEBUG
    if((ITM->TCR&ITM_TCR_ITMENA_Msk) && (ITM->TER&1u)) {
        uint32_t start=HAL_GetTick();
        for(int i=0;i<len;i++) {
            /* Bound each write so a stalled trace receiver cannot hang flight control. */
            while(!ITM->PORT[0].u32) if(HAL_GetTick()-start>=5u)return len;
            ITM->PORT[0].u8=ptr[i];
        }
    }
#else
    (void)ptr;
#endif
    return len;
}
void flashlog_dump_csv(void) {
    char name[16],data[128];uint32_t size;
    for(unsigned i=0;flashlog_list(i,name,&size);i++) {
        printf("# %s\r\n",name);
        for(uint32_t off=0;off<size;){unsigned n=size-off;if(n>sizeof(data))n=sizeof(data);
            if(!flashlog_read_file(name,off,data,n))return;
            _write(1,data,n);off+=n;
        }
    }
}
