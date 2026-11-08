/**
  ******************************************************************************
  * @file           : lfs_config.c
  * @brief          : littlefs configuration and initialization
  *
  *  				  Written for CMRC - Kingfisher (ATS):
  *  				  https://avionics.rocketcommand.org/projects/kingfisher
  *
  ******************************************************************************
 *  Created on: Nov 8, 2026
 *      Author: jyzaki
  ******************************************************************************
  */

#include "lfs.h"
#include "ext_flash_mem.h"
#include "ext_lfs_config.h"

// lfs Function Implementations

static int lfs_mem_read(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, void *buffer, lfs_size_t size) {
    uint32_t address = (block * c->block_size) + off;
    MEM_ReadData(address, (uint8_t *)buffer, (uint16_t)size);
    return LFS_ERR_OK;
}

static int lfs_mem_prog(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, const void *buffer, lfs_size_t size) {
    uint32_t address = (block * c->block_size) + off;
    uint32_t bytes_written = 0;

    // Handle writes that span across 256-byte page boundaries
    while (bytes_written < size) {
        uint3t current_addr = address + bytes_written;
        uint16_t chunk_size = MEM_PAGE_SIZE - (current_addr % MEM_PAGE_SIZE);

        if (chunk_size > (size - bytes_written)) {
            chunk_size = (uint16_t)(size - bytes_written);
        }

        MEM_WritePage(current_addr, (uint8_t *)buffer + bytes_written, chunk_size);
        bytes_written += chunk_size;
    }

    return LFS_ERR_OK;
}

static int lfs_mem_erase(const struct lfs_config *c, lfs_block_t block) {
    uint32_t sector_address = block * c->block_size;
    MEM_EraseSector(sector_address);
    return LFS_ERR_OK;
}

static int lfs_mem_sync(const struct lfs_config *c) {
    // MEM_WritePage and MEM_EraseSector already wait for BUSY to clear,
    // so no hardware sync is required here.
    (void)c;
    return LFS_ERR_OK;
}

// Read buffer and program buffer for littlefs internal caching
static uint8_t lfs_read_buf[LFS_MEM_cache_size];
static uint8_t lfs_prog_buf[LFS_MEM_cache_size];
static uint8_t lfs_lookahead_buf[LFS_MEM_lookahead_size]; // 64 bytes = 512 blocks tracking


const struct lfs_config cfg = {
    // Block Device Drivers
    .read  = lfs_mem_read,
    .prog  = lfs_mem_prog,
    .erase = lfs_mem_erase,
    .sync  = lfs_mem_sync,

    // Hardware Specs & Block Device Configuration
    .read_size      = LFS_MEM_read_size,
    .prog_size      = LFS_MEM_prog_size,
    .block_size     = LFS_MEM_block_size,
    .block_count    = LFS_MEM_block_count,
    .block_cycles   = LFS_MEM_block_cycles,
    .cache_size     = LFS_MEM_cache_size,
    .lookahead_size = LFS_MEM_lookahead_size,

    // Static Buffers (Prevents dynamic allocation overhead)
    .read_buffer      = lfs_read_buf,
    .prog_buffer      = lfs_prog_buf,
    .lookahead_buffer = lfs_lookahead_buf,
};

//simple function to set up the filesystem
void fs_Init(SPI_HandleTypeDef* hspi1,lfs_t* lfs){
    MEM_Init(hspi1);

    // 2. Optional: Verify JEDEC ID (0xEF4018)
    uint32_t id = MEM_ReadJEDECID();
    if ((id & 0x00FFFFFF) != MEM_JEDEC_ID) {
        // Flash chip mismatch or hardware fault
        Error_Handler();
    }

    // 3. Mount littlefs (Format if first boot/mount fails)
    int err = lfs_mount(lfs, &cfg);
    if (err) {
        // First boot or corrupt filesystem: format memory
        err = lfs_format(lfs, &cfg);
        if (err == LFS_ERR_OK) {
            err = lfs_mount(lfs, &cfg);
        }
    }

    if (err != LFS_ERR_OK) {
        // Failed to mount filesystem
        Error_Handler();
    }
}



