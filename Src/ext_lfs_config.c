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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

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
static uint8_t lfs_lookahead_buf[LFS_MEM_lookahead_size];


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

static lfs_t lfs;
static SPI_HandleTypeDef hspi;
static char file_name[32] = "default.txt";

/**
 * @brief Sets up the filesystem
 */
void fs_Init(){
    MEM_Init(&hspi1);

    // Verify JEDEC ID
    uint32_t id = MEM_ReadJEDECID();
    if ((id & 0x00FFFFFF) != MEM_JEDEC_ID) {
        // Flash chip mismatch or hardware fault
        Error_Handler();
    }

    // 3. Mount littlefs (Format if first boot/mount fails)
    int err = lfs_mount(&lfs, &cfg);
    if (err) {
        // First boot or corrupt filesystem: format memory
        err = lfs_format(&lfs, &cfg);
        if (err == LFS_ERR_OK) {
            err = lfs_mount(&lfs, &cfg);
        }
    }

    if (err != LFS_ERR_OK) {
        // Failed to mount filesystem
        Error_Handler();
    }
}

/**
 * @brief Appends text data to the currently determined launch file.
 */
void writeData(const char *text) {
    if (text == NULL) {
        return;
    }

    lfs_file_t file;
    // Open in Read/Write, Append, and Create mode if it doesn't exist
    int err = lfs_file_open(&lfs, &file, file_name, LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);
    if (err >= 0) {
        lfs_file_write(&lfs, &file, text, strlen(text));
        lfs_file_close(&lfs, &file);
    }
}

/**
 * @brief Scans root directory for "launch_X.txt" files and sets file_name to "launch_<X+1>.txt".
 */
void DetermineWriteFile(void) {

    lfs_dir_t dir;
    struct lfs_info info;
    int latestLaunch = 0;

    // Open root directory
    int err = lfs_dir_open(&lfs, &dir, "/");
    if (err < 0) {
        // use default.txt if directory opening fails
    	return;
    }

    // Iterate through directory entries
    while (lfs_dir_read(&lfs, &dir, &info) > 0) {
        // Only inspect regular files (ignore directories)
        if (info.type == LFS_TYPE_REG) {
            if (strncmp(info.name, "launch_", 7) == 0) {
                // Extract number following "launch_"
                int curLaunchFileNumber = atoi(&info.name[7]);
                if (curLaunchFileNumber > latestLaunch) {
                    latestLaunch = curLaunchFileNumber;
                }
            }
        }
    }

    lfs_dir_close(&lfs, &dir);

    // Format new filename: launch_<latest + 1>.txt
    snprintf(file_name, sizeof(file_name), "launch_%d.txt", latestLaunch + 1);
}


