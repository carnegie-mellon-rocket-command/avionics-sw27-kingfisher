/**
  ******************************************************************************
  * @file           : lfs_config.h
  * @brief          : littlefs configuration and initialization
  *
  * 				  Configured for the Winbond W25Q128JV NOR flash memory chip.
  *
  *  				  Written for CMRC - Kingfisher (ATS):
  *  				  https://avionics.rocketcommand.org/projects/kingfisher
  *
  ******************************************************************************
 *  Created on: Nov 8, 2026
 *      Author: jyzaki
  ******************************************************************************
  */

#ifndef LFS_MEM_CONFIG
#define LFS_MEM_CONFIG

// Optimized littlefs Configuration for W25Q128JV (16 MB / 128 Mbit)
#define LFS_MEM_read_size      = 1,                // Byte-level read supported by NOR flash
#define LFS_MEM_prog_size      = 256,              // Matches W25Q128JV Page Size (256 B)
#define LFS_MEM_block_size     = 4096,             // Matches W25Q128JV Sector Size (4 KB)
#define LFS_MEM_block_count    = 4096,             // 16 MB / 4 KB = 4096 sectors
#define LFS_MEM_block_cycles   = 500,              // Wear-leveling threshold (500 cycles before evicting metadata)
#define LFS_MEM_cache_size     = 256,              // Must be a multiple of read_size and prog_size
#define LFS_MEM_lookahead_size = 64,               // 64 bytes * 8 bits = 512 blocks tracked per allocations pass

// Function Declarations
void fs_Init();
void writeData(const char *text);
void DetermineWriteFile(void);

#endif // LFS_MEM_CONFIG
