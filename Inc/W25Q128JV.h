/**
  ******************************************************************************
  * @file           : W25Q128JV.h
  * @brief          : Configuration details for the Winbond W25Q128JV NOR flash memory chip.
  ******************************************************************************
 *  Created on: Nov 7, 2026
 *      Author: jyzaki
  ******************************************************************************
  */

#ifndef W25Q128_H
#define W25Q128_H

// W25Q128JV Details
#define MEM_JEDEC_ID 				0xEF4018

// W25Q128JV Command Instructions
#define MEM_CMD_WRITE_ENABLE       0x06
#define MEM_CMD_WRITE_DISABLE      0x04
#define MEM_CMD_READ_STATUS_REG1   0x05
#define MEM_CMD_READ_DATA          0x03
#define MEM_CMD_PAGE_PROGRAM       0x02
#define MEM_CMD_SECTOR_ERASE_4K    0x20
#define MEM_CMD_JEDEC_ID           0x9F

// Device Specifications
#define MEM_PAGE_SIZE              256
#define MEM_SECTOR_SIZE            4096
#define MEM_SECTOR_COUNT           4096 //16 mb

#endif // W25Q128_H
