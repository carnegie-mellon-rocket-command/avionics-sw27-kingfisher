/**
  ******************************************************************************
  * @file           : ext_flash_memory.c
  * @brief          : A driver to read/write from external flash memory over SPI.
  *
  *  				  Written for CMRC - Kingfisher (ATS):
  *  				  https://avionics.rocketcommand.org/projects/kingfisher
  *
  *  				  Note that writing/erasing will cause the code to block until
  *  				  the flash finishes the respective operation.
  *  				  Consider using interrupts if this becomes an issue.
  *
  ******************************************************************************
 *  Created on: Nov 7, 2026
 *      Author: jyzaki
  ******************************************************************************
  */

#include "ext_flash_mem.h"

static SPI_HandleTypeDef *MEM_spi;

// Chip Select Control
static inline void MEM_Select(void) {
    HAL_GPIO_WritePin(MEM_CS_PORT, MEM_CS_PIN, GPIO_PIN_RESET);
}

static inline void MEM_Deselect(void) {
    HAL_GPIO_WritePin(MEM_CS_PORT, MEM_CS_PIN, GPIO_PIN_SET);
}

// Send Write Enable instruction (required before Page Program or Erase)
static void MEM_WriteEnable(void) {
    uint8_t cmd = MEM_CMD_WRITE_ENABLE;
    MEM_Select();
    HAL_SPI_Transmit(MEM_spi, &cmd, 1, HAL_MAX_DELAY);
    MEM_Deselect();
}

// Poll Status Register 1 until Busy (BUSY / S0) bit is cleared
static void MEM_WaitWhileBusy(void) {
    uint8_t cmd = MEM_CMD_READ_STATUS_REG1;
    uint8_t status = 0;

    MEM_Select();
    HAL_SPI_Transmit(MEM_spi, &cmd, 1, HAL_MAX_DELAY);
    do {
        HAL_SPI_Receive(MEM_spi, &status, 1, HAL_MAX_DELAY);
    } while (status & 0x01); // Bit 0 is BUSY flag
    MEM_Deselect();
}

// Initialize GPIO and SPI peripheral pointer
void MEM_Init(SPI_HandleTypeDef *hspi) {
    MEM_spi = hspi;
    MEM_Deselect(); // Ensure CS starts HIGH
}

// Read Manufacturer & Device ID
uint32_t MEM_ReadJEDECID(void) {
    uint8_t cmd = MEM_CMD_JEDEC_ID;
    uint8_t id_buf[3] = {0};

    MEM_Select();
    HAL_SPI_Transmit(MEM_spi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(MEM_spi, id_buf, 3, HAL_MAX_DELAY);
    MEM_Deselect();

    return ((uint32_t)id_buf[0] << 16) | ((uint32_t)id_buf[1] << 8) | id_buf[2];
}

// Read arbitrary length of data starting at 24-bit address
void MEM_ReadData(uint32_t address, uint8_t *buffer, uint16_t length) {
    uint8_t header[4] = {
        MEM_CMD_READ_DATA,
        (uint8_t)(address >> 16),
        (uint8_t)(address >> 8),
        (uint8_t)(address)
    };

    MEM_Select();
    HAL_SPI_Transmit(MEM_spi, header, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(MEM_spi, buffer, length, HAL_MAX_DELAY);
    MEM_Deselect();
}

// Write up to 256 bytes to a single page
void MEM_WritePage(uint32_t address, uint8_t *buffer, uint16_t length) {
    if (length > MEM_PAGE_SIZE) length = MEM_PAGE_SIZE;

    MEM_WriteEnable();

    uint8_t header[4] = {
        MEM_CMD_PAGE_PROGRAM,
        (uint8_t)(address >> 16),
        (uint8_t)(address >> 8),
        (uint8_t)(address)
    };

    MEM_Select();
    HAL_SPI_Transmit(MEM_spi, header, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(MEM_spi, buffer, length, HAL_MAX_DELAY);
    MEM_Deselect();

    MEM_WaitWhileBusy();
}

// Erase a 4 KB sector
void MEM_EraseSector(uint32_t sector_address) {
    MEM_WriteEnable();

    uint8_t header[4] = {
        MEM_CMD_SECTOR_ERASE_4K,
        (uint8_t)(sector_address >> 16),
        (uint8_t)(sector_address >> 8),
        (uint8_t)(sector_address)
    };

    MEM_Select();
    HAL_SPI_Transmit(MEM_spi, header, 4, HAL_MAX_DELAY);
    MEM_Deselect();

    MEM_WaitWhileBusy();
}

