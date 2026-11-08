/**
  ******************************************************************************
  * @file           : ext_flash_memory.h
  * @brief          : A driver to read/write from external flash memory over SPI.
  *
  *  				  Written for CMRC - Kingfisher (ATS):
  *  				  https://avionics.rocketcommand.org/projects/kingfisher
  *
  *  				  Update corresponding pin definitions and chip commands for use in other boards.
  ******************************************************************************
 *  Created on: Nov 7, 2026
 *      Author: jyzaki
  ******************************************************************************
  */

#ifndef EXT_FLASH_MEM
#define EXT_FLASH_MEM

#include "stm32f4xx_hal.h"
#include "W25Q128JV.h"

// Hardware Pin Definitions (Kingfisher)
#define MEM_CS_PORT                GPIOA
#define MEM_CS_PIN                 GPIO_PIN_4

// Function Declarations
void MEM_Init(SPI_HandleTypeDef *hspi);
uint32_t MEM_ReadJEDECID(void);
void MEM_ReadData(uint32_t address, uint8_t *buffer, uint16_t length);
void MEM_WritePage(uint32_t address, uint8_t *buffer, uint16_t length);
void MEM_EraseSector(uint32_t sector_address);

#endif // EXT_FLASH_MEM
