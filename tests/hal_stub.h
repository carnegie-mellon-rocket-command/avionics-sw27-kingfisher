#ifndef TEST_HAL_STUB_H
#define TEST_HAL_STUB_H
#define __MAIN_H
#define ATS_CONFIG_H
#define DEBUG 0
#define __DMB() ((void)0)
#define ERASE_ON_BOOT 0
#include <stdint.h>
#include <stdbool.h>
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, int);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, uint8_t *, uint16_t, unsigned);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *, uint8_t *, uint16_t, unsigned);
#endif
