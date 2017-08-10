/** 
  ******************************************************************************
  * @file    stm32f4xx_nucleo.h
  * @author  MCD Application Team
  * @version V1.2.4
  * @date    12-January-2016
  * @brief   This file contains definitions for:
  *          - LEDs and push-button available on STM32F4XX-Nucleo Kit 
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD 
  *            shield (reference ID 802)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************  
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_SPI_FLASH_H__
#define __DRV_SPI_FLASH_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
   
#define HAL_SPI_MODULE_ENABLED

/*############################### SPI2 #######################################*/
#ifdef HAL_SPI_MODULE_ENABLED

#define Flash_SPIx                                     SPI1
#define Flash_SPIx_CLK_ENABLE()                        __HAL_RCC_SPI1_CLK_ENABLE()

#define Flash_SPIx_SCK_GPIO_PORT                       GPIOA
#define Flash_SPIx_SCK_PIN                             GPIO_PIN_5
#define Flash_SPIx_SCK_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define Flash_SPIx_SCK_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOA_CLK_DISABLE()

#define Flash_SPIx_MISO_MOSI_GPIO_PORT                 GPIOA
#define Flash_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define Flash_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()
#define Flash_SPIx_MISO_PIN                            GPIO_PIN_6
#define Flash_SPIx_MOSI_PIN                            GPIO_PIN_7


/*############################### LCD control IO PIN #######################################*/
/**
  * @brief  LCD Chip Select Interface pins 
  */
#define Flash_CS_PIN                                 		GPIO_PIN_4
#define Flash_CS_GPIO_PORT                           		GPIOA
#define Flash_CS_GPIO_CLK_ENABLE()                 		__HAL_RCC_GPIOB_CLK_ENABLE()
#define Flash_CS_GPIO_CLK_DISABLE()                		__HAL_RCC_GPIOB_CLK_DISABLE()
    

#define Flash_SPIx_TIMEOUT_MAX                   1000



/**
  * @brief  Flash Control Lines management
  */
#define Flash_CS_LOW()      HAL_GPIO_WritePin(Flash_CS_GPIO_PORT, Flash_CS_PIN, GPIO_PIN_RESET)
#define Flash_CS_HIGH()     HAL_GPIO_WritePin(Flash_CS_GPIO_PORT, Flash_CS_PIN, GPIO_PIN_SET)


/** @defgroup ILI9341_Exported_Functions
  * @{
  */ 
void hw_flash_init(void);
void hw_flash_write_cmd(uint8_t reg);
void hw_flash_write_data(uint8_t data);
uint8_t hw_flash_read_data(void);
void hw_flash_write_multidata(uint8_t *pdata, uint32_t size);
void hw_flash_read_multidata(uint8_t *pdata, uint32_t size);	
void hw_flash_delay(uint32_t delay);
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SPI_Flash_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
