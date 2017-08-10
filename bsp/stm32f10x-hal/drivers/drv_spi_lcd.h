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
#ifndef __DRV_SPI_LCD_H__
#define __DRV_SPI_LCD_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
   
#define HAL_SPI_MODULE_ENABLED

//#if !defined (USE_STM32F1XX_SPI_LCD)
// #define USE_STM32F1XX_NUCLEO
//#endif

/** @defgroup STM32F1XX_SPI_LCD_LOW_LEVEL_BUS 
  * @{ 
The LCD module type is MTF0320CMIL-06V3.0,the SPI interface line are CSX,SCL,DSI,SDO, 
and apply to 4 wire 8 bit data serial interface II in ILI9431 data sheet.
  */
/*############################### SPI2 #######################################*/
#ifdef HAL_SPI_MODULE_ENABLED

#define ILI9341_SPIx                                     SPI2
#define ILI9341_SPIx_CLK_ENABLE()                        __HAL_RCC_SPI2_CLK_ENABLE()

//#define ILI9341_SPIx_SCK_AF                              GPIO_AF5_SPI1
#define ILI9341_SPIx_SCK_GPIO_PORT                       GPIOB
#define ILI9341_SPIx_SCK_PIN                             GPIO_PIN_13
#define ILI9341_SPIx_SCK_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define ILI9341_SPIx_SCK_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOB_CLK_DISABLE()

//#define ILI9341_SPIx_MISO_MOSI_AF                        GPIO_AF5_SPI1
#define ILI9341_SPIx_MISO_MOSI_GPIO_PORT                 GPIOB
#define ILI9341_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define ILI9341_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()
#define ILI9341_SPIx_MISO_PIN                            GPIO_PIN_14
#define ILI9341_SPIx_MOSI_PIN                            GPIO_PIN_15


/*############################### LCD control IO PIN #######################################*/
/**
  * @brief  LCD Chip Select Interface pins 
  */
#define LCD_CS_PIN                                 		GPIO_PIN_12
#define LCD_CS_GPIO_PORT                           		GPIOB
#define LCD_CS_GPIO_CLK_ENABLE()                 		__HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_CS_GPIO_CLK_DISABLE()                		__HAL_RCC_GPIOB_CLK_DISABLE()
    
/**
  * @brief  LCD Data/Command Interface pins (not use)
 */
#define LCD_DC_PIN                                 		GPIO_PIN_9
#define LCD_DC_GPIO_PORT                           		GPIOB
#define LCD_DC_GPIO_CLK_ENABLE()                 		__HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_DC_GPIO_CLK_DISABLE()                		__HAL_RCC_GPIOB_CLK_DISABLE()

/**
  * @brief  LCD Backlight Interface pins (not use)
  */
#define LCD_BL_PIN                                 		GPIO_PIN_8
#define LCD_BL_GPIO_PORT                           		GPIOB
#define LCD_BL_GPIO_CLK_ENABLE()                 		__HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_BL_GPIO_CLK_DISABLE()                		__HAL_RCC_GPIOB_CLK_DISABLE()

/**
  * @brief  LCD Reset Interface pins (not use)
  */
#define LCD_RST_PIN                                 GPIO_PIN_3
#define LCD_RST_GPIO_PORT                           GPIOB
#define LCD_RST_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()
#define LCD_RST_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOB_CLK_DISABLE()


/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define ILI9341_SPIx_TIMEOUT_MAX                   1000



/**
  * @brief  LCD Control Lines management
  */
#define LCD_CS_LOW()      HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_DC_LOW()      HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)
#define LCD_BL_LOW()      HAL_GPIO_WritePin(LCD_BL_GPIO_PORT, LCD_BL_PIN, GPIO_PIN_RESET)
#define LCD_BL_HIGH()     HAL_GPIO_WritePin(LCD_BL_GPIO_PORT, LCD_BL_PIN, GPIO_PIN_SET)
#define LCD_RST_LOW()      HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_RESET)
#define LCD_RST_HIGH()     HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_SET)


/** @defgroup ILI9341_Exported_Functions
  * @{
  */ 
void hw_lcd_init(void);
void hw_lcd_write_cmd(uint8_t reg);
void hw_lcd_write_data(uint8_t data);
uint8_t hw_lcd_read_data(void);
void hw_lcd_write_multidata(uint8_t *pdata, uint32_t size);
void hw_lcd_read_multidata(uint8_t *pdata, uint32_t size);	
void hw_lcd_delay(uint32_t delay);
void hw_lcd_backlight_on(void);
void hw_lcd_backlight_off(void);
void hw_lcd_reset(void);

#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SPI_LCD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
