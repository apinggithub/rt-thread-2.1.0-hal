
#ifndef __DRV_FSMC_LCD_HH
#define __DRV_FSMC_LCD_HH
#ifdef __cplusplus
 extern "C" {_
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SRAM_HandleTypeDef hsram1;
extern void Error_Handler(void);

/* USER CODE BEGIN Private defines */

/* 宏定义 --------------------------------------------------------------------*/
/******************************************************************************
2^26 =0X0400 0000 = 64MB,每个 BANK 有4*64MB = 256MB
64MB:FSMC_Bank1_NORSRAM1:0X6000 0000 ~ 0X63FF FFFF
64MB:FSMC_Bank1_NORSRAM2:0X6400 0000 ~ 0X67FF FFFF
64MB:FSMC_Bank1_NORSRAM3:0X6800 0000 ~ 0X6BFF FFFF
64MB:FSMC_Bank1_NORSRAM4:0X6C00 0000 ~ 0X6FFF FFFF

选择BANK1-BORSRAM4 连接 TFT，地址范围为0X6C00 0000 ~ 0X6FFF FFFF
YS-F1P开发板选择 FSMC_A0 接LCD的DC(寄存器/数据选择)脚
寄存器基地址 = 0X6C00 0000
RAM基地址 = 0X6C00 0002 = 0X6C00 0000+(1<<(0+1))
如果电路设计时选择不同的地址线时，地址要重新计算  
*******************************************************************************/
/******************************* ILI9488 显示屏的 FSMC 参数定义 ***************/
#define FSMC_LCD_CMD                   ((uint32_t)0x60000000)	    //FSMC_Bank1_NORSRAM1用于LCD命令操作的地址
#define FSMC_LCD_DATA                  ((uint32_t)0x61000000)      //FSMC_Bank1_NORSRAM1用于LCD数据操作的地址      
#define LCD_WRITE_CMD(x)               *(__IO uint16_t *)FSMC_LCD_CMD  = x 
#define LCD_WRITE_DATA(x)              *(__IO uint16_t *)FSMC_LCD_DATA = x
#define LCD_READ_DATA()                *(__IO uint16_t *)FSMC_LCD_DATA

#define FSMC_LCD_BANKx                 FSMC_NORSRAM_BANK1



#define RCC_GPIO_LCD_BL_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define GPIO_PORT_LCD_BL              GPIOB
#define GPIO_PIN_LCD_BL               GPIO_PIN_6

#define rt_lcd_bl_on()     HAL_GPIO_WritePin(GPIO_PORT_LCD_BL, GPIO_PIN_LCD_BL, GPIO_PIN_SET)
#define rt_lcd_bl_off()    HAL_GPIO_WritePin(GPIO_PORT_LCD_BL, GPIO_PIN_LCD_BL, GPIO_PIN_RESET)

/* USER CODE END Private defines */

void MX_FSMC_Init(void);
void HAL_SRAM_MspInit(SRAM_HandleTypeDef* hsram);
void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef* hsram);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /*__FSMC_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
