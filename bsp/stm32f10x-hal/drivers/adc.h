/**
  ******************************************************************************
  * File Name          : ADC.h
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#ifndef __adc_H
#define __adc_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
//#include "main.h"
#include <drv_hwadc.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
     
#define ADCx_RCC_CLK_ENABLE()            __HAL_RCC_ADC1_CLK_ENABLE()
#define ADCx_RCC_CLK_DISABLE()           __HAL_RCC_ADC1_CLK_DISABLE()
#define DMAx_RCC_CLK_ENABLE()            __HAL_RCC_DMA1_CLK_ENABLE() 
#define DMAx_RCC_CLK_DISABLE()           __HAL_RCC_DMA1_CLK_DISABLE()     
#define ADCx                             ADC1     
#define ADC_DMAx_CHANNELn                DMA1_Channel1
#define ADC_DMAx_CHANNELn_IRQn           DMA1_Channel1_IRQn
#define ADC_DMAx_CHANNELn_IRQHandler     DMA1_Channel1_IRQHandler     

//#define ADC_GPIO_ClK_ENABLE()            __HAL_RCC_GPIOA_CLK_ENABLE()
//#define ADC_GPIO                         GPIOA   
//#define ADC_GPIO_PIN1                    GPIO_PIN_0        // 连接至板载精密可调电阻(需加跳帽)
//#define ADC_CHANNEL1                     ADC_CHANNEL_0    // 连接至板载精密可调电阻(需加跳帽)
//#define ADC_GPIO_PIN2                    GPIO_PIN_1        // 连接至板载光敏电阻(需加跳帽)
//#define ADC_CHANNEL2                     ADC_CHANNEL_1    // 连接至板载光敏电阻(需加跳帽)     

//extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adcx;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

//extern void Error_Handler(void);

//void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

void HAL_ADC_DMA_MspInit(drv_hwadc_t *dmadc);

/* USER CODE END Prototypes */


#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
