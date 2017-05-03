/**
  ******************************************************************************
  * File Name          : ADC.c
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

/* Includes ------------------------------------------------------------------*/
#include <adc.h>
#include <drv_hwadc.h>
//#include "gpio.h"

/* USER CODE BEGIN 0 */
//ADC_HandleTypeDef hadcx;
DMA_HandleTypeDef hdma_adcx;
/* USER CODE END 0 */
#if 1
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{ 
      GPIO_InitTypeDef GPIO_InitStruct;
      GPIO_InitStruct.Pin = 0;
      if(ADC1 == hadc->Instance)
      {
        /* USER CODE BEGIN ADC1_MspInit 0 */

        /* USER CODE END ADC1_MspInit 0 */
        /* Enalble Peripheral Clock */
        ADCx_RCC_CLK_ENABLE();
    
        /* AD转换通道引脚时钟使能 */
        //ADC_GPIO_ClK_ENABLE();
      
        /**ADC1 GPIO Configuration             
        PA0-WKUP------> ADC1_IN0
        PA1     ------> ADC1_IN1 
        PA2     ------> ADC1_IN2   
        PA3     ------> ADC1_IN3   
        PA4     ------> ADC1_IN4   
        PA5     ------> ADC1_IN5
        PA6     ------> ADC1_IN6
        PA7     ------> ADC1_IN7 
        PB0     ------> ADC1_IN8 
        PB1     ------> ADC1_IN9
        PC0     ------> ADC1_IN10
        PC1     ------> ADC1_IN11
        PC2     ------> ADC1_IN12
        PC3     ------> ADC1_IN13
        PC4     ------> ADC1_IN14
        PC5     ------> ADC1_IN15
        */
        /* AD Configure Regular Channel */
        #if 1 == RT_USING_ADC_IN0_ENABLE
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        #endif 
        #if 1 == RT_USING_ADC_IN1_ENABLE
        GPIO_InitStruct.Pin |= GPIO_PIN_1;
        #endif
        #if 1 == RT_USING_ADC_IN2_ENABLE
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        #endif 
        #if 1 == RT_USING_ADC_IN3_ENABLE
        GPIO_InitStruct.Pin |= GPIO_PIN_3;
        #endif
        #if 1 == RT_USING_ADC_IN4_ENABLE
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        #endif 
        #if 1 == RT_USING_ADC_IN5_ENABLE
        GPIO_InitStruct.Pin |= GPIO_PIN_5;
        #endif
        #if 1 == RT_USING_ADC_IN6_ENABLE
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        #endif 
        #if 1 == RT_USING_ADC_IN7_ENABLE
        GPIO_InitStruct.Pin |= GPIO_PIN_7;
        #endif
        #if (1 == RT_USING_ADC_IN0_ENABLE)||(1 == RT_USING_ADC_IN1_ENABLE)||(1 == RT_USING_ADC_IN2_ENABLE)||(1 == RT_USING_ADC_IN3_ENABLE)\
        ||(1 == RT_USING_ADC_IN4_ENABLE)||(1 == RT_USING_ADC_IN5_ENABLE)||(1 == RT_USING_ADC_IN6_ENABLE)||(1 == RT_USING_ADC_IN7_ENABLE)
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        #endif
        
        #if 1 == RT_USING_ADC_IN8_ENABLE
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        #endif 
        #if 1 == RT_USING_ADC_IN9_ENABLE
        GPIO_InitStruct.Pin |= GPIO_PIN_1;
        #endif  
        #if (1 == RT_USING_ADC_IN8_ENABLE)||(1 == RT_USING_ADC_IN9_ENABLE)
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        #endif
        
        #if 1 == RT_USING_ADC_IN10_ENABLE
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        #endif 
        #if 1 == RT_USING_ADC_IN11_ENABLE
        GPIO_InitStruct.Pin |= GPIO_PIN_1;
        #endif
        #if 1 == RT_USING_ADC_IN12_ENABLE
        GPIO_InitStruct.Pin |= GPIO_PIN_2;
        #endif 
        #if 1 == RT_USING_ADC_IN13_ENABLE
        GPIO_InitStruct.Pin |= GPIO_PIN_3;
        #endif
        #if 1 == RT_USING_ADC_IN14_ENABLE
        GPIO_InitStruct.Pin |= GPIO_PIN_4;
        #endif 
        #if 1 == RT_USING_ADC_IN15_ENABLE
        GPIO_InitStruct.Pin |= GPIO_PIN_5;
        #endif
        #if (1 == RT_USING_ADC_IN10_ENABLE)||(1 == RT_USING_ADC_IN11_ENABLE)||(1 == RT_USING_ADC_IN12_ENABLE)\
        ||(1 == RT_USING_ADC_IN13_ENABLE)||(1 == RT_USING_ADC_IN14_ENABLE)||(1 == RT_USING_ADC_IN15_ENABLE)
        __HAL_RCC_GPIOC_CLK_ENABLE();
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        #endif
        
        #ifdef RT_USING_ADC_INT_MODE 
        /* 外设中断优先级配置和使能中断 */
        /* Peripheral interrupt init */
        HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 1);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn); 
        #endif /* RT_USING_ADC_INT_MODE */ 
        
        #ifdef RT_USING_ADC_DMA_MODE
        /*Enable DMA Clock*/
        DMAx_RCC_CLK_ENABLE();

        /* DMA Peripheral init */  
        hdma_adcx.Instance = ADC_DMAx_CHANNELn;
        hdma_adcx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adcx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adcx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adcx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adcx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        hdma_adcx.Init.Mode = DMA_CIRCULAR;
        hdma_adcx.Init.Priority = DMA_PRIORITY_HIGH;
        HAL_DMA_Init(&hdma_adcx);
    
        /* Link DMA */        
        __HAL_LINKDMA(hadc,DMA_Handle,hdma_adcx);
        
        /* 外设中断优先级配置和使能中断 */
        //HAL_NVIC_SetPriority(ADC_DMAx_CHANNELn_IRQn, 1, 1);
        //HAL_NVIC_EnableIRQ(ADC_DMAx_CHANNELn_IRQn);
        #endif /* RT_USING_ADC_DMA_MODE */
    }   
}
#endif  

#if 1
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {      
    /* Peripheral clock disable */
    ADCx_RCC_CLK_DISABLE();       
        
    /**ADC1 GPIO Configuration    
    PA0-WKUP------> ADC1_IN0
    PA1     ------> ADC1_IN1 
    */   
      
    #if 1 == RT_USING_ADC_IN0_ENABLE
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
    #endif 
    #if 1 == RT_USING_ADC_IN1_ENABLE
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);
    #endif
    #if 1 == RT_USING_ADC_IN2_ENABLE
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
    #endif 
    #if 1 == RT_USING_ADC_IN3_ENABLE
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
    #endif
    #if 1 == RT_USING_ADC_IN4_ENABLE
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
    #endif 
    #if 1 == RT_USING_ADC_IN5_ENABLE
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
    #endif
    #if 1 == RT_USING_ADC_IN6_ENABLE
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
    #endif 
    #if 1 == RT_USING_ADC_IN7_ENABLE
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
    #endif    
    
    #if 1 == RT_USING_ADC_IN8_ENABLE
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);
    #endif 
    #if 1 == RT_USING_ADC_IN9_ENABLE
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);
    #endif      
    
    #if 1 == RT_USING_ADC_IN10_ENABLE
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);
    #endif 
    #if 1 == RT_USING_ADC_IN11_ENABLE
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1);
    #endif
    #if 1 == RT_USING_ADC_IN12_ENABLE
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2);
    #endif 
    #if 1 == RT_USING_ADC_IN13_ENABLE
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3);
    #endif
    #if 1 == RT_USING_ADC_IN14_ENABLE
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
    #endif 
    #if 1 == RT_USING_ADC_IN15_ENABLE
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5);
    #endif
    
    #ifdef RT_USING_ADC_INT_MODE    
    /* disable interrupt */    
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn); 
    #endif /* RT_USING_ADC_INT_MODE */  
    
    #ifdef RT_USING_ADC_DMA_MODE
    /* DMA时钟使能 */
    DMAx_RCC_CLK_DISABLE();  
    /* 外设中断优先级配置和使能中断 */
    //HAL_NVIC_SetPriority(ADC_DMAx_CHANNELn_IRQn, 1, 1);
    //HAL_NVIC_EnableIRQ(ADC_DMAx_CHANNELn_IRQn);
    #endif /* RT_USING_ADC_DMA_MODE */  
         
  }
}
#endif


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
