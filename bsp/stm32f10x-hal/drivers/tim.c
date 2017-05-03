/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include <rtthread.h>
#include "tim.h"
#include "drv_hwtimer.h"



void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
      
  }
  if(tim_baseHandle->Instance==TIM7)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

  }
  if(tim_baseHandle->Instance==TIM2)
  {

    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
  if(tim_baseHandle->Instance==TIM3)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
  if(tim_baseHandle->Instance==TIM4)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
  if(tim_baseHandle->Instance==TIM1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
#ifdef RT_USING_HWTIM_CC_IRQ
    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
#endif
#ifdef RT_USING_HWTIM_UP_IRQ
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);  
#endif      
  }
  
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM6)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM6_IRQn);

  }
  if(tim_baseHandle->Instance==TIM7)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  }
  
  if(tim_baseHandle->Instance==TIM2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  }
  if(tim_baseHandle->Instance==TIM3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  }
  if(tim_baseHandle->Instance==TIM4)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  }
  if(tim_baseHandle->Instance==TIM1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    #ifdef RT_USING_HWTIM_CC_IRQ
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    #endif
    #ifdef RT_USING_HWTIM_UP_IRQ 
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);  
    #endif    
      
  }
     
} 

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
    
#ifdef RT_USING_HWTIM2    
  if(timHandle->Instance==TIM2)
  {
    /**TIM2 GPIO Configuration    
    PA0-WKUP     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    PA2     ------> TIM2_CH3
    PA3     ------> TIM2_CH4 
    */
      
    __HAL_RCC_GPIOA_CLK_ENABLE();  
    /*GPIO_InitStruct.Pin = 0;     
    #ifdef  RT_USING_HWTIM2_CH1   
    GPIO_InitStruct.Pin |= GPIO_PIN_0;  
    #endif
    #ifdef  RT_USING_HWTIM2_CH2   
    GPIO_InitStruct.Pin |= GPIO_PIN_1;
    #endif 
    #ifdef  RT_USING_HWTIM2_CH3  
    GPIO_InitStruct.Pin |= GPIO_PIN_2;
    #endif
    #ifdef  RT_USING_HWTIM2_CH4   
    GPIO_InitStruct.Pin |= GPIO_PIN_3;
    #endif*/
    #if 0
    HAL_GPIO_DeInit(GPIOA,  GPIO_PIN_2|GPIO_PIN_3);
    
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;    
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    #endif
    
    #if 1     
    __HAL_RCC_GPIOB_CLK_ENABLE();      
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);   
    __HAL_AFIO_REMAP_TIM2_PARTIAL_2(); 
    #endif      
   }  
#endif /* RT_USING_HWTIM2 */ 

#ifdef RT_USING_HWTIM3    
  if(timHandle->Instance==TIM3)
  {
    /**TIM3 GPIO Configuration    
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    PB0     ------> TIM3_CH3
    PB1     ------> TIM4_CH4
    */
    __HAL_RCC_GPIOA_CLK_ENABLE();    
    __HAL_RCC_GPIOB_CLK_ENABLE();  
    GPIO_InitStruct.Pin = 0;    
    #ifdef  RT_USING_HWTIM3_CH1   
    GPIO_InitStruct.Pin |= GPIO_PIN_6;  
    #endif
    #ifdef  RT_USING_HWTIM3_CH2 
    GPIO_InitStruct.Pin |= GPIO_PIN_7;    
    #endif 
      
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
     
    GPIO_InitStruct.Pin = 0;      
    #ifdef  RT_USING_HWTIM3_CH3   
    GPIO_InitStruct.Pin |= GPIO_PIN_0;  
    #endif
    #ifdef  RT_USING_HWTIM3_CH4  
    GPIO_InitStruct.Pin |= GPIO_PIN_1;  
    #endif  
         
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
      
   }  
#endif /* RT_USING_HWTIM3 */   

#ifdef RT_USING_HWTIM4    
  if(timHandle->Instance==TIM4)
  {
    /**TIM4 GPIO Configuration    
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    PB8     ------> TIM4_CH3
    PB9     ------> TIM4_CH4 
    */
      
    __HAL_RCC_GPIOB_CLK_ENABLE();
      
    #ifdef  RT_USING_HWTIM4_CH1          
    GPIO_InitStruct.Pin |= GPIO_PIN_6;
    #endif  
    #ifdef  RT_USING_HWTIM4_CH2 
    GPIO_InitStruct.Pin |= GPIO_PIN_7;
    #endif     
    #ifdef  RT_USING_HWTIM4_CH3 
    GPIO_InitStruct.Pin |= GPIO_PIN_8;
    #endif
    #ifdef  RT_USING_HWTIM4_CH4 
    GPIO_InitStruct.Pin |= GPIO_PIN_9;
    #endif 
    
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);    
   }  
#endif /* RT_USING_HWTIM4 */ 

#ifdef RT_USING_HWTIM1    
  if(timHandle->Instance==TIM1)
  {    
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
 
    #if 1 == RT_USING_HWTIM1_EXTERNAL_CLOCK_EN          
    /**TIM1 GPIO Configuration    
    PA12     ------> TIM1_ETR 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    #endif
    #ifdef  RT_USING_HWTIM1_CH1   
    /**TIM1 GPIO Configuration    
    PA8     ------> TIM1_CH1 
    */
    GPIO_InitStruct.Pin |= GPIO_PIN_8;
    #endif       
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);     
    
   }  
#endif /* RT_USING_HWTIM1 */    


}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
