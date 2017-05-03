/*
 * File      : drv_adc.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-01-05     Bernard      the first version
 */

#include <rthw.h>
#include <rtdevice.h>
#include <board.h>
#include <stm32f1xx_hal.h>
#include <drv_hwadc.h>
#include <adc.h>

extern DMA_HandleTypeDef hdma_adcx;
#ifdef RT_USING_HWADC
static rt_err_t drv_hwadc_init(rt_device_hwadc_t *devadc, rt_uint8_t status)
{    
    drv_hwadc_t *hwadc;
    /* check parameters */
    RT_ASSERT(devadc != RT_NULL);
    hwadc = (drv_hwadc_t *)devadc->parent.user_data;    
    
    ADC_ChannelConfTypeDef sConfig;
    sConfig.Rank = 0;  
    
    if ((status & HWADC_ENABLE) == (HWADC_ENABLE))
    {
        if(ADC1 == hwadc->adcHandle.Instance )
        {
            #if defined RT_USING_ADC_GENERIC_MODE
            hwadc->adcHandle.Init.ScanConvMode = ADC_SCAN_DISABLE; //ADC_SCAN_ENABLE;
            hwadc->adcHandle.Init.ContinuousConvMode = ENABLE;//DISABLE;
            hwadc->adcHandle.Init.DiscontinuousConvMode = DISABLE;//ENABLE;
            hwadc->adcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
            hwadc->adcHandle.Init.NbrOfDiscConversion = 1;
            hwadc->adcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
            hwadc->adcHandle.Init.NbrOfConversion = TOTAL_CHANNELS;                           
            HAL_ADC_Init(&(hwadc->adcHandle)); 
                        
            #elif defined RT_USING_ADC_INT_MODE            
            hwadc->adcHandle.Init.ScanConvMode =ADC_SCAN_ENABLE;// ADC_SCAN_DISABLE;//
            hwadc->adcHandle.Init.ContinuousConvMode = ENABLE;
            hwadc->adcHandle.Init.DiscontinuousConvMode = DISABLE;//ENABLE;//      
            hwadc->adcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
            hwadc->adcHandle.Init.NbrOfDiscConversion = 1;
            hwadc->adcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
            hwadc->adcHandle.Init.NbrOfConversion = TOTAL_CHANNELS;
            HAL_ADC_Init(&hwadc->adcHandle);
                                               
            #elif defined RT_USING_ADC_DMA_MODE
            hwadc->adcHandle.Init.ScanConvMode = ADC_SCAN_ENABLE;
            hwadc->adcHandle.Init.ContinuousConvMode = ENABLE;
            hwadc->adcHandle.Init.DiscontinuousConvMode = DISABLE;
            hwadc->adcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
            hwadc->adcHandle.Init.NbrOfDiscConversion = 1;
            hwadc->adcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
            hwadc->adcHandle.Init.NbrOfConversion = TOTAL_CHANNELS;                           
            HAL_ADC_Init(&(hwadc->adcHandle));                             
            #endif /* RT_USING_ADC_DMA_MODE */                                             
            /**Configure Regular Channel 
            */            
            sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5; 
            
            #if 1 == RT_USING_ADC_IN0_ENABLE
            sConfig.Channel = ADC_CHANNEL_0;
            sConfig.Rank++;  
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif 
            
            #if 1 == RT_USING_ADC_IN1_ENABLE
            sConfig.Channel = ADC_CHANNEL_1;
            sConfig.Rank++;
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif
            
            #if 1 == RT_USING_ADC_IN2_ENABLE
            sConfig.Channel = ADC_CHANNEL_2;
            sConfig.Rank++;
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif 
            
            #if 1 == RT_USING_ADC_IN3_ENABLE
            sConfig.Channel = ADC_CHANNEL_3;
            sConfig.Rank++;
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif
            
            #if 1 == RT_USING_ADC_IN4_ENABLE
            sConfig.Channel = ADC_CHANNEL_4;
            sConfig.Rank++;
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif 
            
            #if 1 == RT_USING_ADC_IN5_ENABLE
            sConfig.Channel = ADC_CHANNEL_5;
            sConfig.Rank++; 
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif
            
            #if 1 == RT_USING_ADC_IN6_ENABLE
            sConfig.Channel = ADC_CHANNEL_6;
            sConfig.Rank++;
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif 
            
            #if 1 == RT_USING_ADC_IN7_ENABLE
            sConfig.Channel = ADC_CHANNEL_7;
            sConfig.Rank++;  
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif
            
            #if 1 == RT_USING_ADC_IN8_ENABLE
            sConfig.Channel = ADC_CHANNEL_8;
            sConfig.Rank++;  
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif 
            
            #if 1 == RT_USING_ADC_IN9_ENABLE
            sConfig.Channel = ADC_CHANNEL_9;
            sConfig.Rank++;  
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif 
            
            #if 1 == RT_USING_ADC_IN10_ENABLE
            sConfig.Channel = ADC_CHANNEL_10;
            sConfig.Rank++;
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif 
            
            #if 1 == RT_USING_ADC_IN11_ENABLE
            sConfig.Channel = ADC_CHANNEL_11;
            sConfig.Rank++; 
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif
            
            #if 1 == RT_USING_ADC_IN12_ENABLE
            sConfig.Channel = ADC_CHANNEL_12;
            sConfig.Rank++;
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif 
            
            #if 1 == RT_USING_ADC_IN13_ENABLE
            sConfig.Channel = ADC_CHANNEL_13;
            sConfig.Rank++;  
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif
            
            #if 1 == RT_USING_ADC_IN14_ENABLE
            sConfig.Channel = ADC_CHANNEL_14;
            sConfig.Rank++;
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif 
            
            #if 1 == RT_USING_ADC_IN15_ENABLE
            sConfig.Channel = ADC_CHANNEL_15;
            sConfig.Rank++;
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif  
            
            #if 1 == RT_USING_ADC_TEMPSENSOR_ENABLE
            sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
            sConfig.Rank++;
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif 
            
            #if 1 == RT_USING_ADC_VREFINT_ENABLE
            sConfig.Channel = ADC_CHANNEL_VREFINT;
            sConfig.Rank++;  
            HAL_ADC_ConfigChannel(&(hwadc->adcHandle), &sConfig);
            #endif           
                        
            /* Perform an ADC automatic self-calibration */            
            while(HAL_ADCEx_Calibration_Start(&(hwadc->adcHandle))!= HAL_OK);
            #ifdef USER_HWADC_APP_BUG_TEST
            rt_kprintf("The ADC init OK! \r\n");                       
            #endif            
          }                                
    }    
    else
    {       
        HAL_ADC_DeInit(&(hwadc->adcHandle));
        
        #ifdef RT_USING_ADC_DMA_MODE
        /* Peripheral DMA DeInit*/
        HAL_DMA_DeInit(&hdma_adcx);  
        #endif 
    }  
    return RT_EOK;
}

static rt_err_t drv_hwadc_start(rt_device_hwadc_t *devadc)
{
    rt_err_t result =  RT_ERROR;  
    drv_hwadc_t *hwadc; 
    /* check parameters */
    RT_ASSERT(devadc != RT_NULL);
    hwadc = (drv_hwadc_t*)devadc->parent.user_data;
    
    if(HAL_OK == HAL_ADC_Start(&(hwadc->adcHandle)))
    {
        result = RT_EOK;  
    } 
        
    return result;
}

static rt_err_t drv_hwadc_stop(rt_device_hwadc_t *devadc)
{
    rt_err_t result =  RT_ERROR;  
    drv_hwadc_t *hwadc; 
    /* check parameters */
    RT_ASSERT(devadc != RT_NULL);
    hwadc = (drv_hwadc_t*)devadc->parent.user_data;
    
    if(HAL_OK == HAL_ADC_Stop(&(hwadc->adcHandle)))
    {
        result = RT_EOK;  
    } 
        
    return result;
}

static rt_err_t drv_hwadc_start_IT(rt_device_hwadc_t *devadc)
{
    rt_err_t result =  RT_ERROR;  
    drv_hwadc_t *hwadc; 
    /* check parameters */
    RT_ASSERT(devadc != RT_NULL);
    hwadc = (drv_hwadc_t*)devadc->parent.user_data;
        
    if(HAL_OK == HAL_ADC_Start_IT(&(hwadc->adcHandle)))
    {
        result = RT_EOK;  
    }    
    return result;
}

static rt_err_t drv_hwadc_stop_IT(rt_device_hwadc_t *devadc)
{
    rt_err_t result =  RT_ERROR;  
    drv_hwadc_t *hwadc; 
    /* check parameters */
    RT_ASSERT(devadc != RT_NULL);
    hwadc = (drv_hwadc_t*)devadc->parent.user_data;
     
    if(HAL_OK == HAL_ADC_Stop_IT(&(hwadc->adcHandle)))
    {
        result = RT_EOK;  
    }    
    return result;
}

static rt_err_t drv_hwadc_start_DMA(rt_device_hwadc_t *devadc, uint16_t *buffer, rt_size_t size)
{
    rt_err_t result =  RT_ERROR;  
    drv_hwadc_t *hwadc; 
    /* check parameters */
    RT_ASSERT(devadc != RT_NULL);
    hwadc = (drv_hwadc_t*)devadc->parent.user_data;
    
    /* 启动AD转换并使能DMA传输和中断 */
    if(HAL_OK == HAL_ADC_Start_DMA(&(hwadc->adcHandle), (uint32_t *)buffer, size))
    {
        result = RT_EOK;  
    } 
      
    return result;
}
static rt_err_t drv_hwadc_stop_DMA(rt_device_hwadc_t *devadc)
{
    rt_err_t result =  RT_ERROR;  
    drv_hwadc_t *hwadc; 
    /* check parameters */
    RT_ASSERT(devadc != RT_NULL);
    hwadc = (drv_hwadc_t*)devadc->parent.user_data;
     
    if(HAL_OK == HAL_ADC_Stop_DMA(&(hwadc->adcHandle)))
    {
        result = RT_EOK;  
    }    
    return result;
}

static rt_size_t drv_hwadc_read(rt_device_hwadc_t *devadc, rt_off_t pos, void *buffer, rt_size_t size)
{ 
        
    /* check parameters */
    RT_ASSERT(devadc != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
    
    uint32_t *recv = (uint32_t*)buffer;
       
    #if defined RT_USING_ADC_GENERIC_MODE   
		drv_hwadc_t *hwadc = (drv_hwadc_t *)devadc->parent.user_data;    
    if(HAL_OK == HAL_ADC_PollForConversion(&hwadc->adcHandle,1000))
    {
        recv[0] = devadc->adc_converted_value[0] = HAL_ADC_GetValue(&hwadc->adcHandle);         
    }        
    #else   
    for(uint8_t i = 0; i < TOTAL_CHANNELS; i++)
    {
        recv[i] = devadc->adc_converted_value[i];//HAL_ADC_GetValue(&hwadc->adcHandle);       
    }
    #endif
        
    return size;
}

static drv_hwadc_t hwadc1;
static rt_device_hwadc_t rtadc1;
static uint8_t ADC_Transfer_Complete_Count = 0;
__IO uint16_t adc_convert[TOTAL_CHANNELS][ADC_BUFF_LEN];

#ifdef RT_USING_ADC_INT_MODE
//__IO uint8_t current_adc_channel = 0;
void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&(hwadc1.adcHandle));
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    /* single channel mode*/
    //rtadc1.adc_converted_value[0] = HAL_ADC_GetValue(hadc);
    if(ADC_Transfer_Complete_Count > ADC_BUFF_LEN-1)
    {
        ADC_Transfer_Complete_Count = 0;
    }
    for(uint8_t i = 0; i < TOTAL_CHANNELS; i++)
    {
        rtadc1.adc_converted_value[0] = HAL_ADC_GetValue(hadc);
        adc_convert[i][ADC_Transfer_Complete_Count++] = rtadc1.adc_converted_value[i]&0xFFF;  
    }
   
}
#endif /* RT_USING_ADC_INT_MODE */

#ifdef RT_USING_ADC_DMA_MODE
void ADC_DMAx_CHANNELn_IRQHandler(void)
{   
    HAL_DMA_IRQHandler(&hdma_adcx);
}
/* Conversion complete callback */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    //HAL_ADC_Stop_DMA(hadc);
    //rtadc1.parent.rx_indicate(&rtadc1.parent, sizeof(rtadc1.adc_converted_valid_value[MAX_OF_ADC_CHANNEL]));    
    if(ADC_Transfer_Complete_Count > ADC_BUFF_LEN-1)
    {
        ADC_Transfer_Complete_Count = 0;
    }
    for(uint8_t i = 0; i < TOTAL_CHANNELS; i++)
    {
        adc_convert[i][ADC_Transfer_Complete_Count++] = adc_converted_value[i]&0xFFF;  
    }
}
#endif /* RT_USING_ADC_DMA_MODE */

const static rt_hwadc_ops_t _drv_hwadc_ops =
{
    drv_hwadc_init,   
    drv_hwadc_read,
    drv_hwadc_start,
    drv_hwadc_stop,
    drv_hwadc_start_IT,
    drv_hwadc_stop_IT,
    drv_hwadc_start_DMA,
    drv_hwadc_stop_DMA   
};

int stm32_hwadc_init(void)
{
    int result;
    
    rtadc1.ops = &_drv_hwadc_ops;
       
    hwadc1.adcHandle.Instance = ADC1;    
    
    result = rt_device_hwadc_register(&rtadc1,"hwadc",&hwadc1);
    return result;
}
INIT_BOARD_EXPORT(stm32_hwadc_init);

#endif /* RT_USING_HWADC */
