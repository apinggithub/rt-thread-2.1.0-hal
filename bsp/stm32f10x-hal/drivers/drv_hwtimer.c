/*
 * File      : drv_hwtimer.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author           Notes
 * 2015-09-02     heyuanjie87      the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <drv_hwtimer.h>
#include <tim.h>
#include "light_wave_curer.h"

#ifdef RT_USING_HWTIMER
static void drv_timer_init(rt_device_hwtimer_t *timer, rt_uint8_t status)
{   
    drv_hwtimer_t *hwtim; 
    TIM_MasterConfigTypeDef sMasterConfig;  
    TIM_OC_InitTypeDef sConfigOC;
    TIM_IC_InitTypeDef sConfigIC;
    TIM_ClockConfigTypeDef sClockSourceConfig;    
    
    RT_ASSERT(timer != RT_NULL);   
    hwtim = (drv_hwtimer_t *)timer->parent.user_data;      
    
    if (HW_ENABLE == status)
    {           
        hwtim->TimerHandle.Init.Prescaler = timer->prescaler;
        hwtim->TimerHandle.Init.Period = timer->reload;                
        hwtim->TimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        if((TIM6 == hwtim->TimerHandle.Instance)||(TIM7 == hwtim->TimerHandle.Instance))
        {
            hwtim->TimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
        }
        else
        {
            switch(timer->count_mode)                        
            {
                case DRV_TIM_COUNT_MODE_DOWN:
                {
                   hwtim->TimerHandle.Init.CounterMode = TIM_COUNTERMODE_DOWN; 
                }
                break;
                case DRV_TIM_COUNT_MODE_CENTER_ALIGN1:
                {
                   hwtim->TimerHandle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1; 
                }
                break;
                case DRV_TIM_COUNT_MODE_CENTER_ALIGN2:
                {
                   hwtim->TimerHandle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2; 
                }    
                break;
                case DRV_TIM_COUNT_MODE_CENTER_ALIGN3:
                {
                   hwtim->TimerHandle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3; 
                }    
                break;
                default: /* DRV_TIM_COUNT_MODE_UP */
                {
                    hwtim->TimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
                }    
                break;
            }
        }
        #ifdef RT_USING_HWTIM1
        hwtim->TimerHandle.Init.RepetitionCounter = 0;                                   
        #endif
        
        HAL_TIM_Base_Init(&(hwtim->TimerHandle));
        
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        HAL_TIMEx_MasterConfigSynchronization(&(hwtim->TimerHandle), &sMasterConfig);
        
        //HAL_TIM_Base_MspInit(&(hwtim->TimerHandle));                  
		        
        /*TIM6 and TIM7 are base timer. TIM2,TIM3,TIM4 and TIM5 are general timer , TIM1 and TIM8 are advance timer*/
        if((TIM6 != hwtim->TimerHandle.Instance)&&(TIM7 != hwtim->TimerHandle.Instance))
        {
                                    
                if(HWTIMER_EXTERNAL_CLOCK_SOURCE == timer->clock_source)
                {
                    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;                                   
                    sClockSourceConfig.ClockPolarity = TIM_ETRPOLARITY_INVERTED;
                    sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV8;//TIM_CLOCKPRESCALER_DIV1;
                    sClockSourceConfig.ClockFilter = 10;
                }
                else
                {
                    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; 
                }
                HAL_TIM_ConfigClockSource(&(hwtim->TimerHandle), &sClockSourceConfig);
                                               
                for(uint8_t i = HWTIMER_CH1; i < HWTIMER_CH1 + 4; i++)
                {                   
                    switch(timer->channel_no[i].channel)                        
                    {
                        case TIM_CHANNEL_1:
                        {
                            if(HW_ENABLE == timer->channel_no[i].status)
                            {
                                switch(timer->channel_type[i])
                                {
                                    case DRV_TIM_CH_TYPE_PWM:  
                                    {
                                        HAL_TIM_PWM_Init(&(hwtim->TimerHandle));                                                                           
                                        sConfigOC.OCMode = TIM_OCMODE_PWM1;
                                        sConfigOC.Pulse = 10;      
                                        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;//TIM_OCPOLARITY_LOW
                                        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; 
                                        HAL_TIM_PWM_ConfigChannel(&(hwtim->TimerHandle), &sConfigOC, TIM_CHANNEL_1);     
                                    }            
                                    break;
                                    case DRV_TIM_CH_TYPE_OC:
                                    {
                                        HAL_TIM_OC_Init(&(hwtim->TimerHandle));                                           
                                        sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
                                        sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;//TIM_OCPOLARITY_HIGH;
                                        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; 
                                        HAL_TIM_OC_ConfigChannel(&(hwtim->TimerHandle), &sConfigOC, TIM_CHANNEL_1);   
                                    }                                    
                                    break;
                                    case DRV_TIM_CH_TYPE_IC:  
                                    {
                                        HAL_TIM_IC_Init(&(hwtim->TimerHandle));                                           
                                        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;//TIM_INPUTCHANNELPOLARITY_RISING;
                                        sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
                                        sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
                                        sConfigIC.ICFilter = 8;
                                        HAL_TIM_IC_ConfigChannel(&(hwtim->TimerHandle), &sConfigIC, TIM_CHANNEL_1);   
                                    }                                    
                                    break; 
                                    case DRV_TIM_CH_TYPE_ENCODER:  
                                    {
                                        
                                    }                                    
                                    break; 
                                    #ifdef RT_USING_TIM_ADVANCED_FUNCTION
                                    //case DRV_TIM_CH_TYPE_:  
                                    //{
                                        
                                    //}                                    
                                    //break; 
                                    #endif
                                    default: /*default HWTIMER_TYPE_BASE no output*/
                                        break;                                       
                                }                                
                            }
                            else /* disable */
                            {
                                switch(timer->channel_type[i])
                                {
                                    case DRV_TIM_CH_TYPE_PWM:  
                                    {
                                        HAL_TIMEx_PWMN_Stop_IT(&(hwtim->TimerHandle),  TIM_CHANNEL_1);     
                                        HAL_TIM_PWM_DeInit(&(hwtim->TimerHandle));                                                                                                                                                                                         
                                    }            
                                    break;
                                    case DRV_TIM_CH_TYPE_OC:
                                    {
                                        HAL_TIM_OC_Stop_IT(&(hwtim->TimerHandle), TIM_CHANNEL_1);   
                                        HAL_TIM_OC_DeInit(&(hwtim->TimerHandle));                                                                                                                       
                                    }                                    
                                    break;
                                    case DRV_TIM_CH_TYPE_IC:  
                                    {
                                        HAL_TIM_IC_Stop_IT(&(hwtim->TimerHandle), TIM_CHANNEL_1);   
                                        HAL_TIM_IC_DeInit(&(hwtim->TimerHandle));     
                                    }                                    
                                    break; 
                                    case DRV_TIM_CH_TYPE_ENCODER:  
                                    {
                                        
                                    }                                    
                                    break; 
                                    default: /*default HWTIMER_TYPE_BASE no output*/
                                        break;                                       
                                }
                            }
                        }                            
                        break;
                        case TIM_CHANNEL_2:
                        {
                            if(HW_ENABLE == timer->channel_no[i].status)
                            {
                                switch(timer->channel_type[i])
                                {
                                    case DRV_TIM_CH_TYPE_PWM: 
                                    {                                    
                                        HAL_TIM_PWM_Init(&(hwtim->TimerHandle));                                        
                                        sConfigOC.OCMode = TIM_OCMODE_PWM1;
                                        sConfigOC.Pulse = 10;   
                                        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
                                        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; 
                                        HAL_TIM_PWM_ConfigChannel(&(hwtim->TimerHandle), &sConfigOC, TIM_CHANNEL_2);   
                                    }                                    
                                    break;
                                    case DRV_TIM_CH_TYPE_OC:
                                    {
                                        HAL_TIM_OC_Init(&(hwtim->TimerHandle));                                        
                                        sConfigOC.OCMode = TIM_OCMODE_TOGGLE;//TIM_OCMODE_TIMING;//
                                        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
                                        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; 
                                        HAL_TIM_OC_ConfigChannel(&(hwtim->TimerHandle), &sConfigOC, TIM_CHANNEL_2);    
                                    }
                                    break;
                                    case DRV_TIM_CH_TYPE_IC:  
                                    {
                                        HAL_TIM_IC_Init(&(hwtim->TimerHandle));                                           
                                        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
                                        sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
                                        sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
                                        sConfigIC.ICFilter = 0;
                                        HAL_TIM_IC_ConfigChannel(&(hwtim->TimerHandle), &sConfigIC, TIM_CHANNEL_2); 
                                    }                                    
                                    break; 
                                    case DRV_TIM_CH_TYPE_ENCODER: 
                                    {
                                        
                                    }
                                    break; 
                                    default: /*default HWTIMER_TYPE_BASE no output*/
                                        break; 
                                } 
                            }
                            else /* disable */
                            {
                                switch(timer->channel_type[i])
                                {
                                    case DRV_TIM_CH_TYPE_PWM:  
                                    {
                                        HAL_TIMEx_PWMN_Stop_IT(&(hwtim->TimerHandle),  TIM_CHANNEL_2);     
                                        HAL_TIM_PWM_DeInit(&(hwtim->TimerHandle));                                                                                                                                                                                         
                                    }            
                                    break;
                                    case DRV_TIM_CH_TYPE_OC:
                                    {
                                        HAL_TIM_OC_Stop_IT(&(hwtim->TimerHandle), TIM_CHANNEL_2);   
                                        HAL_TIM_OC_DeInit(&(hwtim->TimerHandle));                                                                                                                       
                                    }                                    
                                    break;
                                    case DRV_TIM_CH_TYPE_IC:  
                                    {
                                        HAL_TIM_IC_Stop_IT(&(hwtim->TimerHandle), TIM_CHANNEL_2);   
                                        HAL_TIM_IC_DeInit(&(hwtim->TimerHandle));
                                    }                                    
                                    break; 
                                    case DRV_TIM_CH_TYPE_ENCODER:  
                                    {
                                        
                                    }                                    
                                    break; 
                                    default: /*default HWTIMER_TYPE_BASE no output*/
                                        break;                                       
                                }
                            }
                        }                            
                        break;
                        case TIM_CHANNEL_3:
                        {
                            if(HW_ENABLE == timer->channel_no[i].status)
                            {
                                switch(timer->channel_type[i])
                                {
                                    case DRV_TIM_CH_TYPE_PWM: 
                                    {                                    
                                        HAL_TIM_PWM_Init(&(hwtim->TimerHandle));                                        
                                        sConfigOC.OCMode = TIM_OCMODE_PWM1;
                                        sConfigOC.Pulse = 10; 
                                        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;//TIM_OCPOLARITY_LOW;
                                        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; 
                                        HAL_TIM_PWM_ConfigChannel(&(hwtim->TimerHandle), &sConfigOC, TIM_CHANNEL_3);   
                                    }                                    
                                    break;
                                    case DRV_TIM_CH_TYPE_OC:
                                    {
                                        HAL_TIM_OC_Init(&(hwtim->TimerHandle));                                        
                                        sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
                                        sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;//TIM_OCPOLARITY_HIGH;
                                        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; 
                                        HAL_TIM_OC_ConfigChannel(&(hwtim->TimerHandle), &sConfigOC, TIM_CHANNEL_3);    
                                    }
                                    break;
                                    case DRV_TIM_CH_TYPE_IC:        
                                    {
                                        HAL_TIM_IC_Init(&(hwtim->TimerHandle));                                           
                                        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
                                        sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
                                        sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
                                        sConfigIC.ICFilter = 0;
                                        HAL_TIM_IC_ConfigChannel(&(hwtim->TimerHandle), &sConfigIC, TIM_CHANNEL_3); 
                                    }                                    
                                    break; 
                                    case DRV_TIM_CH_TYPE_ENCODER:  
                                    {
                                        
                                    }                                    
                                    break; 
                                    default: /*default HWTIMER_TYPE_BASE no output*/
                                        break; 
                                } 
                            }
                            else /* disable */
                            {
                                switch(timer->channel_type[i])
                                {
                                    case DRV_TIM_CH_TYPE_PWM:  
                                    {
                                        HAL_TIMEx_PWMN_Stop_IT(&(hwtim->TimerHandle),  TIM_CHANNEL_3);     
                                        HAL_TIM_PWM_DeInit(&(hwtim->TimerHandle));                                                                                                                                                                                         
                                    }            
                                    break;
                                    case DRV_TIM_CH_TYPE_OC:
                                    {
                                        HAL_TIM_OC_Stop_IT(&(hwtim->TimerHandle), TIM_CHANNEL_3);   
                                        HAL_TIM_OC_DeInit(&(hwtim->TimerHandle));                                                                                                                       
                                    }                                    
                                    break;
                                    case DRV_TIM_CH_TYPE_IC:  
                                    {
                                        HAL_TIM_IC_Stop_IT(&(hwtim->TimerHandle), TIM_CHANNEL_3);   
                                        HAL_TIM_IC_DeInit(&(hwtim->TimerHandle));
                                    }                                    
                                    break; 
                                    case DRV_TIM_CH_TYPE_ENCODER:  
                                    {
                                        
                                    }                                    
                                    break; 
                                    default: /*default HWTIMER_TYPE_BASE no output*/
                                        break;                                       
                                }
                            }
                        }                            
                        break;
                        case TIM_CHANNEL_4:
                        {
                            if(HW_ENABLE == timer->channel_no[i].status)
                            {
                                switch(timer->channel_type[i])
                                {
                                    case DRV_TIM_CH_TYPE_PWM: 
                                    {                                    
                                        HAL_TIM_PWM_Init(&(hwtim->TimerHandle));                                       
                                        sConfigOC.OCMode = TIM_OCMODE_PWM1;
                                        sConfigOC.Pulse = 10;  
                                        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;//TIM_OCPOLARITY_LOW;
                                        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;                                         
                                        HAL_TIM_PWM_ConfigChannel(&(hwtim->TimerHandle), &sConfigOC, TIM_CHANNEL_4);      
                                    }                                    
                                    break;
                                    case DRV_TIM_CH_TYPE_OC:
                                    {
                                        HAL_TIM_OC_Init(&(hwtim->TimerHandle));                                          
                                        sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
                                        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
                                        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; 
                                        HAL_TIM_OC_ConfigChannel(&(hwtim->TimerHandle), &sConfigOC, TIM_CHANNEL_4);    
                                    }
                                    break;
                                    case DRV_TIM_CH_TYPE_IC:  
                                    {
                                        HAL_TIM_IC_Init(&(hwtim->TimerHandle));                                           
                                        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
                                        sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
                                        sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
                                        sConfigIC.ICFilter = 0;
                                        HAL_TIM_IC_ConfigChannel(&(hwtim->TimerHandle), &sConfigIC, TIM_CHANNEL_4); 
                                    }                                    
                                    break; 
                                    case DRV_TIM_CH_TYPE_ENCODER:    
                                    {
                                        
                                    }                                    
                                    break; 
                                    default: /*default HWTIMER_TYPE_BASE no output*/
                                        break; 
                                } 
                            }
                            else /* disable */
                            {
                                switch(timer->channel_type[i])
                                {
                                    case DRV_TIM_CH_TYPE_PWM:  
                                    {
                                        HAL_TIMEx_PWMN_Stop_IT(&(hwtim->TimerHandle),  TIM_CHANNEL_4);     
                                        HAL_TIM_PWM_DeInit(&(hwtim->TimerHandle));                                                                                                                                                                                         
                                    }            
                                    break;
                                    case DRV_TIM_CH_TYPE_OC:
                                    {
                                        HAL_TIM_OC_Stop_IT(&(hwtim->TimerHandle), TIM_CHANNEL_4);   
                                        HAL_TIM_OC_DeInit(&(hwtim->TimerHandle));                                                                                                                       
                                    }                                    
                                    break;
                                    case DRV_TIM_CH_TYPE_IC:  
                                    {
                                        HAL_TIM_IC_Stop_IT(&(hwtim->TimerHandle), TIM_CHANNEL_4);   
                                        HAL_TIM_IC_DeInit(&(hwtim->TimerHandle));
                                    }                                    
                                    break; 
                                    case DRV_TIM_CH_TYPE_ENCODER:  
                                    {
                                        
                                    }                                    
                                    break; 
                                    default: /*default HWTIMER_TYPE_BASE no output*/
                                        break;                                       
                                }
                            }
                        }                            
                        break;                        
                        default:break;
                    }/* end switch timer->channel */                                                           
                }/* end for() */
                HAL_TIM_MspPostInit(&(hwtim->TimerHandle));
            
        }    
    }
    else /* HW_DISABLE */
    {
        HAL_TIM_Base_MspDeInit(&(hwtim->TimerHandle));
    }
    
}

static rt_err_t drv_timer_start(rt_device_hwtimer_t *timer, rt_uint8_t ch)//, rt_hwtimer_mode_t opmode
{
    rt_err_t err = RT_EOK;
    drv_hwtimer_t *hwtim; 
    RT_ASSERT(timer != RT_NULL);    
    hwtim = (drv_hwtimer_t *)timer->parent.user_data;
    if(0 == timer->reload)
    {
        timer->reload = 1;
    }   
    __HAL_TIM_SET_AUTORELOAD(&(hwtim->TimerHandle), timer->reload);
    
    //m = (opmode == HWTIMER_MODE_ONESHOT)? TIM_OPMODE_SINGLE : TIM_OPMODE_REPETITIVE;
    //TIM_SelectOnePulseMode(tim, m);
                                                                                        
    switch(timer->channel_type[ch])
    {
        case DRV_TIM_CH_TYPE_PWM: 
        {
            //HAL_TIM_PWM_Start(&(hwtim->TimerHandle),timer->channel_no[ch].channel);
            HAL_TIM_PWM_Start_IT(&(hwtim->TimerHandle),timer->channel_no[ch].channel);
        }
        break;
        case DRV_TIM_CH_TYPE_OC: 
        {
            //HAL_TIM_OC_Start(&(hwtim->TimerHandle),timer->channel_no[ch].channel);
            HAL_TIM_OC_Start_IT(&(hwtim->TimerHandle),timer->channel_no[ch].channel);
        }
        break;
        case DRV_TIM_CH_TYPE_IC: 
        {
            //HAL_TIM_IC_Start(&(hwtim->TimerHandle),timer->channel_no[ch].channel);
            HAL_TIM_IC_Start_IT(&(hwtim->TimerHandle),timer->channel_no[ch].channel);
        }
        break;
        case DRV_TIM_CH_TYPE_ENCODER: 
        {
            
        }
        break;
        default: /* default HWTIMER_TYPE_BASE on output */
        {
            __HAL_TIM_ENABLE_IT(&(hwtim->TimerHandle), TIM_IT_UPDATE);
            HAL_TIM_Base_Start(&(hwtim->TimerHandle));
        }
        break;
    }           
    return err;
}

static void drv_timer_stop(rt_device_hwtimer_t *timer, rt_uint8_t ch)
{

    drv_hwtimer_t *hwtim;    
    RT_ASSERT(timer != RT_NULL);   
    hwtim = (drv_hwtimer_t *)timer->parent.user_data;

    switch(timer->channel_type[ch])
    {
        case DRV_TIM_CH_TYPE_PWM: 
        {
            //HAL_TIM_PWM_Stop(&(hwtim->TimerHandle),timer->channel_no[ch].channel);
            HAL_TIM_PWM_Stop_IT(&(hwtim->TimerHandle),timer->channel_no[ch].channel);            
        }
        break;
        case DRV_TIM_CH_TYPE_OC: 
        {            
            //HAL_TIM_OC_Stop(&(hwtim->TimerHandle),timer->channel_no[ch].channel);
            HAL_TIM_OC_Stop_IT(&(hwtim->TimerHandle),timer->channel_no[ch].channel);    
        }
        break;
        case DRV_TIM_CH_TYPE_IC: 
        {
            //HAL_TIM_IC_Stop(&(hwtim->TimerHandle),timer->channel_no[ch].channel);
            HAL_TIM_IC_Stop_IT(&(hwtim->TimerHandle),timer->channel_no[ch].channel); 
        }
        break;
        case DRV_TIM_CH_TYPE_ENCODER: 
        {
            
        }
        break;
        default: /* default HWTIMER_TYPE_BASE on output */
        {            
            HAL_TIM_Base_Stop(&(hwtim->TimerHandle));
            __HAL_TIM_DISABLE_IT(&(hwtim->TimerHandle), TIM_IT_UPDATE);     
            __HAL_TIM_CLEAR_IT(&(hwtim->TimerHandle),TIM_IT_UPDATE);    
        }
        break;
    }              
}

static rt_err_t drv_timer_set_frequency(rt_device_hwtimer_t *timer, float freq)
{
    
    rt_err_t err = RT_EOK;
    drv_hwtimer_t *hwtim; 
    uint32_t sysclk;    
    RT_ASSERT(timer != RT_NULL);    
    hwtim = (drv_hwtimer_t *)timer->parent.user_data;
    assert_param(IS_TIM_INSTANCE(htim->Instance));

           
    /* T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK 
    //T is timer cycles,The value of TIM_Period and TIM_Prescaler is (0-65535), TIMxCLK is system clock 72MHz
    //if T =    1 us (1MHz), TIM_Prescaler = 71 ,  and the TIM_Period = 0 < 65535 ok. 
    //if T =  100 us,(10KHz), TIM_Prescaler = 71 ,  and the TIM_Period = 99 < 65535 ok. 
    //if T =  500 us,(2KHz), TIM_Prescaler = 71 ,  and the TIM_Period = 499 < 65535 ok. 
    //if T =    1 ms,(1KHz), TIM_Prescaler = 71 ,  and the TIM_Period = 999 < 65535 ok. 
    //if T =   10 ms,(100Hz), TIM_Prescaler = 71 ,  and the TIM_Period = 9999 < 65535 ok. 
    //if T =   50 ms,(20Hz), TIM_Prescaler = 71 ,  and the TIM_Period = 49999 < 65535 ok. 
    //if T = 1 s, TIM_Prescaler = 7199, and the TIM_Period = 9999 < 65535 ok.
    
    when the timer clock is external pin ETR, user set the T,Period,and Prescaler, will give the timer frequence from ETR pin
    */
    if(0 == timer->reload)
    {
        timer->reload = 1;
    } 
    if(HWTIMER_EXTERNAL_CLOCK_SOURCE == timer->clock_source)
    {        
        __HAL_TIM_SET_PRESCALER(&(hwtim->TimerHandle),timer->prescaler);        
        __HAL_TIM_SET_AUTORELOAD(&(hwtim->TimerHandle), timer->reload);                                                            
    }
    else
    {
        sysclk = HAL_RCC_GetHCLKFreq();/* get system clock 72MHz */  
        if( freq != 0.0) 
        {        
            timer->reload= (uint16_t)(sysclk/(timer->prescaler + 1)/freq - 1);    
            __HAL_TIM_SET_PRESCALER(&(hwtim->TimerHandle), timer->prescaler);        
            __HAL_TIM_SET_AUTORELOAD(&(hwtim->TimerHandle), timer->reload);                                                    
            err = -RT_EOK; 
        }                            
        else
        {            
            err = -RT_ENOSYS;        
        }   
    }        
                
    return err;
}

rt_err_t drv_timer_set_prescaler(rt_device_hwtimer_t *timer,rt_uint32_t val)
{
    rt_err_t err = RT_EOK;
    drv_hwtimer_t *hwtim;    
    RT_ASSERT(timer != RT_NULL);   
    hwtim = (drv_hwtimer_t *)timer->parent.user_data;

    //__HAL_TIM_DISABLE_IT(&(hwtim->TimerHandle), TIM_IT_UPDATE);  
    //__HAL_TIM_CLEAR_IT(&(hwtim->TimerHandle),TIM_IT_UPDATE);    
    __HAL_TIM_SET_PRESCALER(&(hwtim->TimerHandle),val);                                               
    //__HAL_TIM_ENABLE_IT(&(hwtim->TimerHandle), TIM_IT_UPDATE);

   return err; 
}
static rt_uint32_t drv_timer_get_counter(rt_device_hwtimer_t *timer)
{
    drv_hwtimer_t *hwtim;     
    RT_ASSERT(timer != RT_NULL);       
    hwtim = (drv_hwtimer_t *)timer->parent.user_data;
    
    return __HAL_TIM_GET_COUNTER(&(hwtim->TimerHandle));
       
}
static rt_err_t drv_timer_set_counter(rt_device_hwtimer_t *timer,rt_uint32_t val)
{
    rt_err_t err = RT_EOK;
    drv_hwtimer_t *hwtim;     
    RT_ASSERT(timer != RT_NULL);   
    hwtim = (drv_hwtimer_t *)timer->parent.user_data;
    
    //__HAL_TIM_DISABLE_IT(&(hwtim->TimerHandle), TIM_IT_UPDATE);    
    //__HAL_TIM_CLEAR_IT(&(hwtim->TimerHandle),TIM_IT_UPDATE);     
    __HAL_TIM_SET_COUNTER(&(hwtim->TimerHandle),val);                                     
    //__HAL_TIM_ENABLE_IT(&(hwtim->TimerHandle), TIM_IT_UPDATE);
    
    return err;
}
static rt_uint32_t drv_timer_get_autoreload(rt_device_hwtimer_t *timer)
{
    drv_hwtimer_t *hwtim;
    RT_ASSERT(timer != RT_NULL);
    hwtim = (drv_hwtimer_t *)timer->parent.user_data;
    
    return __HAL_TIM_GET_AUTORELOAD(&(hwtim->TimerHandle));
}
static rt_err_t drv_timer_set_autoreload(rt_device_hwtimer_t *timer,rt_uint32_t val)
{
    rt_err_t err = RT_EOK;
    drv_hwtimer_t *hwtim;     
    RT_ASSERT(timer != RT_NULL);
    hwtim = (drv_hwtimer_t *)timer->parent.user_data;
    
    //__HAL_TIM_DISABLE_IT(&(hwtim->TimerHandle), TIM_IT_UPDATE);  
    //__HAL_TIM_CLEAR_IT(&(hwtim->TimerHandle),TIM_IT_UPDATE);       
    __HAL_TIM_SET_AUTORELOAD(&(hwtim->TimerHandle),val);                                                               
    //__HAL_TIM_ENABLE_IT(&(hwtim->TimerHandle), TIM_IT_UPDATE);
    
    return err;
}
static rt_uint32_t drv_timer_get_compare(rt_device_hwtimer_t *timer,rt_uint8_t ch)
{

    drv_hwtimer_t *hwtim; 
    uint16_t val;
    RT_ASSERT(timer != RT_NULL);
    hwtim = (drv_hwtimer_t *)timer->parent.user_data;

    switch(ch)
    {
        case HWTIMER_CH1:
            val = __HAL_TIM_GET_COMPARE(&(hwtim->TimerHandle),TIM_CHANNEL_1);    
        break;
        case HWTIMER_CH2:
            val = __HAL_TIM_GET_COMPARE(&(hwtim->TimerHandle),TIM_CHANNEL_2);    
        break;
        case HWTIMER_CH3:
            val = __HAL_TIM_GET_COMPARE(&(hwtim->TimerHandle),TIM_CHANNEL_3);    
        break;
        case HWTIMER_CH4:
            val = __HAL_TIM_GET_COMPARE(&(hwtim->TimerHandle),TIM_CHANNEL_4);    
        break;
        default:
        break;
    }
    
    return val;
}
static rt_err_t drv_timer_set_compare(rt_device_hwtimer_t *timer,rt_uint8_t ch,rt_uint32_t val)
{
    rt_err_t err = RT_EOK;
    drv_hwtimer_t *hwtim; 
    RT_ASSERT(timer != RT_NULL);
    hwtim = (drv_hwtimer_t *)timer->parent.user_data;

    switch(ch)
    {
        case HWTIMER_CH1:
            __HAL_TIM_DISABLE_IT(&(hwtim->TimerHandle), TIM_IT_CC1); 
            __HAL_TIM_CLEAR_IT(&(hwtim->TimerHandle),TIM_IT_CC1); 
            __HAL_TIM_SET_COMPARE(&(hwtim->TimerHandle),TIM_CHANNEL_1,val);  
            __HAL_TIM_ENABLE_IT(&(hwtim->TimerHandle), TIM_IT_CC1);
        break;
        case HWTIMER_CH2:
            __HAL_TIM_DISABLE_IT(&(hwtim->TimerHandle), TIM_IT_CC2); 
            __HAL_TIM_CLEAR_IT(&(hwtim->TimerHandle),TIM_IT_CC2);
            __HAL_TIM_SET_COMPARE(&(hwtim->TimerHandle),TIM_CHANNEL_2,val);
            __HAL_TIM_ENABLE_IT(&(hwtim->TimerHandle), TIM_IT_CC2);
        break;
        case HWTIMER_CH3:
            __HAL_TIM_DISABLE_IT(&(hwtim->TimerHandle), TIM_IT_CC3); 
            __HAL_TIM_CLEAR_IT(&(hwtim->TimerHandle),TIM_IT_CC3);
            __HAL_TIM_SET_COMPARE(&(hwtim->TimerHandle),TIM_CHANNEL_3,val);   
            __HAL_TIM_ENABLE_IT(&(hwtim->TimerHandle), TIM_IT_CC3);
        break;
        case HWTIMER_CH4:
            __HAL_TIM_DISABLE_IT(&(hwtim->TimerHandle), TIM_IT_CC4); 
            __HAL_TIM_CLEAR_IT(&(hwtim->TimerHandle),TIM_IT_CC4);
            __HAL_TIM_SET_COMPARE(&(hwtim->TimerHandle),TIM_CHANNEL_4,val);
            __HAL_TIM_ENABLE_IT(&(hwtim->TimerHandle), TIM_IT_CC4);
        break;
        default:
        break;
    }
                                                
    return err;
}
/*
static rt_uint32_t drv_timer_get_capturevalue(rt_device_hwtimer_t *timer,rt_uint8_t ch)
{

    drv_hwtimer_t *hwtim; 
    uint16_t val;
    RT_ASSERT(timer != RT_NULL);
    hwtim = (drv_hwtimer_t *)timer->parent.user_data;
    switch(ch)
    {
        case HWTIMER_CH1:
            val = HAL_TIM_ReadCapturedValue(&(hwtim->TimerHandle),TIM_CHANNEL_1);    
        break;
        case HWTIMER_CH2:
            val = HAL_TIM_ReadCapturedValue(&(hwtim->TimerHandle),TIM_CHANNEL_2);    
        break;
        case HWTIMER_CH3:
            val = HAL_TIM_ReadCapturedValue(&(hwtim->TimerHandle),TIM_CHANNEL_3);    
        break;
        case HWTIMER_CH4:
            val = HAL_TIM_ReadCapturedValue(&(hwtim->TimerHandle),TIM_CHANNEL_4);    
        break;
        default:
        break;
    }
    
    return val;
}*/

static const struct rt_hwtimer_info _info =
{
    1000000,           /* the maximum count frequency can be set */
    1,                  /* the minimum count frequency can be set */
    0xFFFF,            /* the maximum counter value */
    HWTIMER_CNTMODE_UP,/* Increment or Decreasing count mode */
};

static const struct rt_hwtimer_ops _ops =
{
    drv_timer_init,
    drv_timer_start,
    drv_timer_stop,
    drv_timer_set_prescaler,
    drv_timer_get_counter,
    drv_timer_set_counter,
    drv_timer_get_autoreload,
    drv_timer_set_autoreload,
    drv_timer_get_compare,
    drv_timer_set_compare,
    drv_timer_set_frequency,
    //drv_timer_get_capturevalue
};

#ifdef RT_USING_HWTIM6
static drv_hwtimer_t hwtimer6;
static rt_device_hwtimer_t rttimer6;

void TIM6_IRQHandler(void)
{ 
    HAL_TIM_IRQHandler(&(hwtimer6.TimerHandle));       
}
#endif /*RT_USING_HWTIM6*/

#ifdef RT_USING_HWTIM2
static drv_hwtimer_t hwtimer2;
static rt_device_hwtimer_t rttimer2;
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&(hwtimer2.TimerHandle));
}
#endif /*RT_USING_HWTIM2*/

#ifdef RT_USING_HWTIM3
static drv_hwtimer_t hwtimer3;
static rt_device_hwtimer_t rttimer3;
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&(hwtimer3.TimerHandle));
}
#endif /*RT_USING_HWTIM3*/

#ifdef RT_USING_HWTIM4
static drv_hwtimer_t hwtimer4;
static rt_device_hwtimer_t rttimer4;
void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&(hwtimer4.TimerHandle));
}
#endif /*RT_USING_HWTIM4*/

#ifdef RT_USING_HWTIM1
static drv_hwtimer_t hwtimer1;
static rt_device_hwtimer_t rttimer1;
#ifdef RT_USING_HWTIM_CC_IRQ
void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&(hwtimer1.TimerHandle));
}
#endif 
#ifdef RT_USING_HWTIM_UP_IRQ
void TIM1_UP_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&(hwtimer1.TimerHandle));
}
#endif

#endif /*RT_USING_HWTIM1*/

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    
} 

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    #ifdef RT_USING_HWTIM2
    if( TIM2 == htim->Instance )
    {       
        if(HAL_TIM_ACTIVE_CHANNEL_1 == htim->Channel)
        {
            rt_device_hwtimer_isr(&rttimer2, HWTIMER_CH1);
        }
        else if(HAL_TIM_ACTIVE_CHANNEL_2 == htim->Channel)
        {
            rt_device_hwtimer_isr(&rttimer2, HWTIMER_CH2);
        }
        else if(HAL_TIM_ACTIVE_CHANNEL_3 == htim->Channel)
        {
            rt_device_hwtimer_isr(&rttimer2, HWTIMER_CH3);
        }
        else if(HAL_TIM_ACTIVE_CHANNEL_4 == htim->Channel)
        {
            rt_device_hwtimer_isr(&rttimer2, HWTIMER_CH4);
        }       
    }
    #endif
    #ifdef RT_USING_HWTIM3
    if( TIM3 == htim->Instance )
    {
        if(HAL_TIM_ACTIVE_CHANNEL_1 == htim->Channel)
        {
            rt_device_hwtimer_isr(&rttimer3, HWTIMER_CH1);
        }
        else if(HAL_TIM_ACTIVE_CHANNEL_2 == htim->Channel)
        {
            rt_device_hwtimer_isr(&rttimer3, HWTIMER_CH2);
        }
        else if(HAL_TIM_ACTIVE_CHANNEL_3 == htim->Channel)
        {
            rt_device_hwtimer_isr(&rttimer3, HWTIMER_CH3);
        }
        else if(HAL_TIM_ACTIVE_CHANNEL_4 == htim->Channel)
        {
            rt_device_hwtimer_isr(&rttimer3, HWTIMER_CH4);
        }        
    }
    #endif
    #ifdef RT_USING_HWTIM4
    if( TIM4 == htim->Instance )
    {        
        if(HAL_TIM_ACTIVE_CHANNEL_1 == htim->Channel)
        {
            rt_device_hwtimer_isr(&rttimer4, HWTIMER_CH1);
        }
        else if(HAL_TIM_ACTIVE_CHANNEL_2 == htim->Channel)
        {
            rt_device_hwtimer_isr(&rttimer4, HWTIMER_CH2);
        }
        else if(HAL_TIM_ACTIVE_CHANNEL_3 == htim->Channel)
        {
            rt_device_hwtimer_isr(&rttimer4, HWTIMER_CH3);
        }
        else if(HAL_TIM_ACTIVE_CHANNEL_4 == htim->Channel)
        {
            rt_device_hwtimer_isr(&rttimer4, HWTIMER_CH4);
        }       
    }
    #endif
}
extern uint32_t freq_music;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
      
    #ifdef RT_USING_HWTIM1
    //timer = (rt_device_hwtimer_t *)dev;
    //RT_ASSERT(timer != RT_NULL);    
    if( TIM1 == htim->Instance )
    {       
        if(HAL_TIM_ACTIVE_CHANNEL_1 == htim->Channel)
        { 
            
            /*rttimer1.capture_buffer[HWTIMER_CH1][rttimer1.capture_count[HWTIMER_CH1]++] \
            = rttimer1.overflow[HWTIMER_BASE] * rttimer1.reload + HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1); 
            rttimer1.overflow[HWTIMER_BASE] = 0;
            freq_music = rttimer1.capture_buffer[HWTIMER_CH1][rttimer1.capture_count[HWTIMER_CH1]]>>4;
            if(rttimer1.capture_count[HWTIMER_CH1] >= HWTMR_IC_BUF_SIZE)
            {
               rttimer1.capture_count[HWTIMER_CH1] = 0;
            }*/
            rttimer1.capture_value[HWTIMER_CH1] \
            = rttimer1.overflow[HWTIMER_BASE] * rttimer1.reload + HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1); 
            rttimer1.overflow[HWTIMER_BASE] = 0;
            //freq_music = rttimer1.capture_value[HWTIMER_CH1]/15;
            
        }
        else if(HAL_TIM_ACTIVE_CHANNEL_2 == htim->Channel)
        {
            rttimer1.capture_value[HWTIMER_CH2] \
            = rttimer1.overflow[HWTIMER_BASE] * rttimer1.reload + HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2); 
            rttimer1.overflow[HWTIMER_BASE] = 0;
            
        }
        else if(HAL_TIM_ACTIVE_CHANNEL_3 == htim->Channel)
        {
            rttimer1.capture_value[HWTIMER_CH3] \
            = rttimer1.overflow[HWTIMER_BASE] * rttimer1.reload + HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);  
            rttimer1.overflow[HWTIMER_BASE] = 0;  
                     
        }
        else if(HAL_TIM_ACTIVE_CHANNEL_4 == htim->Channel)
        {
            rttimer1.capture_value[HWTIMER_CH4] \
            = rttimer1.overflow[HWTIMER_BASE] * rttimer1.reload + HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);  
            rttimer1.overflow[HWTIMER_BASE] = 0;           
        }
        if (rttimer1.parent.rx_indicate != RT_NULL)
        {
            rttimer1.parent.rx_indicate(&rttimer1.parent, sizeof(struct rt_hwtimer_tmr));
        } 
        
    }
    #endif
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* All channel use the same  TIM_FLAG_UPDATE */
    #ifdef RT_USING_HWTIM6
    if( TIM6 == htim->Instance )
    {
        rt_device_hwtimer_isr(&rttimer6,HWTIMER_BASE);
    }
    #endif
    #ifdef RT_USING_HWTIM2
    if( TIM2 == htim->Instance )
    {                                 
        rt_device_hwtimer_isr(&rttimer2, HWTIMER_BASE); /* base timer no channel output*/       
    }
    #endif
    #ifdef RT_USING_HWTIM3
    if( TIM3 == htim->Instance )
    {
        rt_device_hwtimer_isr(&rttimer3, HWTIMER_BASE); /* base timer no channel output*/        
    }
    #endif
    #ifdef RT_USING_HWTIM4
    if( TIM4 == htim->Instance )
    {                
        rt_device_hwtimer_isr(&rttimer4, HWTIMER_BASE); /* base timer no channel output*/       
    }
    #endif
    #ifdef RT_USING_HWTIM1
    if( TIM1 == htim->Instance )
    {  
        //rt_pin_toggle(PB4);
        rt_device_hwtimer_isr(&rttimer1, HWTIMER_BASE); /* base timer no channel output*/       
    }
    #endif
   
}

int stm32_hwtimer_init(void)
{   
#ifdef RT_USING_HWTIM6      
    rttimer6.info = &_info;
    rttimer6.ops  = &_ops;
    rttimer6.channel_no[HWTIMER_BASE].status = HW_ENABLE;
    rttimer6.channel_lock[HWTIMER_BASE] = 0;
    hwtimer6.TimerHandle.Instance = TIM6;       
    rt_device_hwtimer_register(&rttimer6, "timer6", &hwtimer6);
#endif /*RT_USING_HWTIM6*/
    
/////////////////////////////////////////////////////////////////////////////
#ifdef RT_USING_HWTIM2       
    rttimer2.info = &_info;
    rttimer2.ops  = &_ops;
#if 1 == RT_USING_HWTIM2_EXTERNAL_CLOCK_EN    
    rttimer2.clock_source = HWTIMER_EXTERNAL_CLOCK_SOURCE;
#else
    rttimer2.clock_source = HWTIMER_INTERNAL_CLOCK_SOURCE;
#endif    
#if 1 == RT_USING_HWTIM2_CM_DOWN 
    rttimer2.count_mode = DRV_TIM_COUNT_MODE_DOWN;
#elif 1 == RT_USING_HWTIM2_CM_CTRALGN1
    rttimer2.count_mode = DRV_TIM_COUNT_MODE_CENTER_ALIGN1;
#elif 1 == RT_USING_HWTIM2_CM_CTRALGN2 
    rttimer2.count_mode = DRV_TIM_COUNT_MODE_CENTER_ALIGN2;
#elif 1 == RT_USING_HWTIM2_CM_CTRALGN3
    rttimer2.count_mode = DRV_TIM_COUNT_MODE_CENTER_ALIGN3;
#else
    rttimer2.count_mode = DRV_TIM_COUNT_MODE_UP;
#endif    
   
#ifdef RT_USING_HWTIM2_CH1   
    rttimer2.channel_no[HWTIMER_CH1].status = HW_ENABLE;
    rttimer2.channel_no[HWTIMER_CH1].channel = (uint8_t)TIM_CHANNEL_1;
    rttimer2.channel_lock[HWTIMER_CH1] = 0;
#if 1 == RT_USING_HWTIM2_CH1_PWM
    rttimer2.channel_type[HWTIMER_CH1] = DRV_TIM_CH_TYPE_PWM;
    rttimer2.pwm_dutycycle[HWTIMER_CH1] = 10; 
#elif 1 == RT_USING_HWTIM2_CH1_OCM
    rttimer2.channel_type[HWTIMER_CH1] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM2_CH1_IC
#elif 1 == RT_USING_HWTIM2_CH1_ENCODER
#endif 
#else
    rttimer2.channel_no[HWTIMER_CH1].status = HW_DISABLE;
#endif /* RT_USING_HWTIM2_CH1 */  
#ifdef RT_USING_HWTIM2_CH2   
    rttimer2.channel_no[HWTIMER_CH2].status = HW_ENABLE;
    rttimer2.channel_no[HWTIMER_CH2].channel = (uint8_t)TIM_CHANNEL_2;
    rttimer2.channel_lock[HWTIMER_CH2] = 0;
#if 1 == RT_USING_HWTIM2_CH2_PWM
    rttimer2.channel_type[HWTIMER_CH2] = DRV_TIM_CH_TYPE_PWM;
    rttimer2.pwm_dutycycle[HWTIMER_CH2] = 10; 
#elif 1 == RT_USING_HWTIM2_CH2_OCM
    rttimer2.channel_type[HWTIMER_CH2] = DRV_TIM_CH_TYPE_OC;
#elif 1 ==  RT_USING_HWTIM2_CH2_IC
#elif 1 ==  RT_USING_HWTIM2_CH2_ENCODER
#endif 
#else
    rttimer2.channel_no[HWTIMER_CH2].status = HW_DISABLE;
#endif /* RT_USING_HWTIM2_CH2 */ 
#ifdef RT_USING_HWTIM2_CH3  
    rttimer2.channel_no[HWTIMER_CH3].status = HW_ENABLE;
    rttimer2.channel_no[HWTIMER_CH3].channel = (uint8_t)TIM_CHANNEL_3;
    rttimer2.channel_lock[HWTIMER_CH3] = 0;
#if 1 == RT_USING_HWTIM2_CH3_PWM
    rttimer2.channel_type[HWTIMER_CH3] = DRV_TIM_CH_TYPE_PWM;
    rttimer2.pwm_dutycycle[HWTIMER_CH3] = 10; 
#elif 1 == RT_USING_HWTIM2_CH3_OCM
    rttimer2.channel_type[HWTIMER_CH3] = DRV_TIM_CH_TYPE_OC;
#elif 1 ==  RT_USING_HWTIM2_CH3_IC
#elif 1 ==  RT_USING_HWTIM2_CH3_ENCODER
#endif 
#else
    rttimer2.channel_no[HWTIMER_CH3].status = HW_DISABLE;
#endif /* RT_USING_HWTIM2_CH3 */
#ifdef RT_USING_HWTIM2_CH4 
    rttimer2.channel_no[HWTIMER_CH4].status = HW_ENABLE;
    rttimer2.channel_no[HWTIMER_CH4].channel = (uint8_t)TIM_CHANNEL_4;
    rttimer2.channel_lock[HWTIMER_CH4] = 0;
#if 1 == RT_USING_HWTIM2_CH4_PWM
    rttimer2.channel_type[HWTIMER_CH4] = DRV_TIM_CH_TYPE_PWM;
    rttimer2.pwm_dutycycle[HWTIMER_CH4] = 10; 
#elif 1 == RT_USING_HWTIM2_CH4_OCM
    rttimer2.channel_type[HWTIMER_CH4] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM2_CH4_IC
#elif 1 == RT_USING_HWTIM2_CH4_ENCODER
#endif 
#else
    rttimer2.channel_no[HWTIMER_CH4].status = HW_DISABLE;    
#endif /* RT_USING_HWTIM2_CH4 */ 
         
    hwtimer2.TimerHandle.Instance = TIM2;       
    rt_device_hwtimer_register(&rttimer2, "timer2", &hwtimer2);    
#endif /*RT_USING_HWTIM2*/ 

/////////////////////////////////////////////////////////////////////////////
#ifdef  RT_USING_HWTIM3       
    rttimer3.info = &_info;
    rttimer3.ops  = &_ops;
#if 1 == RT_USING_HWTIM3_EXTERNAL_CLOCK_EN    
    rttimer3.clock_source = HWTIMER_EXTERNAL_CLOCK_SOURCE;
#else
    rttimer3.clock_source = HWTIMER_INTERNAL_CLOCK_SOURCE;
#endif        
#if 1 == RT_USING_HWTIM3_CM_DOWN 
    rttimer3.count_mode = DRV_TIM_COUNT_MODE_DOWN;
#elif 1 == RT_USING_HWTIM3_CM_CTRALGN1
    rttimer3.count_mode = DRV_TIM_COUNT_MODE_CENTER_ALIGN1;
#elif 1 == RT_USING_HWTIM3_CM_CTRALGN2 
    rttimer3.count_mode = DRV_TIM_COUNT_MODE_CENTER_ALIGN2;
#elif 1 == RT_USING_HWTIM3_CM_CTRALGN3
    rttimer3.count_mode = DRV_TIM_COUNT_MODE_CENTER_ALIGN3;
#else
    rttimer3.count_mode = DRV_TIM_COUNT_MODE_UP;
#endif   

#ifdef  RT_USING_HWTIM3_CH1       
    rttimer3.channel_no[HWTIMER_CH1].status = HW_ENABLE;
    rttimer3.channel_no[HWTIMER_CH1].channel = (uint8_t)TIM_CHANNEL_1;
    rttimer3.channel_lock[HWTIMER_CH1] = 0;
#if 1 == RT_USING_HWTIM3_CH1_PWM
    rttimer3.channel_type[HWTIMER_CH1] = DRV_TIM_CH_TYPE_PWM;
    rttimer3.pwm_dutycycle[HWTIMER_CH1] = 10; 
#elif 1 == RT_USING_HWTIM3_CH1_OCM
    rttimer3.channel_type[HWTIMER_CH1] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM3_CH1_IC
#elif 1 == RT_USING_HWTIM3_CH1_ENCODER
#endif 
#else
    rttimer3.channel_no[HWTIMER_CH1].status = HW_DISABLE;
#endif /* RT_USING_HWTIM3_CH1 */
#ifdef  RT_USING_HWTIM3_CH2   
    rttimer3.channel_no[HWTIMER_CH2].status = HW_ENABLE;
    rttimer3.channel_no[HWTIMER_CH2].channel = (uint8_t)TIM_CHANNEL_2;
    rttimer3.channel_lock[HWTIMER_CH2] = 0;
#if 1 == RT_USING_HWTIM3_CH2_PWM
    rttimer3.channel_type[HWTIMER_CH2] = DRV_TIM_CH_TYPE_PWM;
    rttimer3.pwm_dutycycle[HWTIMER_CH2] = 10; 
#elif 1 == RT_USING_HWTIM3_CH2_OCM
    rttimer3.channel_type[HWTIMER_CH2] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM3_CH2_IC
#elif 1 == RT_USING_HWTIM3_CH2_ENCODER
#endif 
#else
    rttimer3.channel_no[HWTIMER_CH2].status = HW_DISABLE;
#endif /* RT_USING_HWTIM3_CH2 */
#ifdef  RT_USING_HWTIM3_CH3      
    rttimer3.channel_no[HWTIMER_CH3].status = HW_ENABLE;
    rttimer3.channel_no[HWTIMER_CH3].channel = (uint8_t)TIM_CHANNEL_3;
    rttimer3.channel_lock[HWTIMER_CH3] = 0;
#if(1 == RT_USING_HWTIM3_CH3_PWM )
    rttimer3.channel_type[HWTIMER_CH3] = DRV_TIM_CH_TYPE_PWM;
    rttimer3.pwm_dutycycle[HWTIMER_CH3] = 10; 
#elif 1 == RT_USING_HWTIM3_CH3_OCM
    rttimer3.channel_type[HWTIMER_CH3] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM3_CH3_IC
#elif 1 == RT_USING_HWTIM3_CH3_ENCODER
#endif 
#else
    rttimer3.channel_no[HWTIMER_CH3].status = HW_DISABLE;
#endif /* RT_USING_HWTIM3_CH3 */
#ifdef RT_USING_HWTIM3_CH4  
    rttimer3.channel_no[HWTIMER_CH4].status = HW_ENABLE;
    rttimer3.channel_no[HWTIMER_CH4].channel = (uint8_t)TIM_CHANNEL_4;
    rttimer3.channel_lock[HWTIMER_CH4] = 0;
#if 1 == RT_USING_HWTIM3_CH4_PWM
    rttimer3.channel_type[HWTIMER_CH4] = DRV_TIM_CH_TYPE_PWM;
    rttimer3.pwm_dutycycle[HWTIMER_CH4] = 10; 
#elif 1 == RT_USING_HWTIM3_CH4_OCM
    rttimer3.channel_type[HWTIMER_CH4] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM3_CH4_IC
#elif 1 == RT_USING_HWTIM3_CH4_ENCODER
#endif 
#else
    rttimer3.channel_no[HWTIMER_CH3].status = HW_DISABLE;
#endif /* RT_USING_HWTIM3_CH4 */

    hwtimer3.TimerHandle.Instance = TIM3;       
    rt_device_hwtimer_register(&rttimer3, "timer3", &hwtimer3);    
#endif /*RT_USING_HWTIM3*/

//////////////////////////////////////////////////////////////////////////////
#ifdef RT_USING_HWTIM4       
    rttimer4.info = &_info;
    rttimer4.ops  = &_ops;
#if 1 == RT_USING_HWTIM4_EXTERNAL_CLOCK_EN    
    rttimer4.clock_source = HWTIMER_EXTERNAL_CLOCK_SOURCE;
#else
    rttimer4.clock_source = HWTIMER_INTERNAL_CLOCK_SOURCE;
#endif    
#if 1 == RT_USING_HWTIM4_CM_DOWN 
    rttimer4.count_mode = DRV_TIM_COUNT_MODE_DOWN;
#elif 1 == RT_USING_HWTIM4_CM_CTRALGN1
    rttimer4.count_mode = DRV_TIM_COUNT_MODE_CENTER_ALIGN1;
#elif 1 == RT_USING_HWTIM4_CM_CTRALGN2 
    rttimer4.count_mode = DRV_TIM_COUNT_MODE_CENTER_ALIGN2;
#elif 1 == RT_USING_HWTIM4_CM_CTRALGN3
    rttimer4.count_mode = DRV_TIM_COUNT_MODE_CENTER_ALIGN3;
#else
    rttimer4.count_mode = DRV_TIM_COUNT_MODE_UP;
#endif 

#ifdef RT_USING_HWTIM4_CH1 
    rttimer4.channel_no[HWTIMER_CH1].status = HW_ENABLE;
    rttimer4.channel_no[HWTIMER_CH1].channel = (uint8_t)TIM_CHANNEL_1;
    rttimer3.channel_lock[HWTIMER_CH1] = 0;
#if 1 == RT_USING_HWTIM4_CH1_PWM
    rttimer4.channel_type[HWTIMER_CH1] = DRV_TIM_CH_TYPE_PWM;
    rttimer4.pwm_dutycycle[HWTIMER_CH1] = 10; 
#elif 1 == RT_USING_HWTIM4_CH1_OCM
    rttimer4.channel_type[HWTIMER_CH1] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM4_CH1_IC
#elif 1 == RT_USING_HWTIM4_CH1_ENCODER
#endif 
#else
    rttimer4.channel_no[HWTIMER_CH1].status = HW_DISABLE;
#endif /* RT_USING_HWTIM4_CH1 */
#ifdef RT_USING_HWTIM4_CH2
    rttimer4.channel_no[HWTIMER_CH2].status = HW_ENABLE;
    rttimer4.channel_no[HWTIMER_CH2].channel = (uint8_t)TIM_CHANNEL_2;
    rttimer3.channel_lock[HWTIMER_CH2] = 0;
#if 1 == RT_USING_HWTIM4_CH2_PWM
    rttimer4.channel_type[HWTIMER_CH2] = DRV_TIM_CH_TYPE_PWM;
    rttimer4.pwm_dutycycle[HWTIMER_CH2] = 10; 
#elif 1 == RT_USING_HWTIM4_CH2_OCM
    rttimer4.channel_type[HWTIMER_CH2] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM4_CH2_IC
#elif 1 == RT_USING_HWTIM4_CH2_ENCODER
#endif  
#else
    rttimer4.channel_no[HWTIMER_CH2].status = HW_DISABLE;
#endif /* RT_USING_HWTIM4_CH2 */     
#ifdef RT_USING_HWTIM4_CH3 
    rttimer4.channel_no[HWTIMER_CH3].status = HW_ENABLE;
    rttimer4.channel_no[HWTIMER_CH3].channel = (uint8_t)TIM_CHANNEL_3;
    rttimer3.channel_lock[HWTIMER_CH3] = 0;
#if 1 == RT_USING_HWTIM4_CH3_PWM
    rttimer4.channel_type[HWTIMER_CH3] = DRV_TIM_CH_TYPE_PWM;
    rttimer4.pwm_dutycycle[HWTIMER_CH3] = 10;  
#elif 1 == RT_USING_HWTIM4_CH3_OCM
    rttimer4.channel_type[HWTIMER_CH3] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM4_CH3_IC
#elif 1 == RT_USING_HWTIM4_CH3_ENCODER
#endif 
#else
    rttimer4.channel_no[HWTIMER_CH3].status = HW_DISABLE;
#endif /* RT_USING_HWTIM4_CH3 */   
#ifdef RT_USING_HWTIM4_CH4
    rttimer4.channel_no[HWTIMER_CH4].status = HW_ENABLE; 
    rttimer4.channel_no[HWTIMER_CH4].channel = (uint8_t)TIM_CHANNEL_4;
    rttimer3.channel_lock[HWTIMER_CH4] = 0;
#if 1 == RT_USING_HWTIM4_CH4_PWM
    rttimer4.channel_type[HWTIMER_CH4] = DRV_TIM_CH_TYPE_PWM;
    rttimer4.pwm_dutycycle[HWTIMER_CH4] = 10; 
#elif 1 == RT_USING_HWTIM4_CH4_OCM
    rttimer4.channel_type[HWTIMER_CH4] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM4_CH4_IC
#elif 1 == RT_USING_HWTIM4_CH4_ENCODER
#endif 
#else
    rttimer4.channel_no[HWTIMER_CH4].status = HW_DISABLE;    
#endif /* RT_USING_HWTIM4_CH4 */    
        
    hwtimer4.TimerHandle.Instance = TIM4;       
    rt_device_hwtimer_register(&rttimer4, "timer4", &hwtimer4);    
#endif /*RT_USING_HWTIM4*/  

//////////////////////////////////////////////////////////////////////////////
#ifdef RT_USING_HWTIM1       
    rttimer1.info = &_info;
    rttimer1.ops  = &_ops;
#if 1 == RT_USING_HWTIM1_EXTERNAL_CLOCK_EN    
    rttimer1.clock_source = HWTIMER_EXTERNAL_CLOCK_SOURCE;
#else
    rttimer1.clock_source = HWTIMER_INTERNAL_CLOCK_SOURCE;
#endif        
#if 1 == RT_USING_HWTIM1_CM_DOWN 
    rttimer1.count_mode = DRV_TIM_COUNT_MODE_DOWN;
#elif 1 == RT_USING_HWTIM1_CM_CTRALGN1
    rttimer1.count_mode = DRV_TIM_COUNT_MODE_CENTER_ALIGN1;
#elif 1 == RT_USING_HWTIM1_CM_CTRALGN2 
    rttimer1.count_mode = DRV_TIM_COUNT_MODE_CENTER_ALIGN2;
#elif 1 == RT_USING_HWTIM1_CM_CTRALGN3
    rttimer1.count_mode = DRV_TIM_COUNT_MODE_CENTER_ALIGN3;
#else
    rttimer1.count_mode = DRV_TIM_COUNT_MODE_UP;
#endif 

#ifdef RT_USING_HWTIM1_CH1 
    rttimer1.channel_no[HWTIMER_CH1].status = HW_ENABLE;
    rttimer1.channel_no[HWTIMER_CH1].channel = (uint8_t)TIM_CHANNEL_1;
    rttimer3.channel_lock[HWTIMER_CH1] = 0;
#if 1 == RT_USING_HWTIM1_CH1_PWM
    rttimer1.channel_type[HWTIMER_CH1] = DRV_TIM_CH_TYPE_PWM;
    rttimer1.pwm_dutycycle[HWTIMER_CH1] = 10; 
#elif 1 == RT_USING_HWTIM1_CH1_OCM
    rttimer1.channel_type[HWTIMER_CH1] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM1_CH1_IC
    rttimer1.channel_type[HWTIMER_CH1] = DRV_TIM_CH_TYPE_IC;
#elif 1 == RT_USING_HWTIM1_CH1_ENCODER
#endif 
#else
    rttimer1.channel_no[HWTIMER_CH1].status = HW_DISABLE;
#endif /* RT_USING_HWTIM1_CH1 */
#ifdef RT_USING_HWTIM1_CH2
    rttimer1.channel_no[HWTIMER_CH2].status = HW_ENABLE;
    rttimer1.channel_no[HWTIMER_CH2].channel = (uint8_t)TIM_CHANNEL_2;
    rttimer3.channel_lock[HWTIMER_CH2] = 0;
#if 1 == RT_USING_HWTIM1_CH2_PWM
    rttimer1.channel_type[HWTIMER_CH2] = DRV_TIM_CH_TYPE_PWM;
    rttimer1.pwm_dutycycle[HWTIMER_CH2] = 10; 
#elif 1 == RT_USING_HWTIM1_CH2_OCM
    rttimer1.channel_type[HWTIMER_CH2] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM1_CH1_IC
#elif 1 == RT_USING_HWTIM1_CH1_ENCODER
#endif  
#else
    rttimer1.channel_no[HWTIMER_CH2].status = HW_DISABLE;
#endif /* RT_USING_HWTIM1_CH2 */     
#ifdef RT_USING_HWTIM1_CH3 
    rttimer1.channel_no[HWTIMER_CH3].status = HW_ENABLE;
    rttimer1.channel_no[HWTIMER_CH3].channel = (uint8_t)TIM_CHANNEL_3;
    rttimer3.channel_lock[HWTIMER_CH3] = 0;
#if 1 == RT_USING_HWTIM1_CH3_PWM
    rttimer1.channel_type[HWTIMER_CH3] = DRV_TIM_CH_TYPE_PWM;
    rttimer1.pwm_dutycycle[HWTIMER_CH3] = 10;  
#elif 1 == RT_USING_HWTIM1_CH3_OCM
    rttimer1.channel_type[HWTIMER_CH3] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM1_CH3_IC
#elif 1 == RT_USING_HWTIM1_CH3_ENCODER
#endif 
#else
    rttimer1.channel_no[HWTIMER_CH3].status = HW_DISABLE;
#endif /* RT_USING_HWTIM1_CH3 */   
#ifdef RT_USING_HWTIM1_CH4
    rttimer1.channel_no[HWTIMER_CH4].status = HW_ENABLE; 
    rttimer1.channel_no[HWTIMER_CH4].channel = (uint8_t)TIM_CHANNEL_4;
    rttimer3.channel_lock[HWTIMER_CH4] = 0;
#if 1 == RT_USING_HWTIM1_CH4_PWM
    rttimer1.channel_type[HWTIMER_CH4] = DRV_TIM_CH_TYPE_PWM;
    rttimer1.pwm_dutycycle[HWTIMER_CH4] = 10; 
#elif 1 == RT_USING_HWTIM1_CH4_OCM
    rttimer1.channel_type[HWTIMER_CH4] = DRV_TIM_CH_TYPE_OC;
#elif 1 == RT_USING_HWTIM1_CH4_IC
#elif 1 == RT_USING_HWTIM1_CH4_ENCODER
#endif 
#else
    rttimer1.channel_no[HWTIMER_CH4].status = HW_DISABLE;    
#endif /* RT_USING_HWTIM1_CH4 */    
        
    hwtimer1.TimerHandle.Instance = TIM1;       
    rt_device_hwtimer_register(&rttimer1, "timer1", &hwtimer1);    
#endif /*RT_USING_HWTIM1*/ 
   
    return 0;
}

INIT_BOARD_EXPORT(stm32_hwtimer_init);
#endif /* RT_USING_HWTIMER */
