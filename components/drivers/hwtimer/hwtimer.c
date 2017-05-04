/*
 * File      : hwtimer.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author         Notes
 * 2015-08-31     heyuanjie87    first version
 */

#include <math.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/hwtimer.h>
#include <drv_hwtimer.h>

rt_inline rt_uint32_t timeout_calc(rt_device_hwtimer_t *timer, rt_uint8_t ch, rt_hwtimer_tmrval_t *tv)
{   
    RT_ASSERT(timer != RT_NULL);   
    
    timer->cycles[ch] = (rt_uint32_t)((tv->sec*1.0 + (tv->usec*1.0/1000000) )* timer->freq);    
    //timer->cycles[ch] = (rt_uint32_t)(tv->sec * timer->freq);
    
    if(0 == timer->cycles[ch])
    {
        timer->cycles[ch] = 1;
    }
       
    return timer->cycles[ch];
}

//extern rt_uint32_t SystemCoreClock;
static rt_err_t rt_hwtimer_init(struct rt_device *dev)
{
    rt_err_t result = RT_EOK;
    rt_device_hwtimer_t *timer;
    
    timer = (rt_device_hwtimer_t *)dev;   
    RT_ASSERT(timer != RT_NULL);          
    RT_ASSERT(timer->freq != 0);
   
    RT_ASSERT(timer != RT_NULL);
    //drv_hwtimer_t *hwtim = (drv_hwtimer_t *)timer->parent.user_data;
    
    //double eps = 1e-6; /* eps = 10^-6 */
    
    /* T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK 
    //T is timer cycles,The value of TIM_Period and TIM_Prescaler is (0-65535), TIMxCLK is system clock 72MHz
    //if T =    1 us (1MHz), TIM_Prescaler = 71 ,  and the TIM_Period = 0 < 65535 ok. 
    //if T =  100 us,(10KHz), TIM_Prescaler = 71 ,  and the TIM_Period = 99 < 65535 ok. 
    //if T =  500 us,(2KHz), TIM_Prescaler = 71 ,  and the TIM_Period = 499 < 65535 ok. 
    //if T =    1 ms,(1KHz), TIM_Prescaler = 71 ,  and the TIM_Period = 999 < 65535 ok. 
    //if T =   10 ms,(100Hz), TIM_Prescaler = 71 ,  and the TIM_Period = 9999 < 65535 ok. 
    //if T =   50 ms,(20Hz), TIM_Prescaler = 71 ,  and the TIM_Period = 49999 < 65535 ok. 
    //if T = 1 s, TIM_Prescaler = 7199, and the TIM_Period = 9999 < 65535 ok.
    */
    #if 0
    if(HWTIMER_EXTERNAL_CLOCK_SOURCE == timer->clock_source)
    {
        timer->prescaler = 0;
    }
    else
    {
        //if((fabs(timer->info->minfreq - 20.0) >= eps)&&(fabs(timer->info->maxfreq - 1000000.0) <= eps))
        if(((uint32_t)(timer->info->minfreq*1000) >= (uint32_t)(20*1000))&&((uint32_t)(timer->info->maxfreq*1000) <=  (uint32_t)(1000000*1000)))
        {
            /*if the frequence that user set is in 20Hz ~ 1MHz */
            /* the timer prescaler is the system clock hclk/freq */
            /* if hclk=72MHz ,and the prescaler is 72000000/1000000 = 72 */               
            timer->prescaler = SystemCoreClock/1000000 - 1; /*set the defualt prescaler is 71 */
            
        }
        //else if((fabs(timer->info->minfreq - 20.0) < eps)&&(fabs(timer->info->minfreq - 1.0) > eps))
        else if(((uint32_t)(timer->info->minfreq*1000) < (uint32_t)(20*1000))&&((uint32_t)(timer->info->minfreq*1000) > (uint32_t)(1*1000)))
        {
            //timer->freq = timer->info->minfreq;
            timer->prescaler = SystemCoreClock/10000 - 1; /*set the defualt prescaler is 7199 */
        }
        if(timer->freq != 0.0)
        {
            timer->reload = (rt_uint16_t)(SystemCoreClock/(timer->prescaler + 1)/timer->freq - 1);
            if( 0 == timer->reload )/*if arr is 0 ,the timer will stop*/
            {
                timer->reload = 1;
            }
        }
        else
        {
            timer->freq = 1;
        }
        //timer_period = sysclk/(timer->prescaler + 1)/freq - 1;  
    }            
    //timer->mode = HWTIMER_MODE_PERIOD;
    #endif
    if (RT_NULL != timer->ops->drv_init)
    {
        timer->ops->drv_init(timer, HW_ENABLE);
    }
    else
    {
        result = -RT_ENOSYS;
    }

    return result;
}

static rt_err_t rt_hwtimer_open(struct rt_device *dev, rt_uint16_t oflag)
{
    rt_err_t result = RT_EOK;
    rt_device_hwtimer_t *timer;
    
    timer = (rt_device_hwtimer_t *)dev;
    RT_ASSERT(timer != RT_NULL);    
    
    if (timer->ops->drv_set_frequency != RT_NULL)
    {
        timer->ops->drv_set_frequency(timer, timer->freq);
    }
    else
    {
        result = -RT_ENOSYS;
    }

    return result;
}
/*
static rt_err_t rt_hwtimer_close(struct rt_device *dev)
{
    rt_err_t result = RT_EOK;
    rt_device_hwtimer_t *timer;

    timer = (rt_device_hwtimer_t*)dev;
    if (timer->ops->drv_init != RT_NULL)
    {
        timer->ops->drv_init(timer, 0);
    }
    else
    {
        result = -RT_ENOSYS;
    }

    dev->flag &= ~RT_DEVICE_FLAG_ACTIVATED;
    dev->rx_indicate = RT_NULL;

    return result;
}*/

/* 
pos ---> channel of timer 
buffer ---> rt_hwtimer_tmrval_t data type
*/
static rt_size_t rt_hwtimer_read(struct rt_device *dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_device_hwtimer_t *timer;
    rt_hwtimer_tmrval_t tv;
    rt_uint32_t cnt;
    
    timer = (rt_device_hwtimer_t *)dev;
    RT_ASSERT(timer != RT_NULL);   
    RT_ASSERT(buffer != RT_NULL); 
          
    if (timer->ops->drv_get_counter == RT_NULL)
        return 0;

    cnt = timer->ops->drv_get_counter(timer);
    if (timer->info->cntmode == HWTIMER_CNTMODE_DW)
    {
        cnt = timer->info->maxcnt - cnt;
    }

    RT_ASSERT(timer->freq != 0.0);
    tv.sec =  timer->overflow[pos]/(uint32_t)timer->freq;   
    tv.usec = (timer->overflow[pos]%(uint32_t)timer->freq)*1000000 + cnt;
    
    size = size > sizeof(tv)? sizeof(tv) : size;
    rt_memcpy(buffer, &tv, size);
        
    return size;
}
/*
pos ---> channel of timer 
buffer ---> rt_hwtimer_tmrval_t data type
*/
static rt_size_t rt_hwtimer_write(struct rt_device *dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    //rt_uint32_t cycles;
    //rt_hwtimer_mode_t opm = HWTIMER_MODE_PERIOD;
    rt_device_hwtimer_t *timer;

    timer = (rt_device_hwtimer_t *)dev;
    RT_ASSERT(timer != RT_NULL);  
    
    if ((timer->ops->drv_start == RT_NULL) || (timer->ops->drv_stop == RT_NULL))
        return 0;

    if (size != sizeof(rt_hwtimer_tmrval_t))
        return 0;
    
    if(1 == timer->channel_lock[pos]) 
    {
        return 0;
    }
    else
    {         
        //calculater the cycles to auto reload value
        timer->cycles[pos] = timeout_calc(timer, pos, (rt_hwtimer_tmrval_t*)buffer);

        //if ((timer->cycles <= 1) && (timer->mode == HWTIMER_MODE_ONESHOT))
        //{
        //    opm = HWTIMER_MODE_ONESHOT;
        //}
        if(0 == timer->cycles[pos])
        {
            return 0;
        }
        timer->ops->drv_stop(timer,pos);
        timer->overflow[pos] = 0;
        timer->channel_lock[pos] = 1; 
        if (timer->ops->drv_start(timer, pos) != RT_EOK)
        {
            size = 0;
        }
    }
    return size;
}

static rt_err_t rt_hwtimer_control(struct rt_device *dev, rt_uint8_t cmd, void *args)
{
    rt_err_t result = RT_EOK;
    rt_device_hwtimer_t *timer;
       
    timer = (rt_device_hwtimer_t *)dev;
    RT_ASSERT(timer != RT_NULL);      
    drv_hwtimer_t *hwtim = (drv_hwtimer_t *)timer->parent.user_data;
    
    //double eps = 1e-6; /* eps = 10^-6 */

    switch (cmd)
    {
        case HWTIMER_CTRL_START:
        {
            rt_hwtimer_chval_t *hwc = (rt_hwtimer_chval_t*)args;       
            if (args == RT_NULL)
            {
                result = -RT_EEMPTY;
                break;
            }
            if (HWTIMER_CH4 < hwc->ch) /* ch is in 0~4*/
            {
                result = -RT_ERROR;
                break;
            }
            if(1 == timer->channel_lock[hwc->ch]) 
            {
                result = -RT_EBUSY;
                break;
            }
            else
            {                
                if(timer->ops->drv_start != RT_NULL)
                {
                    timer->ops->drv_start(timer,hwc->ch);
                }
                else
                {
                    result = -RT_ENOSYS;
                }
            }
            
        }
        break;
        case HWTIMER_CTRL_STOP:
        {
            rt_hwtimer_chval_t *hwc = (rt_hwtimer_chval_t*)args;                     
            if (args == RT_NULL)
            {
                result = -RT_EEMPTY;
                break;
            }
            if (HWTIMER_CH4 < hwc->ch) /* ch is in 0~4*/
            {
                result = -RT_ERROR;
                break;
            }
            if (timer->ops->drv_stop != RT_NULL)
            {
                timer->channel_lock[hwc->ch] = 0;
                timer->ops->drv_stop(timer,hwc->ch);
            }
            else
            {
                result = -RT_ENOSYS;
            }
        }
        break;
        case HWTIMER_CTRL_SET_FREQ:
        {
            rt_hwtimer_chfreq_t *hwq = (rt_hwtimer_chfreq_t*)args;            
            if (args == RT_NULL)
            {
                result = -RT_EEMPTY;
                break;
            } 
            if(HWTIMER_EXTERNAL_CLOCK_SOURCE == timer->clock_source)
            {
                timer->prescaler = 0;
            }
            else
            {
                //if((fabs(timer->info->maxfreq - hwq->freq) > eps)&& (fabs(hwq->freq - 20.0) >= eps))
                if(((uint32_t)(timer->info->maxfreq*1000 ) > (uint32_t)(hwq->freq*1000))&& ((uint32_t)(hwq->freq*1000 ) >= (uint32_t)(20*1000)))
                {
                    timer->prescaler = SystemCoreClock/1000000 - 1; /*set the defualt prescaler is 71 */
                }
                //else if((fabs(20.0 - hwq->freq) > eps) && (fabs(hwq->freq - 1.0) >= eps))
                else if(((uint32_t)(20*1000)  > (uint32_t)(hwq->freq*1000)) && ((uint32_t)(hwq->freq*1000) > (uint32_t)(1*1000)))
                {
                    //timer->freq = timer->info->minfreq;
                    timer->prescaler = SystemCoreClock/10000 - 1; /*set the defualt prescaler is 7199 */
                }                         
            }  
            if (timer->ops->drv_set_frequency != RT_NULL)
            {
                result = timer->ops->drv_set_frequency(timer, hwq->freq);
                if (result == RT_EOK)
                {
                    timer->freq = hwq->freq;
                }
            }
            else
            {
                result = -RT_EEMPTY;
            }
        }
        break;
        case HWTIMER_CTRL_SET_PRESCALER:
        {
            if (timer->ops->drv_set_prescaler != RT_NULL)
            {
                result = timer->ops->drv_set_prescaler(timer, timer->prescaler);
                if (result != RT_EOK)
                {
                    result = -RT_ERROR;
                    break;
                }
            }
            else
            {
                result = -RT_EEMPTY;
            }
        }
        case HWTIMER_CTRL_GET_INFO:
        {
            if (args == RT_NULL)
            {
                result = -RT_EEMPTY;
                break;
            }
            *((struct rt_hwtimer_info*)args) = *timer->info;
        }
        break;
        /*case HWTIMER_CTRL_MODE_SET:
        {
            rt_hwtimer_mode_t *m;
            if (args == RT_NULL)
            {
                result = -RT_EEMPTY;
                break;
            }
            m = (rt_hwtimer_mode_t*)args;            
            timer->mode = *m;
        }
        break; */     
        case HWTIMER_CTRL_GET_AUTORELOAD:
        {            
            rt_hwtimer_chval_t *hwc = (rt_hwtimer_chval_t*)args;                     
            if (args == RT_NULL)
            {
                result = -RT_EEMPTY;
                break;
            }  
            
            if (timer->ops->drv_get_autoreload != RT_NULL)
            {                              
                hwc->value = timer->ops->drv_get_autoreload(timer);      
            }
            else
            {
                result = -RT_ENOSYS;
            }
        }   
        break;
        case HWTIMER_CTRL_SET_AUTORELOAD:
        { 
            rt_hwtimer_chval_t *hwc = (rt_hwtimer_chval_t*)args;                     
            if (args == RT_NULL)
            {
                result = -RT_EEMPTY;
                break;
            }  
            if (args == RT_NULL)
            {
                result = -RT_EEMPTY;
                break;
            }           
            if (timer->ops->drv_set_autoreload != RT_NULL)
            {                           
                result = timer->ops->drv_set_autoreload(timer, hwc->value);      
            }
            else
            {
                result = -RT_ENOSYS;
            }
        }   
        break;
        case HWTIMER_CTRL_GET_DUTY_CYCLE:
        {    
            rt_hwtimer_chval_t *hwc = (rt_hwtimer_chval_t*)args;                     
            if (args == RT_NULL)
            {
                result = -RT_EEMPTY;
                break;
            }  
            if ((HWTIMER_CH1 > hwc->ch)||(HWTIMER_CH4 < hwc->ch)) /* ch is in 1~4*/
            {
                result = -RT_ERROR;
                break;
            }
            if (timer->ops->drv_get_compare != RT_NULL)
            {              
                hwc->value = timer->ops->drv_get_compare(timer, hwc->ch);      
            }
            else
            {
                result = -RT_ENOSYS;
            }
        }   
        break;
        case HWTIMER_CTRL_SET_DUTY_CYCLE:
        {
            rt_hwtimer_chval_t *hwc = (rt_hwtimer_chval_t*)args;
            if (args == RT_NULL)
            {
                result = -RT_EEMPTY;
                break;
            }           

            if ((HWTIMER_CH1 > hwc->ch)||(HWTIMER_CH4 < hwc->ch)) /* ch is in 1~4*/
            {
                result = -RT_ERROR;
                break;
            }
            if (timer->ops->drv_set_compare != RT_NULL)
            {                               
                result = timer->ops->drv_set_compare(timer,hwc->ch, hwc->value);
                if (result != RT_EOK)
                {
                    break;
                }
            }
            else
            {
                result = -RT_ENOSYS;
            }
        }   
        break;
        /*case HWTIMER_CTRL_GET_CAPTURE_VALUE:
        {    
            rt_hwtimer_chval_t *hwc = (rt_hwtimer_chval_t*)args;                     
            if (args == RT_NULL)
            {
                result = -RT_EEMPTY;
                break;
            }  
            if ((HWTIMER_CH1 > hwc->ch)||(HWTIMER_CH4 < hwc->ch)) // ch is in 1~4 
            {
                result = -RT_ERROR;
                break;
            }                                                    
            hwc->value = timer->capture_buffer[hwc->ch];
                                                                                  
        }   
        break;*/
        case HWTIMER_CTRL_GET_TIMER_STATUS:
        {    
            rt_hwtimer_chval_t *hwc = (rt_hwtimer_chval_t*)args;                     
            if (args == RT_NULL)
            {
                result = -RT_EEMPTY;
                break;
            }  
            if ((HWTIMER_BASE > hwc->ch)||(HWTIMER_CH4 < hwc->ch)) // ch is in 1~4 
            {
                result = -RT_ERROR;
                break;
            }                                                    
            hwc->value = timer->channel_lock[hwc->ch];                                                                                  
        }   
        break;        
        default:  
        { 
            result = -RT_ENOSYS;   
            break;
        }
    }

    return result;
}
__weak void rt_device_hwtimer_hook(rt_device_hwtimer_t *timer, rt_uint8_t ch )
{
      timer = timer;
}
void rt_device_hwtimer_isr(rt_device_hwtimer_t *timer, rt_uint8_t ch )
{
    RT_ASSERT(timer != RT_NULL);
    //if(HWTIMER_EXTERNAL_CLOCK_SOURCE != timer->clock_source)
    //{
        if(1 == timer->channel_lock[ch])
        {
            #ifdef RT_USING_HWTIMER_HOOK
            rt_device_hwtimer_hook(timer,ch );
            #endif
                   
            timer->overflow[ch]++;/* add in end of the timer period */

            if (timer->cycles[ch] != 0)
            {
                timer->cycles[ch]--;
            }
            else //if (timer->cycles[tmr][ch] == 0)
            {       
                if (timer->ops->drv_stop != RT_NULL)
                {
                    timer->channel_lock[ch] = 0;
                    timer->ops->drv_stop(timer,ch);
                }        

                if (timer->parent.rx_indicate != RT_NULL)
                {
                    timer->parent.rx_indicate(&timer->parent, sizeof(struct rt_hwtimer_tmr));
                }            
            }        
        }
        else if(HWTIMER_BASE == ch)/* HAL_TIM_PeriodElapsedCallback, All channel use the same  TIM_FLAG_UPDATE */
        {
            timer->overflow[ch]++;/* the times overflow */
        }
    //}
}

rt_err_t rt_device_hwtimer_register(rt_device_hwtimer_t *timer, const char *name, void *user_data)
{
    struct rt_device *device;

    RT_ASSERT(timer != RT_NULL);
    RT_ASSERT(timer->ops != RT_NULL);
    RT_ASSERT(timer->info != RT_NULL);

    device = &(timer->parent);

    device->type        = RT_Device_Class_Timer;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

    device->init        = rt_hwtimer_init;
    device->open        = rt_hwtimer_open;
    device->close       = RT_NULL;
    device->read        = rt_hwtimer_read;
    device->write       = rt_hwtimer_write;
    device->control     = rt_hwtimer_control;
    device->user_data   = user_data;

    return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
}
