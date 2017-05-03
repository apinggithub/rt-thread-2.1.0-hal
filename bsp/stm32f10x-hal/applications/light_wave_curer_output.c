/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2013-07-12     aozima       update for auto initial.
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <drivers/lcdht1621b.h>
#include <drv_lcdht1621b.h>
#include <drivers/hwtimer.h>
#include <drv_hwadc.h>
#include <drivers/hwadc.h>
#include "cvt.h"

#include "drv_led.h"
#include "drv_gpio.h"
#include "drv_hwbutton.h"

//#include "adctest/bsp_adc.h"
#include "light_wave_curer.h"
#ifdef RT_USING_LIGHT_WAVE_CURER
static rt_err_t lwc_cure_timer2_output(rt_device_t dev, lwc_cure_t *lc);
static rt_err_t lwc_cure_timer3_output(rt_device_t dev, lwc_cure_t *lc);
static rt_err_t lwc_cure_timer4_output(rt_device_t dev, lwc_cure_t *lc);
static rt_err_t lwc_cure_ion_output(lwc_cure_t *lc);

static const float freq[][6] = {
    {25.0,  16.0,   8.0,    4.5,    0.5,    6.0},/* 中频 */
    {50.0,   25,    12.0,   5.0,    6.0,    1.5},/* 针灸 */
    {8.0,   4.5,    2.0,    5.0,    6.0,    1.5},/* 拍打 */
    {6.0,   3.0,    2.0,    0.5,    1.5,    2.0},/* 推拿 */
    {5.0,   1.5,    3.5,    0.5,    8.5,    2.5},/* 按摩 */
    {1.5,   4.5,    0.5,    0.5,    4.0,    2.0},/* 拔罐 */
    {5.0,   1.0,    0.5,    0.5,    2.5,    1.5},/* 足疗 */
    {0.5,   0.7,    6.0,    1.5,    9.5,    1.5} /* 减肥 */
};
//uint32_t freq_music = 0;

//static rt_sem_t sem = RT_NULL;
extern struct rt_timer timermusic;
extern uint8_t  music_play_wait;
extern __IO uint16_t adc_convert[TOTAL_CHANNELS][ADC_BUFF_LEN];

adc_dat_buffer_t adc[TOTAL_CHANNELS];
uint16_t adc_valid[TOTAL_CHANNELS];

ALIGN(RT_ALIGN_SIZE)
rt_uint8_t lwc_output_stack[5120];
struct rt_thread lwc_output_thread;

struct rt_timer timerions;
static rt_uint16_t ion_count;
lwc_function_fm_t lff;


static rt_err_t timer3_timeout_cb(rt_device_t dev, rt_size_t size)
{
    rt_event_send(&event, RT_EVENT_LWC_TIMER_FINISH_CLOSE);
    return 0;
}

/*switch timer 4 on ch3 and ch4 in timer6*/
void rt_device_hwtimer_hook(rt_device_hwtimer_t *timer, rt_uint8_t ch )
{
    rt_hwtimer_chfreq_t     hwq;
    rt_device_t dev_hwtimer4 = RT_NULL;   
    
    if((0 < lct.lreg.btn.button_gn)&&(10 >= lct.lreg.btn.button_gn))//&&(music_play_wait <= 13)
    {
        //rt_pin_write(PD2_BEEP, lff.fm_switch);
        lff.fm_switch = (~lff.fm_switch)&0x01;
    } 
    else
    {
        lff.fm_switch = 0;
    }
        
    if ((dev_hwtimer4 = rt_device_find(TIMER4)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER4);
        while(1);
    } 
    rt_device_hwtimer_t *timer4 = (rt_device_hwtimer_t* )dev_hwtimer4;    
           
    if((0x01 == lff.fm_switch)&&(0.0 < timer4->freq))
    {
        if(9 >= lct.lreg.btn.button_gn)
        {
            if(0 < lct.lreg.btn.button_zl1)
            {                                             
                hwq.ch = TMR_CH_CUREI_FREQ;           
                rt_device_control(dev_hwtimer4, HWTIMER_CTRL_START, &hwq);                                
            }
            else
            {
               hwq.ch = TMR_CH_CUREI_FREQ;
               rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP, &hwq);    
            }  
            if(0 < lct.lreg.btn.button_zl2)
            {                                                            
                hwq.ch = TMR_CH_CUREII_FREQ;           
                rt_device_control(dev_hwtimer4, HWTIMER_CTRL_START, &hwq);                                
            }
            else
            {
               hwq.ch = TMR_CH_CUREII_FREQ;
               rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP, &hwq);    
            } 
        }       
    }   
    else 
    {
        hwq.ch = TMR_CH_CUREI_FREQ;
        rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP, &hwq); 
        hwq.ch = TMR_CH_CUREII_FREQ;
        rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP, &hwq); 
    }
}
static rt_err_t timer6_timeout_cb(rt_device_t dev, rt_size_t size)
{
    rt_err_t err;
    rt_hwtimer_chfreq_t     hwq;
    rt_hwtimer_tmrval_t     hwt;
    rt_device_t dev_hwtimer4 = RT_NULL;  
    
    //rt_kprintf("HT %d\n", rt_tick_get());  
    if((0 < lct.lreg.btn.button_gn)&&(10 >= lct.lreg.btn.button_gn))
    {
        hwq.ch = HWTIMER_BASE; 
        
        if(5 < lff.fm_idx)/* 0 ~ 5*/
        {
            lff.fm_idx = 0;
        }                   
        if(9 >= lct.lreg.btn.button_gn)
        {
            hwq.freq = (2*freq[lff.func_idx][lff.fm_idx]);
        }       
        if ((dev_hwtimer4 = rt_device_find(TIMER4)) == RT_NULL)
        {
            rt_kprintf("No Device: %s\n", TIMER4);
            while(1);
        }
        //rt_device_hwtimer_t *timer4 = (rt_device_hwtimer_t* )dev_hwtimer4;                       
        //timer4->freq = hwq.freq;   
        
        if(hwq.freq > 0.0)
        {
            err = rt_device_control(dev, HWTIMER_CTRL_SET_FREQ, &hwq);
            if(9 >= lct.lreg.btn.button_gn)
            {
                if (err != RT_EOK)
                {
                    rt_kprintf("Set Freq[%d][%d] = %fHz Fail\n", lff.func_idx,lff.fm_idx, hwq.freq/2);
                    //while(1);
                }
                else   
                {
                    #ifdef USER_HWTIMER_APP_BUG_TEST
                    rt_kprintf("Set Freq[%d][%d] = %fHz ok\n", lff.func_idx,lff.fm_idx, hwq.freq/2);
                    #endif
                }
            }
            
        }
        else
        {                      
            hwq.ch = TMR_CH_CUREI_FREQ;
            rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP, &hwq); 
            
            hwq.ch = TMR_CH_CUREII_FREQ;
            rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP, &hwq); 
                                            
        } 
        if(0 == lct.lreg.btn.button_sd)
        {
            lff.fm_idx++;
        } 
        switch(lct.lreg.btn.button_gn)
        {                
            case FULL_FUNCTION:   /* 1 全功能 */
            {
              if(5 < lff.fm_idx)
              {
                  lff.fm_idx = 0;
                  if(0 == lct.lreg.btn.button_sd)
                  {
                        lff.func_idx++;
                  }
            
              }  
              if(7 < lff.func_idx ) /* 0 ~ 7, 第 8 音频采样频率 */ 
              {              
                  lff.func_idx = 0;
              }             
            } 
            break; 
            case MID_FREQUENCY:          /* 2 中频 */
            {
                lff.func_idx = 0;
            }
            break; 
            case ACUPUNCTURE_NEEDLE:     /* 3 针灸 */
            {    
                lff.func_idx = 1;
            }
            break; 
            case PAT_CURE:               /* 4 拍打 */
            {
                lff.func_idx = 2;
            }
            break;     
            case NAPRAPATHY:             /* 5 推拿 */
            {
                lff.func_idx = 3;
            }
            break;     
            case MASSOTHERAPY:           /* 6 按摩 */
            {
                lff.func_idx = 4;
            }
            break;     
            case CUPPING_THERAPY:        /* 7 拔罐 */
            {
                lff.func_idx = 5;
            }
            break;     
            case PEDICURE:               /* 8 足疗 */
            {
                lff.func_idx = 6;
            }
            break; 
            case LOSE_WEIGHT:            /* 9 减肥 */
            {
                lff.func_idx = 7;
            }
            break;   
            case VOICE_FREQUENCY:            /* 10 音频 不在频率索引列表*/
            {
                lff.func_idx = 8;       
            }
            break;  
            default:
                //lff.func_idx = 0;
            break;        
        } 
        if(9 >= lct.lreg.btn.button_gn)
        {                       
            hwt.sec = 10; /* keep 10s every group */ 
            hwt.usec = 0;
            //rt_kprintf("SetTime: Sec %d, Usec %d\n", hwt.sec, hwt.usec);   
            if (rt_device_write(dev, hwq.ch, &hwt, sizeof(hwt)) != sizeof(hwt))
            {
                rt_kprintf("SetTime Fail\n");
                while(1);
            }
            else
            {
                #ifdef USER_HWTIMER_APP_BUG_TEST
                rt_kprintf("Set timer work on  = %dsec. ok.\n", hwt.sec);
                #endif
            }                  
        }
    }   
    return 0;
}
static void timeout_ionswtich(void* parameter)
{

	if(1200-1 > ion_count)
    {
        ion_count++;
    }
    else
    {
        ion_count = 0;
        //rt_timer_stop(&timerions);
        //rt_event_send(&event, RT_EVENT_LWC_TIMER_FINISH_CLOSE);
    }
}

void lwc_output_thread_entry(void* parameter)
{
    rt_err_t err;
    rt_uint8_t vcno;
    rt_hwtimer_chfreq_t hwq;
    rt_hwtimer_chval_t hwc;
    rt_uint32_t recv_event;
    
    rt_device_t dev_xtp = RT_NULL;
       
#if 1    
#ifdef RT_USING_HWTIM3  
    rt_device_t dev_hwtimer3 = RT_NULL;
    rt_device_hwtimer_t *timer3;
    if ((dev_hwtimer3 = rt_device_find(TIMER3)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER3);
        while(1);
    }
    timer3 = (rt_device_hwtimer_t *)dev_hwtimer3;  
    timer3->freq = 1;
    timer3->prescaler = 7199;
    timer3->reload = 9999;
    //d = (fs/(f*p))-1 fs-->system frequency 72MHz,f --->timer output(interrupt),p-->prescaler    
    if (rt_device_open(dev_hwtimer3, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", TIMER3);
        while(1);
    }
    rt_device_set_rx_indicate(dev_hwtimer3, timer3_timeout_cb);
     /* set the frequency */
    hwq.freq = timer3->freq;
    err = rt_device_control(dev_hwtimer3, HWTIMER_CTRL_SET_FREQ, &hwq);
    if (err != RT_EOK)
    {
        rt_kprintf("Set timer freq = %f Hz Fail! And close the %s \r\n" ,timer3->freq, TIMER3);
        err = rt_device_close(dev_hwtimer3);
        while(1);
    }
#endif /* RT_USING_HWTIM3 */          
#endif
    
#if 1  
#ifdef RT_USING_HWTIM2  
    rt_device_t dev_hwtimer2 = RT_NULL;
    rt_device_hwtimer_t *timer2;
    if ((dev_hwtimer2 = rt_device_find(TIMER2)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER2);
        while(1);
    }
    timer2 = (rt_device_hwtimer_t *)dev_hwtimer2;  
    timer2->freq = 100;
    timer2->prescaler = 71;
    timer2->reload = 9999;
    //d = (fs/(f*p))-1 fs-->system frequency 72MHz,f --->timer output(interrupt),p-->prescaler    
    if (rt_device_open(dev_hwtimer2, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", TIMER2);
        while(1);
    }
    //rt_device_set_rx_indicate(dev_hwtimer2, timer2_timeout_cb);
     /* set the frequency */
    hwq.freq = timer2->freq;
    err = rt_device_control(dev_hwtimer2, HWTIMER_CTRL_SET_FREQ, &hwq);
    if (err != RT_EOK)
    {
        rt_kprintf("Set timer freq = %f Hz Fail! And close the %s \r\n" ,timer2->freq, TIMER2);
        err = rt_device_close(dev_hwtimer2);
        while(1);
    }
    //hwq.ch = TMR_CH_CUREI_PWM;           
    //rt_device_control(dev_hwtimer2, HWTIMER_CTRL_START, &hwq);  
    //hwq.ch = TMR_CH_CUREII_PWM;           
    //rt_device_control(dev_hwtimer2, HWTIMER_CTRL_START, &hwq); 
#endif /* RT_USING_HWTIM2 */   
#endif  
 
#if 1    
#ifdef RT_USING_HWTIM4
    rt_device_t dev_hwtimer4 = RT_NULL;    
    rt_device_hwtimer_t *timer4;
    if ((dev_hwtimer4 = rt_device_find(TIMER4)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER4);
        while(1);
    } 
    timer4 = (rt_device_hwtimer_t *)dev_hwtimer4;  
    timer4->freq = 1200;/*1200 clock / second */
    timer4->prescaler = 71;
    timer4->reload = 833;
    if (rt_device_open(dev_hwtimer4, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", TIMER4);
        while(1);
    }
    //rt_device_set_rx_indicate(dev_hwtimer4, timer4_timeout_cb);
    hwq.freq = timer4->freq;
    err = rt_device_control(dev_hwtimer4, HWTIMER_CTRL_SET_FREQ, &hwq);
    if (err != RT_EOK)
    {
        rt_kprintf("Set timer freq = %f Hz Fail! And close the %s \r\n" ,timer4->freq, TIMER4);
        err = rt_device_close(dev_hwtimer4);
        while(1);
    } 
    //hwq.ch = TMR_CH_CUREI_FREQ;           
    //rt_device_control(dev_hwtimer4, HWTIMER_CTRL_START, &hwq);  
    //hwq.ch = TMR_CH_CUREII_FREQ;           
    //rt_device_control(dev_hwtimer4, HWTIMER_CTRL_START, &hwq); 
#endif /* RT_USING_HWTIM4 */   
#endif    

#if 1    
#ifdef RT_USING_HWTIM6
    rt_device_t dev_hwtimer6 = RT_NULL;
    
    hwq.ch = HWTIMER_BASE;
    //rt_pin_mode(54, PIN_MODE_OUTPUT);// the port PD2
   
    if((dev_hwtimer6 = rt_device_find(TIMER6)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER6);
        while(1);
    }
    
    rt_device_hwtimer_t *timer6 = (rt_device_hwtimer_t *)dev_hwtimer6;
    timer6 = (rt_device_hwtimer_t *)dev_hwtimer6;  
    timer6->freq = 10;
    timer6->prescaler = 7199;
    timer6->reload = 999;       
       
    if (rt_device_open(dev_hwtimer6, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", TIMER6);
        while(1);
    }
    
    rt_device_set_rx_indicate(dev_hwtimer6, timer6_timeout_cb);
    hwq.freq = timer6->freq;
    err = rt_device_control(dev_hwtimer6, HWTIMER_CTRL_SET_FREQ, &hwq);
    if (err != RT_EOK)
    {
        rt_kprintf("Set Freq = %f Hz Fail\n", hwq.freq);
        while(1);
    }

#endif /* RT_USING_HWTIM6 */ 
#endif    

#if 1    
#ifdef RT_USING_HWTIM1
    //arr:重装值
    //psc:分频值
    //Tout=((arr+1)*(psc+1))/Ft ---> Ft = fout*(arr+1)*(psc+1)
    //Ft:定时器工作频率Hz
    rt_device_t dev_hwtimer1 = RT_NULL;    
    rt_device_hwtimer_t *timer1;
    if ((dev_hwtimer1 = rt_device_find(TIMER1)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER1);
        while(1);
    } 
    timer1 = (rt_device_hwtimer_t *)dev_hwtimer1;  
    timer1->freq = 1000;
    timer1->prescaler = 0;
    timer1->reload = 1;
    if (rt_device_open(dev_hwtimer1, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", TIMER1);
        while(1);
    }
    //rt_device_set_rx_indicate(dev_hwtimer1, timer1_timeout_cb);
    hwq.freq = timer1->freq;
    err = rt_device_control(dev_hwtimer1, HWTIMER_CTRL_SET_FREQ, &hwq);
    if (err != RT_EOK)
    {
        rt_kprintf("Set timer freq = %f Hz Fail! And close the %s\n" ,timer1->freq, TIMER1);
        err = rt_device_close(dev_hwtimer1);
        while(1);
    } 
    //hwc.ch = HWTIMER_BASE;
    //rt_device_control(dev_hwtimer1, HWTIMER_CTRL_START, &hwc);
    
#endif /* RT_USING_HWTIM1 */  
        
#endif   
#ifdef RT_USING_HWADC  
    rt_device_t devadc = RT_NULL;
    if ((devadc = rt_device_find(HWADC)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", HWADC);
        while(1);
    }   
    if (rt_device_open(devadc, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", HWADC);
        while(1);
    }
    rt_device_hwadc_t *hwadc = (rt_device_hwadc_t *)devadc;
    
    if(RT_EOK != rt_device_control(devadc, HWTADC_CTRL_START, RT_NULL))
    {
        //rt_device_read(dev, 0, &recv_adc, sizeof(uint32_t));
        while(1);
    }
#endif    
	rt_timer_init(&timerions, "timerions", 
	timeout_ionswtich, 
	RT_NULL, 
	1000, /*1000 tick */
	RT_TIMER_FLAG_PERIODIC); 	
	//rt_timer_start(&timerions);
    
           
    rt_pin_mode(PD2_BEEP, PIN_MODE_OUTPUT);
    rt_pin_mode(PB5_IONTHERAPY_RLY, PIN_MODE_OUTPUT);
    rt_pin_mode(PB12_IONTHERAPY_PWR, PIN_MODE_OUTPUT);
    rt_pin_mode(PB13_IONTHERAPY_CRL1, PIN_MODE_OUTPUT);
    rt_pin_mode(PB14_IONTHERAPY_CRL2, PIN_MODE_OUTPUT);
    rt_pin_mode(PB15_IONTHERAPY_DECT, PIN_MODE_INPUT);
    
    
    while(1)
    {
        
        rt_memcpy(adc, adc_convert, TOTAL_CHANNELS*sizeof(adc_dat_buffer_t));    
        for(uint8_t i = 0; i < TOTAL_CHANNELS; i++)
        {
            bubble16(adc[i].adc_value, ADC_BUFF_LEN, 0);
        }        
        adc_valid[0] = adc[0].adc_value[ADC_BUFF_LEN/2];
        #ifdef USER_HWADC_APP_BUG_TEST
        rt_kprintf("adc[0]  value = %d -> %fV \r\n",adc_valid[0]&0xFFF,(float)(adc_valid[0]&0xFFF)*3.3/4096); 
        #endif
      
        //rt_pin_write(PB5_IONTHERAPY_SW, PIN_HIGH);
        //rt_pin_write(PB12_IONTHERAPY_PWR, PIN_LOW);	
        if (rt_event_recv(&event, (RT_EVENT_LWC_TIMER_FINISH_CLOSE
                                    |RT_EVENT_LWC_DEVICE_FORCE_CLOSE
                                    |RT_EVENT_LWC_RLJG_BUTTON_UPDATE
                                    |RT_EVENT_LWC_GNZL_BUTTON_UPDATE
                                    |RT_EVENT_LWC_LASER_CURE_CLOSE
                                    |RT_EVENT_LWC_HEAT_CURE_CLOSE
                                    |RT_EVENT_LWC_ION_FUNC_START
                                    |RT_EVENT_LWC_ION_CURE_CLOSE
                                    ),
                           RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                           RT_TICK_PER_SECOND/10, &recv_event) == RT_EOK)
        {
            switch(recv_event)
            {
                case RT_EVENT_LWC_TIMER_FINISH_CLOSE:
                case RT_EVENT_LWC_DEVICE_FORCE_CLOSE:    
                {
                    lct.lreg.btn.button_jg = 0;
                    lct.lway[LASER_CURE].status = LWC_INACTIVE;
                    lct.lcf[LASER_CURE].cure_out_actived = LWC_INACTIVE;
                    hwq.ch = TMR_CH_LASER_PWM;
                    rt_device_control(dev_hwtimer3, HWTIMER_CTRL_STOP, &hwq); 
                    
                    lct.lreg.btn.button_rl = 0;
                    lct.lway[HEAT_CURE].status = LWC_INACTIVE;
                    lct.lcf[HEAT_CURE].cure_out_actived = LWC_INACTIVE;
                    hwq.ch = TMR_CH_HEAT_PWM;
                    rt_device_control(dev_hwtimer3, HWTIMER_CTRL_STOP, &hwq); 
                    
                    lct.lreg.btn.button_gn = 0;
                    lct.lreg.btn.button_zl1 = 0;
                    lct.lreg.btn.button_zl2 = 0;
                    lct.lway[FUNCTION].status = LWC_INACTIVE;
                    lct.lcf[FUNCTION].cure_out_actived = LWC_INACTIVE;
                    #if 1
                    hwq.ch = TMR_CH_CUREI_PWM;
                    rt_device_control(dev_hwtimer2, HWTIMER_CTRL_STOP, &hwq); 
                    hwq.ch = TMR_CH_CUREII_PWM;
                    rt_device_control(dev_hwtimer2, HWTIMER_CTRL_STOP, &hwq); 
                    #endif
                    hwq.ch = TMR_CH_CUREI_FREQ;
                    rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP, &hwq);
                    hwq.ch = TMR_CH_CUREII_FREQ;
                    rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP, &hwq);   
                    hwq.ch = TMR_CH_BASE;
                    rt_device_control(dev_hwtimer6, HWTIMER_CTRL_STOP, &hwq);  
                    
                    #ifdef RT_USING_HWTIM1
                    hwc.ch = HWTIMER_CH1;
                    rt_device_control(dev_hwtimer1, HWTIMER_CTRL_STOP, &hwc);
                    #endif /* RT_USING_HWTIM1 */      
                    
                    lct.lreg.btn.button_lzlf = 0;
                    lct.lway[IONICE_CURE].status = LWC_INACTIVE;
                    lct.lcf[IONICE_CURE].cure_out_actived = LWC_INACTIVE;
                    rt_timer_stop(&timerions);
                    rt_pin_write(PB12_IONTHERAPY_PWR, PIN_LOW);
                    rt_pin_write(PB13_IONTHERAPY_CRL1,PIN_LOW);
                    rt_pin_write(PB14_IONTHERAPY_CRL2,PIN_LOW);
                    rt_pin_write(PB5_IONTHERAPY_RLY, PIN_LOW);                                     
                    rt_pin_write(PD2_BEEP, PIN_LOW);                     
                    lct.ion_force_overtimes = 0;                                 
                    rt_event_send(&event, RT_EVENT_LWC_DEVICE_POWER_CLOSE);
                }
                break;
                case RT_EVENT_LWC_RLJG_BUTTON_UPDATE:
                {
                    lwc_cure_timer3_output(dev_hwtimer3, (lwc_cure_t *)&lct);
                }
                break;
                case RT_EVENT_LWC_GNZL_BUTTON_UPDATE:    
                {
                    if(((0 <= lct.lreg.btn.button_gn)&&(9 >= lct.lreg.btn.button_gn))
                        &&((0 == lct.lreg.btn.button_zl1 )&&(0 == lct.lreg.btn.button_zl2 )))
                    {                                                
                        rt_timer_start(&timertip);                        
                        flag_force_voice_play = 1;                        
                        rt_timer_stop(&timermusic);
                        #ifdef RT_USING_HWTIM1
                        hwc.ch = HWTIMER_CH1;
                        rt_device_control(dev_hwtimer1, HWTIMER_CTRL_STOP, &hwc);
                        #endif /* RT_USING_HWTIM1 */                                                 
                    }
                    else if(10 == lct.lreg.btn.button_gn )
                    {
                        rt_timer_start(&timermusic);
                        #ifdef RT_USING_HWTIM1
                        rt_device_t dev_hwtimer1 = rt_device_find(TIMER1);                        
                        rt_hwtimer_chval_t hwc;
                        if (dev_hwtimer1 == RT_NULL)
                        {
                            rt_kprintf("No Device: %s\n", TIMER1);
                            while(1);
                        }
                        else
                        {
                            hwc.ch = HWTIMER_CH1;
                            rt_device_control(dev_hwtimer1, HWTIMER_CTRL_START, &hwc);                                         
                        }
                        #endif 
                        if((0 == lct.lreg.btn.button_zl1 )&&(0 == lct.lreg.btn.button_zl2 ))
                        {
                            //if(0 == flag_voice_close)
                            //{
                            //    rt_thread_delay(RT_TICK_PER_SECOND );
                            //    vcno = 0x5A + 18;/* 请调节治疗输出强度 */
                            //    rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                            //} 
                            
                            rt_thread_delay(2*RT_TICK_PER_SECOND );                                                                                                                     
                            rt_event_send(&event, RT_EVENT_LWC_TIMER_MUSIC_PLAY);
                        }                         
                    }                                                          
                    if((0 == lct.lreg.btn.button_zl1 )&&(0 == lct.lreg.btn.button_zl2 ))
                    {                       
                        lct.lreg.btn.flag_base_timer_status = LWC_BASE_TIMER_IDLE;
                        hwq.ch = TMR_CH_BASE;  
                        rt_device_control(dev_hwtimer6, HWTIMER_CTRL_STOP, &hwq);                       
                    }                                                      
                    lwc_cure_timer2_output(dev_hwtimer2, (lwc_cure_t *)&lct);
                    lwc_cure_timer4_output(dev_hwtimer4, (lwc_cure_t *)&lct);
                }
                break;
                case RT_EVENT_LWC_LASER_CURE_CLOSE:
                {
                    hwq.ch = TMR_CH_LASER_PWM;
                    rt_device_control(dev_hwtimer3, HWTIMER_CTRL_STOP, &hwq); 
                }
                break;
                case RT_EVENT_LWC_HEAT_CURE_CLOSE:
                {
                    hwq.ch = TMR_CH_HEAT_PWM;
                    rt_device_control(dev_hwtimer3, HWTIMER_CTRL_STOP, &hwq); 
                }
                break;
                case RT_EVENT_LWC_ION_FUNC_START:    
                {
                    lct.lreg.btn.button_jg = 0;
                    lct.lway[LASER_CURE].status = LWC_INACTIVE;
                    lct.lcf[LASER_CURE].cure_out_actived = LWC_INACTIVE;
                    hwq.ch = TMR_CH_LASER_PWM;
                    rt_device_control(dev_hwtimer3, HWTIMER_CTRL_STOP, &hwq); 
                    
                    lct.lreg.btn.button_rl = 0;
                    lct.lway[HEAT_CURE].status = LWC_INACTIVE;
                    lct.lcf[HEAT_CURE].cure_out_actived = LWC_INACTIVE;
                    hwq.ch = TMR_CH_HEAT_PWM;
                    rt_device_control(dev_hwtimer3, HWTIMER_CTRL_STOP, &hwq); 
                    
                    lct.lreg.btn.button_gn = 0;
                    lct.lreg.btn.button_zl1 = 0;
                    lct.lreg.btn.button_zl2 = 0;
                    lct.lway[FUNCTION].status = LWC_INACTIVE;
                    lct.lcf[FUNCTION].cure_out_actived = LWC_INACTIVE;
                    #if 1
                    hwq.ch = TMR_CH_CUREI_PWM;
                    rt_device_control(dev_hwtimer2, HWTIMER_CTRL_STOP, &hwq); 
                    hwq.ch = TMR_CH_CUREII_PWM;
                    rt_device_control(dev_hwtimer2, HWTIMER_CTRL_STOP, &hwq); 
                    #endif
                    hwq.ch = TMR_CH_CUREI_FREQ;
                    rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP, &hwq);
                    hwq.ch = TMR_CH_CUREII_FREQ;
                    rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP, &hwq);   
                    hwq.ch = TMR_CH_BASE;
                    rt_device_control(dev_hwtimer6, HWTIMER_CTRL_STOP, &hwq); 
                    
                    //#ifdef RT_USING_HWTIMER1
                    //hwc.ch = HWTIMER_CH1;
                    //rt_device_control(dev_hwtimer1, HWTIMER_CTRL_STOP, &hwc);
                    //#endif
                    
                    lct.lcf[IONICE_CURE].cure_out_actived = LWC_ACTIVED; 
                    rt_event_send(&event, RT_EVENT_LWC_ION_TIME_UPDATE);
                }
                break;
                case RT_EVENT_LWC_ION_CURE_CLOSE:
                {
                    rt_timer_stop(&timerions);
                    rt_pin_write(PB12_IONTHERAPY_PWR, PIN_LOW);
                    rt_pin_write(PB13_IONTHERAPY_CRL1,PIN_LOW);
                    rt_pin_write(PB14_IONTHERAPY_CRL2,PIN_LOW);
                    rt_pin_write(PB5_IONTHERAPY_RLY, PIN_LOW);                                     
                    rt_pin_write(PD2_BEEP, PIN_LOW);  
                    lct.lcf[IONICE_CURE].cure_out_actived = LWC_INACTIVE;
                }
                break;
                default:
                break;
            }                      
        }        
        lwc_cure_ion_output((lwc_cure_t *)&lct);     
      
        rt_thread_delay( RT_TICK_PER_SECOND/100 );
        
    }
}

static rt_err_t lwc_cure_timer3_output(rt_device_t dev, lwc_cure_t *lc)
{
    rt_err_t err = RT_EOK;
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(lc != RT_NULL);   
    rt_hwtimer_chval_t hwc;   
    
    if(LWC_ACTIVED == lc->lway[LASER_CURE].status)
    {
        if(1 == lc->lreg.tval.tmr_lock)
        { 
            if(LWC_INACTIVE == lc->lcf[LASER_CURE].cure_out_actived)
            {                
                lc->lcf[LASER_CURE].cure_out_actived = LWC_ACTIVED;
                                             
            }            
            if(1 == lc->lreg.btn.button_jg)/* low */
            {
                /* get the reload of the timer */
                hwc.ch = TMR_CH_LASER_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_GET_AUTORELOAD, &hwc); 
                if (err != RT_EOK)
                {
                    rt_kprintf("Get the timer reload Fail\n");
                    return err;
                }
                /* set duty cycles */
                uint16_t tval = hwc.value;               
                hwc.value = tval*1/3;
                hwc.ch = TMR_CH_LASER_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_SET_DUTY_CYCLE, &hwc);
                if (err != RT_EOK)
                {
                    rt_kprintf("Set ch  = %d pwm Fail\n", hwc.ch);
                    return err;
                }
                else
                {
                    #ifdef USER_HWTIMER_APP_BUG_TEST
                    rt_kprintf("Set ch = %d pwm = %d ok,\n", hwc.ch, hwc.value);
                    #endif
                }
                hwc.ch = TMR_CH_LASER_PWM;
                rt_device_control(dev, HWTIMER_CTRL_START, &hwc);
            }
            else if(2 == lc->lreg.btn.button_jg) /* middle */
            {
                hwc.ch = TMR_CH_LASER_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_GET_AUTORELOAD, &hwc); 
                if (err != RT_EOK)
                {
                    rt_kprintf("Get the timer reload Fail\n");
                    return err;
                }
                /* set duty cycles */
                uint16_t tval = hwc.value;
                hwc.value = tval*2/3;
                hwc.ch = TMR_CH_LASER_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_SET_DUTY_CYCLE, &hwc);
                if (err != RT_EOK)
                {
                    rt_kprintf("Set ch  = %d pwm Fail\n", hwc.ch);
                    return err;
                } 
                else
                {
                    #ifdef USER_HWTIMER_APP_BUG_TEST
                    rt_kprintf("Set ch = %d pwm = %d ok,\n", hwc.ch, hwc.value);
                    #endif
                }
            }
            else if(3 == lc->lreg.btn.button_jg) /* hight */
            {
                hwc.ch = TMR_CH_LASER_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_GET_AUTORELOAD, &hwc); 
                if (err != RT_EOK)
                {
                    rt_kprintf("Get the timer reload Fail\n");
                    return err;
                }
                /* set duty cycles */
                uint16_t tval = hwc.value;
                hwc.value = tval*3/3;
                hwc.ch = TMR_CH_LASER_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_SET_DUTY_CYCLE, &hwc);
                if (err != RT_EOK)
                {
                    rt_kprintf("Set ch  = %d pwm Fail\n", hwc.ch);
                    return err;
                } 
                else
                {
                    #ifdef USER_HWTIMER_APP_BUG_TEST
                    rt_kprintf("Set ch = %d pwm = %d ok,\n", hwc.ch, hwc.value);
                    #endif
                }
            }                                               
        }             
    }
    if(LWC_ACTIVED == lc->lway[HEAT_CURE].status)
    {
        if(1 == lc->lreg.tval.tmr_lock)
        { 
            if(LWC_INACTIVE == lc->lcf[HEAT_CURE].cure_out_actived)
            {                
                lc->lcf[HEAT_CURE].cure_out_actived = LWC_ACTIVED;
                                             
            }            
            if(1 == lc->lreg.btn.button_rl)/* low */
            {
                /* get the reload of the timer */
                hwc.ch = TMR_CH_HEAT_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_GET_AUTORELOAD, &hwc); 
                if (err != RT_EOK)
                {
                    rt_kprintf("Get the timer reload Fail\n");
                    return err;
                }
                /* set duty cycles */
                uint16_t tval = hwc.value;               
                hwc.value = tval*1/3;
                hwc.ch = TMR_CH_HEAT_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_SET_DUTY_CYCLE, &hwc);
                if (err != RT_EOK)
                {
                    rt_kprintf("Set ch  = %d pwm Fail\n", hwc.ch);
                    return err;
                }
                else
                {
                    #ifdef USER_HWTIMER_APP_BUG_TEST
                    rt_kprintf("Set ch = %d pwm = %d ok,\n", hwc.ch, hwc.value);
                    #endif
                }
                hwc.ch = TMR_CH_HEAT_PWM;
                rt_device_control(dev, HWTIMER_CTRL_START, &hwc);
            }
            else if(2 == lc->lreg.btn.button_rl) /* middle */
            {
                hwc.ch = TMR_CH_HEAT_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_GET_AUTORELOAD, &hwc); 
                if (err != RT_EOK)
                {
                    rt_kprintf("Get the timer reload Fail\n");
                    return err;
                }
                /* set duty cycles */
                uint16_t tval = hwc.value;
                hwc.value = tval*2/3;
                hwc.ch = TMR_CH_HEAT_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_SET_DUTY_CYCLE, &hwc);
                if (err != RT_EOK)
                {
                    rt_kprintf("Set ch  = %d pwm Fail\n", hwc.ch);
                    return err;
                } 
                else
                {
                    #ifdef USER_HWTIMER_APP_BUG_TEST
                    rt_kprintf("Set ch = %d pwm = %d ok,\n", hwc.ch, hwc.value);
                    #endif
                }
            }
            else if(3 == lc->lreg.btn.button_rl) /* hight */
            {
                hwc.ch = TMR_CH_HEAT_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_GET_AUTORELOAD, &hwc); 
                if (err != RT_EOK)
                {
                    rt_kprintf("Get the timer reload Fail\n");
                    return err;
                }
                /* set duty cycles */
                uint16_t tval = hwc.value;
                hwc.value = tval*3/3;
                hwc.ch = TMR_CH_HEAT_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_SET_DUTY_CYCLE, &hwc);
                if (err != RT_EOK)
                {
                    rt_kprintf("Set ch  = %d pwm Fail\n", hwc.ch);
                    return err;
                } 
                else
                {
                    #ifdef USER_HWTIMER_APP_BUG_TEST
                    rt_kprintf("Set ch = %d pwm = %d ok,\n", hwc.ch, hwc.value);
                    #endif
                }
            }                                               
        } 
    }
    return err;
}
#if 1
static rt_err_t lwc_cure_timer2_output(rt_device_t dev, lwc_cure_t *lc)
{
    rt_err_t err = RT_EOK;
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(lc != RT_NULL);
    //rt_device_hwtimer_t *timer = (rt_device_hwtimer_t *)dev;   
    rt_hwtimer_chval_t hwc; 
    rt_hwtimer_tmrval_t hwt; 
    
    if(LWC_ACTIVED == lc->lway[FUNCTION].status)
    {
        if(1 == lc->lreg.tval.tmr_lock)
        { 
            if(LWC_INACTIVE == lc->lcf[FUNCTION].cure_out_actived)
            {                
                lc->lcf[FUNCTION].cure_out_actived = LWC_ACTIVED;
            }   
            /* get the reload of the timer */
            if(0 < lc->lreg.btn.button_zl1)
            {
                hwc.ch = TMR_CH_CUREI_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_GET_AUTORELOAD, &hwc); 
                if (err != RT_EOK)
                {
                    rt_kprintf("Get the timer reload Fail\n");
                    return err;
                }
                /* set duty cycles */
                uint16_t tval = hwc.value;                    
                hwc.value = tval/5 + (lc->lreg.btn.button_zl1)*(tval - tval/5)/23;
                hwc.ch = TMR_CH_CUREI_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_SET_DUTY_CYCLE, &hwc);
                if (err != RT_EOK)
                {
                    rt_kprintf("Set ch  = %d pwm Fail\n", hwc.ch);
                    return err;
                }
                else
                {
                    #ifdef USER_HWTIMER_APP_BUG_TEST
                    rt_kprintf("Set ch = %d pwm = %d ok,\n", hwc.ch, hwc.value);
                    #endif
                }                
                if(1 == lc->lreg.btn.button_zl1)
                {                                                 
                    hwc.ch = TMR_CH_CUREI_PWM;                
                    rt_device_control(dev, HWTIMER_CTRL_START, &hwc);                                      
                }                    
            }
            else
            {
               hwc.ch = TMR_CH_CUREI_PWM;
               rt_device_control(dev, HWTIMER_CTRL_STOP, &hwc);                          
                
            }
            if(0 < lc->lreg.btn.button_zl2)
            {
                hwc.ch = TMR_CH_CUREII_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_GET_AUTORELOAD, &hwc); 
                if (err != RT_EOK)
                {
                    rt_kprintf("Get the timer reload Fail\n");
                    return err;
                }
                /* set duty cycles */
                uint16_t tval = hwc.value;     
                hwc.value = tval/5 + (lc->lreg.btn.button_zl2)*(tval - tval/5)/23;
                hwc.ch = TMR_CH_CUREII_PWM;
                err = rt_device_control(dev, HWTIMER_CTRL_SET_DUTY_CYCLE, &hwc);
                if (err != RT_EOK)
                {
                    rt_kprintf("Set ch  = %d pwm Fail\n", hwc.ch);
                    return err;
                }
                else
                {
                    #ifdef USER_HWTIMER_APP_BUG_TEST
                    rt_kprintf("Set ch = %d pwm = %d ok,\n", hwc.ch, hwc.value);
                    #endif
                }
                if(1 == lc->lreg.btn.button_zl2)
                {                                                    
                    hwc.ch = TMR_CH_CUREII_PWM;
                    rt_device_control(dev, HWTIMER_CTRL_START, &hwc);                     
                }                    
            }
            else
            {
                hwc.ch = TMR_CH_CUREII_PWM;
                rt_device_control(dev, HWTIMER_CTRL_STOP, &hwc);                                 
            }
        }                                                                                                                                          
    }  
    return err;
}
#endif
static rt_err_t lwc_cure_timer4_output(rt_device_t dev, lwc_cure_t *lc)
{
    rt_err_t err = RT_EOK;
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(lc != RT_NULL);
    //rt_device_hwtimer_t *timer = (rt_device_hwtimer_t *)dev;   
    rt_hwtimer_chval_t hwc; 
    rt_hwtimer_tmrval_t hwt; 
    
    if(LWC_ACTIVED == lc->lway[FUNCTION].status)
    {
        if(1 == lc->lreg.tval.tmr_lock)
        { 
            if(LWC_INACTIVE == lc->lcf[FUNCTION].cure_out_actived)
            {                
                lc->lcf[FUNCTION].cure_out_actived = LWC_ACTIVED;
            }   
            /* get the reload of the timer */
            if(0 < lc->lreg.btn.button_zl1)
            {                               
                if(1 == lc->lreg.btn.button_zl1)
                {
                    rt_device_t dev_hwtimer6 = RT_NULL;                                                          
                    hwc.ch = TMR_CH_CUREI_FREQ;                
                    rt_device_control(dev, HWTIMER_CTRL_START, &hwc);
                    if(LWC_BASE_TIMER_READY == lct.lreg.btn.flag_base_timer_status)
                    {
                        lct.lreg.btn.flag_base_timer_status = LWC_BASE_TIMER_RUNNING;
                        if((dev_hwtimer6 = rt_device_find(TIMER6)) == RT_NULL)
                        {
                            rt_kprintf("No Device: %s\n", TIMER6);
                            while(1);
                        }
                        rt_device_hwtimer_t *timer = (rt_device_hwtimer_t *)dev_hwtimer6;                                      
                        if(0 == timer->channel_lock[TMR_CH_BASE])
                        {                       
                            hwt.sec = 1;  
                            hwt.usec = 0;                            
                            hwc.ch = TMR_CH_BASE;                          
                            if (rt_device_write(dev_hwtimer6, hwc.ch, &hwt, sizeof(hwt)) != sizeof(hwt))
                            {
                                rt_kprintf("SetTime Fail\n");
                                while(1);
                            }
                            else
                            {
                                #ifdef USER_HWTIMER_APP_BUG_TEST
                                rt_kprintf("SetTime: Sec %d, Usec %d\n", hwt.sec, hwt.usec); 
                                #endif
                            }
                            
                        }
                    }
                }                    
            }
            else
            {
               hwc.ch = TMR_CH_CUREI_FREQ;  
               rt_device_control(dev, HWTIMER_CTRL_STOP, &hwc);                
                
            }
            if(0 < lc->lreg.btn.button_zl2)
            {               
                if(1 == lc->lreg.btn.button_zl2)
                {
                    rt_device_t dev_hwtimer6 = RT_NULL;                                                           
                    hwc.ch = TMR_CH_CUREII_FREQ;
                    rt_device_control(dev, HWTIMER_CTRL_START, &hwc);  
                    if(LWC_BASE_TIMER_READY == lct.lreg.btn.flag_base_timer_status)
                    {
                        lct.lreg.btn.flag_base_timer_status = LWC_BASE_TIMER_RUNNING;
                        if((dev_hwtimer6 = rt_device_find(TIMER6)) == RT_NULL)
                        {
                            rt_kprintf("No Device: %s\n", TIMER6);
                            while(1);
                        }
                        rt_device_hwtimer_t *timer = (rt_device_hwtimer_t *)dev_hwtimer6;  
                        if(0 == timer->channel_lock[TMR_CH_BASE])
                        {                   
                            hwt.sec = 1;  
                            hwt.usec = 0;
                            hwc.ch = TMR_CH_BASE;                                              
                            if (rt_device_write(dev_hwtimer6, hwc.ch, &hwt, sizeof(hwt)) != sizeof(hwt))
                            {
                                rt_kprintf("SetTime Fail\n");
                                while(1);
                            }
                            else
                            {
                                #ifdef USER_HWTIMER_APP_BUG_TEST
                                rt_kprintf("SetTime: Sec %d, Usec %d\n", hwt.sec, hwt.usec);   
                                #endif
                            }
                        }
                    }
                }                    
            }
            else
            {                
                hwc.ch = TMR_CH_CUREII_FREQ;  
                rt_device_control(dev, HWTIMER_CTRL_STOP, &hwc);    
                
            }
        }                                                                                                                                          
    }  
    return err;
}

//extern uint16_t adc[ADC_NUMOFCHANNEL];
//extern uint32_t ADC_ConvertedValue[ADC_NUMOFCHANNEL];
static rt_err_t lwc_cure_ion_output(lwc_cure_t *lc)
{    
    rt_err_t err = RT_EOK;
    uint8_t flag_beep_warning;
    
    if(LWC_ACTIVED == lc->lway[IONICE_CURE].status)
    {
        if(1 == lc->lreg.tval.tmr_lock)
        { 
            if(LWC_INACTIVE == lc->lcf[IONICE_CURE].cure_out_actived)
            {                
                lc->lcf[IONICE_CURE].cure_out_actived = LWC_ACTIVED;
                rt_timer_start(&timerions);                    
            }            
            if((1 == lc->lreg.btn.button_lzlf)&&(PIN_LOW == rt_pin_read(PB15_IONTHERAPY_DECT)))/* low turn on  control 1 2 */
            {
                                                   
                rt_pin_write(PB12_IONTHERAPY_PWR, PIN_HIGH);
                rt_pin_write(PB13_IONTHERAPY_CRL1, PIN_HIGH);
                rt_pin_write(PB14_IONTHERAPY_CRL2, PIN_HIGH);
                if(600 > ion_count)
                {
                    rt_pin_write(PB5_IONTHERAPY_RLY, PIN_LOW);
                }
                else
                {
                    rt_pin_write(PB5_IONTHERAPY_RLY, PIN_HIGH);
                }
                rt_pin_write(PD2_BEEP, PIN_LOW);   
                flag_beep_warning &= ~(1<<0);
            }
            else if((2 == lc->lreg.btn.button_lzlf)&&(PIN_LOW == rt_pin_read(PB15_IONTHERAPY_DECT))) /* middle turn on  control 2 */
            {
                                             
                rt_pin_write(PB12_IONTHERAPY_PWR, PIN_HIGH);
                rt_pin_write(PB13_IONTHERAPY_CRL1,PIN_LOW);
                rt_pin_write(PB14_IONTHERAPY_CRL2,PIN_HIGH);
                if(600 > ion_count)
                {
                    rt_pin_write(PB5_IONTHERAPY_RLY, PIN_LOW);
                }
                else
                {
                    rt_pin_write(PB5_IONTHERAPY_RLY, PIN_HIGH);
                }
                rt_pin_write(PD2_BEEP, PIN_LOW);
                flag_beep_warning &= ~(1<<0);
            }
            else if((3 == lc->lreg.btn.button_lzlf)&&(PIN_LOW == rt_pin_read(PB15_IONTHERAPY_DECT))) /* hight turn on  control 1 */
            {
                                                             
                rt_pin_write(PB12_IONTHERAPY_PWR, PIN_HIGH);
                rt_pin_write(PB13_IONTHERAPY_CRL1,PIN_HIGH);
                rt_pin_write(PB14_IONTHERAPY_CRL2,PIN_LOW);
                if(600 > ion_count)
                {
                    rt_pin_write(PB5_IONTHERAPY_RLY, PIN_LOW);
                }
                else
                {
                    rt_pin_write(PB5_IONTHERAPY_RLY, PIN_HIGH);
                } 
                rt_pin_write(PD2_BEEP, PIN_LOW);  
                flag_beep_warning &= ~(1<<0);
            }
            else
            {
                flag_beep_warning |= (1<<0);
            }
            
            /*
            电流计算公式 Vout = 2.5 + 0.185*Ip
            因需要串联二极管修正后 Vout = 2.0 + 0.185*Ip (曲线斜率 在 0.18,0.19之间)
            使用电子负载仪测试Vx = Vo + K * Ip,实验得出 Vo = 1.98,K = 0.148
            所以上面计算公式可修正为Vo = 1.98 + 0.148*Ip
            根据测试数据，
            Ip = 0,     Vout = 1.98V, ADC -> 2440
            Ip = 2.0A   Vout = 2.27V, ADC -> 2770
            Ip = 2.5A , Vout = 2.334V,ADC -> 2842
                       
            可用电流范围 0~2.5A > 2440 ~ 2842 -> 0~ 402 
            分12档显示
            检测电流 Id  , 显示强度 Dx　Id*12/402                             
            */
            //if((TMR_DELAY_400ms <= tmr_count)&&(TMR_DELAY_500ms >= tmr_count))
            //{
                if((adc_valid[0] - ADC_I_0A) > 0)
                {
                    lc->ion_force = (uint8_t)((adc_valid[0] - ADC_I_0A) * 12.0 / 402);           
                }
                else
                {
                    lc->ion_force = 0;
                }
            //}
            if(lc->ion_force > 12)
            {
                lc->ion_force = 12;
            } 
            else if(lc->ion_force >= 10)
            {
                lc->ion_force = 11;
            }
            else if(lc->ion_force <= 0)
            {
                lc->ion_force = 0;
            }
            //rt_event_send(&event, RT_EVENT_LWC_ION_FORCE_DISPLAY);
            
            if(adc_valid[0] > ADC_I_2A5 + 100)
            {
                lc->ion_force_overtimes++;               
            }
            else if(adc_valid[0] < ADC_I_2A0)
            {
                lc->ion_force_overtimes = 0;
            }
            
            if((flag_beep_warning&(1<<0))== (1<<0))
            {
                rt_pin_write(PB12_IONTHERAPY_PWR, PIN_LOW);
                rt_pin_write(PB13_IONTHERAPY_CRL1,PIN_LOW);
                rt_pin_write(PB14_IONTHERAPY_CRL2,PIN_LOW);
                //rt_pin_write(PB5_IONTHERAPY_RLY, PIN_LOW);
                //rt_timer_stop(&timerions);
                if((TMR_DELAY_100ms >= tmr_count)
                    ||((TMR_DELAY_200ms <= tmr_count)&&(TMR_DELAY_300ms >= tmr_count))
                    ||((TMR_DELAY_400ms <= tmr_count)&&(TMR_DELAY_500ms >= tmr_count))
                    )
                {
                     rt_pin_write(PD2_BEEP, PIN_HIGH);                
                }
                else
                {
                    rt_pin_write(PD2_BEEP, PIN_LOW);
                }
                
            }
            if(lc->ion_force_overtimes > 30)
            {
                if((TMR_DELAY_100ms >= tmr_count)
                    ||((TMR_DELAY_200ms <= tmr_count)&&(TMR_DELAY_300ms >= tmr_count))
                    ||((TMR_DELAY_400ms <= tmr_count)&&(TMR_DELAY_500ms >= tmr_count))
                    )
                {
                     rt_pin_write(PD2_BEEP, PIN_HIGH);                
                }
                else
                {
                    rt_pin_write(PD2_BEEP, PIN_LOW);
                }
            }               
            if(lc->ion_force_overtimes > 120)  
            {
               rt_event_send(&event, RT_EVENT_LWC_DEVICE_FORCE_CLOSE); 
            }                

        }              
    }    
    return err;        
}

#endif /* RT_USING_LIGHT_WAVE_CURER */
/*@}*/
