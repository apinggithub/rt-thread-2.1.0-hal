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
#include <drivers/hwtimer.h>
#include <drivers/lcdht1621b.h>
#include <drv_lcdht1621b.h>

#include "cvt.h"
#include "drv_led.h"
#include "drv_gpio.h"
#include "drv_hwbutton.h"

#include "light_wave_curer.h"
#ifdef RT_USING_LIGHT_WAVE_CURER
lwc_cure_t lct;

ALIGN(RT_ALIGN_SIZE)
rt_uint8_t lwc_button_stack[ 2048 ];
struct rt_thread lwc_button_thread;
static struct rt_timer timerdec;
struct rt_timer timertip;
struct rt_timer timermusic;
static struct rt_timer timer_shutdown;

struct rt_event event;/* 事件控制�/

static uint32_t tmr_init_val = 0;
static uint32_t tmr_count_dec = 0;
uint32_t tmr_count_tip = 0;
uint16_t  music_play_wait = 0;
uint8_t flag_force_voice_play = 0;
//extern uint32_t freq_music[6];
//extern uint32_t freq_music;

/*定时器超时函�/
static void timeout_dec(void* parameter)
{

	if((tmr_init_val - tmr_count_dec) > 0)
    {
        tmr_count_dec++;
        lct.lreg.tval.tmr_value = (tmr_init_val - tmr_count_dec)/60 + 1;
    }
    else
    {
        tmr_count_dec = 0;       
        rt_event_send(&event, RT_EVENT_LWC_TIMER_FINISH_CLOSE);        
    }
}
/*定时器超时函�/
static void timeout_tip(void* parameter)
{
    tmr_count_tip++;    
	if((9 < tmr_count_tip))
    {
         tmr_count_tip = 0;   
    }   
}
static void timeout_shutdown(void* parameter)
{
    //wait_op_count++;    
	//if((9 < tmr_count_tip))
    //{
    //     tmr_count_tip = 0;   
    //}  
    rt_event_send(&event, RT_EVENT_LWC_DEVICE_FORCE_CLOSE);
}
/*定时器超时函�/
//uint32_t icbuffer[HWTMR_IC_BUF_SIZE];
static void timeout_music(void *parameter)
{ 
    //uint32_t *pdata = icbuffer;
    rt_hwtimer_chfreq_t      hwq;
    rt_hwtimer_tmrval_t      hwt;
    rt_hwtimer_chval_t       hwc;
    rt_device_hwtimer_t *timer1 = (rt_device_hwtimer_t *)parameter;
    rt_device_t dev_hwtimer6 = RT_NULL;
    rt_device_t dev_hwtimer4 = RT_NULL;  
    
    music_play_wait++;
    if(music_play_wait > 160)//16s
    {              
        music_play_wait = 0;        
        rt_event_send(&event, RT_EVENT_LWC_TIMER_MUSIC_PLAY);   
                    
    }       
    /*rt_memset(pdata, 0, HWTMR_IC_BUF_SIZE*sizeof(uint32_t));        
    rt_memcpy(pdata, &timer1->capture_buffer[HWTIMER_CH1], HWTMR_IC_BUF_SIZE*sizeof(uint32_t));
    rt_memset(&timer1->capture_buffer[HWTIMER_CH1], 0, HWTMR_IC_BUF_SIZE*sizeof(uint32_t));        
    bubble32(pdata, HWTMR_IC_BUF_SIZE*sizeof(uint32_t), 0);        
    freq_music = pdata[HWTMR_IC_BUF_SIZE-1]/16; */  
    
    if((dev_hwtimer6 = rt_device_find(TIMER6)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER6);
        while(1);
    } 
    if((dev_hwtimer4 = rt_device_find(TIMER4)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER4);
        while(1);
    } 
    
    hwq.freq = timer1->overflow[HWTIMER_BASE]/4;
    timer1->overflow[HWTIMER_BASE] = 0;
    if((hwq.freq >= 1.0)&&(10 == lct.lreg.btn.button_gn))
    {
        rt_err_t err = rt_device_control(dev_hwtimer6, HWTIMER_CTRL_SET_FREQ, &hwq);        
        if (err != RT_EOK)
        {
            rt_kprintf("Set Freq[%d] = %fHz Fail\n", lff.fm_idx, hwq.freq/2);
            //while(1);
        }
        else   
        {
            #ifdef USER_HWTIMER_APP_BUG_TEST
            rt_kprintf("Set Freq = %fHz ok\n", hwq.freq/2);
            #endif
        }         
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
    else
    {
        
        hwq.ch = TMR_CH_BASE;
        rt_device_control(dev_hwtimer6, HWTIMER_CTRL_STOP, &hwq); 
        hwq.ch = TMR_CH_CUREI_FREQ;
        rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP, &hwq);
        hwq.ch = TMR_CH_CUREII_FREQ;
        rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP, &hwq);  
    }
       
}

void lwc_button_thread_entry(void* parameter)
{
  
    rt_uint8_t val,vcno;
    rt_device_t dev_button = RT_NULL;
    rt_device_t dev_xtp = RT_NULL;
    //rt_uint32_t mb_msg[4];       
    uint8_t flag_tmrval_start = 0;      
    //uint16_t wait_op_count = 0;    
    uint16_t music_play_wait = 0;
    uint8_t flag_voice_close = 0;
    uint8_t flag_voice_tip_enable = 0;
        
    rt_uint32_t recv_event;
    
    rt_pin_mode(PC13_SPEAKER_CTRL, PIN_MODE_OUTPUT);
    rt_pin_write(PC13_SPEAKER_CTRL, PIN_HIGH);
    
    rt_event_init(&event, "event", RT_IPC_FLAG_FIFO);
    
    if ((dev_button = rt_device_find(BUTTON)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", BUTTON);
        while(1);
    }
   
    if (rt_device_open(dev_button, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", BUTTON);
        while(1);
    }
    
     /* 打开 XTP 设备 播放提示�*/        
    if ((dev_xtp = rt_device_find(XTP)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", XTP);
        while(1);
    }  
    if (rt_device_open(dev_xtp, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", XTP);
        while(1);
    }
    
    /* 初始化定时器 */
	rt_timer_init(&timerdec, "timerdec", /* 定时器名为timerdec */
	timeout_dec, /* 超时函数回调处理 */
	RT_NULL, /* 超时函数入口参数*/
	1000, /* 定时长度,OS 以Tick为单��000个OS Tick 产生一次超时处�*/
	RT_TIMER_FLAG_PERIODIC); /* 周期性定�*/
    
    rt_device_t dev_hwtimer1 = RT_NULL;    
    rt_device_hwtimer_t *timer1;
    if ((dev_hwtimer1 = rt_device_find(TIMER1)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER1);
        while(1);
    } 
    timer1 = (rt_device_hwtimer_t *)dev_hwtimer1;  
	rt_timer_init(&timermusic, "timermusic", /* 定时器名为timermusic */
	timeout_music, /* 超时函数回调处理 */
	timer1, /* 超时函数入口参数*/
	100, /* 定时长度,OS 以Tick为单��00个OS Tick 产生一次超时处�*/
	RT_TIMER_FLAG_PERIODIC); /* 周期性定�*/

    /* 初始化定时器 */
	rt_timer_init(&timertip, "timertip", /* 定时器名为timerdec */
	timeout_tip, /* 超时函数回调处理 */
	RT_NULL, /* 超时函数入口参数*/
	100, /* 定时长度,OS 以Tick为单��00个OS Tick 产生一次超时处�*/
	RT_TIMER_FLAG_PERIODIC); /* 周期性定�*/
    
    /* 初始化定时器 */
	rt_timer_init(&timer_shutdown, "timer_shutdown", /* 定时器名为timer_shutdown */
	timeout_shutdown, /* 超时函数回调处理 */
	RT_NULL, /* 超时函数入口参数*/
	1000*60, /* 定时长度,OS 以Tick为单��000个OS Tick (1s)产生一次超时处�*/
	RT_TIMER_FLAG_PERIODIC); /* 周期性定�*/
    
    while (1)
    {
        /*if((0 != lct.lreg.btn.button_dyds)&&(1 == flag_wait_event_start))
        {
            if(300 > wait_op_count ) // add once 0.02s
            {
                wait_op_count++;
            }
            else
            { 
               
                wait_op_count = 0;
                rt_event_send(&event, RT_EVENT_LWC_DEVICE_FORCE_CLOSE);
            }
        }*/
        
        if((1 == lct.lreg.tval.tmr_lock)&&(0 == flag_tmrval_start))           
        {
                      
            rt_timer_stop(&timerdec);
            if(1 <= lct.lreg.btn.button_lzlf)
            {                 
                lct.lreg.tval.tmr_value = 30;                                                                         
            }
            
            tmr_init_val = lct.lreg.tval.tmr_value*60;
            tmr_count_dec = 0;
            rt_timer_start(&timerdec);           
            flag_tmrval_start = 1;  
            
            if(1 <= lct.lreg.btn.button_lzlf)
            {                                        
                rt_timer_start(&timerions);                
                if(0 == flag_voice_close)
                {
                    vcno = 0x5A + 3;/* 30分钟 */
                    rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));                   
                    rt_thread_delay(1*RT_TICK_PER_SECOND );
                    vcno = 0x5A + 64;/* 超音波治疗输出强�弱档 */
                    rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));
                }                   
            }            
        }        
                    
        if (rt_event_recv(&event, (RT_EVENT_LWC_DEVICE_POWER_CLOSE
                                    |RT_EVENT_LWC_ION_TIME_UPDATE
                                    //|RT_EVENT_LWC_WAIT_TMR_START
                                    |RT_EVENT_LWC_TIMER_MUSIC_PLAY
                                    ),
                           RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                           RT_TICK_PER_SECOND/100, &recv_event) == RT_EOK)
        {
            switch(recv_event)
            {
                //case RT_EVENT_LWC_WAIT_TMR_START:
                //{
                //    wait_op_count = 0;
                //    flag_wait_event_start = 1;
                //}
                //break;
                case RT_EVENT_LWC_DEVICE_POWER_CLOSE:
                {
                    rt_timer_stop(&timertip);                        
                    flag_force_voice_play = 0;                    
                    if((0 == flag_voice_close)&&(1 == lct.lreg.tval.tmr_lock))
                    {
                        vcno = 0x5A + 31;/* 治疗结束，请关断电源，谢谢使用，祝您早日康复 */                   
                        rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));       
                    }
                    //flag_wait_event_start = 0;
                    flag_voice_close = 0;
                    rt_pin_write(PC13_SPEAKER_CTRL, PIN_HIGH);
                    rt_timer_stop(&timerdec);
                    rt_timer_stop(&timerions);
                    rt_timer_stop(&timermusic);
                    flag_tmrval_start = 0;
                    lct.lreg.btn.button_dyds = 0;
                    lct.lreg.tval.tmr_lock = 0;
                    lct.lreg.tval.tmr_value = 0;
                    lct.lcf[IONICE_CURE].cure_out_actived = LWC_INACTIVE;   
                    lct.lway[SET_TIMER].status = LWC_INACTIVE;
                }
                break;
                case RT_EVENT_LWC_ION_TIME_UPDATE:
                {
                    //flag_ion_timer_update = 1;
                    rt_timer_stop(&timerdec);
                    lct.lreg.tval.tmr_value = 30;
                    tmr_init_val = lct.lreg.tval.tmr_value*60;
                    tmr_count_dec = 0;
                    rt_timer_start(&timerdec);
                    
                    rt_timer_start(&timerions);                                                            
                    if(0 == flag_voice_close)
                    {
                        vcno = 0x5A + 3;/* 30分钟 */
                        rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));                   
                        rt_thread_delay(1*RT_TICK_PER_SECOND );
                        vcno = 0x5A + 64;/* 超音波治疗输出强�弱档 */
                        rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));
                    }                                                                          
                }
                break;
                case RT_EVENT_LWC_TIMER_MUSIC_PLAY:
                {
                    if(10 == lct.lreg.btn.button_gn)
                    {
                        //rt_thread_delay(2*RT_TICK_PER_SECOND );
                        vcno = 0x5A + 41;/* 音频治疗 26s */
                        rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));                         
                    }
                }
                default:
                break;
            }
        }       
        /* read the button value */
        rt_device_read(dev_button, 0, &val, sizeof(val));        
        //rt_kprintf("Read button = %x \n", val);       
        switch(val)
        {
           
            case BUTTON_DY_DS:/* 0x12 电源 定时 */
            {
                               
                if(0 == lct.lreg.tval.tmr_lock )
                {
                    lct.lreg.btn.button_dyds++;                
                    if( lct.lreg.btn.button_dyds > 4)                  
                    {
                         if(0 == lct.lreg.tval.tmr_lock )
                         {
                            lct.lreg.btn.button_dyds = 2;
                         }                     
                    }
                    if(1 == lct.lreg.btn.button_dyds) 
                    {
                        lct.lway[SET_TIMER].status = LWC_ACTIVED;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 62;/* 欢迎使用光波康复理疗仪，请定�*/
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));                             
                            //rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));                                
                        }
                        //rt_event_send(&event, RT_EVENT_LWC_BUTTON_UPDATE);
                    } 
                    if(2 == lct.lreg.btn.button_dyds) 
                    {
                        lct.lreg.tval.tmr_value = 10;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 1;/* 10分钟 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));  
                            rt_thread_delay(1*RT_TICK_PER_SECOND );
                            vcno = 0x5A + 4;/* 请设置治疗方�*/
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }
                        //rt_event_send(&event, RT_EVENT_LWC_BUTTON_UPDATE);
                    }
                    else if(3 == lct.lreg.btn.button_dyds) 
                    {
                        lct.lreg.tval.tmr_value = 20;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 2;/* 20分钟 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));
                            rt_thread_delay(1*RT_TICK_PER_SECOND );
                            vcno = 0x5A + 4;/* 请设置治疗方�*/
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }
                        //rt_event_send(&event, RT_EVENT_LWC_BUTTON_UPDATE);
                    }
                    else if(4 == lct.lreg.btn.button_dyds) 
                    {
                        lct.lreg.tval.tmr_value = 30;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 3;/* 30分钟 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));
                            rt_thread_delay(1*RT_TICK_PER_SECOND );
                            vcno = 0x5A + 4;/* 请设置治疗方�*/
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }
                        //rt_event_send(&event, RT_EVENT_LWC_BUTTON_UPDATE);
                    } 
                    //rt_event_send(&event, RT_EVENT_LWC_WAIT_TMR_START);
                    rt_timer_start(&timer_shutdown);
                }
                else
                {
                    rt_event_send(&event, RT_EVENT_LWC_DEVICE_FORCE_CLOSE);
                }
            }
            break;
            case BUTTON_JG:/* 0x22 激�*/
            {
                if((1 < lct.lreg.btn.button_dyds)&&(LWC_INACTIVE == lct.lcf[IONICE_CURE].cure_out_actived))
                {
                    //wait_op_count = 0;
                    //flag_wait_event_start = 0;
                    lct.lreg.tval.tmr_lock = 1;              
                    lct.lreg.btn.button_jg++;
                    if( lct.lreg.btn.button_jg > 3)
                    {    
                        if((0 == flag_voice_close)&&(10 != lct.lreg.btn.button_gn))
                        {
                            vcno = 0x5A + 28;/* 激光治疗关�*/                   
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));   
                        }                        
                        lct.lreg.btn.button_jg = 0;
                        lct.lway[LASER_CURE].status = LWC_INACTIVE;
                        rt_event_send(&event, RT_EVENT_LWC_LASER_CURE_CLOSE);
                        //rt_event_send(&event, RT_EVENT_LWC_WAIT_TMR_START);
                        rt_timer_start(&timer_shutdown);
                    } 
                    else if(1 == lct.lreg.btn.button_jg) 
                    {
                        lct.lway[LASER_CURE].status = LWC_ACTIVED;                           
                        if((0 == flag_voice_close)&&(10 != lct.lreg.btn.button_gn))
                        {
                            vcno = 0x5A + 49;/* 激光治疗输出强�弱档 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }
                        rt_timer_stop(&timer_shutdown);
                    } 
                    else if(2 == lct.lreg.btn.button_jg) 
                    {
                        if((0 == flag_voice_close)&&(10 != lct.lreg.btn.button_gn))
                        {
                            vcno = 0x5A + 50;/* 激光治疗输出强�中档 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));
                        }
                        rt_timer_stop(&timer_shutdown);
                    }
                    else if(3 == lct.lreg.btn.button_jg) 
                    {
                        if((0 == flag_voice_close)&&(10 != lct.lreg.btn.button_gn))
                        {
                            vcno = 0x5A + 51;/* 激光治疗输出强�强档 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        } 
                        rt_timer_stop(&timer_shutdown);
                    }
                    rt_event_send(&event, RT_EVENT_LWC_RLJG_BUTTON_UPDATE);
                }                
            }
            break;
            case BUTTON_RL:/* 0x11 热疗 */
            {
                if((1 < lct.lreg.btn.button_dyds)&&(LWC_INACTIVE == lct.lcf[IONICE_CURE].cure_out_actived))
                {
                    //wait_op_count = 0;
                    //flag_wait_event_start = 0;
                    lct.lreg.tval.tmr_lock = 1;         
                    lct.lreg.btn.button_rl++;
                    if( lct.lreg.btn.button_rl > 3)
                    {
                        if((0 == flag_voice_close)&&(10 != lct.lreg.btn.button_gn))
                        {
                            vcno = 0x5A + 8;/* 热疗停止 */                   
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));     
                        }                        
                        lct.lreg.btn.button_rl = 0;
                        lct.lway[HEAT_CURE].status = LWC_INACTIVE;
                        rt_event_send(&event, RT_EVENT_LWC_HEAT_CURE_CLOSE);
                        //rt_event_send(&event, RT_EVENT_LWC_WAIT_TMR_START);
                        rt_timer_start(&timer_shutdown);
                    } 
                    else if(1 == lct.lreg.btn.button_rl) 
                    {                       
                        if((0 == flag_voice_close)&&(10 != lct.lreg.btn.button_gn))
                        {
                            vcno = 0x5A + 5;/* 低温热疗 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));                              
                        }
                        lct.lway[HEAT_CURE].status = LWC_ACTIVED;  
                        rt_timer_stop(&timer_shutdown);
                    } 
                    else if(2 == lct.lreg.btn.button_rl) 
                    {
                        if((0 == flag_voice_close)&&(10 != lct.lreg.btn.button_gn))
                        {
                            vcno = 0x5A + 6;/* 中温热疗 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));
                        } 
                        rt_timer_stop(&timer_shutdown);
                    }
                    else if(3 == lct.lreg.btn.button_rl) 
                    {
                        if((0 == flag_voice_close)&&(10 != lct.lreg.btn.button_gn))
                        {
                            vcno = 0x5A + 7;/* 高温热疗 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }
                        rt_timer_stop(&timer_shutdown);
                    }
                    rt_event_send(&event, RT_EVENT_LWC_RLJG_BUTTON_UPDATE);
                }                
            }
            break;            
            case BUTTON_LZLF:/* 0x28 离子治疗 */
            {
                if(1 <= lct.lreg.btn.button_dyds)
                {
                    //wait_op_count = 0;
                    //flag_wait_event_start = 0;                    
                    lct.lreg.btn.button_lzlf++;
                    if( lct.lreg.btn.button_lzlf > 3)
                    { 
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 67;/* 超音波治疗关�*/                   
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }                            
                        lct.lreg.btn.button_lzlf = 0;
                        lct.ion_force = 0;
                        rt_event_send(&event, RT_EVENT_LWC_ION_CURE_CLOSE);
                        lct.lcf[IONICE_CURE].cure_out_actived = LWC_INACTIVE;
                        lct.lway[IONICE_CURE].status = LWC_INACTIVE;
                        //rt_event_send(&event, RT_EVENT_LWC_WAIT_TMR_START);
                        rt_timer_start(&timer_shutdown);
                        
                    } 
                    else if(1 == lct.lreg.btn.button_lzlf) 
                    {
                        if((1 == lct.lreg.tval.tmr_lock)&&(LWC_INACTIVE == lct.lcf[IONICE_CURE].cure_out_actived)) 
                        {
                            rt_event_send(&event, RT_EVENT_LWC_ION_FUNC_START);    
                        }
                        else
                        {
                            lct.lreg.tval.tmr_lock = 1;
                            lct.lcf[IONICE_CURE].cure_out_actived = LWC_ACTIVED; 
                        }
                        lct.lway[IONICE_CURE].status = LWC_ACTIVED;     
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 64;/* 超音波治疗输出强�弱档 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));
                        } 
                        rt_timer_stop(&timer_shutdown);
                        
                    } 
                    else if(2 == lct.lreg.btn.button_lzlf) 
                    {
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 65;/* 超音波治疗输出强�中档 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));
                        }
                        rt_timer_stop(&timer_shutdown);
                    }
                    else if(3 == lct.lreg.btn.button_lzlf) 
                    {
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 66;/* 超音波治疗输出强�强档 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }
                        rt_timer_stop(&timer_shutdown);
                    } 
                }
            }
            break;
            case BUTTON_GN:/* 0x32 功能 */
            {
                if((1 < lct.lreg.btn.button_dyds)&&(LWC_INACTIVE == lct.lcf[IONICE_CURE].cure_out_actived))
                {
                    //wait_op_count = 0;        
                    lct.lreg.btn.button_gn++;                    
                    if( lct.lreg.btn.button_gn > 10)
                    {
                        lct.lreg.btn.button_gn = 1;
                    } 
                    if(1 == lct.lreg.btn.button_gn) /* 1 全功�*/
                    { 
                        lct.lreg.btn.button_zl1 = 0;
                        lct.lreg.btn.button_zl2 = 0;
                        lct.lway[FUNCTION].status = LWC_ACTIVED; 
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 9; /* 全功�*/
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));   
                        }                                                
                    } 
                    else if(2 == lct.lreg.btn.button_gn) /* 2 中频治疗 */
                    {
                        lct.lreg.btn.button_zl1 = 0;
                        lct.lreg.btn.button_zl2 = 0;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 10;/* 中频治疗 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));  
                        }                                               
                    }
                    else if(3 == lct.lreg.btn.button_gn) /* 3 针灸 */
                    {
                        lct.lreg.btn.button_zl1 = 0;
                        lct.lreg.btn.button_zl2 = 0;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 14;/* 针灸 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }                                                 
                    } 
                    else if(4 == lct.lreg.btn.button_gn) /* 4 拍打 */
                    {
                        lct.lreg.btn.button_zl1 = 0;
                        lct.lreg.btn.button_zl2 = 0;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 15;/* 拍打 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));
                        }                                               
                    } 
                    else if(5 == lct.lreg.btn.button_gn) /* 5 推拿 */
                    {
                        lct.lreg.btn.button_zl1 = 0;
                        lct.lreg.btn.button_zl2 = 0;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 12;/* 推拿 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno));
                        }                                                
                    } 
                    else if(6 == lct.lreg.btn.button_gn) /* 6 按摩 */
                    {
                        lct.lreg.btn.button_zl1 = 0;
                        lct.lreg.btn.button_zl2 = 0;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 17;/* 按摩 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }                                              
                    } 
                    else if(7 == lct.lreg.btn.button_gn)/* 7 拔罐 */ 
                    {
                        lct.lreg.btn.button_zl1 = 0;
                        lct.lreg.btn.button_zl2 = 0;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 13;/* 拔罐 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }                                                
                    }
                    else if(8 == lct.lreg.btn.button_gn) /*8 足疗 */
                    {
                        lct.lreg.btn.button_zl1 = 0;
                        lct.lreg.btn.button_zl2 = 0;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 42;/* 足疗 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }                                               
                    }
                    else if(9 == lct.lreg.btn.button_gn) /* 9 减肥 */
                    {
                        lct.lreg.btn.button_zl1 = 0;
                        lct.lreg.btn.button_zl2 = 0;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 43;/* 减肥 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }                                             
                    }
                    else if(10 == lct.lreg.btn.button_gn) /* 10 音频 */
                    {
                        lct.lreg.btn.button_zl1 = 0;
                        lct.lreg.btn.button_zl2 = 0;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 38;/* 音频 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }                        
                    } 
                    tmr_count_tip = 0;   
                    flag_voice_tip_enable = 1;
                    rt_event_send(&event, RT_EVENT_LWC_GNZL_BUTTON_UPDATE);                                                                                           
                }                
            }
            break;
            case BUTTON_SD:/* 0x24 锁定 */
            {
                if((1 < lct.lreg.btn.button_dyds)&&((1 <= lct.lreg.btn.button_zl1)
                    ||(1 <=lct.lreg.btn.button_zl2))&&(LWC_INACTIVE == lct.lcf[IONICE_CURE].cure_out_actived))
                {
                    lct.lreg.btn.button_sd++;
                    if(1 < lct.lreg.btn.button_sd)
                    {
                        lct.lreg.btn.button_sd = 0;
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 30;/* 循环 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }
                    }
                    else
                    {
                        if(0 == flag_voice_close)
                        {
                            vcno = 0x5A + 29;/* 锁定 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                        }
                    }
                }
            }
            break;            
            case BUTTON_JY:/* 0x31 静音 */
            {
                if(1 <= lct.lreg.btn.button_dyds) 
                {
                    lct.lreg.btn.button_jy++;
                       
                    if(1 < lct.lreg.btn.button_jy)
                    {
                        flag_voice_close = 0;                        
                        lct.lreg.btn.button_jy = 0;
                        rt_pin_write(PC13_SPEAKER_CTRL, PIN_HIGH);
                        if((10 != lct.lreg.btn.button_gn))
                        { 
                            vcno = 0x5A + 39;/* 语音功能开�*/
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                            rt_thread_delay(RT_TICK_PER_SECOND);
                        }                        
                    }
                    else
                    {
                        flag_voice_close = 1;                          
                        if((10 != lct.lreg.btn.button_gn))
                        { 
                            vcno = 0x5A + 40;/* 语音功能关闭 */
                            rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                            rt_thread_delay(2*RT_TICK_PER_SECOND);
                        }                        
                        rt_pin_write(PC13_SPEAKER_CTRL, PIN_LOW);
                    }
                }                               
            }
            break;
            case BUTTON_ZL1_INC:/* 0x14 治疗1+ */
            { 
                if((1 < lct.lreg.btn.button_dyds)&&(LWC_INACTIVE == lct.lcf[IONICE_CURE].cure_out_actived))
                {
                    //wait_op_count = 0;
                    //flag_wait_event_start = 0;   
                    if(LWC_ACTIVED == lct.lway[FUNCTION].status)
                    {  
                        lct.lreg.tval.tmr_lock = 1;      
                        if(23 > lct.lreg.btn.button_zl1)
                        {
                            lct.lreg.btn.button_zl1++;
                            
                            if((0 == flag_voice_close)&&(10 != lct.lreg.btn.button_gn))
                            {
                                vcno = 0x5A + 19;/* 治疗输出强度1 增加 */
                                rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                            }
                            rt_timer_stop(&timer_shutdown);
                        }
                        
                        if(1 == lct.lreg.btn.button_zl1)
                        {
                                                        
                            if(LWC_BASE_TIMER_RUNNING!= lct.lreg.btn.flag_base_timer_status)
                            {
                                lct.lreg.btn.flag_base_timer_status = LWC_BASE_TIMER_READY;
                            }
                        }
                        
                        rt_event_send(&event, RT_EVENT_LWC_GNZL_BUTTON_UPDATE);
                    }
                }
            }
            break;
            case BUTTON_ZL2_INC:/* 0x18 治疗2+ */
            {
                  
                if((1 < lct.lreg.btn.button_dyds)&&(LWC_INACTIVE == lct.lcf[IONICE_CURE].cure_out_actived))
                {
                    //wait_op_count = 0;
                    //flag_wait_event_start = 0;   
                    if(LWC_ACTIVED == lct.lway[FUNCTION].status) 
                    {
                        lct.lreg.tval.tmr_lock = 1;    
                        if(23 > lct.lreg.btn.button_zl2)
                        {
                            lct.lreg.btn.button_zl2++;
                            if((0 == flag_voice_close)&&(10 != lct.lreg.btn.button_gn))
                            {
                                vcno = 0x5A + 21;/* 治疗输出强度2 增加 */
                                rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                            }
                            rt_timer_stop(&timer_shutdown);
                        }
                        
                        if(1 == lct.lreg.btn.button_zl2)
                        {
                            
                            if(LWC_BASE_TIMER_RUNNING!= lct.lreg.btn.flag_base_timer_status)
                            {
                                lct.lreg.btn.flag_base_timer_status = LWC_BASE_TIMER_READY;
                            }
                        }
                        
                        rt_event_send(&event, RT_EVENT_LWC_GNZL_BUTTON_UPDATE);
                    }
                }
            }
            break;                    
            case BUTTON_ZL1_DEC:/* 0x34 治疗1- */
            {
                if((1 < lct.lreg.btn.button_dyds)&&(LWC_INACTIVE == lct.lcf[IONICE_CURE].cure_out_actived))
                {
                    //wait_op_count = 0;
                    //flag_wait_event_start = 0;    
                    if(LWC_ACTIVED == lct.lway[FUNCTION].status)
                    {
                        lct.lreg.tval.tmr_lock = 1;     
                        if(0 < lct.lreg.btn.button_zl1 )
                        {
                            lct.lreg.btn.button_zl1--;
                            if((0 == flag_voice_close)&&(10 != lct.lreg.btn.button_gn))
                            {
                                vcno = 0x5A + 20;/* 治疗输出强度1 减少 */
                                rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                            }
                            rt_timer_stop(&timer_shutdown);
                        }                        
                        rt_event_send(&event, RT_EVENT_LWC_GNZL_BUTTON_UPDATE);
                    }
                    if((0 == lct.lreg.btn.button_zl1)&&(0 == lct.lreg.btn.button_zl2))
                    {
                        //rt_event_send(&event, RT_EVENT_LWC_WAIT_TMR_START);
                        rt_timer_start(&timer_shutdown);
                    }
                    if(0 == lct.lreg.btn.button_zl1)
                    {
                       flag_voice_tip_enable = 0;
                    }
                }
            }
            break;
            case BUTTON_ZL2_DEC:/* 0x38 治疗2- */
            {
                if((1 < lct.lreg.btn.button_dyds)&&(LWC_INACTIVE == lct.lcf[IONICE_CURE].cure_out_actived))
                {
                    //wait_op_count = 0;
                    //flag_wait_event_start = 0;   
                    if(LWC_ACTIVED == lct.lway[FUNCTION].status) 
                    {
                        lct.lreg.tval.tmr_lock = 1;      
                        if(0 < lct.lreg.btn.button_zl2 )
                        {
                            lct.lreg.btn.button_zl2--;
                            if((0 == flag_voice_close)&&(10 != lct.lreg.btn.button_gn))
                            {
                                vcno = 0x5A + 22;/* 治疗输出强度2 减少 */
                                rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                            }
                            rt_timer_stop(&timer_shutdown);
                        }                        
                        rt_event_send(&event, RT_EVENT_LWC_GNZL_BUTTON_UPDATE);
                    }
                    if((0 == lct.lreg.btn.button_zl1)&&(0 == lct.lreg.btn.button_zl2))
                    {
                        //rt_event_send(&event, RT_EVENT_LWC_WAIT_TMR_START);
                        rt_timer_start(&timer_shutdown);
                    }
                    if(0 == lct.lreg.btn.button_zl2)
                    {
                       flag_voice_tip_enable = 0;
                    }
                }
                
            }
            break;
            default:            
            break;
        }                  						
        rt_thread_delay( RT_TICK_PER_SECOND/10 );        
        if((1 == flag_force_voice_play)&&(10 != lct.lreg.btn.button_gn)&&(1 == flag_voice_tip_enable))
        {               
            if(0 == flag_voice_close)
            {
                
                if(9 <= tmr_count_tip)
                {
                    tmr_count_tip = 0;
                    flag_force_voice_play = 0;
                    rt_timer_stop(&timertip);
                    vcno = 0x5A + 18;/*  */
                    rt_device_write(dev_xtp, 0, &vcno, sizeof(vcno)); 
                    //rt_thread_delay(RT_TICK_PER_SECOND/2 );
                }
            }
            else
            {
                flag_force_voice_play = 0;
            }
        }
    }
}

#endif /* RT_USING_LIGHT_WAVE_CURER */
/*@}*/
