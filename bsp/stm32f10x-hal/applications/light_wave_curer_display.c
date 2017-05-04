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


#include "drv_led.h"
#include "drv_gpio.h"
#include "drv_hwbutton.h"

#include "light_wave_curer.h"
#ifdef RT_USING_LIGHT_WAVE_CURER
extern lwc_cure_t lct;
//static rt_err_t seglcd_dislay_time(rt_device_t dev, rt_lcd_ramdat_t *lcds, uint8_t tmr_value, uint8_t flag);
static rt_err_t seglcd_display(rt_device_t dev, lwc_cure_t *lc);

//static rt_sem_t sem = RT_NULL;
rt_uint16_t tmr_count = 0;
static struct rt_timer timerblink;

ALIGN(RT_ALIGN_SIZE)
rt_uint8_t lwc_display_stack[2048];
struct rt_thread lwc_display_thread;


/*定时器超时函�/
static void timeout_blink(void* parameter)
{
	//rt_kprintf("periodic timer is timeout\n");	
	//rt_pin_write(19, led2sw);
	//led2sw = (~led2sw)&0x01;
	tmr_count++;
    if(tmr_count > 1000-1)
    {
        tmr_count = 0;
    }
}
void lwc_display_thread_entry(void* parameter)
{
    //rt_err_t result;
    //lwc_cure_display_t *rlwc;
    rt_device_t dev_slcd = RT_NULL;
    //rt_uint32_t recv_event;
    
    rt_memset(&lct.lreg.btn, 0, sizeof(lwc_button_t));
    rt_memset(&lct.lreg.tval, 0, sizeof(timer_val_t));
    rt_memset(&lct.lway, 0, 5*sizeof(lwc_cure_way_t));
        
    if ((dev_slcd = rt_device_find(LCD)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", LCD);
        while(1);
    }           
    if (rt_device_open(dev_slcd, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", LCD);
        while(1);
    }
    rt_device_control(dev_slcd, LCDHT_CMD_FULL_SREEN_OFF, RT_NULL);   
   
    /* 初始化定时器*/
	rt_timer_init(&timerblink, "timerblink", /* 定时器名为timerblink */
	timeout_blink, /* 超时函数回调处理 */
	RT_NULL, /* 超时函数入口参数*/
	1, /* 定时长度,OS 以Tick为单��个OS Tick 产生一次超时处�/
	RT_TIMER_FLAG_PERIODIC); /* 周期性定�/
	
	rt_timer_start(&timerblink);
    
    //rt_sem_init(sem, "sem_update", 0, RT_IPC_FLAG_FIFO);
    while (1)
    {
       
        if((0 == lct.lreg.power_ok)&&(1 == lct.lreg.btn.button_dyds))
        {
            lct.lreg.power_ok = 1; 
            for(rt_uint8_t i = 0; i < 20; i++)
            {
                if(18 == i)
                {
                   lct.lcdr[i].segno = 30; 
                }
                else if(19 == i)
                {
                   lct.lcdr[i].segno = 31; 
                }
                else
                {
                    lct.lcdr[i].segno = i;                   
                } 
                lct.lcdr[i].dat = 0;
            }  
            
            rt_device_control(dev_slcd, LCDHT_CMD_BKLON, RT_NULL);
            rt_device_control(dev_slcd, LCDHT_CMD_LCDON, RT_NULL);
            rt_device_control(dev_slcd, LCDHT_CMD_FULL_SREEN_ON, RT_NULL);
            rt_thread_delay( RT_TICK_PER_SECOND );
            rt_device_control(dev_slcd, LCDHT_CMD_FULL_SREEN_OFF, RT_NULL);            
            
        }
        else if(1 == lct.lreg.power_ok)
        {                             
            seglcd_display(dev_slcd, (lwc_cure_t *)&lct);
        }
        /*if (rt_event_recv(&event, (RT_EVENT_LWC_ION_FORCE_DISPLAY                                                                      
                                    ),
                           RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                           RT_TICK_PER_SECOND/100, &recv_event) == RT_EOK)
        {
            switch(recv_event)
            {
                                
                case RT_EVENT_LWC_ION_FORCE_DISPLAY:
                {
                      lwc_force_display(dev_slcd, (lwc_cure_t *)&lct);                                          
                }
                break;
                default:
                break;
            }
        }*/
        if(LWC_ACTIVED == lct.lcf[IONICE_CURE].cure_out_actived)
        {
            lwc_force_display(dev_slcd, (lwc_cure_t *)&lct);     
        }            
        rt_thread_delay( RT_TICK_PER_SECOND/10 ); /* sleep 0.01 second and switch to other thread */       
    }
}
/*
display the 0~9 in the segement lcd
dev ---> the segement device
time ---> the sent time value
flag ---> display on left or right
*/
static rt_err_t seglcd_dislay_time(rt_device_t dev, rt_lcd_ramdat_t *lcds, uint8_t tmr_value, uint8_t flag)
{
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(lcds != RT_NULL);
    
    switch( tmr_value )
    {
        
        case 0:
        {
            if(0 == flag)/* low segement */
            {

                lcds[13].dat = 0x0E;
                rt_device_write(dev, 0, &lcds[13], sizeof(lcds[13]));
                lcds[14].dat = 0x0B;
                rt_device_write(dev, 0, &lcds[14], sizeof(lcds[14]));
            }
            else /* hight segement */
            {
                lcds[15].dat = 0x0F;
                rt_device_write(dev, 0, &lcds[15], sizeof(lcds[15]));
                lcds[16].dat = 0x0B;
                rt_device_write(dev, 0, &lcds[16], sizeof(lcds[16]));
            }
        }
        break;  
        case 1:
        {
            if(0 == flag)/* low segement */
            {
                lcds[13].dat = 0x06;
                rt_device_write(dev, 0, &lcds[13], sizeof(lcds[13]));
                lcds[14].dat = 0x00;
                rt_device_write(dev, 0, &lcds[14], sizeof(lcds[14]));
            }
            else /* left segement */
            {
                lcds[15].dat = 0x07;
                rt_device_write(dev, 0, &lcds[15], sizeof(lcds[15]));
                lcds[16].dat = 0x00;
                rt_device_write(dev, 0, &lcds[16], sizeof(lcds[16]));
            }
        }
        break; 
        case 2:
        {
            if(0 == flag)/* right segement */
            {
                lcds[13].dat = 0x0C;
                rt_device_write(dev, 0, &lcds[13], sizeof(lcds[13]));
                lcds[14].dat = 0x07;
                rt_device_write(dev, 0, &lcds[14], sizeof(lcds[14]));
            }
            else /* left segement */
            {
                lcds[15].dat = 0x0D;
                rt_device_write(dev, 0, &lcds[15], sizeof(lcds[15]));
                lcds[16].dat = 0x07;
                rt_device_write(dev, 0, &lcds[16], sizeof(lcds[16]));
            } 
        }            
        break;   
        case 3:
        {
            if(0 == flag)/* right segement */
            {
                lcds[13].dat = 0x0E;
                rt_device_write(dev, 0, &lcds[13], sizeof(lcds[13]));
                lcds[14].dat = 0x05;
                rt_device_write(dev, 0, &lcds[14], sizeof(lcds[14]));
            }
            else /* left segement */
            {
                lcds[15].dat = 0x0F;
                rt_device_write(dev, 0, &lcds[15], sizeof(lcds[15]));
                lcds[16].dat = 0x05;
                rt_device_write(dev, 0, &lcds[16], sizeof(lcds[16]));
            } 
        }           
        break;   
        case 4:
        {
            if(0 == flag)/* right segement */
            {
                lcds[13].dat = 0x06;
                rt_device_write(dev, 0, &lcds[13], sizeof(lcds[13]));
                lcds[14].dat = 0x0C;
                rt_device_write(dev, 0, &lcds[14], sizeof(lcds[14]));
            }
            else /* left segement */
            {
                lcds[15].dat = 0x07;
                rt_device_write(dev, 0, &lcds[15], sizeof(lcds[15]));
                lcds[16].dat = 0x0C;
                rt_device_write(dev, 0, &lcds[16], sizeof(lcds[16]));
            } 
        }           
        break; 
        case 5:
        {
            if(0 == flag)/* right segement */
            {
                lcds[13].dat = 0x0A;
                rt_device_write(dev, 0, &lcds[13], sizeof(lcds[13]));
                lcds[14].dat = 0x0D;
                rt_device_write(dev, 0, &lcds[14], sizeof(lcds[14]));
            }
            else /* left segement */
            {
                lcds[15].dat = 0x0B;
                rt_device_write(dev, 0, &lcds[15], sizeof(lcds[15]));
                lcds[16].dat = 0x0D;
                rt_device_write(dev, 0, &lcds[16], sizeof(lcds[16]));
            } 
        }           
        break; 
        case 6:
        {
            if(0 == flag)/* right segement */
            {
                lcds[13].dat = 0x0A;
                rt_device_write(dev, 0, &lcds[13], sizeof(lcds[13]));
                lcds[14].dat = 0x0F;
                rt_device_write(dev, 0, &lcds[14], sizeof(lcds[14]));
            }
            else /* left segement */
            {
                lcds[15].dat = 0x0B;
                rt_device_write(dev, 0, &lcds[15], sizeof(lcds[15]));
                lcds[16].dat = 0x0F;
                rt_device_write(dev, 0, &lcds[16], sizeof(lcds[16]));
            } 
        }           
        break; 
        case 7:
        {
            if(0 == flag)/* right segement */
            {
                lcds[13].dat = 0x0E;
                rt_device_write(dev, 0, &lcds[13], sizeof(lcds[13]));
                lcds[14].dat = 0x00;
                rt_device_write(dev, 0, &lcds[14], sizeof(lcds[14]));
            }
            else /* left segement */
            {
                lcds[15].dat = 0x0F;
                rt_device_write(dev, 0, &lcds[15], sizeof(lcds[15]));
                lcds[16].dat = 0x00;
                rt_device_write(dev, 0, &lcds[16], sizeof(lcds[16]));
            } 
        }           
        break; 
        case 8:
        {
            if(0 == flag)/* right segement */
            {
                lcds[13].dat = 0x0E;
                rt_device_write(dev, 0, &lcds[13], sizeof(lcds[13]));
                lcds[14].dat = 0x0F;
                rt_device_write(dev, 0, &lcds[14], sizeof(lcds[14]));
            }
            else /* left segement */
            {
                lcds[15].dat = 0x0F;
                rt_device_write(dev, 0, &lcds[15], sizeof(lcds[15]));
                lcds[16].dat = 0x0F;
                rt_device_write(dev, 0, &lcds[16], sizeof(lcds[16]));
            } 
        }           
        break; 
        case 9:
        {
            if(0 == flag)/* right segement */
            {
                lcds[13].dat = 0x0E;
                rt_device_write(dev, 0, &lcds[13], sizeof(lcds[13]));
                lcds[14].dat = 0x0D;
                rt_device_write(dev, 0, &lcds[14], sizeof(lcds[14]));
            }
            else /* left segement */
            {
                lcds[15].dat = 0x0F;
                rt_device_write(dev, 0, &lcds[15], sizeof(lcds[15]));
                lcds[16].dat = 0x0D;
                rt_device_write(dev, 0, &lcds[16], sizeof(lcds[16]));
            } 
        }           
        break; 
        default:
        break;
    }
    return RT_EOK;
}

/*
active the segement display function
dev ---> the segement lcd device
mode ---> cure mode
*/
rt_err_t seglcd_display(rt_device_t dev, lwc_cure_t *lc)
{

    rt_err_t err = RT_EOK;
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(lc != RT_NULL);
    
    if(0 == lc->lreg.btn.button_jy)/* 非静�*/
    {
        lc->lcdr[12].dat |= 0x01;
        rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
        lc->lcdr[11].dat &= ~0x04;
        rt_device_write(dev, 0, &lc->lcdr[11], sizeof(lc->lcdr[11]));
    }
    else /* 静音 */
    {
        lc->lcdr[12].dat &= ~0x01;
        rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
        lc->lcdr[11].dat |= 0x04;
        rt_device_write(dev, 0, &lc->lcdr[11], sizeof(lc->lcdr[11]));
    }
       
    if(LWC_ACTIVED == lc->lway[SET_TIMER].status)/* display on */
    {
        lc->lcdr[11].dat |= 0x08;/* wecome to use...*/
        rt_device_write(dev, 0, &lc->lcdr[11], sizeof(lc->lcdr[11]));
        lc->lreg.tval.lcd_lbyte =  lc->lreg.tval.tmr_value%10;
        seglcd_dislay_time( dev, lc->lcdr , lc->lreg.tval.lcd_lbyte, 0);
        lc->lreg.tval.lcd_hbyte =  lc->lreg.tval.tmr_value/10;
        seglcd_dislay_time( dev, lc->lcdr , lc->lreg.tval.lcd_hbyte, 1);
        
        /*display full function*/
        lc->lcdr[10].dat |= 0x0F;
        rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10])); 
        lc->lcdr[9].dat |= 0x07;                
        rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9])); 
        lc->lcdr[8].dat |= 0x07;                
        rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
        
        /*display heat cure*/
        lc->lcdr[6].dat |= 0x0F;  
        rt_device_write(dev, 0, &lc->lcdr[6], sizeof(lc->lcdr[6])); 
        lc->lcdr[0].dat |= 0x0F;
        rt_device_write(dev, 0, &lc->lcdr[0], sizeof(lc->lcdr[0]));   
        lc->lcdr[1].dat |= 0x0F;
        rt_device_write(dev, 0, &lc->lcdr[1], sizeof(lc->lcdr[1]));   
        lc->lcdr[2].dat |= 0x0F;
        rt_device_write(dev, 0, &lc->lcdr[2], sizeof(lc->lcdr[2]));    
        
    }
    else /* display off */
    {                              
                
        lc->lreg.power_ok = 0;
        lc->lreg.tval.tmr_value = 0;
        err = rt_device_control(dev, LCDHT_CMD_FULL_SREEN_OFF, RT_NULL);  
        err = rt_device_control(dev, LCDHT_CMD_LCDOFF, RT_NULL);        
        err = rt_device_control(dev, LCDHT_CMD_BKLOFF, RT_NULL);
        rt_memset(&lc->lreg.btn, 0, sizeof(lwc_button_t));
        rt_memset(&lc->lreg.tval, 0, sizeof(timer_val_t));
        rt_memset(&lc->lway, 0, 5*sizeof(lwc_cure_way_t));
    }           

    if(LWC_ACTIVED == lc->lway[LASER_CURE].status)/* display on */
    {
        if(TMR_DELAY_500ms > tmr_count)
        {
            lc->lcdr[8].dat |= 0x08;
            rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));                    
        }               
        else
        {    
            lc->lcdr[8].dat &= ~0x08;
            rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));
        }
    }
    else /* display off */
    {
        lc->lcdr[8].dat &= ~0x08;
        rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));
    }           

    if(LWC_ACTIVED == lc->lway[HEAT_CURE].status)/* display on */
    {               
        
        if(1 == lc->lreg.btn.button_rl) /* low  temperature */
        {
            lc->lcdr[6].dat |= 0x08; 
            lc->lcdr[6].dat |= 0x04;
            lc->lcdr[6].dat |= 0x02;                    
            if(TMR_DELAY_500ms > tmr_count)
            {                        
                lc->lcdr[6].dat |= 0x01;                        
                rt_device_write(dev, 0, &lc->lcdr[6], sizeof(lc->lcdr[6]));   
            }
            else 
            {                                                                         
                lc->lcdr[6].dat &= ~0x01;
                rt_device_write(dev, 0, &lc->lcdr[6], sizeof(lc->lcdr[6]));
            }
        }
        else if(2 == lc->lreg.btn.button_rl) /* middle  temperature */
        {
            lc->lcdr[6].dat |= 0x08; 
            lc->lcdr[6].dat |= 0x02;
            lc->lcdr[6].dat |= 0x01;                                       
            if(TMR_DELAY_500ms > tmr_count)
            {
                lc->lcdr[6].dat |= 0x04;
                rt_device_write(dev, 0, &lc->lcdr[6], sizeof(lc->lcdr[6]));                                           
            }
            else
            {                        
                lc->lcdr[6].dat &= ~0x04;
                rt_device_write(dev, 0, &lc->lcdr[6], sizeof(lc->lcdr[6]));                       
            }
        }
        else if(3 == lc->lreg.btn.button_rl) /* hight temperature */
        {
            lc->lcdr[6].dat |= 0x08; 
            lc->lcdr[6].dat |= 0x04;
            lc->lcdr[6].dat |= 0x01;                                     
            if(TMR_DELAY_500ms > tmr_count)
            {
                lc->lcdr[6].dat |= 0x02;  
                rt_device_write(dev, 0, &lc->lcdr[6], sizeof(lc->lcdr[6]));                      
            }
            else
            {
                lc->lcdr[6].dat &= ~0x02;
                rt_device_write(dev, 0, &lc->lcdr[6], sizeof(lc->lcdr[6]));
            }
        }                               
        if(0 != lc->lreg.btn.button_rl)
        {
            /* Flicker  */
            if(TMR_DELAY_300ms > tmr_count)
            {
                lc->lcdr[0].dat |= 0x0F;                   
                rt_device_write(dev, 0, &lc->lcdr[0], sizeof(lc->lcdr[0])); 
                lc->lcdr[1].dat &= ~0x0F;                    
                rt_device_write(dev, 0, &lc->lcdr[1], sizeof(lc->lcdr[1])); 
                lc->lcdr[2].dat &= ~0x0F;                    
                rt_device_write(dev, 0, &lc->lcdr[2], sizeof(lc->lcdr[2])); 
            }
            else if(TMR_DELAY_600ms > tmr_count)
            {
                lc->lcdr[0].dat &= ~0x0F;                   
                rt_device_write(dev, 0, &lc->lcdr[0], sizeof(lc->lcdr[0])); 
                lc->lcdr[1].dat |= 0x0F;                    
                rt_device_write(dev, 0, &lc->lcdr[1], sizeof(lc->lcdr[1])); 
                lc->lcdr[2].dat &= ~0x0F;                    
                rt_device_write(dev, 0, &lc->lcdr[2], sizeof(lc->lcdr[2])); 
            }
            else
            {                
                lc->lcdr[0].dat &= ~0x0F;                   
                rt_device_write(dev, 0, &lc->lcdr[0], sizeof(lc->lcdr[0])); 
                lc->lcdr[1].dat &= ~0x0F;                    
                rt_device_write(dev, 0, &lc->lcdr[1], sizeof(lc->lcdr[1])); 
                lc->lcdr[2].dat |= 0x0F;                    
                rt_device_write(dev, 0, &lc->lcdr[2], sizeof(lc->lcdr[2])); 
            }
        }                 
    }
    else /* heat cure stop */
    {               
        lc->lcdr[6].dat |= 0x0F;  
        rt_device_write(dev, 0, &lc->lcdr[6], sizeof(lc->lcdr[6])); 
        lc->lcdr[0].dat |= 0x0F;
        rt_device_write(dev, 0, &lc->lcdr[0], sizeof(lc->lcdr[0]));   
        lc->lcdr[1].dat |= 0x0F;
        rt_device_write(dev, 0, &lc->lcdr[1], sizeof(lc->lcdr[1]));   
        lc->lcdr[2].dat |= 0x0F;
        rt_device_write(dev, 0, &lc->lcdr[2], sizeof(lc->lcdr[2]));  
    }              

    if(LWC_ACTIVED == lc->lway[IONICE_CURE].status)/* display on */
    {               

        if(1 == lc->lreg.btn.button_lzlf) /* low  force */
        {
            lc->lcdr[7].dat |= 0x02;
            rt_device_write(dev, 0, &lc->lcdr[7], sizeof(lc->lcdr[7])); 
        }
        else if(2 == lc->lreg.btn.button_lzlf) /* middle force*/
        {

            lc->lcdr[7].dat |= 0x02;
            lc->lcdr[7].dat |= 0x04;
            rt_device_write(dev, 0, &lc->lcdr[7], sizeof(lc->lcdr[7])); 
        }
        else if(3 == lc->lreg.btn.button_lzlf) /* hight force*/
        {

            lc->lcdr[7].dat |= 0x02;
            lc->lcdr[7].dat |= 0x04;
            lc->lcdr[7].dat |= 0x08;
            rt_device_write(dev, 0, &lc->lcdr[7], sizeof(lc->lcdr[7]));
        }               
        
    }
    else /* display off */
    {
        
        lc->lcdr[7].dat = 0x00;
        rt_device_write(dev, 0, &lc->lcdr[7], sizeof(lc->lcdr[7]));                
    } 

    if(LWC_ACTIVED == lc->lway[FUNCTION].status)/* display on */
    { 
        if((0 != lc->lreg.btn.button_zl1)||(0 != lc->lreg.btn.button_zl2))
        {
            if(0 == lc->lreg.btn.button_sd)/* 循环 */
            {
                lc->lcdr[9].dat &= ~0x08;
                rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));
                if(TMR_DELAY_300ms > tmr_count)
                {                                                                    
                    lc->lcdr[12].dat |= 0x02;
                    lc->lcdr[12].dat &= ~0x04;
                    lc->lcdr[12].dat &= ~0x08;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
                }
                else if(TMR_DELAY_600ms > tmr_count)
                {
                    lc->lcdr[12].dat &= ~0x02;
                    lc->lcdr[12].dat |= 0x04;
                    lc->lcdr[12].dat &= ~0x08;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
                }
                else
                { 
                    lc->lcdr[12].dat &= ~0x02;
                    lc->lcdr[12].dat &= ~0x04;
                    lc->lcdr[12].dat |= 0x08;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
                }                        
                
            }               
            else /* 锁定 */
            {
                lc->lcdr[9].dat |= 0x08;
                rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));
                lc->lcdr[12].dat &= ~0x0E;
                rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
            }
        }
                            
        switch(lc->lreg.btn.button_gn)
        {
            case FULL_FUNCTION: 
            { 
                if((0 == lc->lreg.btn.button_zl1)&&(0 == lc->lreg.btn.button_zl2))/*The label is normal on ,and the others is normal off */ 
                {
                    lc->lcdr[10].dat |= 0x08;  
                    lc->lcdr[10].dat &= ~0x07; 
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                           
                    lc->lcdr[9].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9])); 
                    lc->lcdr[8].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));  
                    lc->lcdr[12].dat &= ~0x0E;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
                    
                }
                else /*The label is blinking ,and the others is normal on */  
                {
                    
                    lc->lcdr[10].dat |= 0x07;
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10])); 
                    lc->lcdr[9].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9])); 
                    lc->lcdr[8].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    if(TMR_DELAY_500ms > tmr_count)
                    {
                        lc->lcdr[10].dat |= 0x08;  
                        rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));
                    }
                    else
                    {    
                        lc->lcdr[10].dat &= ~0x08;                    
                        rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                            
                    }                                                                                      
                 }                                                
            }
            break;                    
            case MID_FREQUENCY:                  
            { 
                if((0 == lc->lreg.btn.button_zl1)&&(0 == lc->lreg.btn.button_zl2))/*The label is normal on ,and the others is normal off */ 
                {
                    lc->lcdr[10].dat |= 0x04;                            
                    lc->lcdr[10].dat &= ~0x0B;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                                     
                    lc->lcdr[9].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9])); 
                    lc->lcdr[8].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    lc->lcdr[12].dat &= ~0x0E;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
                    
                }
                else  /* The label is blinking ,and the others is normal on */ 
                {
                                                
                    lc->lcdr[10].dat |= 0x0B;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                                     
                    lc->lcdr[9].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9])); 
                    lc->lcdr[8].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    if(TMR_DELAY_500ms > tmr_count)
                    {
                        lc->lcdr[10].dat |= 0x04; 
                        rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));
                    }
                    else
                    {
                        lc->lcdr[10].dat &= ~0x04;                    
                        rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));
                    }                             
                } 
            }    
            break;    
            case ACUPUNCTURE_NEEDLE:/*针灸*/
            {
                if((0 == lc->lreg.btn.button_zl1)&&(0 == lc->lreg.btn.button_zl2))/*The label is normal on ,and the others is normal off */    
                {
                     
                    lc->lcdr[10].dat &= ~0x0F;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));  
                    lc->lcdr[9].dat |= 0x04;                            
                    lc->lcdr[9].dat &= ~0x03;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9])); 
                    lc->lcdr[8].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    lc->lcdr[12].dat &= ~0x0E;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
                    
                }
                else  /* The label is blinking ,and the others is normal on */ 
                {                           
                    
                    lc->lcdr[10].dat |= 0x0F;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                              
                    lc->lcdr[9].dat |= 0x03;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9])); 
                    lc->lcdr[8].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));
                    if(TMR_DELAY_500ms > tmr_count)
                    {
                        lc->lcdr[9].dat |= 0x04; 
                        rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));
                    }
                    else
                    {
                        lc->lcdr[9].dat &= ~0x04;                    
                        rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));
                    }                            
                }
            }
            break;        
            case PAT_CURE:/* 拍打 */
            {
                if((0 == lc->lreg.btn.button_zl1)&&(0 == lc->lreg.btn.button_zl2))/*The label is normal on ,and the others is normal off */  
                {
                     
                    lc->lcdr[10].dat &= ~0x0F;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                                  
                    lc->lcdr[9].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));   
                    lc->lcdr[8].dat |= 0x04;                            
                    lc->lcdr[8].dat &= ~0x03;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    lc->lcdr[12].dat &= ~0x0E;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
                    
                }
                else  /* The label is blinking ,and the others is normal on */ 
                {                           
                    
                    lc->lcdr[10].dat |= 0x0F;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                               
                    lc->lcdr[9].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));                             
                    lc->lcdr[8].dat |= 0x03;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    if(TMR_DELAY_500ms > tmr_count)
                    {
                        lc->lcdr[8].dat |= 0x04;
                        rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));
                    }
                    else
                    {
                        lc->lcdr[8].dat &= ~0x04;                    
                        rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));
                    }
                } 
            }                        
            break;        
            case NAPRAPATHY:/* 推拿 */
            {
                if((0 == lc->lreg.btn.button_zl1)&&(0 == lc->lreg.btn.button_zl2))/*The label is normal on ,and the others is normal off */      
                {
                    lc->lcdr[10].dat |= 0x02;                            
                    lc->lcdr[10].dat &= ~0x0D;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                                  
                    lc->lcdr[9].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));                            
                    lc->lcdr[8].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));
                    lc->lcdr[12].dat &= ~0x0E;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
                                               
                }
                else  /* The label is blinking ,and the others is normal on */ 
                {                           
                    
                    lc->lcdr[10].dat |= 0x0D;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                               
                    lc->lcdr[9].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));                            
                    lc->lcdr[8].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    if(TMR_DELAY_500ms > tmr_count)
                    {
                        lc->lcdr[10].dat |= 0x02;
                        rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));
                    }
                    else
                    {
                        lc->lcdr[10].dat &= ~0x02;                    
                        rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));
                    } 
                }
            }     
            break;
            case MASSOTHERAPY: /* 按摩 */
            {
                if((0 == lc->lreg.btn.button_zl1)&&(0 == lc->lreg.btn.button_zl2))/*The label is normal on ,and the others is normal off */    
                {
                                              
                    lc->lcdr[10].dat &= ~0x0F;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));  
                    lc->lcdr[9].dat |= 0x02;  
                    lc->lcdr[9].dat &= ~0x05;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));                            
                    lc->lcdr[8].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));
                    lc->lcdr[12].dat &= ~0x0E;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
                                               
                }
                else  /* The label is blinking ,and the others is normal on */ 
                {                           
                    
                    lc->lcdr[10].dat |= 0x0F;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                                                       
                    lc->lcdr[9].dat |= 0x05;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));                            
                    lc->lcdr[8].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    if(TMR_DELAY_500ms > tmr_count)
                    {    
                        lc->lcdr[9].dat |= 0x02;  
                        rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));
                    }
                    else
                    {
                        lc->lcdr[9].dat &= ~0x02;                    
                        rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));
                    } 
                }
                    
            }     
            break;
            case CUPPING_THERAPY:/* 拔罐 */
            {
                if((0 == lc->lreg.btn.button_zl1)&&(0 == lc->lreg.btn.button_zl2))/*The label is normal on ,and the others is normal off */ 
                {
                                                
                    lc->lcdr[10].dat &= ~0x0F;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                                                       
                    lc->lcdr[9].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9])); 
                    lc->lcdr[8].dat |= 0x02; 
                    lc->lcdr[8].dat &= ~0x05;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    lc->lcdr[12].dat &= ~0x0E;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
                                                
                }
                else  /* The label is blinking ,and the others is normal on */ 
                {                           
                    
                    lc->lcdr[10].dat |= 0x0F;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                                                          
                    lc->lcdr[9].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));                             
                    lc->lcdr[8].dat |= 0x05;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    if(TMR_DELAY_500ms > tmr_count)
                    {
                        lc->lcdr[8].dat |= 0x02;  
                        rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));
                    }
                    else
                    {
                        lc->lcdr[8].dat &= ~0x02;                    
                        rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));
                    }
                } 
                    
            } 
            break;
            case PEDICURE: /* 足疗 */
            {
                if((0 == lc->lreg.btn.button_zl1)&&(0 == lc->lreg.btn.button_zl2))/*The label is normal on ,and the others is normal off */     
                {
                    lc->lcdr[10].dat |= 0x01;                                                                
                    lc->lcdr[10].dat &= ~0x0E;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                                                       
                    lc->lcdr[9].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));                                                    
                    lc->lcdr[8].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    lc->lcdr[12].dat &= ~0x0E;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));

                }
                else  /* The label is blinking ,and the others is normal on */ 
                {
                                               
                    lc->lcdr[10].dat |= 0x0E;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                                                          
                    lc->lcdr[9].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));                                                      
                    lc->lcdr[8].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    if(TMR_DELAY_500ms > tmr_count)
                    {
                        lc->lcdr[10].dat |= 0x01;   
                        rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));
                    }
                    else
                    {
                        lc->lcdr[10].dat &= ~0x01;                    
                        rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));
                    } 
                }
                    
            } 
            break;
            case LOSE_WEIGHT: /* 减肥 */
            {
                if((0 == lc->lreg.btn.button_zl1)&&(0 == lc->lreg.btn.button_zl2))/*The label is normal on ,and the others is normal off */     
                {
                                              
                    lc->lcdr[10].dat &= ~0x0F;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10])); 
                    lc->lcdr[9].dat |= 0x01; 
                    lc->lcdr[9].dat &= ~0x06;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));                                                    
                    lc->lcdr[8].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    lc->lcdr[12].dat &= ~0x0E;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));

                }
                else  /* The label is blinking ,and the others is normal on */ 
                {                           
                    
                    lc->lcdr[10].dat |= 0x0F;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                                                                                       
                    lc->lcdr[9].dat |= 0x06;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));                                                      
                    lc->lcdr[8].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    if(TMR_DELAY_500ms > tmr_count)
                    {
                        lc->lcdr[9].dat |= 0x01;  
                        rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));
                    }
                    else
                    {
                        lc->lcdr[9].dat &= ~0x01;                    
                        rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));
                    }
                } 
            }
            break;
            case VOICE_FREQUENCY:/* 音频 */
            {
                if((0 == lc->lreg.btn.button_zl1)&&(0 == lc->lreg.btn.button_zl2))/*The label is normal on ,and the others is normal off */   
                {
                                              
                    lc->lcdr[10].dat &= ~0x0F;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                                                                                           
                    lc->lcdr[9].dat &= ~0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));    
                    lc->lcdr[8].dat |= 0x01;    
                    lc->lcdr[8].dat &= ~0x06;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    lc->lcdr[12].dat &= ~0x0E;
                    rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));

                }
                else  /* The label is blinking ,and the others is normal on */ 
                {                           
                    
                    lc->lcdr[10].dat |= 0x0F;   
                    rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10]));                                                                                         
                    lc->lcdr[9].dat |= 0x07;                
                    rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));                                                                              
                    lc->lcdr[8].dat |= 0x06;                
                    rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
                    if(TMR_DELAY_500ms > tmr_count)
                    {
                        lc->lcdr[8].dat |= 0x01;    
                        rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));
                    }
                    else
                    {
                        lc->lcdr[8].dat &= ~0x01;                    
                        rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));
                    } 
                }
                
            }           
            break;           
            default:
                break;
        }
        switch((lc->lreg.btn.button_zl1 + 1) / 2) /* y = (x+1)/2, x[0:23] */
        {
            case 0:
            {
                                
                lc->lcdr[18].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19]));   
                lc->lcdr[17].dat = 0; 
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17])); 
                
            }    
            break;                   
            case 1:           
            {                        
                
                lc->lcdr[18].dat = 0x01; 
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
                lc->lcdr[17].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                
            }
            break;           
            case 2:
            {                        
                
                lc->lcdr[18].dat = 0x03; 
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
                lc->lcdr[17].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                
            }
            break;
            case 3:
            {                        
                
                lc->lcdr[18].dat = 0x07; 
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
                lc->lcdr[17].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                
            }                    
            break;
            case 4:    
            {
                
                lc->lcdr[18].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
                lc->lcdr[17].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                
            }
            break;            
            case 5:
            {                        
                
                lc->lcdr[18].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0x08;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
                lc->lcdr[17].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                
            }
            break;                        
            case 6:    
            {                        
                
                lc->lcdr[18].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0x0C;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
                lc->lcdr[17].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                
            }
            break;           
            case 7:    
            {   
                
                lc->lcdr[18].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0x0E;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
                lc->lcdr[17].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                
            }
            break;           
            case 8:    
            {                        
                
                lc->lcdr[18].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0x0F;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
                lc->lcdr[17].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                
            }
            break;
            case 9:
            {                        
                
                lc->lcdr[18].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0x0F;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
                lc->lcdr[17].dat = 0x01;                
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                
            }
            break;
            case 10:
            {                        
                
                lc->lcdr[18].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0x0F;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
                lc->lcdr[17].dat = 0x03;                
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                
            }
            break;
            case 11:
            {                        
               
                lc->lcdr[18].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0x0F;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
                lc->lcdr[17].dat = 0x07;                
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                
            }
            break;           
            case 12:
            {                        
                
                lc->lcdr[18].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
                lc->lcdr[19].dat = 0x0F;                
                rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
                lc->lcdr[17].dat = 0x0F;                
                rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                
            }
            break;
            default:
            break;
        }
        switch((lc->lreg.btn.button_zl2 + 1) / 2) /* y = (x+1)/2, x[0:23] */
        {
            case 0:
            {
                                               
                lc->lcdr[3].dat = 0; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
                
            }    
            break;                   
            case 1:
            {                        
                                                
                lc->lcdr[3].dat = 0x01; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
                
            }
            break;
            case 2:
            {                        
                                               
                lc->lcdr[3].dat = 0x03; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
                
            }
            break;
            case 3:
            {                        
                                                
                lc->lcdr[3].dat = 0x07; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
               
            }                    
            break;
            case 4:    
            {
                                                
                lc->lcdr[3].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
                
            }
            break;            
            case 5:    
            {                        
                                                
                lc->lcdr[3].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0x08;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
                
            }
            break;
            case 6:
            {                        
                                                
                lc->lcdr[3].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0x0C;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
                
            }
            break;
            case 7:
            {                        
                                                
                lc->lcdr[3].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0x0E;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
                
            }
            break;            
            case 8:    
            {                        
                                               
                lc->lcdr[3].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0x0F;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
                
            }
            break;
            case 9:
            {                        
                                               
                lc->lcdr[3].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0x0F;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0x01;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
                
            }
            break;
            case 10:
            {                        
                                               
                lc->lcdr[3].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0x0F;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0x03;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
                
            }
            break;
            case 11:
            {                        
                                                
                lc->lcdr[3].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0x0F;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0x07;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
                
            }
            break;            
            case 12:
            {                        
                                               
                lc->lcdr[3].dat = 0x0F; 
                rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
                lc->lcdr[4].dat = 0x0F;                
                rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
                lc->lcdr[5].dat = 0x0F;                
                rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
                
            }
            break;
            default:
            break;
        }         
    }
    if(LWC_ACTIVED == lc->lcf[IONICE_CURE].cure_out_actived)
    {
        /*laser cure display off*/
        lc->lcdr[8].dat &= ~0x08;
        rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8]));
        /*heat cure display off*/
        lc->lcdr[6].dat &= ~0x0F;
        rt_device_write(dev, 0, &lc->lcdr[6], sizeof(lc->lcdr[6]));
        lc->lcdr[0].dat &= ~0x0F;
        rt_device_write(dev, 0, &lc->lcdr[0], sizeof(lc->lcdr[0]));   
        lc->lcdr[1].dat &= ~0x0F;
        rt_device_write(dev, 0, &lc->lcdr[1], sizeof(lc->lcdr[1]));   
        lc->lcdr[2].dat &= ~0x0F;
        rt_device_write(dev, 0, &lc->lcdr[2], sizeof(lc->lcdr[2]));  
        /*display full function*/
        lc->lcdr[10].dat &= ~0x0F;
        rt_device_write(dev, 0, &lc->lcdr[10], sizeof(lc->lcdr[10])); 
        lc->lcdr[9].dat &= ~0x07;                
        rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9])); 
        lc->lcdr[8].dat &= ~0x07;                
        rt_device_write(dev, 0, &lc->lcdr[8], sizeof(lc->lcdr[8])); 
        /*zl1 force reset*/
        lc->lcdr[17].dat = 0; 
        rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17])); 
        lc->lcdr[18].dat = 0;                
        rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
        lc->lcdr[19].dat = 0;                
        rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
        /*zl2 force reset*/
        lc->lcdr[3].dat = 0; 
        rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
        lc->lcdr[4].dat = 0;                
        rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
        lc->lcdr[5].dat = 0;                
        rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
        
        /*cycle and lock lable reset*/
        lc->lcdr[12].dat &= ~0x0E;
        rt_device_write(dev, 0, &lc->lcdr[12], sizeof(lc->lcdr[12]));
        lc->lcdr[9].dat &= ~0x08;
        rt_device_write(dev, 0, &lc->lcdr[9], sizeof(lc->lcdr[9]));
        
    }
              
    return err;   
}

void lwc_force_display(rt_device_t dev, lwc_cure_t *lc)
{
    //rt_err_t err = RT_EOK;
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(lc != RT_NULL);
    switch(lc->ion_force)
    {
        case 0:
        {
                       
            lc->lcdr[18].dat = 0;               
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
            lc->lcdr[19].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0; 
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17])); 
                                                      
            lc->lcdr[3].dat = 0; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
            
        }    
        break;  
        case 1:
        {                        
            
            lc->lcdr[18].dat = 0x01; 
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
            lc->lcdr[19].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                                          
                                            
            lc->lcdr[3].dat = 0x01; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
            
        }
        break;
        case 2:
        {                        
                                                                                              
            lc->lcdr[18].dat = 0x03; 
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
            lc->lcdr[19].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));   
            
            lc->lcdr[3].dat = 0x03; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
        }
        break;
        case 3:
        {                        
            
            lc->lcdr[18].dat = 0x07; 
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
            lc->lcdr[19].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                                                   
            lc->lcdr[3].dat = 0x07; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
           
        }                    
        break;
        case 4:
        {
            
            lc->lcdr[18].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
            lc->lcdr[19].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                   
                                            
            lc->lcdr[3].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
            
        }
        break;
        case 5:
        {                        
            
            lc->lcdr[18].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
            lc->lcdr[19].dat = 0x08;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
            
                                         
            lc->lcdr[3].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0x08;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
            
        }
        break;
        case 6:
        {                        
            
            lc->lcdr[18].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
            lc->lcdr[19].dat = 0x0C;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
            
                                        
            lc->lcdr[3].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0x0C;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
            
        }
        break;
        case 7:
        {   
            
            lc->lcdr[18].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
            lc->lcdr[19].dat = 0x0E;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
                                                  
            lc->lcdr[3].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0x0E;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
            
        }
        break;
        case 8:
        {                        
            
            lc->lcdr[18].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
            lc->lcdr[19].dat = 0x0F;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
            
                                       
            lc->lcdr[3].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0x0F;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
            
        }
        break;
        case 9:
        {                        
            
            lc->lcdr[18].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
            lc->lcdr[19].dat = 0x0F;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0x01;                
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
            
                                      
            lc->lcdr[3].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0x0F;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0x01;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
            
        }
        break;
        case 10:
        {                        
            
            lc->lcdr[18].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
            lc->lcdr[19].dat = 0x0F;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0x03;                
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
            
                                       
            lc->lcdr[3].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0x0F;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0x03;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
            
        }
        break;
        case 11:
        {                        
           
            lc->lcdr[18].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18])); 
            lc->lcdr[19].dat = 0x0F;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0x07;                
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
            
                                       
            lc->lcdr[3].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0x0F;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0x07;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
            
        }               
        break;
        case 12:
        {                        
           
            lc->lcdr[18].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[18], sizeof(lc->lcdr[18]));            
            lc->lcdr[19].dat = 0x0F;                
            rt_device_write(dev, 0, &lc->lcdr[19], sizeof(lc->lcdr[19])); 
            lc->lcdr[17].dat = 0x0F;                
            rt_device_write(dev, 0, &lc->lcdr[17], sizeof(lc->lcdr[17]));                               
            
                                       
            lc->lcdr[3].dat = 0x0F; 
            rt_device_write(dev, 0, &lc->lcdr[3], sizeof(lc->lcdr[3])); 
            lc->lcdr[4].dat = 0x0F;                
            rt_device_write(dev, 0, &lc->lcdr[4], sizeof(lc->lcdr[4])); 
            lc->lcdr[5].dat = 0x0F;                
            rt_device_write(dev, 0, &lc->lcdr[5], sizeof(lc->lcdr[5])); 
            
        }               
        break;
        default:
        break;
    }
           
}    

#endif /* RT_USING_LIGHT_WAVE_CURER */

/*@}*/
