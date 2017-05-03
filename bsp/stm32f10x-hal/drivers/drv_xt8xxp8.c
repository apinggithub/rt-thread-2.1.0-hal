/*
 * File      : gpio.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-03-24     Bright      the first version
 */

#include <rthw.h>
#include <rtdevice.h>
#include <board.h>
#include <drv_xt8xxp8.h>
#include <drivers/xt8xxp8.h>
//#include <core_cm3.h>

#ifdef RT_USING_XT8XXP8
#if 1
static void rt_hw_us_delay(int us)
{
    rt_uint32_t delta;
    rt_uint32_t current_delay;

    /* ge ticks to delay  */
    us = us * (SysTick->LOAD/(1000000/RT_TICK_PER_SECOND));

    /* get current time */
    delta = SysTick->VAL;

    /* get current time by cycles,until the given time to exit */
    do
    {
        if ( delta > SysTick->VAL )
            current_delay = delta - SysTick->VAL;
        else
        /* the delay crossing a OS tick critical */
        current_delay = SysTick->LOAD + delta - SysTick->VAL;
    } while( current_delay < us );
}
#endif
#if 0
__asm void wait() 
{ 
    nop 
    nop 
    nop 
    nop 
    nop 
    nop
    nop
    nop
    nop 
    BX lr 
} 
#endif 

/*发送单字节，低位在前*/
static void XTP_SendBitLsb(uint8_t dat)
{
#ifdef RT_USING_XTP_ONE_WRIRE   
    XTP_DAT_L();/*data lin set low about 5ms to wakeup voice IC*/
    rt_thread_delay(RT_TICK_PER_SECOND*5/1000);
    
    /*low level 200us and high level 600us express 0; high level 600us and low level 200us express 1*/
    for (uint8_t i = 0; i < 8; i++)
    {
        //(dat & 0x01) ? XTP_DAT_H(): XTP_DAT_L();
        
        if(dat & 0x01) 
        {
            XTP_DAT_H();
            rt_hw_us_delay(600);
            //rt_thread_delay(RT_TICK_PER_SECOND*2/1000);
            XTP_DAT_L();
            rt_hw_us_delay(200);
            //rt_thread_delay(RT_TICK_PER_SECOND/1000);
        }
        else
        {            
            XTP_DAT_L();
            rt_hw_us_delay(600);
            //rt_thread_delay(RT_TICK_PER_SECOND*2/1000);
            XTP_DAT_H();
            rt_hw_us_delay(200);
            //rt_thread_delay(RT_TICK_PER_SECOND/1000);
        }                          
        dat >>= 1;
       
    }
    XTP_DAT_H();
    
#else
    XTP_RST_H();//reset pin
    XTP_DAT_H();//data pin
    XTP_CLK_L();/* clock pin rising edge to lock data */   
    XTP_RST_L();
    rt_thread_delay(RT_TICK_PER_SECOND*10/1000);  //30ms
    XTP_RST_H();
    rt_thread_delay(RT_TICK_PER_SECOND*10/1000); 
    for (uint8_t i = 0; i < 8; i++)
    {
        (dat & 0x01) ? XTP_DAT_H(): XTP_DAT_L(); 
        //rt_thread_delay(RT_TICK_PER_SECOND/1000); //1ms
        //rt_hw_us_delay(100);       
        XTP_CLK_L();
        //rt_thread_delay(RT_TICK_PER_SECOND*1/1000); //2ms   
        rt_hw_us_delay(100);
        XTP_CLK_H();
        //rt_thread_delay(RT_TICK_PER_SECOND*1/1000); //2ms
        rt_hw_us_delay(100);        
        dat >>= 1;      
    }
    XTP_DAT_H(); 
    XTP_CLK_H();   
    
#endif    
}

  
static rt_err_t drv_xtp_init(rt_device_t dev)
{
    
    rt_device_xtp_t *xtp = (rt_device_xtp_t *)dev;

    /* check parameters */
    RT_ASSERT(xtp != RT_NULL);
    
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();

#ifdef RT_USING_XTP_ONE_WRIRE    
    /*Configure GPIO pins : PC10 ---> chip data in single wire mode */    
    GPIO_InitStruct.Pin = GPIO_PIN_10; /*PC10 the voice chip data*/
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#else
    /*Configure GPIO pins : PC11 PC12 
    PC11 ---> the voice chip data 
    PC12 ---> the voice chip clock 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif       
    return RT_EOK;
}

static rt_size_t drv_xtp_write(rt_device_t dev, const void *buffer, rt_size_t size)
{
    rt_device_xtp_t *xtp = (rt_device_xtp_t *)dev;
    
    /* check parameters */
    RT_ASSERT(xtp != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
    
    rt_uint8_t dat;
           
    dat = *(rt_uint8_t *)buffer;
    
    XTP_SendBitLsb(dat); 
        
    return sizeof(dat);
}


const static rt_xtp_ops_t _drv_xtp_ops =
{
    drv_xtp_init,   
    drv_xtp_write,    
};

int stm32_xtp_init(void)
{
    int result;
    
    result = rt_device_xtp_register("xtp", &_drv_xtp_ops, RT_NULL);
    return result;
}
INIT_BOARD_EXPORT(stm32_xtp_init);

#endif /* RT_USING_XT8XXP8 */
