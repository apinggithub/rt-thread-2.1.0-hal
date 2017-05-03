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
#include <drv_lcdht1621b.h>
#include <drivers/lcdht1621b.h>


#ifdef RT_USING_LCDHT1621B

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

/*发送多字节，高位在前*/
static void HTB_SendBitMsb(uint8_t dat, uint8_t cnt)
{
    for (uint8_t i=0; i<cnt; i++)
    {
        (dat & 0x80)? LCDHT_DAT_H(): LCDHT_DAT_L();
        
        dat <<= 1;
        LCDHT_WR_L();
        wait();
        LCDHT_WR_H();
    }
}
/*发送多字节，低位在前*/
static void HTB_SendBitLsb(uint8_t dat, uint8_t cnt)
{
    for (uint8_t i=0; i<cnt; i++)
    {
        (dat & 0x01) ? LCDHT_DAT_H(): LCDHT_DAT_L();

        dat >>= 1;
        LCDHT_WR_L();
        wait();
        LCDHT_WR_H();
    }
}

/* Send a control comand */
static void HTB_SendCmd(uint8_t cmd)
{
    LCDHT_CS_L();
    HTB_SendBitMsb(0x80, 3);//前面3位为命令代码

    HTB_SendBitMsb(cmd, 9);//后面10位A5~A0(RAM地址)+D3~D0(RAM数据)
    
    LCDHT_CS_H();
}
/* Send a RAM dat */
static void HTB_SendDat(uint8_t addr, uint8_t dat)
{
    LCDHT_CS_L();
    HTB_SendBitMsb(0xA0, 3);//前面3位为命令代码
    HTB_SendBitMsb(addr<<2, 6);//A5~A0(RAM地址)
    
    HTB_SendBitLsb(dat, 4);//RAM数据,宽度为4位
    
    LCDHT_CS_H();
}

static void HTB_SendFullScreenOn(void)
{
    /* seg0 ~ seg19 in use */
    for(uint8_t i = 0; i < 32; i++)
    { 
        if(7 == i)//||(8 == i)
        {
            HTB_SendDat(i, 0x0E);   
        }
        else
        {
            HTB_SendDat(i, 0x0F);   
        }       
        
        if(17 == i)/* segement 18 ~ 29 not use */
        {
            i = 30-1;
        }
    }
}
static void HTB_SendFullScreenOff(void)
{
    /* seg0 ~ seg19 in use */
    for(uint8_t i = 0; i < 32; i++)
    {       
        HTB_SendDat(i, 0x00);            
    }
}
    
static rt_err_t drv_lcdht_init(rt_device_t dev, rt_uint8_t status)
{
    
    rt_device_lcdht_t *lcdht = (rt_device_lcdht_t *)dev;

    /* check parameters */
    RT_ASSERT(lcdht != RT_NULL);
    
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
  
    /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
    if(1 == status)
    {
        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9; /*PC6~PC9*/
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        
        /*initialize the HT lcd */
        //LCDHT_BKL_H();  /*turn on the backlight*/      
        HTB_SendCmd( HTB_CMD_BIAS );  
        HTB_SendCmd( HTB_CMD_SYSEN );   
        HTB_SendCmd( HTB_CMD_LCDON );
    }
    else
    {
        HTB_SendCmd( HTB_CMD_LCDOFF );
        //LCDHT_BKL_L();
        HAL_GPIO_DeInit(GPIOC, GPIO_InitStruct.Pin);
    }
        
    return RT_EOK;
}

static rt_size_t drv_lcdth_write(rt_device_t dev, const void *buffer, rt_size_t size)
{
    rt_device_lcdht_t *lcdht = (rt_device_lcdht_t *)dev;
    
    /* check parameters */
    RT_ASSERT(lcdht != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
    
    rt_lcd_ramdat_t *rcv;
           
    rcv = (rt_lcd_ramdat_t *)buffer;
    
    HTB_SendDat(rcv->segno, rcv->dat); 
        
    return sizeof(rcv->dat);
}

static rt_err_t drv_lcdth_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    rt_device_lcdht_t *lcdht = (rt_device_lcdht_t *)dev;
    
    /* check parameters */
    RT_ASSERT(lcdht != RT_NULL);
    
    rt_err_t  err = RT_EOK;
     
    switch (cmd)
    {
        case LCDHT_CMD_LCDON:
            HTB_SendCmd( HTB_CMD_LCDON );
            break;
        case LCDHT_CMD_LCDOFF:
            HTB_SendCmd( HTB_CMD_LCDOFF );
            break;
        case LCDHT_CMD_BKLON:
            LCDHT_BKL_H();
            break;
        case LCDHT_CMD_BKLOFF:
            LCDHT_BKL_L();
            break;
        case LCDHT_CMD_FULL_SREEN_ON:
            HTB_SendFullScreenOn();
            break;
        case LCDHT_CMD_FULL_SREEN_OFF:
            HTB_SendFullScreenOff();
            break;
        default:
            break;
    }
                
    return err;
}


const static rt_lcdht_ops_t _drv_lcdht_ops =
{
    drv_lcdht_init,   
    drv_lcdth_write,
    drv_lcdth_control,
   
};

int stm32_lcdht1621b_init(void)
{
    int result;
    
    result = rt_device_lcdht_register("lcdht", &_drv_lcdht_ops, RT_NULL);
    return result;
}
INIT_BOARD_EXPORT(stm32_lcdht1621b_init);

#endif /* RT_USING_LCDHT1621B */
