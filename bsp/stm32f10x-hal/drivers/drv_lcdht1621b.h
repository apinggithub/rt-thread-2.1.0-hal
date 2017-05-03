/*
 * File      : gpio.h
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
#ifndef DRV_LCDHT1621B_H__
#define DRV_LCDHT1621B_H__
 
#include <stdint.h> 
#include <drivers/lcdht1621b.h>

#define LCDHT_CS_H()  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET)                                             
#define LCDHT_CS_L()  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET)  

#define LCDHT_WR_H()  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET)                                                      
#define LCDHT_WR_L()  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET)

#define LCDHT_DAT_H() HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET)                                             
#define LCDHT_DAT_L() HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET)

#define LCDHT_BKL_H()  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET)                                                        
#define LCDHT_BKL_L()  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET)

enum LCDHT_CTRL
{
    LCDHT_CMD_LCDON  =   0x00,
    LCDHT_CMD_LCDOFF,    
    LCDHT_CMD_BKLON,    
    LCDHT_CMD_BKLOFF, 
    LCDHT_CMD_FULL_SREEN_ON,
    LCDHT_CMD_FULL_SREEN_OFF,    
};

enum HTB_CMD
{
    /* BIAS&COM : 100 0010 abXcX = 0010 1001
    c=0:可选1/2偏压
    c=1:可选1/3偏压
    ab=00:可选2个COM
    ab=01:可选3个COM
    ab=10:可选4个COM
    */
    HTB_CMD_BIAS    = 0x28, // 0B:0010 abXc -ab控制占空比,-c控制偏压
    HTB_CMD_SYSEN   = 0x01, //
    HTB_CMD_LCDOFF  = 0x02, //
    HTB_CMD_LCDON   = 0x03, //

};



int stm32_lcdht1621b_init(void);

#endif
