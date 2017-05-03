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
#ifndef GPIO_H__
#define GPIO_H__
 
#include <stdint.h> 
#include <drivers/pin.h>

typedef struct stm32_hw_pin
{
    int pin; /* the pin NO. on the chip*/
    uint32_t mode;
}stm32_hw_pin_t;

#define PIN_USERDATA_END {-1,0}

extern  stm32_hw_pin_t pins_app[];

int stm32_hw_pin_init(void);

#endif
