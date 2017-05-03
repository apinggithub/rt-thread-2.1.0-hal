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
#ifndef DRV_HWBUTTON_H__
#define DRV_HWBUTTON_H__
 
#include <stdint.h> 
#include <drivers/hwbutton.h>

/*Configure GPIO pins : PC0 PC1 PC2 PC3 PC4 PC5 */
                                                   
struct stm32_hw_button_port_app
{
    int port;
    uint32_t mode;
};

//#define PIN_USERDATA_END {-1,0}

extern struct stm32_hw_button_port_app stm32_ports[];

int stm32_hw_button_init(void);

#endif
