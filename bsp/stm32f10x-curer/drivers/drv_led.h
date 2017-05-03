/*
 * File      : drv_led.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-08-01     xiaonong     the first version
 */
#ifndef __DRV_LED_H
#define __DRV_LED_H

//#include "board.h"

#define RCC_GPIO_LED1_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define GPIO_PORT_LED1              GPIOA
#define GPIO_PIN_LED1               GPIO_PIN_15

#define rt_led_on()     HAL_GPIO_WritePin(GPIO_PORT_LED1, GPIO_PIN_LED1, GPIO_PIN_SET)
#define rt_led_off()    HAL_GPIO_WritePin(GPIO_PORT_LED1, GPIO_PIN_LED1, GPIO_PIN_RESET)

int rt_led_hw_init(void);

#endif
