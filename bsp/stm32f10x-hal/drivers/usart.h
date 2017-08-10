/*
 * File      : usart.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

#ifndef __USART_H__
#define __USART_H__

#include <rthw.h>
#include <rtthread.h>

#define UART_ENABLE_IRQ(n)            NVIC_EnableIRQ((n))
#define UART_DISABLE_IRQ(n)           NVIC_DisableIRQ((n))

#define RT_USART_MODE_0  0
#define RT_USART_MSB      1

struct rt_usart_configuration{
	rt_uint32_t data_width;
	rt_uint32_t mode;
	rt_uint32_t max_hz;
        };
struct rt_usart_message{
	rt_uint32_t *send_buf;
  rt_uint32_t *recv_buf;
  rt_uint8_t      length;
  rt_uint8_t    cs_take;
  rt_uint8_t    cs_release;
                       };

void rt_hw_usart_init(void);

#endif
