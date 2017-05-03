/*
 * File      : pin.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-01-20     Bernard      the first version
 */

#ifndef HW_BUTTON_H__
#define HW_BUTTON_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

/* button device and operations for RT-Thread */
typedef struct rt_device_button
{
    struct rt_device parent;
    const struct rt_button_ops *ops;
}rt_device_button_t;


typedef struct rt_button_ops
{       
    rt_err_t (*drv_init)(rt_device_t dev,rt_uint8_t flag);    
    rt_size_t (*drv_read)(rt_device_t dev);    
    rt_size_t (*drv_close)(rt_device_t dev);   
    /* TODO: add GPIO interrupt */
}rt_button_ops_t;

int rt_device_button_register(const char *name, const rt_button_ops_t *ops, void *user_data);

/* RT-Thread Hardware button APIs */
//void rt_port_mode(rt_base_t port, rt_base_t mode);
//void rt_port_write(rt_base_t port, rt_base_t value);
//int  rt_port_read(rt_base_t port);

int  rt_port_read(void);

#ifdef __cplusplus
}
#endif

#endif
