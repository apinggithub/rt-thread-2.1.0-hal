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

#ifndef HW_ADC_H__
#define HW_ADC_H__

#include "stm32f1xx_hal.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <drv_hwadc.h>

#ifdef __cplusplus
extern "C" {
#endif

#define     TOTAL_CHANNELS          1 
    
/* ADC Control Command */
#define    HWADC_ENABLE             0x01
#define    HWADC_DISABLE            0x02  
 

/* ADC Control Command */
enum rt_hwadc_ctrl
{
    HWTADC_CTRL_START = 0x01,           /* start the adc device */
    HWTADC_CTRL_STOP,                   /* stop the adc device */    
};   
      
/* button device and operations for RT-Thread */
typedef struct rt_device_hwadc
{
    struct rt_device parent;
    const struct rt_hwadc_ops *ops;        
    __IO uint16_t adc_converted_value[TOTAL_CHANNELS];   
    
}rt_device_hwadc_t;


typedef struct rt_hwadc_ops
{       
    rt_err_t (*drv_init)(rt_device_hwadc_t *devadc, rt_uint8_t status);        
    rt_size_t (*drv_read)(rt_device_hwadc_t *devadc,rt_off_t pos,void *buffer,rt_size_t size);    
    rt_err_t (*drv_start)(rt_device_hwadc_t *devadc);
    rt_err_t (*drv_stop)(rt_device_hwadc_t *devadc);
    rt_err_t (*drv_start_IT)(rt_device_hwadc_t *devadc);
    rt_err_t (*drv_stop_IT)(rt_device_hwadc_t *devadc);
    rt_err_t (*drv_start_DMA)(rt_device_hwadc_t *devadc, uint16_t *buffer, rt_size_t size);
    rt_err_t (*drv_stop_DMA)(rt_device_hwadc_t *devadc);
        
    /* TODO: add GPIO interrupt */
}rt_hwadc_ops_t;

rt_err_t rt_device_hwadc_register(rt_device_hwadc_t *devadc, const char *name, void *user_data);

/* RT-Thread Hardware button APIs */
//void rt_port_mode(rt_base_t port, rt_base_t mode);
//void rt_port_write(rt_base_t port, rt_base_t value);
//int  rt_port_read(rt_base_t port);

int  rt_port_read(void);

#ifdef __cplusplus
}
#endif

#endif
