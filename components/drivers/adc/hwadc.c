/*
 * File      : pin.c
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

#include <drivers/hwadc.h>
#include <drv_hwadc.h>

#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

#ifdef RT_USING_HWADC
//static rt_device_hwadc_t _devadc;

static rt_err_t rt_hwadc_init(rt_device_t dev)
{
    rt_err_t err = RT_EOK;
    rt_device_hwadc_t *devadc = (rt_device_hwadc_t *)dev;

    /* check parameters */
    RT_ASSERT(devadc != RT_NULL);

    err = devadc->ops->drv_init(devadc, HWADC_ENABLE);
    return err;
} 

static rt_size_t rt_hwadc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
   
    rt_device_hwadc_t *devadc = (rt_device_hwadc_t *)dev;

    /* check parameters */
    RT_ASSERT(devadc != RT_NULL);
    
    size = devadc->ops->drv_read(devadc, pos, buffer, size);
    return size;
}

static rt_err_t rt_hwadc_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    rt_err_t result = RT_EOK;
    rt_device_hwadc_t *devadc = (rt_device_hwadc_t *)dev;
    RT_ASSERT(devadc != RT_NULL)
    //drv_hwadc_t *hwadc;      
    //hwadc = (drv_hwadc_t *)adc->parent.user_data;
      
    switch (cmd)
    {
        case HWTADC_CTRL_START:
        {
            #if defined RT_USING_ADC_GENERIC_MODE
            if(devadc->ops->drv_start != RT_NULL)
            {
                result = devadc->ops->drv_start(devadc);                
            }
            #elif defined RT_USING_ADC_INT_MODE
            if(devadc->ops->drv_start_IT != RT_NULL)
            {
                result = devadc->ops->drv_start_IT(devadc);                
            }
            #elif defined RT_USING_ADC_DMA_MODE           
            if(devadc->ops->drv_start_DMA != RT_NULL)
            {
                result = devadc->ops->drv_start_DMA(devadc ,devadc->adc_converted_value, TOTAL_CHANNELS);            
            }
            #endif
        }
        break;
        case HWTADC_CTRL_STOP:/* stop the adc device */
        {
            #if defined RT_USING_ADC_GENERIC_MODE
            if(devadc->ops->drv_stop != RT_NULL)
            {
                result = devadc->ops->drv_stop(devadc);
            }
            #elif defined RT_USING_ADC_INT_MODE
            if(devadc->ops->drv_stop_IT != RT_NULL)
            {
                result = devadc->ops->drv_stop_IT(devadc);                
            }
            #elif defined RT_USING_ADC_DMA_MODE
            if(devadc->ops->drv_start_DMA != RT_NULL)
            {
                result = devadc->ops->drv_stop_DMA(devadc);                
            }
            #endif            
        }
        break;        
        default:
        break;
    }
    return result;
    
}

static rt_err_t rt_hwadc_close(rt_device_t dev)
{
    rt_err_t result = RT_EOK;
    rt_device_hwadc_t *devadc = (rt_device_hwadc_t *)dev;
    
    /* check parameters */
    RT_ASSERT(devadc != RT_NULL);
      
    if (devadc->ops->drv_init != RT_NULL)
    {
        #if defined RT_USING_ADC_GENERIC_MODE
        if(devadc->ops->drv_stop != RT_NULL)
        {
            result = devadc->ops->drv_stop(devadc);
        }
        #elif defined RT_USING_ADC_INT_MODE
        if(devadc->ops->drv_stop_IT != RT_NULL)
        {
            result = devadc->ops->drv_stop_IT(devadc);                
        }
        #elif defined RT_USING_ADC_DMA_MODE
        if(devadc->ops->drv_stop_DMA != RT_NULL)
        {
            result = devadc->ops->drv_stop_DMA(devadc);                
        }
        #endif   
        devadc->ops->drv_init(devadc, HWADC_DISABLE);
    }
    else
    {
        result = -RT_ENOSYS;
    }

    dev->flag &= ~RT_DEVICE_FLAG_ACTIVATED;
    dev->rx_indicate = RT_NULL;

    return result;
}

rt_err_t rt_device_hwadc_register(rt_device_hwadc_t *devadc, const char *name, void *user_data)
{
    struct rt_device *device;

    RT_ASSERT(devadc != RT_NULL);
    RT_ASSERT(devadc->ops != RT_NULL);
    
    device = &(devadc->parent);

    device->type        = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

    device->init        = rt_hwadc_init;
    device->open        = RT_NULL;
    device->close       = rt_hwadc_close;
    device->read        = rt_hwadc_read;
    device->write       = RT_NULL;
    device->control     = rt_hwadc_control;
    
    device->user_data   = user_data;

    return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
}

/* RT-Thread Hardware PIN APIs */
#if 0
void rt_port_mode(rt_base_t port, rt_base_t mode)
{
	RT_ASSERT(_hw_button.ops != RT_NULL);
    _hw_button.ops->port_mode(&_hw_button.parent, port, mode);
}
//FINSH_FUNCTION_EXPORT_ALIAS(rt_port_mode, portMode, set hardware port mode);

void rt_port_write(rt_base_t port, rt_base_t value)
{
	RT_ASSERT(_hw_button.ops != RT_NULL);
    _hw_button.ops->port_write(&_hw_button.parent, port, value);
}
//FINSH_FUNCTION_EXPORT_ALIAS(rt_port_write, portWrite, write value to hardware port);

int  rt_port_read(rt_base_t port)
{
	RT_ASSERT(_hw_button.ops != RT_NULL);
    return _hw_button.ops->port_read(&_hw_button.parent, port);
}
//FINSH_FUNCTION_EXPORT_ALIAS(rt_port_read, portRead, read status from hardware port);
#endif
#endif /* RT_USING_HWADC */

