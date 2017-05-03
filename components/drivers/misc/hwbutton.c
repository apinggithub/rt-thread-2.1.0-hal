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

#include <drivers/hwbutton.h>

#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

static rt_device_button_t _hw_button;

static rt_err_t rt_hwbutton_init(rt_device_t dev)
{
    rt_err_t err = RT_EOK;
    rt_device_button_t *button = (rt_device_button_t *)dev;

    /* check parameters */
    RT_ASSERT(button != RT_NULL);

    err = button->ops->drv_init(dev, 1);
    return err;
}

static rt_size_t rt_hwbutton_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
   
    rt_uint8_t *pbuf;
    rt_device_button_t *button = (rt_device_button_t *)dev;

    /* check parameters */
    RT_ASSERT(button != RT_NULL);

    pbuf = (rt_uint8_t *) buffer;
    if (pbuf == RT_NULL || size != sizeof(*pbuf)) 
        return 0;

    *pbuf = button->ops->drv_read(dev);
    return size;
}

static rt_err_t rt_hwbutton_close(rt_device_t dev)
{
    rt_err_t result = RT_EOK;
    rt_device_button_t *button = (rt_device_button_t *)dev;
    
    /* check parameters */
    RT_ASSERT(button != RT_NULL);
      
    if (button->ops->drv_init != RT_NULL)
    {
        button->ops->drv_init(dev, 0);
    }
    else
    {
        result = -RT_ENOSYS;
    }

    dev->flag &= ~RT_DEVICE_FLAG_ACTIVATED;
    dev->rx_indicate = RT_NULL;

    return result;
}

int rt_device_button_register(const char *name, const rt_button_ops_t *ops, void *user_data)
{
    _hw_button.parent.type         = RT_Device_Class_Miscellaneous;
    _hw_button.parent.rx_indicate  = RT_NULL;
    _hw_button.parent.tx_complete  = RT_NULL;

    _hw_button.parent.init         = rt_hwbutton_init;
    _hw_button.parent.open         = RT_NULL;
    _hw_button.parent.close        = rt_hwbutton_close;
    _hw_button.parent.read         = rt_hwbutton_read;
    _hw_button.parent.write        = RT_NULL;//_port_write;
    _hw_button.parent.control      = RT_NULL;//_port_control;

    _hw_button.ops                 = ops;
    _hw_button.parent.user_data    = user_data;

    /* register a character device */
    rt_device_register(&_hw_button.parent, name, RT_DEVICE_FLAG_RDWR);

    return 0;
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

