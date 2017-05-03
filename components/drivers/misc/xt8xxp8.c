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
 
#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/xt8xxp8.h>

#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

static rt_device_xtp_t _xtp;

static rt_err_t rt_xtp_init(rt_device_t dev)
{
    rt_err_t err = RT_EOK;
    rt_device_xtp_t *xtp = (rt_device_xtp_t *)dev;

    /* check parameters */
    RT_ASSERT(xtp != RT_NULL);

    err = xtp->ops->drv_init(dev);
    return err;
}

static rt_size_t rt_xtp_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{

    rt_device_xtp_t *xtp = (rt_device_xtp_t *)dev;

    /* check parameters */
    RT_ASSERT(xtp != RT_NULL);
    
    RT_ASSERT(buffer != RT_NULL);

    return  xtp->ops->drv_write(dev,buffer,size);
    
}


int rt_device_xtp_register(const char *name, const rt_xtp_ops_t *ops, void *user_data)
{
    _xtp.parent.type         = RT_Device_Class_Miscellaneous;
    _xtp.parent.rx_indicate  = RT_NULL;
    _xtp.parent.tx_complete  = RT_NULL;

    _xtp.parent.init         = rt_xtp_init;
    _xtp.parent.open         = RT_NULL;
    _xtp.parent.close        = RT_NULL;
    _xtp.parent.read         = RT_NULL;
    _xtp.parent.write        = rt_xtp_write;
    _xtp.parent.control      = RT_NULL;

    _xtp.ops                 = ops;
    _xtp.parent.user_data    = user_data;

    /* register a character device */
    rt_device_register(&_xtp.parent, name, RT_DEVICE_FLAG_RDWR);

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


