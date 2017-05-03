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
#include <drivers/lcdht1621b.h>

#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

static rt_device_lcdht_t _lcd_ht1621b;

static rt_err_t rt_lcdht_init(rt_device_t dev)
{
    rt_err_t err = RT_EOK;
    rt_device_lcdht_t *lcdht = (rt_device_lcdht_t *)dev;

    /* check parameters */
    RT_ASSERT(lcdht != RT_NULL);

    err = lcdht->ops->drv_init(dev, 1);
    return err;
}

static rt_size_t rt_lcdht_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{

    rt_device_lcdht_t *lcdht = (rt_device_lcdht_t *)dev;

    /* check parameters */
    RT_ASSERT(lcdht != RT_NULL);
    
    RT_ASSERT(buffer != RT_NULL);

    return  lcdht->ops->drv_write(dev,buffer,size);
    
}

static rt_err_t rt_lcdht_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{

    rt_device_lcdht_t *lcdht = (rt_device_lcdht_t *)dev;

    /* check parameters */
    RT_ASSERT(lcdht != RT_NULL);     

    return  lcdht->ops->drv_control(dev,cmd,args);
    
}

static rt_err_t rt_lcdht_close(rt_device_t dev)
{
    rt_err_t result = RT_EOK;
    rt_device_lcdht_t *lcdht = (rt_device_lcdht_t *)dev;
    
    /* check parameters */
    RT_ASSERT(lcdht != RT_NULL);
      
    if (lcdht->ops->drv_init != RT_NULL)
    {
        lcdht->ops->drv_init(dev, 0);
    }
    else
    {
        result = -RT_ENOSYS;
    }

    dev->flag &= ~RT_DEVICE_FLAG_ACTIVATED;
    dev->rx_indicate = RT_NULL;

    return result;
}



int rt_device_lcdht_register(const char *name, const rt_lcdht_ops_t *ops, void *user_data)
{
    _lcd_ht1621b.parent.type         = RT_Device_Class_Miscellaneous;
    _lcd_ht1621b.parent.rx_indicate  = RT_NULL;
    _lcd_ht1621b.parent.tx_complete  = RT_NULL;

    _lcd_ht1621b.parent.init         = rt_lcdht_init;
    _lcd_ht1621b.parent.open         = RT_NULL;
    _lcd_ht1621b.parent.close        = rt_lcdht_close;
    _lcd_ht1621b.parent.read         = RT_NULL;
    _lcd_ht1621b.parent.write        = rt_lcdht_write;
    _lcd_ht1621b.parent.control      = rt_lcdht_control;

    _lcd_ht1621b.ops                 = ops;
    _lcd_ht1621b.parent.user_data    = user_data;

    /* register a character device */
    rt_device_register(&_lcd_ht1621b.parent, name, RT_DEVICE_FLAG_RDWR);

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


