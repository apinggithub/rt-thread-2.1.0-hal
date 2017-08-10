#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <stdio.h>
#include <drivers/lcdht1621b.h>
#include <drv_lcdht1621b.h>
#include "light_wave_curer.h"

#ifdef RT_USING_LCDHT1621B_TEST

//extern rt_err_t seglcd_display_time(rt_device_t dev, uint8_t time, uint8_t flag);
//extern rt_err_t seglcd_display_cure_mode(rt_device_t dev, uint8_t cure_mode, uint8_t flag);

int lcdht(void)
{
    rt_err_t err;   
    rt_device_t devlcdht = RT_NULL;
    
    
    //rt_lcd_ramdat_t val;
    //uint8_t tm = 0;  
    
     
    #define LCD   "lcdht"
       
    //rt_pin_mode(20, PIN_MODE_OUTPUT);// the port PF8
   
    if ((devlcdht = rt_device_find(LCD)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", LCD);
        return -1;
    }
   
    if (rt_device_open(devlcdht, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", LCD);
        return -1;
    }
          
    while(1)
    {
          
        rt_device_control(devlcdht, LCDHT_CMD_FULL_SREEN_OFF, RT_NULL);
        rt_kprintf("full screen the seg lcd OFF.\n");
        rt_thread_delay(RT_TICK_PER_SECOND);
        rt_device_control(devlcdht, LCDHT_CMD_BKLON, RT_NULL);
        rt_kprintf("Open the seg lcd backlight.\n");
        rt_device_control(devlcdht, LCDHT_CMD_LCDON, RT_NULL);
        rt_kprintf("Open the seg lcd .\n");
        
        /* write the lcd */
        for(uint8_t i = 0;i < 10;i++)
        {
            #if 0
            val.segno = i;
            if(7 == i)
            {
                val.dat = 0x0E;
            }
            else
            {
                val.dat = 0x0F;
            }
            rt_device_write(devlcdht, 0, &val, sizeof(val));
            #endif
            //seglcd_display_time(devlcdht, i, 0);
            //seglcd_display_time(devlcdht, i, 1);

                        
            //rt_kprintf("Send to lcd segno = %d , dat = %x \n", val.segno, val.dat);
            rt_kprintf("Send to lcd dat = %d , \n",  i);
            rt_thread_delay(1*RT_TICK_PER_SECOND);  
            

            //val.dat = 0x00;
            //rt_device_write(devlcdht, 0, &val, sizeof(val));
            //rt_kprintf("Send to lcd segno = %d , dat = %x \n", val.segno, val.dat);
            rt_thread_delay(RT_TICK_PER_SECOND);  
            #if 0
            if(i == 17)
            {
                i = 30-1;
            }
            #endif
                                                   
        }
        break;  
        /*
        tm++;
        if(tm > 2)
        {    
            //tm = 0;  
            rt_kprintf("the tm value = %d  \n", tm);              
            break;        
        }
        rt_kprintf("the tm value = %d  \n", tm);              
         */                
        //rt_thread_delay(RT_TICK_PER_SECOND/10);         
                
    } 
    
    rt_device_control(devlcdht, LCDHT_CMD_FULL_SREEN_ON, RT_NULL);
    rt_kprintf("full screen the seg lcd ON.\n");
    rt_thread_delay(RT_TICK_PER_SECOND);  
    
    rt_device_control(devlcdht, LCDHT_CMD_FULL_SREEN_OFF, RT_NULL);
    rt_kprintf("full screen the seg lcd OFF.\n");
    rt_thread_delay(RT_TICK_PER_SECOND); 
    
    rt_device_control(devlcdht, LCDHT_CMD_FULL_SREEN_ON, RT_NULL);
    rt_kprintf("full screen the seg lcd ON.\n");
    rt_thread_delay(RT_TICK_PER_SECOND); 
    
    rt_device_control(devlcdht, LCDHT_CMD_LCDOFF, RT_NULL);
    rt_kprintf("Close the seg lcd .\n");
    rt_device_control(devlcdht, LCDHT_CMD_BKLOFF, RT_NULL);
    rt_kprintf("Close the seg lcd backlight.\n");
    
    err = rt_device_close(devlcdht);
    rt_kprintf("Close %s test.\n", LCD);
   
    return err;

}
FINSH_FUNCTION_EXPORT(lcdht, Test Seg lcd ht1621b);
#endif /* RT_USING_LCDHT1621B_TEST */
