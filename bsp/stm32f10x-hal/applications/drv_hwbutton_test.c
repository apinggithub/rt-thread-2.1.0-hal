#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <stdio.h>
#include <stdint.h>

#ifdef RT_USING_HWBUTTON_TEST

int hwbutton(void)
{
    rt_err_t err;   
    rt_device_t devbtn = RT_NULL;
    
    
    uint8_t val;
    uint32_t tm;   
     
    #define BUTTON   "button"
       
    //rt_pin_mode(20, PIN_MODE_OUTPUT);// the port PF8
   
    if ((devbtn = rt_device_find(BUTTON)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", BUTTON);
        return -1;
    }
   
    if (rt_device_open(devbtn, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", BUTTON);
        return -1;
    }
          
    while(1)
    {
               
        /* read the counter */
        rt_device_read(devbtn, 0, &val, sizeof(val));
        if(val != 0)
        {
            rt_kprintf("Read button = %x \n", val);
            tm = 0;
        }
        rt_thread_delay(RT_TICK_PER_SECOND/10);   
                
        tm++;
        if(tm > 100)
        {                      
            break;        
        }
                  
        //rt_thread_delay(RT_TICK_PER_SECOND/10);         
                
    } 
    
    err = rt_device_close(devbtn);
    rt_kprintf("Close %s test.\n", BUTTON);
    return err;

}
FINSH_FUNCTION_EXPORT(hwbutton, Test hardware button);
#endif /* RT_USING_HWBUTTON_TEST */
