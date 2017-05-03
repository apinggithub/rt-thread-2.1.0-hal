#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <stdio.h>
#include <drivers/xt8xxp8.h>
#include <drv_xt8xxp8.h>

#ifdef RT_USING_XT8XXP8_TEST

int xtp(void)
{
    rt_err_t err;   
    rt_device_t devxtp = RT_NULL;
    
    
    rt_uint8_t vcno;
    //uint8_t tm = 0;  

    //enum lcdcmd;
     
    #define XTP   "xtp"
       
    //rt_pin_mode(20, PIN_MODE_OUTPUT);// the port PF8
   
    if ((devxtp = rt_device_find(XTP)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", XTP);
        return -1;
    }
   
    if (rt_device_open(devxtp, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", XTP);
        return -1;
    }
          
    while(1)
    {
                 
        rt_thread_delay(RT_TICK_PER_SECOND);
        
        /* write the lcd */
        for(uint8_t i = 60;i < 69; i++)
        {
            vcno = 0x5A+i;     
            rt_device_write(devxtp, 0, &vcno, sizeof(vcno));
                        
            rt_kprintf("Send to the voice chip  segno = %d  \n", i);
            
            rt_thread_delay(5*RT_TICK_PER_SECOND);  
            
            //rt_thread_delay(RT_TICK_PER_SECOND);                          
                                                   
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
      
    err = rt_device_close(devxtp);
    rt_kprintf("Close %s test.\n", XTP);
   
    return err;

}
FINSH_FUNCTION_EXPORT(xtp, Test voice chip xt8xxp8);
#endif /* RT_USING_XT8XXP8_TEST */
