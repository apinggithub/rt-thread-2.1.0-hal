#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include <stdio.h>
#include <drivers/hwadc.h>
#include <drv_hwadc.h>
#include "stm32f1xx_hal.h"

//#ifdef RT_USING_HWADC_TEST

rt_err_t hwadc(void)
{
    rt_err_t err;   
    rt_device_t dev = RT_NULL;
        
    uint16_t Current_Temperature;
    uint8_t delay_tm = 0;  
    __IO uint32_t recv_adc[TOTAL_CHANNELS]; //*sizeof(uint16_t)
    float adc[TOTAL_CHANNELS];//*sizeof(uint16_t)

    //enum lcdcmd;
     
    #define HWADC   "hwadc"
       
    //rt_pin_mode(20, PIN_MODE_OUTPUT);// the port PF8
   
    if ((dev = rt_device_find(HWADC)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", HWADC);
        return -1;
    }
   
    if (rt_device_open(dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", HWADC);
        return -1;
    }
    rt_device_hwadc_t *devadc = (rt_device_hwadc_t *)dev;
    
    if(RT_EOK != rt_device_control(dev, HWTADC_CTRL_START, RT_NULL))
    {
        //rt_device_read(dev, 0, &recv_adc, sizeof(uint32_t));
        while(1);
    }
    
    
    #if 1
    //rt_device_read(devhwadc, 0, &adc, sizeof(adc[2]));
    //adcrv = rt_device_read(dev, 0, (float *)&adc, TOTAL_CHANNELS*sizeof(adc[0]));      
    while(1)
    {
          
        rt_thread_delay(RT_TICK_PER_SECOND);       
        rt_device_read(dev, 0, (void*)&recv_adc, TOTAL_CHANNELS);  
        for(uint8_t i = 0; i < TOTAL_CHANNELS; i++)
        {        
            adc[i] = ((float)(recv_adc[i]&0x0FFF)*3.3/4096);
            rt_kprintf("adc[%d]  value = %d -> %fV \r\n",i,recv_adc[i]&0x0FFF,adc[i]); 
        }
        //adc[1] = ((float)(recv_adc[1]&0x0FFF)*3.3/4096);
        //rt_kprintf("adc[1]  value = %d -> %fV \r\n",recv_adc[1]&0x0FFF,adc[1]); 
        //adc[2] = ((float)(recv_adc[2]&0x0FFF)*3.3/4096);
        //rt_kprintf("adc[2]  value = %d -> %fV \r\n",recv_adc[2]&0x0FFF,adc[2]);
        //adc[3] = ((float)(recv_adc[3]&0x0FFF)*3.3/4096);
        //rt_kprintf("adc[3]  value = %d -> %fV \r\n",recv_adc[3]&0x0FFF,adc[3]);
        //Current_Temperature = (V25-recv_adc[1]&0x0FFF)/AVG_SLOPE+25;	
        //rt_kprintf("The IC current temperature = %d->%3d 'C \r\n",recv_adc[1],Current_Temperature); 
        
        
        /* print the adc value */ 
        //rt_kprintf("The ADC_IN0 = %f  \n",adc[0]);
        //adcrv = rt_device_read(dev, 0, (float *)&adc, TOTAL_CHANNELS*sizeof(adc[0]));
        
        //adc[0] = ((float)(devadc->adc_converted_value[0]&0xFFF)*3.3/4096);
        //adc[1] = ((float)(devadc->adc_converted_value[1]&0xFFF)*3.3/4096);
        //if(TOTAL_CHANNELS*sizeof(adc[0]) == adcrv)   
        //{            
        //    rt_kprintf("The ADC_IN0 = %f  \r\n",adc[0]);
        //    rt_kprintf("The ADC_IN1 = %f  \r\n",adc[1]);  
            //rt_kprintf("The ADC_IN OK  \r\n");
        //}
        //else
        //{
            //rt_kprintf("the adc buffer size is %d error ! \r\n" ,adcrv);
        //}
        //rt_kprintf("The ADC_IN0 = %d  \r\n",devadc->adc_converted_value[0]);
        //rt_kprintf("The ADC_IN1 = %d  \r\n",devadc->adc_converted_value[1]);
        
        //rt_thread_delay(1*RT_TICK_PER_SECOND);  
        
        //rt_thread_delay(RT_TICK_PER_SECOND);                          
                                                             
        delay_tm++;
        if(delay_tm > 10)
        {    
            delay_tm = 0;  
            rt_kprintf("the adc test is over! \r\n");              
            break;        
        }                            
    }    
    #endif
    
      
    err = rt_device_close(dev);
    rt_kprintf("Close %s test.\n", HWADC);
   
    return err;

}
FINSH_FUNCTION_EXPORT(hwadc, Test the adc convert);
//#endif /* RT_USING_XT8XXP8_TEST */
