/*

 */

#include <rtthread.h>
#include <rthw.h>
#include "drv_uart_finger.h"
#include <rtdevice.h>
/*#ifdef RT_USING_FINGERPRINT*/

#define REV_DATA      0x01
#define FP_REG        0x02
#define FP_SUCESS     0x04
#define FP_FAILD      0x08
//rt_uint8_t fp_recv_buf[509] = {0};
extern rt_uint8_t temp_uart2_recv[];
extern rt_uint16_t temp_uart_i;
//#define REV_MASK      ( REV_DATA | FP_REG | FP_CHECK )
static struct rt_event fp_rev_event;
static struct rt_mailbox fp_mail;
int rt_hw_finger_io_init(void);
//static struct rt_event fp_reg_event;
//static struct rt_event fp_check_event;
struct uart_finger_device
{   
	  rt_device_t                		  parent;
		rt_err_t  (*open)   						(rt_device_t dev);
    rt_err_t  (*close)  						(rt_device_t dev);
}uart_finger_device_t;
static rt_device_t finger_device = NULL;

rt_uint8_t rt_fp_get_data_check(rt_uint8_t *buf,rt_uint16_t len)
{
	rt_uint8_t check_data1 = *(buf+1);
	rt_uint16_t i = 2;
	for(;i < len - 1 ;i++)
	{
		check_data1 = check_data1^*(buf+i);
	}
	return check_data1;
}

rt_device_t rt_finger_get_device(void)
{
    return finger_device;
}
RTM_EXPORT(rt_finger_get_device);

/**
 * This function will set a device as console device.
 * After set a device to console, all output of rt_kprintf will be
 * redirected to this new device.
 *
 * @param name the name of new console device
 *
 * @return the old console device handler
 */
rt_device_t rt_finger_set_device(const char *name)
{
    rt_device_t new, old;

    /* save old device */
    old = finger_device;

    /* find new console device */
    new = rt_device_find(name);
    if (new != RT_NULL)
    {
        if (finger_device != RT_NULL)
        {
            /* close old console device */
            rt_device_close(finger_device);
        }
        /* set new console device */
        rt_device_open(new, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_STREAM | RT_DEVICE_FLAG_INT_RX);
        finger_device= new;
    }

    return old;
}
RTM_EXPORT(rt_console_set_device);



/********************************* RT-Thread Ethernet interface begin **************************************/
static rt_err_t ta0702_fp_init(rt_device_t dev)
{
		//rt_serial_init(dev);
	//uint8_t fp_recv_buf[507];
	//rt_memset(fp_recv_buf,0x00,sizeof(fp_recv_buf));
	//rt_device_read(finger_device, 0, fp_recv_buf, 507);
	//rt_event_send(&fp_rev_event, REV_DATA);
  return RT_EOK;
}

static rt_err_t ta0702_fp_open(rt_device_t dev)
{
	
	  FP_POWER_ON();
    return RT_EOK;
}

static rt_err_t ta0702_fp_close(rt_device_t dev)
{
	  FP_POWER_OFF();
	  FP_CRTL_ON();
	  rt_thread_delay( RT_TICK_PER_SECOND/2 );
		FP_CRTL_OFF();
    return RT_EOK;
}

static rt_err_t fp_rx_ind(rt_device_t dev, rt_size_t size)
{
    //RT_ASSERT(shell != RT_NULL);

    /* release semaphore to let finsh thread rx data */
    //rt_sem_release(&_finger_device->rx_sem);
	 //ta0702_fp_read(finger_device,FP_CMD_REG_FIRST,fp_recv_buf,507);
	uint32_t temp_msg =REV_DATA;
	uint32_t* temp_msg_addr = &temp_msg;
	
	rt_mb_send(&fp_mail,temp_msg);
	//rt_event_send(&fp_rev_event, REV_DATA);

    return RT_EOK;
}


static rt_size_t ta0702_fp_write(rt_device_t dev)
{
	
	//rt_err_t result = RT_EOK;
	
	rt_uint8_t send_buf[8];
	send_buf[0] = 0xf5;
	 send_buf[1] = FP_CMD_GET_FP;
	 send_buf[2] = 0x00;
	 send_buf[3] = 0x00;
	 send_buf[4] = 0x00;
	 send_buf[5] = 0x00;
	 send_buf[6] = FP_CMD_GET_FP;
	 send_buf[7] = 0xf5;
	 rt_device_write(dev,0,send_buf,8);
	//switch(cmd):
		//case 
		
	  ///t_fp_tx();
    //rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t ta0702_fp_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    rt_err_t result = RT_EOK;
    return result;
}



static void usart_fp_data_thread_entry(void *parameter)
{
    //rt_uint32_t e;
		rt_uint32_t event;
    rt_err_t result = RT_EOK;
    //rt_err_t result_reg = RT_EOK;
	  //rt_err_t result_check = RT_EOK;
	  rt_uint8_t check_flag = 0;
	  rt_uint8_t reg_flag = 0;
	  rt_uint32_t fp_mail_temp;
		uint8_t fpmsg_pool[4];
    rt_uint8_t fp_recv_buf[509] = {0};
	  //rt_mailbox_t rt_mb_create ("fp_mail", rt_size_t size, rt_uint8_t flag);
		rt_mb_init(&fp_mail,"fp_mail",&fpmsg_pool,(sizeof(fpmsg_pool))/4,RT_IPC_FLAG_FIFO);
	  //rt_hw_finger_io_init();
		//ta0702_fp_read(finger_device,FP_CMD_REG_FIRST,fp_recv_buf,8);
	  ta0702_fp_write(finger_device);
    while (1)
    {
	    //ta0702_fp_write(finger_device);
			#if 1
      result = rt_mb_recv(&fp_mail,&event,2*RT_TICK_PER_SECOND);
			//ta0702_fp_read(finger_device,FP_CMD_REG_FIRST,fp_recv_buf,8);
      if (result == RT_EOK)
      { 
				
        if (event == REV_DATA)
        {
					#if 0
					if((0x23 == fp_recv_buf[1])&&(0 == fp_recv_buf[4])&&(0xf5 == fp_recv_buf[507]))
					{
						/*uint16_t i = 0;
						for(;i<508;i++)
						{
							fp_recv_buf[i] = temp_uart2_recv[i];
							temp_uart2_recv[i] = 0;	
						}*/
						rt_memcpy(fp_recv_buf,temp_uart2_recv,508);
						rt_memset(temp_uart2_recv,0,508);
						temp_uart_i = 0;
						rt_mb_send(&fp_mail,(rt_uint32_t)(&fp_recv_buf));
					}
					else if((0x23 == fp_recv_buf[1])&&(ACK_SUCCESS != fp_recv_buf[4]))
					{
						//rt_memcpy(temp_uart2_recv,0,508);
						rt_memset(temp_uart2_recv,0,508);
						temp_uart_i = 0;
					}
          /*//rt_memset(gprs_rx_buffer,0x00,sizeof(gprs_rx_buffer));
					//fp_count++;
					if((0xf5 == fp_recv_buf[0])&&(0 == fp_recv_buf[1]))
						rt_device_read(finger_device,0,fp_recv_buf+1,507);
					else
						rt_device_read(finger_device,0,fp_recv_buf,507);
					if((0x23 == fp_recv_buf[1])&&(0 == fp_recv_buf[4])&&(0xf5 == fp_recv_buf[507]))
						rt_mb_send(&fp_mail,(rt_uint32_t)(&fp_recv_buf));
          //rt_thread_delay(RT_TICK_PER_SECOND*2);*/
					#else
					rt_device_read(finger_device,0,fp_recv_buf,508);
          rt_kprintf("xxxx");
					#endif
					//ta0702_fp_read(finger_device,FP_CMD_REG_FIRST,fp_recv_buf,8);
				}
        else if (event == FP_REG)
        {
          ta0702_fp_write(finger_device);
        }
				else
				{
					return;
				}
				
      }
      #endif
        /* receive first event */
			//ta0702_fp_read(finger_device,FP_CMD_REG_FIRST,fp_recv_buf,8);
			rt_thread_delay(RT_TICK_PER_SECOND/5);
        
    }
}

int rt_hw_finger_io_init(void)
{
		GPIO_InitTypeDef GPIO_InitStruct;
	
		__HAL_RCC_GPIOC_CLK_ENABLE();
		
    GPIO_InitStruct.Pin = FP_POWER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FP_POWER_PORT, &GPIO_InitStruct);
	
		GPIO_InitStruct.Pin = FP_CRTL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FP_CRTL_PORT, &GPIO_InitStruct);
	
		HAL_GPIO_WritePin(FP_POWER_PORT,FP_POWER_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(FP_CRTL_PORT,FP_CRTL_PIN,GPIO_PIN_RESET);
	
	
	/*Configure GPIO pin : CTP_INT_Pin */
	GPIO_InitStruct.Pin = FP_OUT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(FP_OUT_PORT, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	return 0;
}


int rt_hw_finger_init(void)
{  

	struct uart_finger_device fp_finger;
	fp_finger.parent = finger_device;
	fp_finger.open   = ta0702_fp_open;
	fp_finger.close  = ta0702_fp_close;	
	rt_hw_finger_io_init();
	finger_device = rt_device_find("uart2");
	
	
  if (finger_device != RT_NULL)    
  {
    rt_kprintf("\r\n fingerprint port initialized!\r\n");
    /* 设置回调函数及打开设备*/
    rt_device_set_rx_indicate(finger_device, fp_rx_ind);
    rt_device_open(finger_device, RT_DEVICE_OFLAG_RDWR| RT_DEVICE_FLAG_STREAM | RT_DEVICE_FLAG_INT_RX); 	
  }
	else
  {
    rt_kprintf("\r\n fingerprint port not find !\r\n");
    return RT_FALSE;
  }
	/*指纹串口打开后，初始化串口接收事件*/
  /*rt_event_init(&fp_rev_event, "fp_rev_ev", RT_IPC_FLAG_FIFO);*/
	{
		rt_thread_t thread = rt_thread_create("fingerprint",
		usart_fp_data_thread_entry, RT_NULL,
		1024, 25, 7);
		
		
		/* 创建成功则启动线程*/
		if (thread != RT_NULL)
			rt_thread_startup(thread);
	}
  return RT_TRUE;
}
INIT_COMPONENT_EXPORT(rt_hw_finger_init);
//INIT_DEVICE_EXPORT(rt_hw_finger_io_init);
//#endif
