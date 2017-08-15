/*

 */

#include <rtthread.h>
#include <rthw.h>
#include "drv_uart_finger.h"
#include <rtdevice.h>
/*#ifdef RT_USING_FINGERPRINT*/


//rt_uint8_t fp_recv_buf[509] = {0};
extern rt_uint8_t temp_uart2_recv[];
extern rt_uint16_t temp_uart_i;
//#define REV_MASK      ( REV_DATA | FP_REG | FP_CHECK )
static struct rt_event fp_recv_event;
static struct rt_mailbox fp_mail;
//rt_event_t fp_recv_event;
uint8_t fpmsg_pool[4];
int rt_hw_finger_io_init(void);
//static struct rt_event fp_reg_event;
//static struct rt_event fp_check_event;
static rt_uart_extent_device_t _finger_dev;
rt_thread_t finger_thread = RT_NULL;

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

/********************************* RT-Thread Ethernet interface begin **************************************/
static rt_err_t ta0702_fp_init(rt_device_t dev)
{
  return RT_EOK;
}

static rt_err_t ta0702_fp_open(rt_device_t dev,rt_uint16_t oflag)
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
	uint32_t temp_msg =REV_DATA;
	uint32_t* temp_msg_addr = &temp_msg;
	
	//rt_mb_send(fp_recv_mail,temp_msg);
	rt_event_send(&fp_recv_event, REV_DATA);
  return RT_EOK;
}


static rt_err_t ta0702_fp_write(rt_device_t dev)
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
	 if(8 == rt_device_write(dev,0,send_buf,8))
		 return RT_EOK;
	 else
		 return RT_ERROR;
}

rt_uint8_t fp_recv_buf[509] = {0};
static void usart_fp_data_thread_entry(void *parameter)
{
    //rt_uint32_t e;
		rt_uint32_t event;
    rt_err_t result = RT_EOK;
	  ta0702_fp_write(_finger_dev.uart_dev);
    while (1)
    {
			//struct rt_mailbox fp_recv_mail;
			#if 1
      result = rt_event_recv(&fp_recv_event,REV_DATA,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,2*RT_TICK_PER_SECOND,&event);
			//ta0702_fp_read(finger_device,FP_CMD_REG_FIRST,fp_recv_buf,8);
      if (result == RT_EOK)
      { 
				rt_event_detach(&fp_recv_event);
				//&fp_recv_event = RT_NULL;
        if (event == REV_DATA)
        {
					#if 1
					if((0x23 == temp_uart2_recv[1])&&(0 == temp_uart2_recv[4])&&(rt_fp_get_data_check(temp_uart2_recv+8,499)==temp_uart2_recv[506]))
					{
						rt_memcpy(fp_recv_buf,temp_uart2_recv,508);
						rt_memset(temp_uart2_recv,0,508);
						temp_uart_i = 0;
						rt_mb_send(&fp_mail,(rt_uint32_t)(fp_recv_buf+12));
						rt_thread_delete(finger_thread);
						finger_thread = RT_NULL;
					}
					else if((0x23 == temp_uart2_recv[1])&&(ACK_SUCCESS != temp_uart2_recv[4]))
					{
						//rt_memcpy(temp_uart2_recv,0,508);
						rt_memset(temp_uart2_recv,0,508);
						temp_uart_i = 0;
						rt_thread_delete(finger_thread);
						finger_thread = RT_NULL;
					}
					#endif
					//ta0702_fp_read(finger_device,FP_CMD_REG_FIRST,fp_recv_buf,8);
				}	
      }
      #endif
        /* receive first event */
			//ta0702_fp_read(_finger_dev.fp_uart_dev,FP_CMD_REG_FIRST,fp_recv_buf,8);
			rt_thread_delay(RT_TICK_PER_SECOND);
        
    }
}

static rt_err_t ta0702_fp_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    rt_err_t result = RT_EOK;
	  switch(cmd)
		{
			case FP_REG:
				 {
					 
					 if(RT_NULL == finger_thread)
					 {
						//fp_recv_event = *
						rt_event_init(&fp_recv_event,"fp_recv_ok",RT_IPC_FLAG_FIFO);
						finger_thread = rt_thread_create("fingerprint",
						usart_fp_data_thread_entry, RT_NULL,
						1024, 25, 7);
						/* 创建成功则启动线程*/
						if (finger_thread != RT_NULL)
							rt_thread_startup(finger_thread);
						else
							result = RT_ERROR;
					}
					}	
				break;
			default:
				break;
		}
	
	
    return result;
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

rt_err_t rt_hw_finger_register(struct rt_uart_extent_device *finger_dev,
                               const char              *name,
                               rt_uint32_t              flag)
{
    struct rt_device *device;
    RT_ASSERT(finger_dev != RT_NULL);

    device = &(finger_dev->parent);

    device->type        = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

    device->init        = ta0702_fp_init;
    device->open        = ta0702_fp_open;
    device->close       = ta0702_fp_close;
    device->read        = NULL;
    device->write       = NULL;
    device->control     = ta0702_fp_control;
    device->user_data   = NULL;

    /* register a character device */
    return rt_device_register(device, name, flag);
}



int rt_hw_finger_init(void)
{  
   _finger_dev.uart_dev = RT_NULL;
	//struct rt_finger_device fp_finger;
	
	//fp_finger.parent = *finger_device;
	rt_hw_finger_io_init();
	
	_finger_dev.uart_dev = rt_device_find(FP_DEVICE_NAME);
	
  if (_finger_dev.uart_dev != RT_NULL)    
  {
    rt_kprintf("\r\n fingerprint port initialized!\r\n");
    /* 设置回调函数及打开设备*/
    rt_device_set_rx_indicate(_finger_dev.uart_dev, fp_rx_ind);
    rt_device_open(_finger_dev.uart_dev, RT_DEVICE_OFLAG_RDWR| RT_DEVICE_FLAG_STREAM | RT_DEVICE_FLAG_INT_RX); 	
  }
	else
  {
    rt_kprintf("\r\n fingerprint port not find !\r\n");
    return RT_FALSE;
  }
	/*指纹串口打开后，初始化邮箱事件*/
	 rt_mb_init(&fp_mail,"fp_mail",&fpmsg_pool,(sizeof(fpmsg_pool))/4,RT_IPC_FLAG_FIFO);
  /*rt_event_init(&fp_rev_event, "fp_rev_ev", RT_IPC_FLAG_FIFO);*/
	rt_hw_finger_register(&_finger_dev,"finger_dev",RT_DEVICE_FLAG_RDWR);
	
  return RT_TRUE;
}
INIT_COMPONENT_EXPORT(rt_hw_finger_init);
//INIT_DEVICE_EXPORT(rt_hw_finger_io_init);
//#endif
