/***************************2012-2016, NJUT, Edu.******************************* 
FileName: esp8266.c 
Author:  孙冬梅       Version :  1.0       Date: 2016.11.26
Description:   wifi模块通信
Version:         1.0
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    16/11/10    1.0       文件创建  

Description：设备初始化，查找wifi串口(UART4)设备打开，注册回调函数，初始化串口接收事件。
开启线程，监视串口数据。
发送AT指令时，关闭串口监视线程。
串口收到数据后，运行数据到达回调函数,发送事件到wifi_send_data_package。
wifi_send_data_package中接收数据，并检查数据。
使用  PC10-USART4Tx PC11-USART4Rx CS-PC9
Others:   串口接收数据后，检测 关键字{"value":**}来获取 GET方法得到的数据
  Function List:  
   1. wificonfig() 查找wifi串口设备并打开，注册回调函数，初始化串口接收事件
   2. wifiinit(); wifi设置
   3. wifijap() ；加入AP
   4. wificonnect(); 打开远程连接网络
   5. wifisend("abc"); 网络数据发送
   6. wificloseconnect(); 关闭远程连接网络
   7. wifiexit();  wifi退出AP 并关闭
*******************************************************************************/ 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include  <rtthread.h>
#include "drivers/pin.h"
#include "drivers/serial.h"
#include "drv_led.h"
#include "usart_wifi_esp8266.h"

#ifdef RT_USING_UART_WIFI_ESP8266


/***************************WIFI模块串口接收數據事件******************************/
#define REV_DATA      0x01
#define REV_WATCH      0x02
#define REV_STOPWATCH      0x04
#define JOIN_AP      0x08

#define REV_MASK      ( REV_DATA | REV_WATCH | REV_STOPWATCH|JOIN_AP )

//#define WIFI_DEBUG_ON

#ifdef WIFI_DEBUG_ON
#define WIFI_DEBUG         rt_kprintf("[ESP-12F] ");rt_kprintf
//#define USART_DEBUG         rt_kprintf("[USART] ");rt_kprintf
#define USART_DEBUG(...)
#else
#define WIFI_DEBUG(...)
#define WSART_DEBUG(...)
#endif /* #ifdef WIFI_DEBUG_ON */

static struct rt_event rev_event;
static rt_device_t wifi_device;
rt_bool_t wifi_send(char * str);
rt_bool_t wifi_jap(void);
rt_bool_t quit_wifi_connect(void);

/* 监视WIFI串口线程入口*/
void wifiwatch_entry(void* parameter)
{
	rt_err_t result = RT_EOK;
	rt_uint32_t event;
	char wifi_rx_buffer[512]={0x00};
	rt_size_t  readnum;
	char * charaddr;
	char * charstartaddr;
	char *  charendaddr;
	uint8_t valuestr[3] = {0x00};
	uint8_t value = 0;
  
  
  
	while(1)
	{
		result = rt_event_recv(&rev_event,	  
								REV_MASK, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &event);
		if (result == RT_EOK)
		{
			if (event & REV_DATA)
			{
				rt_memset(wifi_rx_buffer,0x00,sizeof(wifi_rx_buffer));
				rt_thread_delay(RT_TICK_PER_SECOND/2);
				readnum = rt_device_read(wifi_device, 0, wifi_rx_buffer, 512);
				WIFI_DEBUG(wifi_rx_buffer);
				
				/*以下获取串口接收数据 中的 value值 */
				/*charaddr = rt_strstr(wifi_rx_buffer,"\"value\":");
				if(charaddr!=RT_NULL)
				{
					WIFI_DEBUG(charaddr);    
					charstartaddr = charaddr + 8;
					charendaddr = rt_strstr(charaddr,"}") ;
					if(charendaddr!=RT_NULL)
					{
						int i=0;
						while (charstartaddr!=charendaddr)
						{
							valuestr[i] = *charstartaddr;
							charstartaddr++;i++;
						}
						value = atoi((char const*)valuestr);
						WIFI_DEBUG("\r\n Wifi Device receive value = %d \r\n",value );
						if(value==1)
						{
							//LEDOn(LED1);
							LED1_ON();
						}
						else if(value==0)
						{
							//LEDOff(LED1);
							LED1_OFF(); 
						}              
					}
				}*/
				/*获取串口接收数据 中的 value值 结束 */
			}
			if (event & REV_STOPWATCH)
			{
				return;
			}
		}
    }
}

void wifiwatch(void)
{
	/* 创建wifi watch线程*/
	rt_thread_t thread = rt_thread_create("wifiwatch",
										wifiwatch_entry, RT_NULL,
										1024, 25, 7);
  
	/* 创建成功则启动线程*/
	if (thread != RT_NULL)
	{
		rt_thread_startup(thread);
	}
}
void wifistopwatch(void)
{
	rt_event_send(&rev_event, REV_STOPWATCH);
}

/*数据到达回调函数,发送事件到wifi_send_data_package*/
static rt_err_t wifi_uart_input(rt_device_t dev, rt_size_t size)
{
	rt_event_send(&rev_event, REV_DATA);
	return RT_EOK;
}

/*WIFI串口发送和接收*/
rt_bool_t wifi_send_data_package( char *cmd,char *ack,uint16_t waittime, uint8_t retrytime)
{
	rt_bool_t res = RT_FALSE; 
	rt_err_t result = RT_EOK;
	rt_uint32_t event;
	char wifi_rx_buffer[512]={0x00};
	rt_thread_t thread;	
  
	thread = rt_thread_find("wifiwatch");
	if( thread != RT_NULL)
	{
		rt_thread_delete(thread);
	}
  
	do 
	{
		rt_device_write(wifi_device, 0, cmd, rt_strlen(cmd));   
		
		result = rt_event_recv(&rev_event, 
							REV_MASK, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
							waittime*RT_TICK_PER_SECOND, &event);
		if (result == RT_EOK)
		{
			if (event & REV_DATA)
			{
				rt_memset(wifi_rx_buffer,0x00,sizeof(wifi_rx_buffer));
				rt_device_read(wifi_device, 0, wifi_rx_buffer, 512);
				//WIFI_DEBUG(wifi_rx_buffer);
				if((rt_strstr(wifi_rx_buffer,ack))||(rt_strstr(wifi_rx_buffer,"OK")))
				{
					res = RT_TRUE;
				}
				else
				{
					res = RT_FALSE;
					rt_thread_delay( RT_TICK_PER_SECOND);
				}
			}
		}
		retrytime--;
	}while((RT_FALSE == res)&&(retrytime >= 1));
	wifiwatch();
	return res;
} 
void wifijap_entry(void* parameter)
{
	rt_err_t result = RT_EOK;
	rt_uint32_t event;
	while(1)
	{
		result = rt_event_recv(&rev_event,	  
								JOIN_AP, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &event);
		if (result == RT_EOK)
		{
			if (event & JOIN_AP)
			{
					wifi_jap();
				  rt_thread_t	thread = rt_thread_find("wifijap");
					if( thread != RT_NULL)
					{
						rt_thread_delete(thread);
					}
			}
		}
		
		rt_thread_delay( RT_TICK_PER_SECOND);		
	}
}



/*WIFI端口初始化，打开设备，注册回调函数*/
rt_bool_t wifi_config(void)
{
	ESP8266_CS_Init();
	ESP8266_CS_HIGH();

	wifi_device = rt_device_find(ESP8266_USARTx);//need to find the usart and open it
  
	if (wifi_device != RT_NULL)    
	{		
		/* 设置回调函数及打开设备*/
		rt_device_open(wifi_device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_STREAM | RT_DEVICE_FLAG_INT_RX); 
		rt_device_set_rx_indicate(wifi_device, wifi_uart_input);
				
		WIFI_DEBUG("\r\n Wifi port initialized!\r\n");
	}
	else
	{
		WIFI_DEBUG("\r\n Wifi port not find !\r\n");
		return RT_FALSE;
	}
	/* WIFI串口打开后，初始化串口接收事件 */
	rt_event_init(&rev_event, "rev_ev", RT_IPC_FLAG_FIFO);
	
	WIFI_DEBUG("\r\n Wifi link AT cmd test ... \r\n");
	//quit_wifi_connect();
	if(wifi_send_data_package(ESP8266_ATCMD,"OK",3,3))
	{
		WIFI_DEBUG("\r\n Wifi AT test OK !\r\n");
		//return RT_TRUE;
		wifi_jap();
		{
			//rt_thread_t thread;	
  
			/* 创建wifi watch线程*/
			rt_thread_t thread = rt_thread_create("wifijap",
										wifijap_entry, RT_NULL,
										1024, 25, 7);
  
			/* 创建成功则启动线程*/
			if (thread != RT_NULL)
			{
				rt_thread_startup(thread);
				rt_event_send(&rev_event, JOIN_AP);
			}
		}
	}
	else
	{
		WIFI_DEBUG("\r\n Wifi AT test Faild !\r\n");
		rt_device_close(wifi_device);
		return RT_FALSE;
	}
	
	return RT_TRUE;	
}

rt_bool_t wifi_init(void) //wifi接入AP
{
	 
	if(wifi_send_data_package(ESP8266_CWMODE_STA,"OK",2,1))
	{
		WIFI_DEBUG("\r\n Wifi AT OK !\r\n");
	}

	WIFI_DEBUG("\r\n Wifi Reset! Display information from module:\r\n");
	if(wifi_send_data_package(ESP8266_RESET,"ready",4,1))
	{
		WIFI_DEBUG("\r\n Wifi Reset OK !\r\n");
	}
	rt_thread_delay(RT_TICK_PER_SECOND*5);
 
	//rt_thread_delay(RT_TICK_PER_SECOND*5);
  
	//WIFI_DEBUG("\r\n Wifi  当前AP：\r\n");
	//if(wifi_send_data_package(ESP8266_CWLAP,"OK",1,1))//列出当前接入点
	// {
	//   rt_device_write(wifi_device, 0, ESP8266_CWLAP, rt_strlen(ESP8266_CWLAP)); 
	// }
	// rt_thread_delay(RT_TICK_PER_SECOND*10);
	//  else
	//    return RT_FALSE; 
  
  
	//  if(wifi_send_data_package(ESP8266_CIPSTATUS,"OK",2,1))// 获得TCP/UDP连接状态
	//  {
	//  }
	//  WIFI_DEBUG("\r\n Wifi 获得TCP/UDP连接状态");
	//  rt_thread_delay(RT_TICK_PER_SECOND*1);
	//  else
	//    return RT_FALSE; 
    return RT_TRUE; 
}

rt_bool_t wifi_jap(void) //加入AP
{
	
	quit_wifi_connect();
	/*if(wifi_send_data_package(ESP8266_CWQAP,"OK",2,6))
	{
		WIFI_DEBUG("\r\n Wifi connect to tcp server OK  !\r\n");
	}
	else
	{
		WIFI_DEBUG("\r\n Wifi connect to tcp server faild  !\r\n");
		goto END;
	}*/
	
	WIFI_DEBUG("\r\n Wifi will join into AP \r\n");
	if(wifi_send_data_package(ESP8266_CWMODE_APSTA,"OK",2,6))
	{
		WIFI_DEBUG("\r\n Wifi STA and AP OK  !\r\n");
	}
	else
	{
		WIFI_DEBUG("\r\n Wifi STA and AP faild  !\r\n");
		goto END;
	}
	if(wifi_send_data_package(ESP8266_CWLAP,"OK",2,6))
	{
		WIFI_DEBUG("\r\n Wifi list AP OK  !\r\n");
	}
	else
	{
		WIFI_DEBUG("\r\n Wifi list AP faild  !\r\n");
		goto END;
	}
	rt_thread_delay( RT_TICK_PER_SECOND*3 );
	if(wifi_send_data_package(ESP8266_CWQAP,"OK",2,6))
	{
		WIFI_DEBUG("\r\n Wifi connect to tcp server OK  !\r\n");
	}
	
	if(wifi_send_data_package(ESP8266_CWJAP,"OK",5,6))
	{
		rt_thread_delay( RT_TICK_PER_SECOND/100);
		//WIFI_DEBUG("\r\n Wifi join AP OK  !\r\n");
	}
	else
	{
		if(wifi_send_data_package(ESP8266_CWJAP,"busy",5,6))
		{
			WIFI_DEBUG("\r\n Wifi join AP already  !\r\n");
		}
		else
		{
			WIFI_DEBUG("\r\n Wifi join AP faild  !\r\n");
			goto END;
		}
		//WIFI_DEBUG("\r\n Wifi join AP faild  !\r\n");
		//goto END;
	}
	rt_thread_delay( RT_TICK_PER_SECOND*2 );
	if(wifi_send_data_package(ESP8266_CIFSR,"OK",10,12))
	{
		//WIFI_DEBUG("\r\n Wifi get host ip OK  !\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/100);
	}
	else
	{
		//WIFI_DEBUG("\r\n Wifi get host ip faild  !\r\n");
		goto END;
	}
	rt_thread_delay( RT_TICK_PER_SECOND*3 );
	/*if(wifi_send_data_package(ESP8266_CIPEXITSERVER,"OK",2,6))
	{
		//WIFI_DEBUG("\r\n Wifi set single link OK  !\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/100);
	}*/
	if(wifi_send_data_package(ESP8266_CIPMUX,"OK",2,6))
	{
		//WIFI_DEBUG("\r\n Wifi set single link OK  !\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/100);
	}
	else
	{
		//WIFI_DEBUG("\r\n Wifi set single link faild  !\r\n");
		goto END;
	}
	rt_thread_delay( RT_TICK_PER_SECOND/10 );
	if(wifi_send_data_package(ESP8266_CIPMODE,"OK",2,6))
	{
		//WIFI_DEBUG("\r\n Wifi set SerialNet OK  !\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/100);
	}
	else
	{
		//WIFI_DEBUG("\r\n Wifi set SerialNet faild  !\r\n");
		goto END;
	}
	rt_thread_delay(RT_TICK_PER_SECOND);
	if(wifi_send_data_package(ESP8266_CIPSTART,"OK",10,6))
	{
		//WIFI_DEBUG("\r\n Wifi connect to tcp server OK  !\r\n");
		rt_thread_delay( RT_TICK_PER_SECOND/100);
	}
	else
	{
		//WIFI_DEBUG("\r\n Wifi connect to tcp server faild  !\r\n");
		goto END;
	}
	rt_thread_delay(RT_TICK_PER_SECOND);
	if(wifi_send_data_package(ESP8266_CIPSEND,">",2,6))
	{
		char send_buf[8] = "AASDFGHJ";
		//WIFI_DEBUG("\r\n Wifi is ready to send data !\r\n");
		//wifi_send(send_buf);
		wifi_send_data_package("AASDFGHJ","OK",3,1);
	}
	else
	{
		//WIFI_DEBUG("\r\n Wifi is not ready to send data !\r\n");
		goto END;
	}
	rt_thread_delay(RT_TICK_PER_SECOND*5);
	return RT_TRUE; 
END:
	WIFI_DEBUG("\r\n Wifi closed !\r\n");
	rt_device_close(wifi_device);
	return RT_FALSE;

}

rt_bool_t wifi_connect(void) //打开远程连接网络
{
	WIFI_DEBUG("\r\n Wifi Connect %s \r\n",ESP8266_CIPSTART);

	if(wifi_send_data_package(ESP8266_CIPSTART,"OK",2,1))
	{
		WIFI_DEBUG("\r\n Wifi connect OK !\r\n");
	}

    rt_thread_delay(RT_TICK_PER_SECOND*5);
    return RT_TRUE; 
}

rt_bool_t wifi_send(char * str) //网络数据发送
{
	uint8_t lenstr[64]={0x00};
	
	//WIFI_DEBUG("\r\n Wifi send data! Display information from module:\r\n");
	//sprintf((char*)lenstr,"%s%d\r\n",ESP8266_CIPSEND,strlen(str));
	if(wifi_send_data_package((char*)lenstr,"OK",4,6))
	{
		//WIFI_DEBUG("\r\n Wifi send %d data  OK !\r\n",strlen(str));
	}

	if(wifi_send_data_package(str,"OK",4,6))
	{
	}
	rt_thread_delay(RT_TICK_PER_SECOND*5);
  
    return RT_TRUE; 
}


rt_bool_t wifi_closeconnect(void)// 关闭远程连接网络
{
	WIFI_DEBUG("\r\n Wifi closeconnect! Display information from module:\r\n");

	if(wifi_send_data_package(ESP8266_CIPCLOSE,"OK",3,1))
	{
		WIFI_DEBUG("\r\n Wifi closeconnect OK !\r\n");
	}
    rt_thread_delay(RT_TICK_PER_SECOND*5);

    return RT_TRUE; 
}

rt_bool_t quit_wifi_connect(void)// 关闭透传模式
{
	if(wifi_send_data_package(ESP8266_CIPMODE0,"OK",3,1))
	{
		//WIFI_DEBUG("\r\n Wifi closeconnect OK !\r\n");
		rt_thread_delay(RT_TICK_PER_SECOND/10000);
	}
    //rt_thread_delay(RT_TICK_PER_SECOND*5);

    return RT_TRUE; 
}






rt_bool_t wifi_exit(void) // wifi退出AP
{
	WIFI_DEBUG("\r\n Wifi wifi exit! Display information from module:\r\n");

	wifi_device->close(wifi_device);
	if(wifi_send_data_package(ESP8266_CWQAP,"OK",2,1))
	{
		WIFI_DEBUG("\r\n Wifi exit OK !\r\n");
	}
	else
	{
		return RT_FALSE; 
	}
    return RT_TRUE; 
}

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(wifi_config, connect to usart and AT test.);
//FINSH_FUNCTION_EXPORT(wifi_init, connect to usart.);
//FINSH_FUNCTION_EXPORT(wifi_scan, SACN and list AP.);
FINSH_FUNCTION_EXPORT(wifi_jap, ESP12F join to AP.);
//FINSH_FUNCTION_EXPORT(wifi_rssi, get ESP12F current AP rssi.);
#endif /* RT_USING_FINSH */

#endif /* RT_USING_UART_WIFI_ESP8266 */
