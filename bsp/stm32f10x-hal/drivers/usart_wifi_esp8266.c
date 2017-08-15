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
#include "cvt.h"
#include <string.h>

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

#define WIFI_LIST_AP_MAX 				5	

static struct rt_mailbox wifi_mail;
static uint32_t wifi_msg_pool[4];
static struct rt_event rev_event;
static rt_device_t wifi_device;
//rt_bool_t wifi_send(char * str);
//rt_bool_t wifi_jap(void);
rt_bool_t quit_wifi_connect(void);
static rt_uart_extent_device_t _wifi_dev;

static wifi_list_t _wifi_list_AP[WIFI_LIST_AP_MAX];
static char wifi_status[1] = {0};
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
	//uint8_t valuestr[3] = {0x00};
	//uint8_t value = 0;
  
  
  
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
				rt_mb_send(&wifi_mail,(rt_uint32_t)wifi_rx_buffer);
				WIFI_DEBUG(wifi_rx_buffer);
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
rt_bool_t wifi_send_data_package(char *cmd,char *ack,uint16_t waittime, uint8_t retrytime,char *ret)
{
	rt_bool_t res = RT_FALSE; 
	rt_err_t result = RT_EOK;
	rt_uint32_t event;
	rt_uint32_t temp_offset = 0;;
	rt_uint16_t count;
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
		//count = 3;
		temp_offset = 0;
		for(count = 100;count > 0;)
		{
			result = rt_event_recv(&rev_event, 
								REV_MASK, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								waittime*RT_TICK_PER_SECOND, &event);
			if(result == RT_EOK)
			{
				if (event & REV_DATA)
				{
					if(100 == count)
						rt_memset(wifi_rx_buffer,0x00,sizeof(wifi_rx_buffer));
					rt_device_read(wifi_device, 0, wifi_rx_buffer+temp_offset, 512);
					//WIFI_DEBUG(wifi_rx_buffer);
					if((rt_strstr(wifi_rx_buffer,ack))||(rt_strstr(wifi_rx_buffer,"OK")))
					{
						res = RT_TRUE;
						if(RT_NULL != ret)
								rt_strncpy(ret,wifi_rx_buffer,strlen(wifi_rx_buffer)); 
						count = 0;
					}
					else if(rt_strstr(wifi_rx_buffer,"ERROR"))
					{
						res = RT_FALSE;
						count = 0;
						rt_thread_delay( RT_TICK_PER_SECOND/2 );
					}
					else
					{
						count--;
						temp_offset = rt_strlen(wifi_rx_buffer);
						if(temp_offset >500)
							temp_offset = 0;
					}
					
				}
			}
			else
			{
				count = 0;
			}
			 
		}
		retrytime--;
	}while((RT_FALSE == res)&&(retrytime >= 1));
	//wifiwatch();
	return res;
} 
#if 0
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
#endif
static rt_err_t rt_esp8266_init(rt_device_t dev)
{
  return RT_EOK;
}

static rt_err_t rt_esp8266_open(rt_device_t dev,rt_uint16_t oflag)
{
	
	   ESP8266_CS_HIGH();
    return RT_EOK;
}

static rt_err_t rt_esp8266_close(rt_device_t dev)
{
     ESP8266_CS_LOW();
    return RT_EOK;
}
#if 0
static rt_size_t rt_esp8266_write(rt_device_t dev,rt_off_t pos, void *buffer, rt_size_t size)
{
	return 0;
}
static rt_size_t rt_esp8266_read(rt_device_t dev,rt_off_t pos, const void *buffer, rt_size_t size)
{
	rt_err_t result = RT_EOK;
	return 0;
}
#endif
rt_uint16_t rt_SearchInsert(char *src,char *args)
{
	
}
rt_err_t wifi_send(void *args)
{
	//char temp_buf[20]={0};
	//strcpy(temp_buf,"+++");
	rt_device_write(wifi_device, 0, ESP8266_CIPMODE_CLOSED, 3);
	//strcpy(temp_buf,"AT+CWMODE=1\x0D\x0A");
	if(!wifi_send_data_package(ESP8266_CWMODE_STA,"OK",2,3,RT_NULL))//station 模式
	{
		return RT_ERROR;
	}
	//strcpy(temp_buf,"AT+CIPMUX=0\x0D\x0A");
	if(!wifi_send_data_package(ESP8266_CIPMUX,"OK",2,3,RT_NULL))//单链接模式
	{
		return RT_ERROR;
	}
	//strcpy(temp_buf,"AT+CIPMODE=1\x0D\x0A");
	if(!wifi_send_data_package(ESP8266_CIPMODE,"OK",2,3,RT_NULL))//透传
	{
		return RT_ERROR;
	}
	//strcpy(temp_buf,"AT+CIPSTART=\"TCP\",\"192.168.8.109\",8080\x0D\x0A");
	if(!wifi_send_data_package(ESP8266_CIPSTART,"OK",2,3,RT_NULL))//连接远程服务器
	{
		return RT_ERROR;
	}
	if(!wifi_send_data_package(ESP8266_CIPSEND,">",2,3,RT_NULL))
	{
		return RT_ERROR;
	}
	wifiwatch();
  rt_device_write(wifi_device, 0, args, rt_strlen(args)); 
	
	//rt_device_write(wifi_device, 0, ESP8266_CIPMODE_CLOSED, 3);
  //rt_device_write(wifi_device, 0, temp_buf, 3);	
	return RT_EOK;
}
rt_err_t wifi_list(char *buf)
{
	rt_size_t  temp_offset = 0;
	char *list_begin = rt_strstr(buf+temp_offset,"+CWLAP:(");
  for(uint8_t i = 0; list_begin&&(i < WIFI_LIST_AP_MAX); i++)
	{
		//rt_size_t temp_offset2 = list_begin - buf + sizeof("+CWLAP:(");
		//char *list_begin = rt_strstr(buf+temp_offset,"+CWLAP:(");
		char *ssid_end = rt_strstr(list_begin+sizeof("+CWLAP:(")+3,",");
		char *rssi_end = rt_strstr(list_begin+sizeof("+CWLAP:(")+3,")");
		rt_size_t ssid_len = (rt_size_t)(ssid_end - (list_begin+sizeof("+CWLAP:(")+3));
		rt_size_t rssi_len = (rt_size_t)(rssi_end - (ssid_end+1));
		rt_strncpy(_wifi_list_AP[i].ecn,list_begin+sizeof("+CWLAP:(")-1,1);
		rt_strncpy(_wifi_list_AP[i].ssid,list_begin+sizeof("+CWLAP:(")+2,ssid_len);
		rt_strncpy(_wifi_list_AP[i].rssi,ssid_end+1,rssi_len);
		temp_offset = rssi_end - buf;
		list_begin = rt_strstr(buf+temp_offset,"+CWLAP:(");
	}	
	if( 0 == temp_offset)
		return RT_ERROR;
	else
		return RT_EOK;
}	
	
rt_err_t joap_wifi(void *args)
{
	char recv_buf[100];
	rt_device_write(wifi_device,0,ESP8266_CIPMODE_CLOSED, 3);
	//strcpy(recv_buf,"AT+CWQAP\x0D\x0A");
	if(wifi_send_data_package("AT+CWQAP\x0D\x0A","OK",3,6,RT_NULL))
	{
		if(wifi_send_data_package("AT+CWMODE=1\x0D\x0A","OK",2,3,RT_NULL))
		{
			char *ssid_end;
			char *password; 
      rt_size_t len1,len2,len3;
			//strcat(args,"\"\0x0D\0xA");
			ssid_end = rt_strstr(args,",");
			len1 =  rt_strlen(args);
			len2 =  rt_strlen(ssid_end);
			len3 = len1 - len2;
			//strcpy(args+len1,"\"\0x0D\0xA");
			strcpy(recv_buf,"AT+CWJAP=\"\",\"\"\x0D\x0A");
			strcpy(recv_buf+10,args);
		  strcpy(recv_buf+10+len3,"\",\"");
			strcpy(recv_buf+13+len3,ssid_end+1);
			strcpy(recv_buf+12+len1,"\"\x0D\x0A");
			if(wifi_send_data_package(recv_buf,"OK",2,3,RT_NULL))
				return RT_EOK;
			else
				return RT_ERROR;
		}
		else
			return RT_ERROR;
	}
	else
		return RT_ERROR;
	 
}	
	

static rt_err_t rt_esp8266_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    rt_err_t result = RT_EOK;
	  char buf[512]={0};
		#if 1
		 switch(cmd)
		 {
			 case WIFI_LIST:
				    if(wifi_send_data_package(ESP8266_CWLAPOPT,"OK",2,3,RT_NULL))//
						{	
							//strcpy(buf,"AT+CWLAP\x0D\x0A");
							wifi_send_data_package(ESP8266_CWLAP,"OK",2,3,buf);
							wifi_list(buf);
						}
						break;
			 case WIFI_JAP:
				    return(joap_wifi(args));
			 case WIFI_QAP:
				    return(!wifi_send_data_package(ESP8266_CWQAP,"OK",2,3,RT_NULL));
			 case WIFI_SEND:
				   return(wifi_send(args));
			 case WIFI_STATUS:
				    //strcpy(buf,"AT+CIPSTATUS\x0D\x0A");
						rt_device_write(wifi_device,0,ESP8266_CIPMODE_CLOSED,3);
				    if(wifi_send_data_package(ESP8266_CIPSTATUS,"OK",2,3,buf))
						{	
							char* status;
							status = rt_strstr(buf,"STATUS:");
              rt_strncpy(wifi_status,status+sizeof("STATUS:")-1,1);
							rt_mb_send(&wifi_mail,(rt_uint32_t)wifi_status);
							return RT_EOK;							
						}
				    else
							return RT_ERROR;
			 default:
				 break;
		 }			 
		#endif
    return result;
}
rt_err_t rt_hw_wifi_register(struct rt_uart_extent_device *finger_dev,
                               const char              *name,
                               rt_uint32_t              flag)
{
    struct rt_device *device;
    RT_ASSERT(finger_dev != RT_NULL);

    device = &(finger_dev->parent);

    device->type        = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

    device->init        = rt_esp8266_init;
    device->open        = rt_esp8266_open;
    device->close       = rt_esp8266_close;
    device->read        = RT_NULL;
    device->write       = RT_NULL;
    device->control     = rt_esp8266_control;
    device->user_data   = RT_NULL;

    /* register a character device */
    return rt_device_register(device, name, flag);
}

int rt_hw_wifi_init(void)
{  
	 ESP8266_CS_Init();
   _wifi_dev.uart_dev = RT_NULL;
	 
	_wifi_dev.uart_dev = rt_device_find(ESP8266_USARTx);
	wifi_device = _wifi_dev.uart_dev;
  if (_wifi_dev.uart_dev != RT_NULL)    
  {
    /* 设置回调函数及打开设备*/
    rt_device_set_rx_indicate(_wifi_dev.uart_dev, wifi_uart_input);
    rt_device_open(_wifi_dev.uart_dev, RT_DEVICE_OFLAG_RDWR| RT_DEVICE_FLAG_STREAM | RT_DEVICE_FLAG_INT_RX); 	
  }
	else
  {
    return RT_FALSE;
  }
	rt_event_init(&rev_event, "rev_ev", RT_IPC_FLAG_FIFO);
	rt_mb_init(&wifi_mail,"wifi_mail",&wifi_msg_pool,(sizeof(wifi_msg_pool))/4,RT_IPC_FLAG_FIFO);
	rt_hw_wifi_register(&_wifi_dev,"wifi_dev",RT_DEVICE_FLAG_RDWR);
	
  return RT_TRUE;
}
INIT_COMPONENT_EXPORT(rt_hw_wifi_init);


#ifdef RT_USING_FINSH
#include <finsh.h>
//FINSH_FUNCTION_EXPORT(wifi_config, connect to usart and AT test.);
//FINSH_FUNCTION_EXPORT(wifi_init, connect to usart.);
//FINSH_FUNCTION_EXPORT(wifi_scan, SACN and list AP.);
//FINSH_FUNCTION_EXPORT(wifi_jap, ESP12F join to AP.);
//FINSH_FUNCTION_EXPORT(wifi_rssi, get ESP12F current AP rssi.);
#endif /* RT_USING_FINSH */

#endif /* RT_USING_UART_WIFI_ESP8266 */
