/************************************************* 
  Copyright (C), 2012, NJUT
  File name:      gprsio.c
  Author:  sundm     Version:  1.0      Date: 2013.1.3 
  Description:    sundm GPRS模块驱动接口函数 
  Others:        采用底层驱动，填充函数  
  Function List:  
*************************************************/ 

#ifndef _ESP8266_H_
#define _ESP8266_H_

#include <rtthread.h>
#include "stm32f1xx.h"

//#define ESP8266_RCC_GPIO_CLK_ENABLE()  	__HAL_RCC_GPIOB_CLK_ENABLE()
//#define ESP8266_CS_PORT              	GPIOB
//#define ESP8266_CS_PIN               	GPIO_PIN_4

#define PB4 					  90 // PB4 Pin is NO. 90 @STM32F103VET6

#define	ESP8266_CS_Init() 		rt_pin_mode(PB4, PIN_MODE_OUTPUT)
#define ESP8266_CS_LOW()		rt_pin_write(PB4,PIN_LOW)
#define ESP8266_CS_HIGH()		rt_pin_write(PB4,PIN_HIGH)

#define ESP8266_USARTx 		"uart3"

//定义AT指令

#define WIFI_LIST   0x20
#define WIFI_JAP    0x21
#define WIFI_QAP    0x22
#define WIFI_SEND   0x23
#define WIFI_STATUS 0x24


#if 1
#define ESP8266_ATCMD "AT\x0D\x0A"               // AT查询
#define ESP8266_RESET "AT+RST\x0D\x0A"           // 模块复位

#define ESP8266_CWMODE_STA "AT+CWMODE=1\x0D\x0A"      // 选择WiFi应用模式  Station模式
#define ESP8266_CWMODE_AP "AT+CWMODE=2\x0D\x0A"      // 选择WiFi应用模式  AP模式
#define ESP8266_CWMODE_APSTA "AT+CWMODE=3\x0D\x0A"      // 选择WiFi应用模式  Station+AP模式

#define ESP8266_CWLAP "AT+CWLAP\x0D\x0A"      // 列出当前接入点
#define ESP8266_CWQAP "AT+CWQAP\x0D\x0A"      // 退出
#define ESP8266_CIFSR "AT+CIFSR\x0D\x0A"      // 获取本地IP地址
//#define ESP8266_CWJAP "AT+CWJAP=\"lzt02\",\"lzt123456\"\x0D\x0A"

#define ESP8266_CWJAP "AT+CWJAP=\"lzt02\",\"lzt123456\"\x0D\x0A"      // 加入接入点 TP-LINK_sundm
#define ESP8266_CIPEXITSERVER "AT+CIPSERVER=0\x0D\x0A"      // 退出server
#define ESP8266_CIPMUX "AT+CIPMUX=0\x0D\x0A"      // 设置单连接
#define ESP8266_CIPMODE "AT+CIPMODE=1\x0D\x0A"      // 设置透传模式

#define ESP8266_CIPSTART "AT+CIPSTART=\"TCP\",\"192.168.2.102\",8080\x0D\x0A"      // 建立TCP/UDP连接
#define ESP8266_CIPSTATUS "AT+CIPSTATUS\x0D\x0A"      // 获得TCP/UDP连接状态
#define ESP8266_CIPSEND "AT+CIPSEND\x0D\x0A"      // 发送数据

#define ESP8266_CIPCLOSE "AT+CIPCLOSE\x0D\x0A"      // 关闭TCP/UDP连接
#define ESP8266_CIPMODE_CLOSED "+++"      // 关闭透传
#define ESP8266_CWLAPOPT "AT+CWLAPOPT=0,7\0x0D\x0A"                   //设置cwlap的显示属性 1(0)(不)根据信号强度排序(bit6)(bit5)(bit4)ch(bit3)mac(bit2)信号强度(bit1)ssid(bit0)加密方式
#endif

typedef struct wifi_list{
	char ecn[1];
	char ssid[32];
	char rssi[4];
}wifi_list_t;



#if 0
#define ESP8266_ATCMD         0x01               // AT查询
#define ESP8266_RESET         0x02           // 模块复位

#define ESP8266_CWMODE_STA    0x03      // 选择WiFi应用模式  Station模式
#define ESP8266_CWMODE_AP     0x04     // 选择WiFi应用模式  AP模式
#define ESP8266_CWMODE_APSTA  0x05      // 选择WiFi应用模式  Station+AP模式

#define ESP8266_CWLAP         0x06      // 列出当前接入点
#define ESP8266_CWQAP         0x07      // 退出
#define ESP8266_CIFSR         0x08      // 获取本地IP地址
//#define ESP8266_CWJAP "AT+CWJAP=\"lzt02\",\"lzt123456\"\x0D\x0A"

#define ESP8266_CWJAP         0x0a      // 加入接入点 TP-LINK_sundm
#define ESP8266_CIPEXITSERVER 0x0b      // 退出server
#define ESP8266_CIPMUX        0x0c      // 设置单连接
#define ESP8266_CIPMODE       0x0e      // 设置透传模式

#define ESP8266_CIPSTART      0x0f     // 建立TCP/UDP连接
#define ESP8266_CIPSTATUS     0x10     // 获得TCP/UDP连接状态
#define ESP8266_CIPSEND       0x11      // 发送数据

#define ESP8266_CIPCLOSE      0x12     // 关闭TCP/UDP连接
#define ESP8266_CIPMODE0      0x13      // 关闭透传
#define ESP8266_CWLAPOPT      0x14 
#define ESP8266_SERIALNET     0x15      //设置透传模式
#endif


//rt_bool_t wificonfig(void);// 查找wifi串口设备并打开，注册回调函数，初始化串口接收事件
//rt_bool_t wifiinit(void); //wifi接入热点
//rt_bool_t wifijap(void) ;
//rt_bool_t wificonnect(void); //打开远程连接网络
//rt_bool_t wifisend(char * str); //网络数据发送
//rt_bool_t wificloseconnect(void);// 关闭远程连接网络
//rt_bool_t wifiexit(void); // wifi退出热点

#endif
