/************************************************* 
  Copyright (C), 2012, NJUT
  File name:      gprsio.c
  Author:  sundm     Version:  1.0      Date: 2013.1.3 
  Description:    sundm GPRSģ�������ӿں��� 
  Others:        ���õײ���������亯��  
  Function List:  
*************************************************/ 

#ifndef _ESP8266_H_
#define _ESP8266_H_

#include <rtthread.h>
#include "stm32f10x.h"


//#define ESP8266_RCC_GPIO_CLK_ENABLE()  	__HAL_RCC_GPIOB_CLK_ENABLE()
//#define ESP8266_CS_PORT              	GPIOB
//#define ESP8266_CS_PIN               	GPIO_PIN_4

#define PB4 					  90 // PB4 Pin is NO. 90 @STM32F103VET6

#define	ESP8266_CS_Init() 		rt_pin_mode(PB4, PIN_MODE_OUTPUT)
#define ESP8266_CS_LOW()		rt_pin_write(PB4,PIN_LOW)
#define ESP8266_CS_HIGH()		rt_pin_write(PB4,PIN_HIGH)

#define ESP8266_USARTx 		"uart3"

//����ATָ��
#define ESP8266_ATCMD "AT\x0D\x0A"               // AT��ѯ
#define ESP8266_RESET "AT+RST\x0D\x0A"           // ģ�鸴λ

#define ESP8266_CWMODE_STA "AT+CWMODE=1\x0D\x0A"      // ѡ��WiFiӦ��ģʽ  Stationģʽ
#define ESP8266_CWMODE_AP "AT+CWMODE=2\x0D\x0A"      // ѡ��WiFiӦ��ģʽ  APģʽ
#define ESP8266_CWMODE_APSTA "AT+CWMODE=3\x0D\x0A"      // ѡ��WiFiӦ��ģʽ  Station+APģʽ

#define ESP8266_CWLAP "AT+CWLAP\x0D\x0A"      // �г���ǰ�����
#define ESP8266_CWQAP "AT+CWQAP\x0D\x0A"      // �˳�
#define ESP8266_CIFSR "AT+CIFSR\x0D\x0A"      // ��ȡ����IP��ַ
//#define ESP8266_CWJAP "AT+CWJAP=\"lzt02\",\"lzt123456\"\x0D\x0A"

#define ESP8266_CWJAP "AT+CWJAP=\"lzt\",\"lzt123456\"\x0D\x0A"      // �������� TP-LINK_sundm
#define ESP8266_CIPEXITSERVER "AT+CIPSERVER=0\x0D\x0A"      // �˳�server
#define ESP8266_CIPMUX "AT+CIPMUX=0\x0D\x0A"      // ���õ�����
#define ESP8266_CIPMODE "AT+CIPMODE=1\x0D\x0A"      // ����͸��ģʽ

#define ESP8266_CIPSTART "AT+CIPSTART=\"TCP\",\"192.168.8.109\",8080\x0D\x0A"      // ����TCP/UDP����
#define ESP8266_CIPSTATUS "AT+CIPSTATUS\x0D\x0A"      // ���TCP/UDP����״̬
#define ESP8266_CIPSEND "AT+CIPSEND\x0D\x0A"      // ��������

#define ESP8266_CIPCLOSE "AT+CIPCLOSE\x0D\x0A"      // �ر�TCP/UDP����
#define ESP8266_CIPMODE0 "+++"      // �ر�͸��

//rt_bool_t wificonfig(void);// ����wifi�����豸���򿪣�ע��ص���������ʼ�����ڽ����¼�
//rt_bool_t wifiinit(void); //wifi�����ȵ�
//rt_bool_t wifijap(void) ;
//rt_bool_t wificonnect(void); //��Զ����������
//rt_bool_t wifisend(char * str); //�������ݷ���
//rt_bool_t wificloseconnect(void);// �ر�Զ����������
//rt_bool_t wifiexit(void); // wifi�˳��ȵ�

#endif
