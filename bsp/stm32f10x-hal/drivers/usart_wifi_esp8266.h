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
#include "stm32f1xx.h"

//#define ESP8266_RCC_GPIO_CLK_ENABLE()  	__HAL_RCC_GPIOB_CLK_ENABLE()
//#define ESP8266_CS_PORT              	GPIOB
//#define ESP8266_CS_PIN               	GPIO_PIN_4

#define PB4 					  90 // PB4 Pin is NO. 90 @STM32F103VET6

#define	ESP8266_CS_Init() 		rt_pin_mode(PB4, PIN_MODE_OUTPUT)
#define ESP8266_CS_LOW()		rt_pin_write(PB4,PIN_LOW)
#define ESP8266_CS_HIGH()		rt_pin_write(PB4,PIN_HIGH)

#define ESP8266_USARTx 		"uart3"

//����ATָ��

#define WIFI_LIST   0x20
#define WIFI_JAP    0x21
#define WIFI_QAP    0x22
#define WIFI_SEND   0x23
#define WIFI_STATUS 0x24


#if 1
#define ESP8266_ATCMD "AT\x0D\x0A"               // AT��ѯ
#define ESP8266_RESET "AT+RST\x0D\x0A"           // ģ�鸴λ

#define ESP8266_CWMODE_STA "AT+CWMODE=1\x0D\x0A"      // ѡ��WiFiӦ��ģʽ  Stationģʽ
#define ESP8266_CWMODE_AP "AT+CWMODE=2\x0D\x0A"      // ѡ��WiFiӦ��ģʽ  APģʽ
#define ESP8266_CWMODE_APSTA "AT+CWMODE=3\x0D\x0A"      // ѡ��WiFiӦ��ģʽ  Station+APģʽ

#define ESP8266_CWLAP "AT+CWLAP\x0D\x0A"      // �г���ǰ�����
#define ESP8266_CWQAP "AT+CWQAP\x0D\x0A"      // �˳�
#define ESP8266_CIFSR "AT+CIFSR\x0D\x0A"      // ��ȡ����IP��ַ
//#define ESP8266_CWJAP "AT+CWJAP=\"lzt02\",\"lzt123456\"\x0D\x0A"

#define ESP8266_CWJAP "AT+CWJAP=\"lzt02\",\"lzt123456\"\x0D\x0A"      // �������� TP-LINK_sundm
#define ESP8266_CIPEXITSERVER "AT+CIPSERVER=0\x0D\x0A"      // �˳�server
#define ESP8266_CIPMUX "AT+CIPMUX=0\x0D\x0A"      // ���õ�����
#define ESP8266_CIPMODE "AT+CIPMODE=1\x0D\x0A"      // ����͸��ģʽ

#define ESP8266_CIPSTART "AT+CIPSTART=\"TCP\",\"192.168.2.102\",8080\x0D\x0A"      // ����TCP/UDP����
#define ESP8266_CIPSTATUS "AT+CIPSTATUS\x0D\x0A"      // ���TCP/UDP����״̬
#define ESP8266_CIPSEND "AT+CIPSEND\x0D\x0A"      // ��������

#define ESP8266_CIPCLOSE "AT+CIPCLOSE\x0D\x0A"      // �ر�TCP/UDP����
#define ESP8266_CIPMODE_CLOSED "+++"      // �ر�͸��
#define ESP8266_CWLAPOPT "AT+CWLAPOPT=0,7\0x0D\x0A"                   //����cwlap����ʾ���� 1(0)(��)�����ź�ǿ������(bit6)(bit5)(bit4)ch(bit3)mac(bit2)�ź�ǿ��(bit1)ssid(bit0)���ܷ�ʽ
#endif

typedef struct wifi_list{
	char ecn[1];
	char ssid[32];
	char rssi[4];
}wifi_list_t;



#if 0
#define ESP8266_ATCMD         0x01               // AT��ѯ
#define ESP8266_RESET         0x02           // ģ�鸴λ

#define ESP8266_CWMODE_STA    0x03      // ѡ��WiFiӦ��ģʽ  Stationģʽ
#define ESP8266_CWMODE_AP     0x04     // ѡ��WiFiӦ��ģʽ  APģʽ
#define ESP8266_CWMODE_APSTA  0x05      // ѡ��WiFiӦ��ģʽ  Station+APģʽ

#define ESP8266_CWLAP         0x06      // �г���ǰ�����
#define ESP8266_CWQAP         0x07      // �˳�
#define ESP8266_CIFSR         0x08      // ��ȡ����IP��ַ
//#define ESP8266_CWJAP "AT+CWJAP=\"lzt02\",\"lzt123456\"\x0D\x0A"

#define ESP8266_CWJAP         0x0a      // �������� TP-LINK_sundm
#define ESP8266_CIPEXITSERVER 0x0b      // �˳�server
#define ESP8266_CIPMUX        0x0c      // ���õ�����
#define ESP8266_CIPMODE       0x0e      // ����͸��ģʽ

#define ESP8266_CIPSTART      0x0f     // ����TCP/UDP����
#define ESP8266_CIPSTATUS     0x10     // ���TCP/UDP����״̬
#define ESP8266_CIPSEND       0x11      // ��������

#define ESP8266_CIPCLOSE      0x12     // �ر�TCP/UDP����
#define ESP8266_CIPMODE0      0x13      // �ر�͸��
#define ESP8266_CWLAPOPT      0x14 
#define ESP8266_SERIALNET     0x15      //����͸��ģʽ
#endif


//rt_bool_t wificonfig(void);// ����wifi�����豸���򿪣�ע��ص���������ʼ�����ڽ����¼�
//rt_bool_t wifiinit(void); //wifi�����ȵ�
//rt_bool_t wifijap(void) ;
//rt_bool_t wificonnect(void); //��Զ����������
//rt_bool_t wifisend(char * str); //�������ݷ���
//rt_bool_t wificloseconnect(void);// �ر�Զ����������
//rt_bool_t wifiexit(void); // wifi�˳��ȵ�

#endif
