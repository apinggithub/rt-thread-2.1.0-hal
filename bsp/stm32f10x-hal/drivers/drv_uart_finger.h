/*
 * File      : usart_wifi_esp12f.h
 * This file is part of RT-Thread RTOS
 * Copyright by Shanghai Real-Thread Electronic Technology Co.,Ltd
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-07-31     aozima       the first version
 * 2014-09-18     aozima       update command & response.
 */

#ifndef _DRV_UART_FINGER_H
#define _DRV_UART_FINGER_H


//#ifdef RT_USING_FINGERPRIN
#include <stdint.h>
#include <rtdef.h>
#include "stm32f1xx.h"
#include "board.h"


	#define FINGER_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()


  #define FP_POWER_PIN    GPIO_PIN_9
	#define FP_CRTL_PIN  	  GPIO_PIN_8
	#define FP_OUT_PIN  	  GPIO_PIN_7
	#define FP_POWER_PORT   GPIOC
	#define FP_CRTL_PORT  	GPIOC
	#define FP_OUT_PORT  	  GPIOC

	#define FP_CRTL_ON()  HAL_GPIO_WritePin(FP_CRTL_PORT,FP_CRTL_PIN,GPIO_PIN_SET)
	#define FP_CRTL_OFF() HAL_GPIO_WritePin(FP_CRTL_PORT,FP_CRTL_PIN,GPIO_PIN_RESET)

	#define FP_POWER_ON()  HAL_GPIO_WritePin(FP_POWER_PORT,FP_POWER_PIN,GPIO_PIN_SET)
	#define FP_POWER_OFF() HAL_GPIO_WritePin(FP_POWER_PORT,FP_POWER_PIN,GPIO_PIN_RESET)

#define FINGER_RECIVE_MAX 500

#define REV_DATA      0x01
#define FP_REG        0x02
#define FP_SUCESS     0x04
#define FP_FAILD      0x08

#define FP_CMD_REG_FIRST            0xc1
#define FP_CMD_REG_SECOND           0xc2
#define FP_CMD_REG_THIRD            0xc3
#define FP_CMD_REG_CHECK            0xc4


#define ACK_SUCCESS 								0x00 		//执行成功
#define ACK_FAIL 										0x01 			//执行失败
#define ACK_FULL 										0x04 			//数据库满
#define ACK_NOUSER 									0x05 		//没有这个用户
#define ACK_USER_EXIST 							0x07 //用户已存在
#define ACK_TIMEOUT 								0x08 		//图像采集超时
#define ACK_BREAK 									0x18 			//终止当前命令
#define ACK_IMAGEFAIL 							0x84 	//图像不合格

#define FP_CMD_REG         					0x01 //注册单次注册多次返回（1CR3）
#define FP_CMD_REG1        					0x01 //注册3次注册3啻畏祷兀�3CR3）
#define FP_CMD_REG2        					0x02 //注册3次注册3次返回（3CR3）
#define FP_CMD_REG3        					0x03 //注册3次注册3次返回（3CR3）
#define FP_CMD_DEL_ONE     					0x04 //删除指定编号的用户
#define FP_CMD_DEL_ALL     					0x05 //删除所有的用户
#define FP_CMD_GET_USER_NUM     		0x09 //获取用户总数
#define FP_CMD_GET_USER_ON     			0x0A //获取用户是否存在
#define FP_CMD_GET_MATCH_BY_NUM     0x0B //指纹一比一比对
#define FP_CMD_GET_MATCH_BY_ALL     0x0C //指纹一对多比对
#define FP_CMD_GET_UNUSED_USER     	0x0D //指纹一对多比对
#define FP_CMD_SET_BAUD     				0x21 //设定波特率

#define FP_CMD_GET_FP						    0x23 //获取指纹特征值特征值长度494

#define FP_CMD_GET_FP_IMAGE					0x24 //获取指纹图像
#define FP_CMD_GET_FP_BY_NUM				0x31 //获取指纹特征值特征值长度494
#define FP_CMD_GET_FP_IMAGE					0x24 //获取指纹图像

#define FP_CMD_TIMEOUT              1000




/*struct fp_cmdack{
	rt_uint8_t   begin;
	rt_uint8_t   cmd;
	rt_uint8_t   num1_high;
	rt_uint8_t   num1_low;
	rt_uint8_t   num2_high;
	rt_uint8_t   num2_low;
	rt_uint8_t   num2_check;
	rt_uint8_t   end;
};*/



//#endif // SPI_WIFI_H_INCLUDED
#endif
