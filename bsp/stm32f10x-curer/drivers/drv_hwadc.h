/*
 * File      : drv_adc.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-01-05     Bernard      the first version
 */
#ifndef DRV_HWADC_H__
#define DRV_HWADC_H__
 
#include <stdint.h> 
#include <drivers/hwadc.h>
#include "stm32f1xx.h"

//#define RT_USING_ADC_GENERIC_MODE
#define  RT_USING_ADC_INT_MODE 
//#define  RT_USING_ADC_DMA_MODE       
  
#define RT_USING_ADC_IN0_ENABLE     1 /* ADC Channel PA0*/
#define RT_USING_ADC_IN1_ENABLE     0 /* ADC Channel PA1*/
#define RT_USING_ADC_IN2_ENABLE     0 /* ADC Channel PA2*/
#define RT_USING_ADC_IN3_ENABLE     0 /* ADC Channel PA3*/
#define RT_USING_ADC_IN4_ENABLE     0 /* ADC Channel PA4*/
#define RT_USING_ADC_IN5_ENABLE     0 /* ADC Channel PA5*/
#define RT_USING_ADC_IN6_ENABLE     0 /* ADC Channel PA6*/
#define RT_USING_ADC_IN7_ENABLE     0 /* ADC Channel PA7*/

#define RT_USING_ADC_IN8_ENABLE     0 /* ADC Channel PB0*/
#define RT_USING_ADC_IN9_ENABLE     0 /* ADC Channel PB1*/

#define RT_USING_ADC_IN10_ENABLE     0 /* ADC Channel PC0*/
#define RT_USING_ADC_IN11_ENABLE     0 /* ADC Channel PC1*/
#define RT_USING_ADC_IN12_ENABLE     0 /* ADC Channel PC2*/
#define RT_USING_ADC_IN13_ENABLE     0 /* ADC Channel PC3*/
#define RT_USING_ADC_IN14_ENABLE     0 /* ADC Channel PC4*/
#define RT_USING_ADC_IN15_ENABLE     0 /* ADC Channel PC5*/

#define RT_USING_ADC_TEMPSENSOR_ENABLE  0 /* Chip tempreture sensor*/
#define RT_USING_ADC_VREFINT_ENABLE     0 /* Chip reference voltage*/

//对于12位的ADC，3.3V的ADC值为0xfff,温度为25度时对应的电压值为1.43V即0x6EE
#define V25  0x6EE
//斜率 每摄氏度4.3mV 对应每摄氏度0x05
#define AVG_SLOPE 0x05 

/* STM32 timer driver */
typedef struct drv_hwadc
{
    ADC_HandleTypeDef adcHandle;      
    IRQn_Type irq;
}drv_hwadc_t;

#define ADC_BUFF_LEN                    32
typedef struct adc_dat
{
    uint16_t adc_value[ADC_BUFF_LEN];
}adc_dat_buffer_t;

//#define PIN_USERDATA_END {-1,0}

//extern struct stm32_adc_port_app stm32_ports[];

int stm32_hwadc_init(void);

#endif
