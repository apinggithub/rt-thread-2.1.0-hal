/*
 * File      : application.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2013-07-12     aozima       update for auto initial.
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#ifndef LIGHT_WAVE_CURER_H__
#define LIGHT_WAVE_CURER_H__

#include <rtthread.h>
#include <stdint.h>
#include <drivers/lcdht1621b.h>

#include <drv_hwtimer.h>

#ifdef RT_USING_LIGHT_WAVE_CURER
/* Using hardware button */
//#define RT_USING_HWBUTTON 
//#define RT_USING_HWBUTTON_TEST /*the button driver test cmd in finsh*/ 
 
/* Using segment lcd ht1621b */
//#define RT_USING_LCDHT1621B 
//#define RT_USING_LCDHT1621B_TEST /*the segment lcd  driver chip ht1621b test cmd in finsh*/ 
 
/* Using voice chip xt8xxp8 */
//#define RT_USING_XT8XXP8
//#define RT_USING_XT8XXP8_TEST /*the voice chip driver test cmd in finsh*/ 
//#define RT_USING_XTP_ONE_WRIRE /* single wire mode*/
#endif /* RT_USING_LIGHT_WAVE_CURER */

/* debug switch */

//#define USER_HWTIMER_APP_BUG_TEST
//#define USER_HWADC_APP_BUG_TEST

#define LWC_ACTIVED                         1
#define LWC_INACTIVE                        0

#define LWC_BASE_TIMER_IDLE                 0x00
#define LWC_BASE_TIMER_READY                0x55
#define LWC_BASE_TIMER_RUNNING              0xAA


/* The pin NO. on the chip*/
#define PC13_SPEAKER_CTRL               2 
#define PD2_BEEP                        54 
#define PB5_IONTHERAPY_RLY              57
#define PB12_IONTHERAPY_PWR             33
#define PB13_IONTHERAPY_CRL1            34
#define PB14_IONTHERAPY_CRL2            35
#define PB15_IONTHERAPY_DECT            36
#define PA8_YY_IC_DECT                  41
#define PB8_CUREI_FREQ                  61
#define PB9_CUREII_FREQ                 62

#define PC13                       2
#define PC14                       3
#define PC15                       4
#define PB3                       55
#define PB4                       56

//电流检测
#define ADC_I_0A             2440 /*I = 0 ADC转换值*/
#define ADC_I_2A0            2770 /*I = 2.0A ADC转换值*/ 
#define ADC_I_2A5            2842 /*I = 2.5A ADC转换值*/ 


/* the registered device */
#define BUTTON                          "button"
#define LCD                             "lcdht"
#define TIMER2                          "timer2"
#define TIMER3                          "timer3"
#define TIMER4                          "timer4"
#define TIMER6                          "timer6"
#define TIMER1                          "timer1"
#define XTP                             "xtp"
#define HWADC                           "hwadc"


/* define the button value */
#define BUTTON_RL                       0x11
#define BUTTON_DY_DS                    0x12
#define BUTTON_ZL1_INC                  0x14
#define BUTTON_ZL2_INC                  0x18
#define BUTTON_JG                       0x22
#define BUTTON_SD                       0x28
#define BUTTON_LZLF                     0x24
#define BUTTON_JY                       0x31
#define BUTTON_GN                       0x32
#define BUTTON_ZL1_DEC                  0x34
#define BUTTON_ZL2_DEC                  0x38

#define TMR_DELAY_100ms                 100
#define TMR_DELAY_200ms                 200
#define TMR_DELAY_250ms                 250
#define TMR_DELAY_300ms                 300
#define TMR_DELAY_400ms                 400
#define TMR_DELAY_500ms                 500
#define TMR_DELAY_600ms                 600
#define TMR_DELAY_800ms                 800
#define TMR_DELAY_900ms                 900
#define TMR_DELAY_1000ms                1000

#define TMR_CH_BASE             HWTIMER_BASE
#define TMR_CH_LASER_PWM        HWTIMER_CH4
#define TMR_CH_HEAT_PWM         HWTIMER_CH3

#define TMR_CH_CUREI_PWM        HWTIMER_CH3
#define TMR_CH_CUREII_PWM       HWTIMER_CH4

#define TMR_CH_CUREI_FREQ       HWTIMER_CH3
#define TMR_CH_CUREII_FREQ      HWTIMER_CH4

/* the event type rt-thread event API */
#define RT_EVENT_LWC_TIMER_FINISH_CLOSE     1<<0 /* 定时器超时输出关闭 */
#define RT_EVENT_LWC_DEVICE_POWER_CLOSE     1<<1 /* 定时递减到0 */
#define RT_EVENT_LWC_DEVICE_FORCE_CLOSE     1<<2 /* 设备强制关闭 */
#define RT_EVENT_LWC_RLJG_BUTTON_UPDATE     1<<3 /* 激光热疗按键更新 */
#define RT_EVENT_LWC_GNZL_BUTTON_UPDATE     1<<4 /* 功能治疗按键更新 */
#define RT_EVENT_LWC_LASER_CURE_CLOSE       1<<5 /* 激光关闭*/
#define RT_EVENT_LWC_HEAT_CURE_CLOSE        1<<6 /* 热疗关闭*/
#define RT_EVENT_LWC_IONICE_CURE_CLOSE      1<<7 /* 离子治疗关闭*/
#define RT_EVENT_LWC_FUNCTION_CLOSE         1<<8 /* 功能治疗关闭*/
#define RT_EVENT_LWC_ION_FUNC_START         1<<9 /* 离子治疗功能开启 */
#define RT_EVENT_LWC_ION_TIME_UPDATE        1<<10 /* 离子治疗时间更新 */
#define RT_EVENT_LWC_ION_CURE_CLOSE         1<<11 /* 离子治疗关闭 */
//#define RT_EVENT_LWC_WAIT_TMR_START         1<<12 /* 待机事件触发 */
//#define RT_EVENT_LWC_ION_FORCE_DISPLAY      1<<13 /* 离子治疗强度显示 */
#define RT_EVENT_LWC_TIMER_MUSIC_PLAY       1<<14 /* 音频治疗重播*/


/* define cure mode */
enum cure_mod
{
    SET_TIMER  = 0x00,  /*  定时 */
    LASER_CURE,         /*  激光治疗 */
    HEAT_CURE,          /*  热疗 */
    IONICE_CURE,        /*  离子疗法 */
    FUNCTION            /*  功能 */  
};

enum func_mod
{
    FULL_FUNCTION = 1,   /* 全功能 */
    MID_FREQUENCY,          /* 中频 */
    ACUPUNCTURE_NEEDLE,     /* 针灸 */
    PAT_CURE,               /* 拍打 */
    NAPRAPATHY,             /* 推拿 */
    MASSOTHERAPY,           /* 按摩 */
    CUPPING_THERAPY,        /* 拔罐 */
    PEDICURE,               /* 足疗 */
    LOSE_WEIGHT,            /* 减肥 */
    VOICE_FREQUENCY         /* 音频 */
};


typedef struct lwc_button
{
    uint8_t button_rl; /* 热疗 */
    uint8_t button_dyds;/* 电源定时 */   
    uint8_t button_jg;/* 激光 */
    uint8_t button_sd;/* 锁定 */
    uint8_t button_lzlf;/* 离子疗法 */
    uint8_t button_jy/* 静音 */;
    uint8_t button_gn;/*功能*/
    uint8_t button_zl1;/* 治疗1 */
    uint8_t button_zl2;/* 治疗2 */
    uint8_t flag_base_timer_status; /* 基本定时器工作状态 */
    
}lwc_button_t;

typedef struct timer_val
{
    uint8_t tmr_value;  /* 定时时间长度 */
    uint8_t lcd_hbyte;  /* LCD 显示高位 */
    uint8_t lcd_lbyte;  /* LCD 显示低位 */ 
    uint8_t tmr_lock;   /* 定时时间锁定 */    
}timer_val_t;

typedef struct lwc_cure_way
{
    uint8_t status;     /* 激活状态 */      
}lwc_cure_way_t;    

typedef struct lwc_cure_output
{
    //uint8_t ion_current_force;      /* 治疗强度 */
    //uint8_t ion_func_actived;       /* 离子功能激活 */
    uint8_t cure_out_actived;
}lwc_cure_output_t;  

typedef struct lwc_data_reg
{   
    lwc_button_t btn;       /* 按键类型 */
    
    uint8_t    power_ok;    /* 系统上电标记*/
    timer_val_t tval;       /* 定时长度 */   
}lwc_data_reg_t;

typedef struct lwc_cure_display   
{   
    lwc_data_reg_t lreg;
    lwc_cure_way_t lway[5];     /* 治疗方式 */    
    rt_lcd_ramdat_t lcdr[20];   /* 共用20个段位 */
    int8_t ion_force;          /* 离子治疗强度 */
    int8_t ion_force_overtimes; /* 强度超限次数*/
    lwc_cure_output_t lcf[5];    /* 共四路输出*/
}lwc_cure_t;

typedef struct lwc_function_fm
{   
    uint8_t  fm_idx;       /* 调频索引 */
    uint8_t  func_idx;     /* 功能索引 */
    uint8_t  fm_switch;    /* 频段输出开关 */   
}lwc_function_fm_t;


extern uint8_t flag_force_voice_play;
extern rt_uint16_t tmr_count;
extern struct rt_mailbox mb;
extern char mb_pool[128];

extern lwc_cure_t lct;
extern lwc_function_fm_t lff;
extern struct rt_event event;
extern struct rt_timer timerions;
extern struct rt_timer timertip;
extern uint32_t tmr_count_tip;

extern rt_uint8_t lwc_adc_stack[2048];
extern struct rt_thread lwc_adc_thread;

extern rt_uint8_t lwc_button_stack[2048];
extern struct rt_thread lwc_button_thread;
extern rt_uint8_t lwc_display_stack[2048];
extern struct rt_thread lwc_display_thread;
extern rt_uint8_t lwc_output_stack[5120];
extern struct rt_thread lwc_output_thread;

extern void lwc_adc_thread_entry(void* parameter);
extern void lwc_button_thread_entry(void* parameter);
extern void lwc_display_thread_entry(void* parameter);
extern void lwc_output_thread_entry(void* parameter);

//rt_err_t seglcd_display_time(rt_device_t dev, lwc_cure_t *lc);
void lwc_force_display(rt_device_t dev, lwc_cure_t *lc);


#endif

/*@}*/
