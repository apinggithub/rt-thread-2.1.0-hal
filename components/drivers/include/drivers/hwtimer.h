#ifndef __HWTIMER_H__
#define __HWTIMER_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HW_ENABLE           1
#define HW_DISABLE          0       

#define HWTMR_IC_BUF_SIZE    8    
    
/* Timer Control Command */
enum rt_hwtimer_ctrl
{
    HWTIMER_CTRL_SET_FREQ = 0x01,       /* set the count frequency */
    HWTIMER_CTRL_START,                 /* start timer */
    HWTIMER_CTRL_STOP,                  /* stop timer */
    HWTIMER_CTRL_GET_INFO,              /* get a timer feature information */
    HWTIMER_CTRL_SET_MODE,              /* Setting the timing mode(pwm,oc,ic or encoder) */
    HWTIMER_CTRL_GET_AUTORELOAD,        /* get the timer autoreload */
    HWTIMER_CTRL_SET_AUTORELOAD,        /* set the timer autoreload */
    HWTIMER_CTRL_SET_PRESCALER,         /* set the timer prescaler */
    HWTIMER_CTRL_GET_DUTY_CYCLE,        /* get pwm duty cycle */
    HWTIMER_CTRL_SET_DUTY_CYCLE,        /* set pwm duty cycle */
    HWTIMER_CTRL_GET_CAPTURE_VALUE,     /* get capture value  */
    HWTIMER_CTRL_GET_TIMER_STATUS        /* get the timer run status */
};


/* Timing Mode */
/*typedef enum
{
    HWTIMER_MODE_ONESHOT = 0x01,
    HWTIMER_MODE_PERIOD
} rt_hwtimer_mode_t;*/

/* Timer Channel */
typedef enum
{
    HWTIMER_BASE = 0x00,
    HWTIMER_CH1,    
    HWTIMER_CH2,
    HWTIMER_CH3,
    HWTIMER_CH4    
} rt_hwtimer_ch_t;

/* Time Value */
typedef struct rt_hwtimer_tmr
{
    int32_t sec;      /* second */
    int32_t usec;     /* microsecond */
} rt_hwtimer_tmrval_t;

typedef struct rt_hwtimer_val
{
    int8_t ch;      /* channel no. */
    int32_t value;  /* counter */
} rt_hwtimer_chval_t;

typedef struct rt_hwtimer_freq
{
    int8_t ch;      /* channel no. */
    float freq;  /* frequency */
} rt_hwtimer_chfreq_t;

#define HWTIMER_INTERNAL_CLOCK_SOURCE    0x00 /*  timer internal clock source */
#define HWTIMER_EXTERNAL_CLOCK_SOURCE    0x01 /*  timer external clock source */

#define HWTIMER_CNTMODE_UP          0x01 /* increment count mode */
#define HWTIMER_CNTMODE_DW          0x02 /* decreasing count mode */

typedef struct rt_channel
{
    int8_t status;      /* channel status is actived or inactive. */
    int8_t channel;
} rt_channel_t;

typedef struct rt_device_hwtimer
{
    struct rt_device parent;
    const struct rt_hwtimer_ops *ops;
    const struct rt_hwtimer_info *info;

    float freq;                     /* counting frequency(Hz) set by the user */
    uint8_t clock_source;   /* the flag of timer counter ,0 --> internal clock or 1 --> external clock */
    uint16_t prescaler;          /* timer prescaler */
    int16_t reload;              /* reload cycles(using in period mode) */
    uint8_t count_mode;          /* timer count mode */
    
    rt_channel_t channel_no[5];    /* the timer channel no.*/   
    uint8_t channel_type[5];     /* the timer work type ic,oc,pwm,and encode */    
    uint16_t pwm_dutycycle[5];    /* the pwm duty cycle */
    uint32_t capture_value[5];
    //uint32_t capture_buffer[5][HWTMR_IC_BUF_SIZE];    /* the capture value buffer */
    //uint8_t capture_count[5];     /* the flag capture */    
    uint8_t channel_lock[5];     /* channel visit lock */
    uint32_t overflow[5];      /* timer overflows,the max. is 6 of timer total,every timer has 4 channels. */  
    uint32_t cycles[5];        /* how many times will generate a timeout event after overflow */
    
    //rt_hwtimer_mode_t mode;       /* timing mode(oneshot/period) */
} rt_device_hwtimer_t;

struct rt_hwtimer_ops
{
    void (*drv_init)( rt_device_hwtimer_t *timer, rt_uint8_t status);
    rt_err_t (*drv_start)(rt_device_hwtimer_t *timer, rt_uint8_t ch);
    void (*drv_stop)(rt_device_hwtimer_t *timer, rt_uint8_t ch);
    rt_err_t (*drv_set_prescaler)(rt_device_hwtimer_t *timer,rt_uint32_t val);
    rt_uint32_t (*drv_get_counter)(rt_device_hwtimer_t *timer);
    rt_err_t (*drv_set_counter)(rt_device_hwtimer_t *timer,rt_uint32_t val);
    rt_uint32_t (*drv_get_autoreload)(rt_device_hwtimer_t *timer);
    rt_err_t (*drv_set_autoreload)(rt_device_hwtimer_t *timer,rt_uint32_t val);
    rt_uint32_t (*drv_get_compare)(rt_device_hwtimer_t *timer,rt_uint8_t ch);
    rt_err_t (*drv_set_compare)(rt_device_hwtimer_t *timer,rt_uint8_t ch,rt_uint32_t val);    
    rt_err_t (*drv_set_frequency)(rt_device_hwtimer_t *timer, float freq);
    //rt_uint32_t (*drv_get_capturevalue)(rt_device_hwtimer_t *timer,rt_uint8_t ch);
};

/* Timer Feature Information */
struct rt_hwtimer_info
{
    float maxfreq;    /* the maximum count frequency timer support */
    float minfreq;    /* the minimum count frequency timer support */
    rt_uint32_t maxcnt;    /* counter maximum value */
    rt_uint8_t  cntmode;   /* count mode (inc/dec) */
};

rt_err_t rt_device_hwtimer_register(rt_device_hwtimer_t *timer, const char *name, void *user_data);
void rt_device_hwtimer_isr(rt_device_hwtimer_t *timer, rt_uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif
