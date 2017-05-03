#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>

#ifdef RT_USING_HWTIMER
//#ifdef RT_USING_HWTIMER_TEST 
#if 0
extern const float frq[][6];/* = {
    {26.0,  17.0,   9.0,    5.0,    0.5,    4.0},
    {1.0,   0.5,    0.5,    2.5,    1.5,    5.0},
    {0.5,   1.5,    2.0,    6.5,    3.5,    2.5},
    {1.5,   5.0,    0.5,    0.5,    4.5,    2.0},
    {26.0,  13.0,   5.0,    4.0,    1.5,    50.0},
    {9.0,   4.5,    2.0,    5.5,    6.5,    1.5},
    {0.5,   0.75,   6.5,    1.5,    9.5,    1.5},
    {3.5,   0.5,    9.0,    2.6,    5.0,    1.3}
};*/
#ifdef RT_USING_HWTIM6
static rt_err_t timer6a_timeout_cb(rt_device_t dev, rt_size_t size)
{
    //rt_kprintf("HT %d\n", rt_tick_get());    
    //if(dev == dev_hwtimer3)
    //{
        rt_kprintf("timer6 stop in timeout_cb \n");  
    //}
    
    return 0;
} 
#endif /* RT_USING_HWTIM6 */
#if 0
#ifdef RT_USING_HWTIM3
static rt_err_t timer3a_timeout_cb(rt_device_t dev, rt_size_t size)
{
    //rt_kprintf("HT %d\n", rt_tick_get());    
    //if(dev == dev_hwtimer3)
    //{
        rt_kprintf("timer3 stop in timeout_cb \n");  
    //}
    
    return 0;
} 
#endif /* RT_USING_HWTIM3 */
#ifdef RT_USING_HWTIM4
static rt_err_t timer4a_timeout_cb(rt_device_t dev, rt_size_t size)
{
    //rt_kprintf("HT %d\n", rt_tick_get());    
    //if(dev == dev_hwtimer4)
    //{
        rt_kprintf("timer4 stop in timeout_cb \n");  
    //}
    return 0;
}
#endif /*RT_USING_HWTIM4 */
#endif
#endif
int hwtimer(void)
{
    rt_err_t err;
    //rt_hwtimer_tmrval_t tmr;

    rt_tick_t tick;
    //rt_hwtimer_mode_t mode;
    //int freq = 2000;//Hz, the timer out frequence (20Hz~1MHz)
    int t = 30;//sec. the timer work on time
    //int temp = 0;
    //uint8_t ch;
    
    //rt_device_hwtimer_t *timer;
#if 0    
#ifdef RT_USING_HWTIM6
    
    #define TIMER6   "timer6"
    rt_device_t dev_hwtimer6 = RT_NULL;
    //static uint16_t val6;
    rt_hwtimer_chval_t hwc6;
    rt_hwtimer_tmrval_t hwt6;
    
    hwc6.ch = HWTIMER_BASE;
    rt_pin_mode(54, PIN_MODE_OUTPUT);// the port PD2
   
    if((dev_hwtimer6 = rt_device_find(TIMER6)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER6);
        return -1;
    }
    
    rt_device_hwtimer_t *timer6 = (rt_device_hwtimer_t *)dev_hwtimer6;
    timer6 = (rt_device_hwtimer_t *)dev_hwtimer6;  
    timer6->freq = 1;
    timer6->prescaler = 71;
    timer6->reload = 0;
    
    rt_kprintf("Now test the %s \n", TIMER6);
       
    if (rt_device_open(dev_hwtimer6, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", TIMER6);
        return -1;
    }
    
    rt_device_set_rx_indicate(dev_hwtimer6, timer6a_timeout_cb);
    /* 计数时钟设置(默认1Mhz或支持的最小计数频率) */
    hwc6.value = timer6->freq;
    err = rt_device_control(dev_hwtimer6, HWTIMER_CTRL_SET_FREQ, &hwc6);
    if (err != RT_EOK)
    {
        rt_kprintf("Set Freq = %dHz Fail\n", hwc6.value);
        goto EXIT_TIM6;
    }
    else
    {
        rt_kprintf("Set Freq = %dHz ok\n", hwc6.value);
    }

    /* 周期模式 */
    //mode = HWTIMER_MODE_PERIOD;
    //err = rt_device_control(dev, HWTIMER_CTRL_MODE_SET, &mode);

    tick = rt_tick_get();
    rt_kprintf("Start Timer> Tick: %d\n", tick);
    /* 设置定时器超时值并启动定时器 */
    hwt6.sec = 10;
    hwt6.usec = 0;
    rt_kprintf("SetTime: Sec %d, Usec %d\n", hwt6.sec, hwt6.usec);   
    if (rt_device_write(dev_hwtimer6, hwc6.ch, &hwt6, sizeof(hwt6)) != sizeof(hwt6))
    {
        rt_kprintf("SetTime Fail\n");
        goto EXIT_TIM6;
    }
    else
    {
        rt_kprintf("Set timer work on  = %dsec. on channel No.%d ok.\n", hwt6.sec,hwc6.ch);
    }
      
    rt_kprintf("NO. 1 timer on %d sec.\n", t*2);
    rt_thread_delay(2*t*RT_TICK_PER_SECOND);
    /*
    tempfreq = freq - 200;
    err = rt_device_control(dev_hwtimer6, HWTIMER_CTRL_SET_FREQ, &tempfreq);
    if (err != RT_EOK)
    {
        rt_kprintf("Set Freq = %dHz Fail\n", tempfreq);
        goto EXIT_TIM6;
    }
    else
    {
        rt_kprintf("Set Freq = %dHz ok\n", tempfreq);
    }
    rt_kprintf("NO. 2 timer on %d sec.\n", t/2);
    rt_thread_delay(t*RT_TICK_PER_SECOND/2);
    */
    /* stop the timer */
    err = rt_device_control(dev_hwtimer6, HWTIMER_CTRL_STOP, RT_NULL);
    rt_kprintf("Timer Stoped\n");
    /* read the counter */
    rt_device_read(dev_hwtimer6, 0, &hwt6, sizeof(hwt6));
    rt_kprintf("Read: Sec = %d, Usec = %d\n", hwt6.sec, hwt6.usec);

EXIT_TIM6:
    err = rt_device_close(dev_hwtimer6);
    rt_kprintf("Close %s\n", TIMER6);
    
#endif /*RT_USING_HWTIM6*/    
#endif
#if 1
#ifdef RT_USING_HWTIM2
    rt_device_t dev_hwtimer2 = RT_NULL;
    #define TIMER2   "timer2"
    rt_hwtimer_chval_t hwc2;
    rt_hwtimer_chfreq_t hwq2;
    rt_hwtimer_tmrval_t hwt2;
    //rt_pin_mode(20, PIN_MODE_OUTPUT);// the port PF8
    uint16_t tempfreq;
    
    rt_kprintf("Now test the %s ... \n", TIMER2);
   
    if ((dev_hwtimer2 = rt_device_find(TIMER2)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER2);
        return -1;
    }
    rt_device_hwtimer_t *timer2 = (rt_device_hwtimer_t *)dev_hwtimer2;
    timer2 = (rt_device_hwtimer_t *)dev_hwtimer2;  
    timer2->freq = 1200;
    timer2->prescaler = 71;
    //timer2->reload = 0;

    if (rt_device_open(dev_hwtimer2, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", TIMER2);
        return -1;
    }
    
    //rt_device_set_rx_indicate(dev_hwtimer2, timer_timeout_cb);
    /* 计数时钟设置(默认1Mhz或支持的最小计数频率) */
    hwq2.freq = timer2->freq;
    err = rt_device_control(dev_hwtimer2, HWTIMER_CTRL_SET_FREQ, &hwq2);
    if (err != RT_EOK)
    {
        rt_kprintf("Set Freq = %dHz Fail\n", timer2->freq);
        goto EXIT_TIM2;
    }
    else
    {
        rt_kprintf("Set Freq = %dHz ok\n", timer2->freq);
    }

    /* 周期模式 */
    //mode = HWTIMER_MODE_PERIOD;
    //err = rt_device_control(dev, HWTIMER_CTRL_MODE_SET, &mode);

    tick = rt_tick_get();
    rt_kprintf("Start Timer> Tick: %d\n", tick);
    /* 设置定时器超时值并启动定时器 */
    //val.sec = t;
    //val.usec = 0;
    //rt_kprintf("SetTime: Sec %d, Usec %d\n", val.sec, val.usec);
    
    /*for( uint8_t i = 0; i < 4; i++)
    {       
        if (rt_device_write(dev_hwtimer2, i, &val, sizeof(val)) != sizeof(val))
        {
            rt_kprintf("SetTime Fail\n");
            goto EXIT_TIM2;
        }
        else
        {
            rt_kprintf("Set %s work on  = %d sec. on channel PA%d ok.\n", TIMER2, val.sec, i);
        }       
    }*/
    
    hwc2.ch = HWTIMER_CH3;
    rt_device_control(dev_hwtimer2, HWTIMER_CTRL_START, &hwc2);
    hwc2.ch = HWTIMER_CH4;
    rt_device_control(dev_hwtimer2, HWTIMER_CTRL_START, &hwc2);
    
    
    //rt_kprintf("The timer will work on %d sec.\n", t/2);
    //rt_thread_delay(t*RT_TICK_PER_SECOND + 1);
     
    rt_thread_delay(t*RT_TICK_PER_SECOND/2);
    
    /* stop the timer */
    hwc2.ch = HWTIMER_CH3;
    rt_device_control(dev_hwtimer2, HWTIMER_CTRL_STOP,  &hwc2);
    hwc2.ch = HWTIMER_CH4;
    rt_device_control(dev_hwtimer2, HWTIMER_CTRL_STOP,  &hwc2);
    rt_kprintf("Timer Stoped\n");    
    /* read the counter */
    //rt_device_read(dev_hwtimer2, 0, &val, sizeof(val));
    //rt_kprintf("Read: Sec = %d, Usec = %d\n", val.sec, val.usec);

EXIT_TIM2:
    err = rt_device_close(dev_hwtimer2);
    rt_kprintf("Close %s\n", TIMER2);
    
#endif /*RT_USING_HWTIM2*/  
#endif 
    
#if 0
#ifdef RT_USING_HWTIM3
    #define TIMER3   "timer3"
    rt_device_t dev_hwtimer3 = RT_NULL;
    
    static uint16_t val3;
    rt_hwtimer_chval_t hwc3;
    hwc3.ch = 3;
    
    rt_kprintf("Now test the %s ... \n", TIMER3);
    
    if ((dev_hwtimer3 = rt_device_find(TIMER3)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER3);
        return -1;
    }
    rt_device_hwtimer_t *timer3 = (rt_device_hwtimer_t *)dev_hwtimer3;
    timer3 = (rt_device_hwtimer_t *)dev_hwtimer3;  
    timer3->freq = 1200;
    timer3->prescaler = 71;
    timer3->reload = 0;

    if (rt_device_open(dev_hwtimer3, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", TIMER3);
        return -1;
    }
    
    rt_device_set_rx_indicate(dev_hwtimer3, timer3a_timeout_cb);
    /* 计数时钟设置(默认1Mhz或支持的最小计数频率) */
    hwc3.value = timer3->freq;
    err = rt_device_control(dev_hwtimer3, HWTIMER_CTRL_SET_FREQ, &hwc3);
    if (err != RT_EOK)
    {
        rt_kprintf("Set Freq = %dHz Fail\n", timer3->freq);
        goto EXIT_TIM3;
    }
    else
    {
        rt_kprintf("Set Freq = %dHz ok\n", timer3->freq);
    }
    
    /* 周期模式 */
    //mode = HWTIMER_MODE_PERIOD;
    //err = rt_device_control(dev, HWTIMER_CTRL_MODE_SET, &mode);
    
    rt_kprintf(" default reload = %d \n", timer3->reload);  
       
    err = rt_device_control(dev_hwtimer3, HWTIMER_CTRL_GET_AUTORELOAD, &hwc3); 
    if (err != RT_EOK)
    {
        rt_kprintf("Get the timer reload Fail\n");
        goto EXIT_TIM3;
    }
    else
    {
        rt_kprintf("Get the timer reload  = %d \n", hwc3.value);
    }
    //rt_kprintf(" default Get ch = %d pwm = %d \n", ch, timer->pwm_duty_cycle[ch]);
    
    
    err = rt_device_control(dev_hwtimer3, HWTIMER_CTRL_GET_DUTY_CYCLE, &hwc3);
    if (err != RT_EOK)
    {
        rt_kprintf("Get ch = %d pwm Fail\n", hwc3.ch);
        goto EXIT_TIM3;
    }
    else
    {
        rt_kprintf("Get ch = %d pwm = %d ok\n", hwc3.ch, hwc3.value);
    }
    err = rt_device_control(dev_hwtimer3, HWTIMER_CTRL_GET_AUTORELOAD, &hwc3); 
    if (err != RT_EOK)
    {
        rt_kprintf("Get the timer reload Fail\n");
        goto EXIT_TIM3;
    }
    else
    {
        rt_kprintf("Get the timer reload  = %d \n", hwc3.value);
    }
    //timer->pwm_duty_cycle[ch] += 100;
    val3 = hwc3.value;
    hwc3.value = val3*1/3;
    err = rt_device_control(dev_hwtimer3, HWTIMER_CTRL_SET_DUTY_CYCLE, (rt_hwtimer_chval_t*)&hwc3);
    if (err != RT_EOK)
    {
        rt_kprintf("Set ch  = %d pwm Fail\n", hwc3.ch);
        goto EXIT_TIM3;
    }
    else
    {
        rt_kprintf("Set ch = %d pwm = %d ok,\n", hwc3.ch, hwc3.value);
    }
    //tick = rt_tick_get();
    //rt_kprintf("Start Timer> Tick: %d\n", tick);
    /* 设置定时器超时值并启动定时器 */
    tmr.sec = t;
    tmr.usec = 0;
    
    rt_kprintf("SetTime: Sec %d, Usec %d\n", tmr.sec, tmr.usec);
    rt_device_control(dev_hwtimer3, HWTIMER_CTRL_START, &hwc3);
    /*if (rt_device_write(dev_hwtimer3, hwc3.ch, &tmr, sizeof(tmr)) != sizeof(tmr))
    {
        rt_kprintf("SetTime Fail\n");
        goto EXIT_TIM3;
    }*/
    rt_kprintf("The timer will work on %d sec.\n", t/2);
    
    rt_thread_delay(t*RT_TICK_PER_SECOND/2 );
    rt_device_control(dev_hwtimer3, HWTIMER_CTRL_STOP, &hwc3);
    //ch = 2;   
    err = rt_device_control(dev_hwtimer3, HWTIMER_CTRL_GET_DUTY_CYCLE, &hwc3);
    if (err != RT_EOK)
    {
        rt_kprintf("Get ch = %d pwm Fail\n", hwc3.ch);
        goto EXIT_TIM3;
    }
    else
    {
        rt_kprintf("Get ch = %d pwm = %d ok\n", hwc3.ch, hwc3.value);
    }
    err = rt_device_control(dev_hwtimer3, HWTIMER_CTRL_GET_AUTORELOAD, &hwc3); 
    if (err != RT_EOK)
    {
        rt_kprintf("Get the timer reload Fail\n");
        goto EXIT_TIM3;
    }
    else
    {
        rt_kprintf("Get the timer reload  = %d \n", hwc3.value);
    }
    val3 = hwc3.value;
    hwc3.value = val3*2/3;
    err = rt_device_control(dev_hwtimer3, HWTIMER_CTRL_SET_DUTY_CYCLE, &hwc3);
    if (err != RT_EOK)
    {
        rt_kprintf("Set ch  = %d pwm Fail\n", hwc3.ch);
        goto EXIT_TIM3;
    }
    else
    {
        rt_kprintf("Set ch = %d pwm = %d ok,\n", hwc3.ch, hwc3.value);
    }
    //rt_device_control(dev_hwtimer3, HWTIMER_CTRL_START, &hwc3);
    tmr.sec = t/2;
    if (rt_device_write(dev_hwtimer3, hwc3.ch, &tmr, sizeof(tmr)) != sizeof(tmr))
    {
        rt_kprintf("SetTime Fail\n");
        goto EXIT_TIM3;
    }
    rt_kprintf("The timer will work on %d sec.\n", t/2);
    
    rt_thread_delay(t*RT_TICK_PER_SECOND/2);
    rt_device_control(dev_hwtimer3, HWTIMER_CTRL_STOP, &hwc3);
    rt_device_read(dev_hwtimer3, hwc3.ch, &tmr, sizeof(tmr));
    rt_kprintf("Read: Sec = %d, Usec = %d\n", tmr.sec, tmr.usec);
    rt_kprintf("Read: timer->cycle[%d] = %d, \n", hwc3.ch, timer3->cycles[hwc3.ch]);
    
EXIT_TIM3:
    err = rt_device_close(dev_hwtimer3);
    rt_kprintf("Close %s\n", TIMER3);
    
#endif /*RT_USING_HWTIM3*/ 
#endif
    
#if 0   
#ifdef RT_USING_HWTIM4
    #define TIMER4   "timer4"
    rt_device_t dev_hwtimer4 = RT_NULL;  
    rt_hwtimer_chval_t hwc4;
    rt_hwtimer_chfreq_t hwq4;
    rt_hwtimer_tmrval_t hwt4;
    static uint8_t val4 = 0;
    hwq4.ch = 0;
    //rt_pin_mode(20, PIN_MODE_OUTPUT);// the port PF8
   
    rt_kprintf("Now test the %s ... \n", TIMER4);
    
    if ((dev_hwtimer4 = rt_device_find(TIMER4)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER4);
        return -1;
    }
    rt_device_hwtimer_t *timer4 = (rt_device_hwtimer_t *)dev_hwtimer4;
    timer4 = (rt_device_hwtimer_t *)dev_hwtimer4;  
    timer4->freq = 2400;
    timer4->prescaler = 71;
    timer4->reload = 0;

    if (rt_device_open(dev_hwtimer4, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", TIMER4);
        return -1;
    }
    hwq4.freq = timer4->freq = 2400;
    //rt_device_set_rx_indicate(dev_hwtimer4, timer4a_timeout_cb);
    /* 计数时钟设置(默认1Mhz或支持的最小计数频率) */
    err = rt_device_control(dev_hwtimer4, HWTIMER_CTRL_SET_FREQ, &hwq4);
    if (err != RT_EOK)
    {
        rt_kprintf("Set Freq = %fHz Fail\n", hwq4.freq);
        goto EXIT_TIM4;
    }
    else
    {
        rt_kprintf("Set Freq = %fHz ok\n", hwq4.freq);
    }

    /* 周期模式 */
    //mode = HWTIMER_MODE_PERIOD;
    //err = rt_device_control(dev, HWTIMER_CTRL_MODE_SET, &mode);

    tick = rt_tick_get();
    rt_kprintf("Start Timer> Tick: %d\n", tick);
    /* 设置定时器超时值并启动定时器 */
    /*
    hwt4.sec = t;
    hwt4.usec = 0;
    rt_kprintf("SetTime: Sec %d, Usec %d\n", hwt4.sec, hwt4.usec);
    
    for( uint8_t i = HWTIMER_CH1; i < HWTIMER_CH1 + 4; i++)
    {
        
        if (rt_device_write(dev_hwtimer4, i, &hwt4, sizeof(hwt4)) != sizeof(hwt4))
        {
            rt_kprintf("SetTime Fail\n");
            goto EXIT_TIM4;
        }
        else
        {
            rt_kprintf("Set %s work on  = %d sec. on channel PB%d ok.\n", TIMER4, hwt4.sec, i+5);
        }       
    }
    rt_kprintf("The timer will work on %d sec.\n", t/2);
    rt_thread_delay(t*RT_TICK_PER_SECOND/2 + 1);
     
    hwc4.ch = HWTIMER_CH1;    
    rt_kprintf(" default reload = %d \n", timer4->reload);       
    err = rt_device_control(dev_hwtimer4, HWTIMER_CTRL_GET_AUTORELOAD, &hwc4);    
    if (err != RT_EOK)
    {
        rt_kprintf("Get the timer reload Fail\n");
        goto EXIT_TIM4;
    }
    else
    {
        rt_kprintf("Get the timer reload  = %d \n", val4);
    }
    //rt_kprintf(" default Get ch = %d pwm = %d \n", ch, timer->pwm_duty_cycle[ch]);
    
    rt_kprintf("Get the ch No.%d duty cycle ...\n", hwc4.ch);
    err = rt_device_control(dev_hwtimer4, HWTIMER_CTRL_GET_DUTY_CYCLE, &hwc4);
    if (err != RT_EOK)
    {
        rt_kprintf("Get ch  = %d pwm Fail\n", hwc4.ch);
        goto EXIT_TIM4;
    }
    else
    {
        rt_kprintf("Get ch = %d pwm = %d ok,\n", hwc4.ch, hwc4.value);
    }
    //hwc4.ch = HWTIMER_CH1;
    hwc4.value += 100;
    rt_kprintf("Set the ch No.%d duty cycle ...\n", hwc4.ch);   
    err = rt_device_control(dev_hwtimer4, HWTIMER_CTRL_SET_DUTY_CYCLE, &hwc4);
    if (err != RT_EOK)
    {
        rt_kprintf("Set ch  = %d pwm Fail\n", hwc4.ch);
        goto EXIT_TIM4;
    }
    else
    {
        rt_kprintf("Set ch = %d pwm = %d ok,\n", hwc4.ch, hwc4.value);
    }*/
    //hwc4.ch = HWTIMER_CH1;
    //rt_device_control(dev_hwtimer4, HWTIMER_CTRL_START, &hwc4);
    //hwc4.ch = HWTIMER_CH2;
    //rt_device_control(dev_hwtimer4, HWTIMER_CTRL_START, &hwc4);
    hwc4.ch = HWTIMER_CH3;
    rt_device_control(dev_hwtimer4, HWTIMER_CTRL_START, &hwc4);
    hwc4.ch = HWTIMER_CH4;
    rt_device_control(dev_hwtimer4, HWTIMER_CTRL_START, &hwc4);
    
    
    rt_kprintf("The timer will work on %d sec.\n", t/2);
    rt_thread_delay(t*RT_TICK_PER_SECOND + 1);
    
    /* stop the timer */
    //hwc4.ch = HWTIMER_CH1;
    //rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP,  &hwc4);
    //hwc4.ch = HWTIMER_CH2;
    //rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP,  &hwc4);
    hwc4.ch = HWTIMER_CH3;
    rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP,  &hwc4);
    hwc4.ch = HWTIMER_CH4;
    rt_device_control(dev_hwtimer4, HWTIMER_CTRL_STOP,  &hwc4);
    rt_kprintf("Timer Stoped\n");
    /* read the counter */
    //rt_device_read(dev_hwtimer4, HWTIMER_CH1, &hwt4, sizeof(hwt4));
    //rt_kprintf("Read: Sec = %d, Usec = %d\n", hwt4.sec, hwt4.usec);

EXIT_TIM4:
    err = rt_device_close(dev_hwtimer4);
    rt_kprintf("Close %s\n", TIMER4);
    
#endif /*RT_USING_HWTIM4*/     
#endif


#if 0  
#ifdef RT_USING_HWTIM1

    #define TIMER1   "timer1"
    rt_device_t dev_hwtimer1 = RT_NULL;    
    rt_hwtimer_chval_t hwc;
    rt_hwtimer_chfreq_t hwq;
    rt_device_hwtimer_t *timer1;
    if ((dev_hwtimer1 = rt_device_find(TIMER1)) == RT_NULL)
    {
        rt_kprintf("No Device: %s\n", TIMER1);
        goto EXIT_TIM1;
    } 
    timer1 = (rt_device_hwtimer_t *)dev_hwtimer1;  
    timer1->freq = 1000;
    timer1->prescaler = 71;
    timer1->reload = 833;
    if (rt_device_open(dev_hwtimer1, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Open %s Fail\n", TIMER1);
        goto EXIT_TIM1;
    }
    //rt_device_set_rx_indicate(dev_hwtimer1, timer1_timeout_cb);
    hwq.freq = timer1->freq = 1000;
    err = rt_device_control(dev_hwtimer1, HWTIMER_CTRL_SET_FREQ, &hwq);
    if (err != RT_EOK)
    {
        rt_kprintf("Set timer freq = %d Hz Fail! And close the %s\n" ,timer1->freq, TIMER1);
        //err = rt_device_close(dev_hwtimer1);
        goto EXIT_TIM1;
    }
    hwc.ch = HWTIMER_CH1;
    rt_device_control(dev_hwtimer1, HWTIMER_CTRL_START, &hwc);
    
    if (err != RT_EOK)
    {
        rt_kprintf("Timer freq = %d Hz Fail! And close the %s\n" ,timer1->freq, TIMER1);
        //err = rt_device_close(dev_hwtimer1);
        goto EXIT_TIM1;
    } 
    rt_thread_delay(10*RT_TICK_PER_SECOND);   
EXIT_TIM1:
    err = rt_device_close(dev_hwtimer1);
    rt_kprintf("Close %s\n", TIMER1);    
#endif /* RT_USING_HWTIM1 */   
#endif      
    
    return err;

}

FINSH_FUNCTION_EXPORT(hwtimer, Test hardware timer);
//#endif /* RT_USING_HWTIMER_TEST */
#endif /* RT_USING_HWTIMER */
