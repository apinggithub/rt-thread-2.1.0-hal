/* RT-Thread config file */
#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__

/*Using application mode*/
//#define RT_USING_LIGHT_WAVE_CURER

/* RT_NAME_MAX*/
#define RT_NAME_MAX	8

/* RT_ALIGN_SIZE*/
#define RT_ALIGN_SIZE	4

/* PRIORITY_MAX */
#define RT_THREAD_PRIORITY_MAX	32

/* Tick per Second */
#define RT_TICK_PER_SECOND	1000

/* SECTION: RT_DEBUG */
/* Thread Debug */
#define RT_DEBUG
#define RT_THREAD_DEBUG

#define RT_USING_OVERFLOW_CHECK

/* Using Hook */
#define RT_USING_HOOK

 
/*Using adc converter */
//#define RT_USING_HWADC
//#define RT_USING_HWADC_TEST /*ADC1 channel test*/

/* Using Software Timer */
/* #define RT_USING_TIMER_SOFT */
#define RT_TIMER_THREAD_PRIO		4
#define RT_TIMER_THREAD_STACK_SIZE	512
#define RT_TIMER_TICK_PER_SECOND	10

/* Using independent hardware Timer, the channel and mode set of the timer is in drv_hwtimer.h*/
//#define RT_USING_HWTIMER  /*hardware timer driver switch*/
//#define RT_USING_HWTIMER_TEST  /*hardware timer driver test cmd in finsh*/
#define RT_USING_HWTIMER_HOOK
#define RT_USING_HWTIM6 /*the base hardtimer TIM6*/

#define RT_USING_HWTIM2   /*the general hardtimer TIM2*/
//#define RT_USING_HWTIM2_ETR /*external clock*/

#define RT_USING_HWTIM3   /*the general hardtimer TIM2*/
//#define RT_USING_HWTIM3_ETR /*external clock*/

#define RT_USING_HWTIM4   /*the general hardtimer TIM4*/

#define RT_USING_HWTIM1   /*the advance hardtimer TIM1*/
//#define RT_USING_HWTIM_CC_IRQ /* the capture or compare interrupt */
#define RT_USING_HWTIM_UP_IRQ /* the update interrupt */


/* SECTION: IPC */
/* Using Semaphore*/
#define RT_USING_SEMAPHORE

/* Using Mutex */
#define RT_USING_MUTEX

/* Using Event */
#define RT_USING_EVENT

/* Using MailBox */
#define RT_USING_MAILBOX

/* Using Message Queue */
#define RT_USING_MESSAGEQUEUE

/* SECTION: Memory Management */
/* Using Memory Pool Management*/
#define RT_USING_MEMPOOL

/* Using Dynamic Heap Management */
#define RT_USING_HEAP

/* Using Small MM */
#define RT_USING_SMALL_MEM

// <bool name="RT_USING_COMPONENTS_INIT" description="Using RT-Thread components initialization" default="true" />
#define RT_USING_COMPONENTS_INIT

/* SECTION: Device System */
/* Using Device System */
#define RT_USING_DEVICE
// <bool name="RT_USING_DEVICE_IPC" description="Using device communication" default="true" />
#define RT_USING_DEVICE_IPC
// <bool name="RT_USING_SERIAL" description="Using Serial" default="true" />
#define RT_USING_SERIAL

#define RT_USING_UART1
#define RT_USING_UART2
#define RT_USING_UART3
/*SECTION: fingerprint mode*/
#define RT_USING_FINGERPRINT
/* SECTION: WIFI mode */
#define RT_USING_UART_WIFI_ESP8266/* WIFI mode ESP8266*/



/*Using Pin select as device*/
#define RT_USING_PIN
#define RT_USING_PIN_NUMBERS        100 /*[ 64, 100, 144 ] */

/* SECTION: Console options */
#define RT_USING_CONSOLE
/* the buffer size of console*/
#define RT_CONSOLEBUF_SIZE	        256
// <string name="RT_CONSOLE_DEVICE_NAME" description="The device name for console" default="uart1" />
// if you use the console, and you need to configure the 'STM32_CONSOLE_USART' in board.h .
#define RT_CONSOLE_DEVICE_NAME	    "uart1"
#define FP_DEVICE_NAME	    				"uart2"



/* SECTION: finsh, a C-Express shell */
#define RT_USING_FINSH
/* Using symbol table */
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION

/*#define RT_USING_CAN*/
#define RT_CAN_USING_BUS_HOOK
#define RT_CAN_USING_HDR

#define RT_USING_SPI
#define RT_USING_SPI1
#define RT_USING_W25QXX
#define RT_USING_SPI_FLASH

/* SECTION: device filesystem */
#define RT_USING_DFS
#define RT_USING_DFS_ELMFAT
/* Reentrancy (thread safe) of the FatFs module.  */
#define RT_DFS_ELM_REENTRANT
/* Number of volumes (logical drives) to be used. */
#define RT_DFS_ELM_DRIVES			1
/* #define RT_DFS_ELM_USE_LFN			1 */
/* #define RT_DFS_ELM_CODE_PAGE			936 */
#define RT_DFS_ELM_MAX_LFN			255
/* Maximum sector size to be handled. */
#define RT_DFS_ELM_MAX_SECTOR_SIZE  4096

/* the max number of mounted filesystem */
#define DFS_FILESYSTEMS_MAX			2
/* the max number of opened files 		*/
#define DFS_FD_MAX					4


#define RT_USING_USB_DEVICE
#define RT_USB_DEVICE_MSTORAGE



/* SECTION: STemWin GUI */
//#define RT_USING_SPI_LCD
#define RT_USING_FSMC_LCD //fsmc data bus
#define RT_USING_LCD_ILI9341
#define RT_USING_IIC_TOUCH_FT5216
#define RT_USING_STEMWIN



/* SECTION: RT-Thread/GUI */
/* #define RT_USING_RTGUI //(RT_USING_GUIENGINE) */

/* name length of RTGUI object */
#define RTGUI_NAME_MAX		12
/* support 16 weight font */
#define RTGUI_USING_FONT16
/* support Chinese font */
#define RTGUI_USING_FONTHZ
/* use DFS as file interface */
#define RTGUI_USING_DFS_FILERW
/* use font file as Chinese font */
#define RTGUI_USING_HZ_FILE
/* use Chinese bitmap font */
#define RTGUI_USING_HZ_BMP
/* use small size in RTGUI */
#define RTGUI_USING_SMALL_SIZE
/* use mouse cursor */
/* #define RTGUI_USING_MOUSE_CURSOR */
/* default font size in RTGUI */
#define RTGUI_DEFAULT_FONT_SIZE	16

/* image support */
/* #define RTGUI_IMAGE_XPM */
/* #define RTGUI_IMAGE_BMP */

/* SECTION: WIFI mode */
//#define RT_USING_ESP12F /* WIFI mode ESP-12F*/

/* SECTION: lwip, a lighwight TCP/IP protocol stack */
#define RT_USING_LWIP 
/* LwIP uses RT-Thread Memory Management */
#define RT_LWIP_USING_RT_MEM
/* Enable ICMP protocol*/
#define RT_LWIP_ICMP
/* Enable UDP protocol*/
#define RT_LWIP_UDP
/* Enable TCP protocol*/
#define RT_LWIP_TCP
/* Enable DNS */
#define RT_LWIP_DNS

/* the number of simulatenously active TCP connections*/
#define RT_LWIP_TCP_PCB_NUM	5

/* Using DHCP */
/* #define RT_LWIP_DHCP */

/* ip address of target*/
#define RT_LWIP_IPADDR0	192
#define RT_LWIP_IPADDR1	168
#define RT_LWIP_IPADDR2	1
#define RT_LWIP_IPADDR3	30

/* gateway address of target*/
#define RT_LWIP_GWADDR0	192
#define RT_LWIP_GWADDR1	168
#define RT_LWIP_GWADDR2	1
#define RT_LWIP_GWADDR3	1

/* mask address of target*/
#define RT_LWIP_MSKADDR0	255
#define RT_LWIP_MSKADDR1	255
#define RT_LWIP_MSKADDR2	255
#define RT_LWIP_MSKADDR3	0

/* tcp thread options */
#define RT_LWIP_TCPTHREAD_PRIORITY		12
#define RT_LWIP_TCPTHREAD_MBOX_SIZE		10
#define RT_LWIP_TCPTHREAD_STACKSIZE		1024

/* ethernet if thread options */
#define RT_LWIP_ETHTHREAD_PRIORITY		15
#define RT_LWIP_ETHTHREAD_MBOX_SIZE		10
#define RT_LWIP_ETHTHREAD_STACKSIZE		512

/* TCP sender buffer space */
#define RT_LWIP_TCP_SND_BUF	8192
/* TCP receive window. */
#define RT_LWIP_TCP_WND		8192

// <bool name="RT_USING_CMSIS_OS" description="Using CMSIS OS API" default="true" />
// #define RT_USING_CMSIS_OS
// <bool name="RT_USING_RTT_CMSIS" description="Using CMSIS in RTT" default="true" />
#define RT_USING_RTT_CMSIS
// <bool name="RT_USING_BSP_CMSIS" description="Using CMSIS in BSP" default="true" />
// #define RT_USING_BSP_CMSIS

/* nanopb support */
/* #define RT_USING_NANOPB */

#endif
