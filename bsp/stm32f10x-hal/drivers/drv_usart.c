/*
 * File      : drv_usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
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
 * 2015-08-01     xiaonong     the first version for stm32f7xx
 */

#include "stm32f1xx.h"
#include "drv_usart.h"
#include "board.h"

#include <rtdevice.h>


static rt_err_t drv_configure(struct rt_serial_device *serial,struct serial_configure *cfg)															
{
    struct drv_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct drv_uart *)serial->parent.user_data;

    uart->UartHandle.Init.BaudRate   = cfg->baud_rate;
    uart->UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    uart->UartHandle.Init.Mode       = UART_MODE_TX_RX;
    uart->UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

    switch (cfg->data_bits)
    {
    case DATA_BITS_8:
        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case DATA_BITS_9:
        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        uart->UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    }
    switch (cfg->stop_bits)
    {
    case STOP_BITS_1:
        uart->UartHandle.Init.StopBits   = UART_STOPBITS_1;
        break;
    case STOP_BITS_2:
        uart->UartHandle.Init.StopBits   = UART_STOPBITS_2;
        break;
    default:
        uart->UartHandle.Init.StopBits   = UART_STOPBITS_1;
        break;
    }
    switch (cfg->parity)
    {
    case PARITY_NONE:
        uart->UartHandle.Init.Parity     = UART_PARITY_NONE;
        break;
    case PARITY_ODD:
        uart->UartHandle.Init.Parity     = UART_PARITY_ODD;
        break;
    case PARITY_EVEN:
        uart->UartHandle.Init.Parity     = UART_PARITY_EVEN;
        break;
    default:
        uart->UartHandle.Init.Parity     = UART_PARITY_NONE;
        break;
    }

    if (HAL_UART_Init(&uart->UartHandle) != HAL_OK)
    {
        return RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t drv_control(struct rt_serial_device *serial,int cmd, void *arg)
{
    struct drv_uart *uart;
    //int len;
    RT_ASSERT(serial != RT_NULL);
    uart = (struct drv_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        NVIC_DisableIRQ(uart->irq);
        /* disable interrupt */
        __HAL_UART_DISABLE_IT(&uart->UartHandle, UART_IT_RXNE);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        NVIC_EnableIRQ(uart->irq);
        /* enable interrupt */
        __HAL_UART_ENABLE_IT(&uart->UartHandle, UART_IT_RXNE);
        break;
    }

    return RT_EOK;
}

static int drv_putc(struct rt_serial_device *serial, char c)
{
    struct drv_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct drv_uart *)serial->parent.user_data;
    
    while((__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_TXE) == RESET));
    uart->UartHandle.Instance->DR = c;
    return 1;
}
//rt_uint8_t temp_uart3_recv[512]={0};
//rt_uint16_t temp_uart3_i =0;
rt_uint8_t temp_uart2_recv[508]={0};
rt_uint16_t temp_uart_i = 0;
static int drv_getc(struct rt_serial_device *serial)
{
    int ch;
    struct drv_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct drv_uart *)serial->parent.user_data;

    ch = -1;
    if (__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_RXNE) != RESET)
    {			
        ch = uart->UartHandle.Instance->DR & 0xff;
				if(1024 > temp_uart_i)
				{
					if(USART2 == uart->UartHandle.Instance)
					{
						temp_uart2_recv[temp_uart_i++] = ch;
						if(507 < temp_uart_i)
						{
							temp_uart_i = 0;
						}
					}
					/*if(USART3 == uart->UartHandle.Instance)
					{
						temp_uart3_recv[temp_uart3_i++] = ch;
						if(511 < temp_uart3_i)
						{
							temp_uart3_i = 0;
						}
					}*/
			   }
				
		}	
    return ch;
}

static const struct rt_uart_ops drv_uart_ops =
{
    drv_configure,
    drv_control,
    drv_putc,
    drv_getc,
};

#if defined(RT_USING_UART1)
/* UART1 device driver structure */
static struct drv_uart uart1;
struct rt_serial_device serial1;

void USART1_IRQHandler(void)
{
    struct drv_uart *uart;

    uart = &uart1;
    /* enter interrupt */
    rt_interrupt_enter();

    /* UART in mode Receiver -------------------------------------------------*/
    if ((__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&uart->UartHandle, UART_IT_RXNE) != RESET))
    {
        rt_hw_serial_isr(&serial1, RT_SERIAL_EVENT_RX_IND);
        /* Clear RXNE interrupt flag */
        __HAL_UART_CLEAR_FLAG(&uart->UartHandle, UART_FLAG_RXNE);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
/* UART2 device driver structure */
static struct drv_uart uart2;
struct rt_serial_device serial2;
uint8_t fp_count = 0;
void USART2_IRQHandler(void)
{
    struct drv_uart *uart;

    uart = &uart2;
    /* enter interrupt */
    rt_interrupt_enter();
    //fp_count++;
    /* UART in mode Receiver -------------------------------------------------*/
    if ((__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&uart->UartHandle, UART_IT_RXNE) != RESET))
    {
			 
			  fp_count++;
        rt_hw_serial_isr(&serial2, RT_SERIAL_EVENT_RX_IND);
        /* Clear RXNE interrupt flag */
        __HAL_UART_CLEAR_FLAG(&uart->UartHandle, UART_FLAG_RXNE);
    }
    /* leave interrupt */
    rt_interrupt_leave();
		//__HAL_UART_CLEAR_FLAG(&uart->UartHandle, UART_FLAG_RXNE);
}
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
/* UART1 device driver structure */
static struct drv_uart uart3;
struct rt_serial_device serial3;

void USART3_IRQHandler(void)
{
    struct drv_uart *uart;

    uart = &uart3;
    /* enter interrupt */
    rt_interrupt_enter();

    /* UART in mode Receiver -------------------------------------------------*/
    if ((__HAL_UART_GET_FLAG(&uart->UartHandle, UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&uart->UartHandle, UART_IT_RXNE) != RESET))
    {
			
        rt_hw_serial_isr(&serial3, RT_SERIAL_EVENT_RX_IND);
        /* Clear RXNE interrupt flag */
        __HAL_UART_CLEAR_FLAG(&uart->UartHandle, UART_FLAG_RXNE);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART3 */

/**
* @brief UART MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration
*           - NVIC configuration for UART interrupt request enable
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
#if defined(RT_USING_UART1)	
    if (huart->Instance == USART1)
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable GPIO TX/RX clock */
        USART1_TX_GPIO_CLK_ENABLE();
        USART1_RX_GPIO_CLK_ENABLE();

        /* Enable USARTx clock */
        USART1_CLK_ENABLE(); 

        /*##-2- Configure peripheral GPIO ##########################################*/  
        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = USART1_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;        
        HAL_GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStruct);
        
        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin = USART1_RX_PIN;       
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_INPUT;
        HAL_GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStruct);
        
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 2);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }	
#endif /* RT_USING_UART1 */		

#if defined(RT_USING_UART2)		
    if (huart->Instance == USART2)
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable GPIO TX/RX clock */
        USART2_TX_GPIO_CLK_ENABLE();
        USART2_RX_GPIO_CLK_ENABLE();

        /* Enable USARTx clock */
        USART2_CLK_ENABLE(); 

        /*##-2- Configure peripheral GPIO ##########################################*/  
        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = USART2_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;        
        HAL_GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStruct);
        
        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = USART2_RX_PIN;       
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_INPUT;
        HAL_GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStruct);
        
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }	
#endif /* RT_USING_UART2 */
	
#if defined(RT_USING_UART3)	
    if (huart->Instance == USART3)
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable GPIO TX/RX clock */
        USART3_TX_GPIO_CLK_ENABLE();
        USART3_RX_GPIO_CLK_ENABLE();

        /* Enable USARTx clock */
        USART3_CLK_ENABLE(); 

        /*##-2- Configure peripheral GPIO ##########################################*/  
        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = USART3_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;        
        HAL_GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStruct);
        
        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin = USART3_RX_PIN;       
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_INPUT;
        HAL_GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStruct);
        
        HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }	
#endif /* RT_USING_UART3 */		
}

/**
* @brief UART MSP De-Initialization
*        This function frees the hardware resources used in this example:
*          - Disable the Peripheral's clock
*          - Revert GPIO and NVIC configuration to their default state
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
#if defined(RT_USING_UART1)
    if (huart->Instance == USART1)
    {
        /*##-1- Reset peripherals ##################################################*/
        USART1_FORCE_RESET();
        USART1_RELEASE_RESET();

        /*##-2- Disable peripherals and GPIO Clocks #################################*/
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(USART1_TX_GPIO_PORT, USART1_TX_PIN);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(USART1_RX_GPIO_PORT, USART1_RX_PIN);

        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }	
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)		
	if (huart->Instance == USART2)
    {
        /*##-1- Reset peripherals ##################################################*/
        USART2_FORCE_RESET();
        USART2_RELEASE_RESET();

        /*##-2- Disable peripherals and GPIO Clocks #################################*/
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(USART2_TX_GPIO_PORT, USART2_TX_PIN);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(USART2_RX_GPIO_PORT, USART2_RX_PIN);

        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }		
#endif /* RT_USING_UART2 */	
	
#if defined(RT_USING_UART3)
    if (huart->Instance == USART3)
    {
        /*##-1- Reset peripherals ##################################################*/
        USART3_FORCE_RESET();
        USART3_RELEASE_RESET();

        /*##-2- Disable peripherals and GPIO Clocks #################################*/
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(USART3_TX_GPIO_PORT, USART3_TX_PIN);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(USART3_RX_GPIO_PORT, USART3_RX_PIN);

        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }	
#endif /* RT_USING_UART3 */	
	
}

int hw_usart_init(void)
{
    struct drv_uart *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef RT_USING_UART1
	
    uart = &uart1;
    uart->UartHandle.Instance = USART1;

    serial1.ops    = &drv_uart_ops;
    serial1.config = config;

    /* register UART1 device */
    rt_hw_serial_register(&serial1, "uart1",
							RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
							uart);
	
#endif /* RT_USING_UART1 */

#ifdef RT_USING_UART2
	  {
			struct serial_configure uart2_cfg;

			uart2_cfg.baud_rate = BAUD_RATE_115200;
			uart2_cfg.bit_order = BIT_ORDER_LSB;
			uart2_cfg.bufsz     = RT_SERIAL_UART2_RB_BUFSZ;
			uart2_cfg.data_bits = DATA_BITS_8;
			uart2_cfg.invert    = NRZ_NORMAL;//NRZ_NORMAL;
			uart2_cfg.parity    = PARITY_NONE;
			uart2_cfg.reserved  = 0;
			uart2_cfg.stop_bits = STOP_BITS_1;
			
			uart = &uart2;
			uart->UartHandle.Instance = USART2;

			serial2.ops    = &drv_uart_ops;
			serial2.config = uart2_cfg;

			/* register UART2 device */
			rt_hw_serial_register(&serial2, "uart2", 
								RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
								uart);
	  }
#endif /* RT_USING_UART2 */

#ifdef RT_USING_UART3
	
    uart = &uart3;
    uart->UartHandle.Instance = USART3;

    serial3.ops    = &drv_uart_ops;
    serial3.config = config;

    /* register UART1 device */
    rt_hw_serial_register(&serial3, "uart3",
							RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,
							uart);
	
#endif /* RT_USING_UART3 */

    return 0;
}
INIT_BOARD_EXPORT(hw_usart_init);
