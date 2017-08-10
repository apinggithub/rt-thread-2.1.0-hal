#ifndef STM32_SPI_H_INCLUDED
#define STM32_SPI_H_INCLUDED

#include <rtdevice.h>

#include "stm32f1xx.h"
#include "board.h"


/* STM32 spi bus driver */
typedef struct drv_spi_bus
{
    struct rt_spi_bus parent;
    SPI_HandleTypeDef spi_handle;      
    IRQn_Type irq;
	
#ifdef RT_USING_SPI_DMA
    DMA_Channel_TypeDef * DMA_Channel_TX;
    DMA_Channel_TypeDef * DMA_Channel_RX;
    uint32_t DMA_Channel_TX_FLAG_TC;
    uint32_t DMA_Channel_TX_FLAG_TE;
    uint32_t DMA_Channel_RX_FLAG_TC;
    uint32_t DMA_Channel_RX_FLAG_TE;
#endif /* RT_USING_SPI_DMA */
	
}drv_spi_bus_t;


typedef struct drv_spi_cs
{
    GPIO_TypeDef * GPIOx;
    uint16_t GPIO_Pin;
}drv_spi_cs_t;

//int rt_platform_init(void);
/* public function list */
rt_err_t stm32_spi_register( struct rt_spi_bus * spi_bus, const char * spi_bus_name);

#endif // STM32_SPI_H_INCLUDED
