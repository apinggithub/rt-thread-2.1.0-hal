#if 0
#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include <rthw.h>

#ifdef RT_USING_LWIP
//#include "stm32_eth.h"
#endif /* RT_USING_LWIP */

#ifdef RT_USING_SPI
#include "drv_stm32f10x_spi.h"

#if defined(RT_USING_DFS) && defined(RT_USING_DFS_ELMFAT)
#include <spi_flash_w25qxx.h>
#endif /* RT_USING_DFS */

/*
 * SPI1_MOSI: PA7
 * SPI1_MISO: PA6
 * SPI1_SCK : PA5
 *
 * CS0: PA4  SD card.
 */
 #ifdef  RT_USING_SPI_FLASH
 static struct rt_spi_device spi_device_flash;
 static struct drv_spi_cs  spi_flash_cs;
 #endif

static void rt_hw_spi_init(void)
{

#ifdef RT_USING_SPI
    /* register spi bus */
    {
		#ifdef RT_USING_SPI1
        //static struct stm32_spi_bus stm32_spi1;
        GPIO_InitTypeDef GPIO_InitStruct;

        /* Enable GPIO clock */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        //__HAL_RCC_GPIOC_CLK_ENABLE();
        //__HAL_RCC_GPIOD_CLK_ENABLE();
        __SPI1_CLK_ENABLE();
        
        /* SCK->PC10, MOSI->PC12, MISO->PC11*/
        GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        //GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        //GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
        stm32_spi_register(&stm32_spi1, "spi1");
							
        /*NSS->PA15,flash_cs */
        GPIO_InitStruct.Pin = GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
        #if 0
        /*NSS->PD2,msd_cs*/
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
				#endif
		#endif
    }

    /* attach cs */
    {
        
    #ifdef  RT_USING_SPI_FLASH				
        /* spi_flash cs: PA4 */
        spi_flash_cs.GPIOx = GPIOA;
        spi_flash_cs.GPIO_Pin = GPIO_PIN_4;
        rt_spi_bus_attach_device(&spi_device_flash, "spi_flash", "spi1", (void*)&spi_flash_cs);
		#endif   		
    }
#endif /* RT_USING_SPI1 */
}
#endif /* RT_USING_SPI */


int rt_spi_init(void)
{
#ifdef RT_USING_SPI
    rt_hw_spi_init();
#endif 
#ifdef RT_USING_SPI_FLASH
	  w25qxx_init("sd0", "spi_flash");
#endif 
	return 0;
}//INIT_DEVICE_EXPORT(rt_spi_init);

#endif
