#include "drv_stm32f10x_spi.h"

#define CS_HIGH()  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
#define CS_LOW()  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)

#define Dummy_Byte 0xFF
static rt_err_t configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration);
static rt_uint32_t xfer(struct rt_spi_device* device, struct rt_spi_message* message);

static struct rt_spi_ops stm32_spi_ops =
{
    configure,
    xfer
};

#ifdef RT_USING_SPI1
static struct rt_spi_bus  _spibus1;
SPI_HandleTypeDef hspi1;
#define SPI1_NAME "spi1"
#endif /* #ifdef USING_SPI1 */

#ifdef RT_USING_SPI2
static struct rt_spi_bus  _spibus2;
SPI_HandleTypeDef hspi2;
#define SPI2_NAME "spi2"
#endif /* #ifdef USING_SPI2 */

#ifdef RT_USING_SPI3
static struct rt_spi_bus  _spibus3;
SPI_HandleTypeDef hspi3;
#define SPI3_NAME "spi3"
#endif /* #ifdef USING_SPI3 */

#ifdef  RT_USING_SPI_FLASH
 static struct rt_spi_device spi_flash_device;
 static struct drv_spi_cs  spi_flash_cs;
 #endif
//SPI_HandleTypeDef hspi3;
//------------------ DMA ------------------
#ifdef RT_USING_SPI_DMA
static uint8_t dummy = 0xFF;
#endif



#ifdef RT_USING_SPI_DMA
static void DMA_Configuration(struct stm32_spi_bus * stm32_spi_bus, const void * send_addr, void * recv_addr, rt_size_t size)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_ClearFlag(stm32_spi_bus->DMA_Channel_RX_FLAG_TC
                  | stm32_spi_bus->DMA_Channel_RX_FLAG_TE
                  | stm32_spi_bus->DMA_Channel_TX_FLAG_TC
                  | stm32_spi_bus->DMA_Channel_TX_FLAG_TE);

    /* RX channel configuration */
    DMA_Cmd(stm32_spi_bus->DMA_Channel_RX, DISABLE);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(stm32_spi_bus->SPI->DR));
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_InitStructure.DMA_BufferSize = size;

    if(recv_addr != RT_NULL)
    {
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32) recv_addr;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    }
    else
    {
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32) (&dummy);
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }

    DMA_Init(stm32_spi_bus->DMA_Channel_RX, &DMA_InitStructure);

    DMA_Cmd(stm32_spi_bus->DMA_Channel_RX, ENABLE);

    /* TX channel configuration */
    DMA_Cmd(stm32_spi_bus->DMA_Channel_TX, DISABLE);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(stm32_spi_bus->SPI->DR));
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    DMA_InitStructure.DMA_BufferSize = size;

    if(send_addr != RT_NULL)
    {
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32)send_addr;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    }
    else
    {
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(&dummy);;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }

    DMA_Init(stm32_spi_bus->DMA_Channel_TX, &DMA_InitStructure);

    DMA_Cmd(stm32_spi_bus->DMA_Channel_TX, ENABLE);
}
#endif

rt_inline uint16_t get_spi_BaudRatePrescaler(rt_uint32_t max_hz)
{
    uint16_t SPI_BaudRatePrescaler;

    /* STM32F10x SPI MAX 18Mhz */
    if(max_hz >= SystemCoreClock/2 && SystemCoreClock/2 <= 18000000)
    {
        SPI_BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    }
    else if(max_hz >= SystemCoreClock/4)
    {
        SPI_BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    }
    else if(max_hz >= SystemCoreClock/8)
    {
        SPI_BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    }
    else if(max_hz >= SystemCoreClock/16)
    {
        SPI_BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    }
    else if(max_hz >= SystemCoreClock/32)
    {
        SPI_BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    }
    else if(max_hz >= SystemCoreClock/64)
    {
        SPI_BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    }
    else if(max_hz >= SystemCoreClock/128)
    {
        SPI_BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    }
    else
    {
        /* min prescaler 256 */
        SPI_BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    }

    return SPI_BaudRatePrescaler;
}

static rt_err_t configure(struct rt_spi_device* device, struct rt_spi_configuration* configuration)
{
    //drv_spi_bus_t * spibus = (drv_spi_bus_t *)device;
	//drv_spi_bus_t * spibus = (drv_spi_bus_t *)(device->bus);
		SPI_HandleTypeDef *hspix;
		#ifdef RT_USING_SPI1
		if(&_spibus1 == device->bus)
		{
			hspix = &hspi1;
		}
		#endif
		#ifdef RT_USING_SPI2
		if(&_spibus2 == device->bus)
		{
			hspix = &hspi2;
		}
		#endif
		#ifdef RT_USING_SPI3
	  if(&_spibus3 == device->bus)
		{
			hspix = &hspi3;
		}
		#endif
		__HAL_SPI_ENABLE(hspix);
		//SPI_TypeDef *  SPIx;
    /* data_width */    
	#if 1
    if(configuration->data_width <= 8)
    {
				hspix->Init.DataSize = SPI_DATASIZE_8BIT;
    }
    else if(configuration->data_width <= 16)
    {
        hspix->Init.DataSize = SPI_DATASIZE_16BIT;
    }
    else
    {
        return RT_EIO;
    }
    /* baudrate */
    hspix->Init.BaudRatePrescaler = get_spi_BaudRatePrescaler(configuration->max_hz);
    /* CPOL */
    if(configuration->mode & RT_SPI_CPOL)
    {
        hspix->Init.CLKPolarity = SPI_POLARITY_HIGH;
    }
    else
    {
        hspix->Init.CLKPolarity = SPI_POLARITY_LOW;
    }
    /* CPHA */
    if(configuration->mode & RT_SPI_CPHA)
    {
        hspix->Init.CLKPhase = SPI_PHASE_2EDGE;
    }
    else
    {
        hspix->Init.CLKPhase = SPI_PHASE_1EDGE;
    }
    /* MSB or LSB */
    if(configuration->mode & RT_SPI_MSB)
    {
        hspix->Init.FirstBit = SPI_FIRSTBIT_MSB;
    }
    else
    {
        hspix->Init.FirstBit = SPI_FIRSTBIT_LSB;
    }
    hspix->Init.Mode = SPI_MODE_MASTER;
    hspix->Init.Direction = SPI_DIRECTION_2LINES;
    hspix->Init.NSS = SPI_NSS_SOFT;
    hspix->Init.TIMode = SPI_TIMODE_DISABLE;//SPI_TIMODE_ENABLE;
    hspix->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspix->Init.CRCPolynomial = 7;
 
    //hspix->Instance = stm32_spi_bus->spiHandle.Instance;
    HAL_SPI_Init(hspix);
    //__HAL_SPI_DISABLE(&hspi);
    //__HAL_SPI_ENABLE(&hspi);
    //SPIx = stm32_spi_bus->spiHandle.Instance;
		#if 1
    if(SPI1 == hspix->Instance)
		{
				__HAL_RCC_SPI1_CLK_DISABLE();
        __HAL_RCC_SPI1_CLK_ENABLE();
		}
    else if(SPI2 == hspix->Instance)
		{
				__HAL_RCC_SPI2_CLK_DISABLE();
        __HAL_RCC_SPI2_CLK_ENABLE();
		}
    else if(SPI3 == hspix->Instance)
		{
				__HAL_RCC_SPI3_CLK_DISABLE();
        __HAL_RCC_SPI3_CLK_ENABLE();
		}
    #endif
    #endif
			
    return RT_EOK;
};

uint8_t SPI_FLASH_ReadByte(SPI_HandleTypeDef *hspi)
{
  uint8_t d_read,d_send=Dummy_Byte;
  if(HAL_SPI_TransmitReceive(hspi,&d_send,&d_read,1,0xFFFFFF)!=HAL_OK)
    d_read=Dummy_Byte;
  
  return d_read;    
}

/**
  * 函数功能: 往串行Flash读取写入一个字节数据并接收一个字节数据
  * 输入参数: byte：待发送数据
  * 返 回 值: uint8_t：接收到的数据
  * 说    明：无
  */
uint8_t SPI_FLASH_SendByte(SPI_HandleTypeDef *hspi,uint8_t byte)
{
  uint8_t d_read,d_send=byte;
  if(HAL_SPI_TransmitReceive(hspi,&d_send,&d_read,1,0xFFFFFF)!=HAL_OK)
    d_read=Dummy_Byte;
  
  return d_read; 
}

static rt_uint32_t xfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    //struct stm32_spi_bus * stm32_spi_bus = (struct stm32_spi_bus *)device->bus;
    struct rt_spi_configuration * config = &device->config;
    //SPI_TypeDef * SPI = stm32_spi_bus->SPI->Instance;
    drv_spi_cs_t * spi_cs = device->parent.user_data;
    rt_uint32_t size = message->length;
	//SPI_HandleTypeDef hspix = ;
		SPI_HandleTypeDef hspix ;
		#ifdef RT_USING_SPI1
		if(&_spibus1 == device->bus)
		{
			hspix = hspi1;
		}
		#endif
		#ifdef RT_USING_SPI2
		if(&_spibus2 == device->bus)
		{
			hspix = hspi2;
		}
		#endif
		#ifdef RT_USING_SPI3
	  if(&_spibus3 == device->bus)
		{
			hspix = hspi3;
		}
		#endif
    if(message->cs_take)
    {
        //__HAL_SPI_ENABLE(&hspi);
        HAL_GPIO_WritePin(spi_cs->GPIOx,spi_cs->GPIO_Pin,GPIO_PIN_RESET);			  
    }
    
#ifdef RT_USING_SPI_DMA
    if(message->length > 32)
    {
        if(config->data_width <= 8)
        {
            /*DMA_Configuration(stm32_spi_bus, message->send_buf, message->recv_buf, message->length);
            SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
            while (DMA_GetFlagStatus(stm32_spi_bus->DMA_Channel_RX_FLAG_TC) == RESET
                    || DMA_GetFlagStatus(stm32_spi_bus->DMA_Channel_TX_FLAG_TC) == RESET);
            SPI_I2S_DMACmd(SPI, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);*/
					//SPI_DMAEndTransmitReceive(stm32_spi_bus->SPI);
			HAL_SPI_TransmitReceive_DMA(hspi,message->send_buf,message->recv_buf, message->length);
        }
//        rt_memcpy(buffer,_spi_flash_buffer,DMA_BUFFER_SIZE);
//        buffer += DMA_BUFFER_SIZE;
    }
    else
#endif
    {
        if(config->data_width <= 8)
        {
            rt_uint8_t* send_ptr = (rt_uint8_t*)(message->send_buf);
			//const rt_uint8_t  send_ptr1 = message->send_buf;
            rt_uint8_t*  recv_ptr =    message->recv_buf;
            #if 1					   
            if((send_ptr != RT_NULL)&&(recv_ptr != RT_NULL))
            {
							
                HAL_SPI_TransmitReceive(&hspix,send_ptr,message->recv_buf,size, 1000);
            }
            else if(send_ptr != RT_NULL)
            {
							/*uint8_t dumy[size];
							for(rt_uint32_t i = 0; i < size; i++)
							{
								dumy[i] = 0xff;
							}
               HAL_SPI_TransmitReceive(&hspix,send_ptr,dumy,size, 1000);*/
                HAL_SPI_Transmit(&hspix,send_ptr,size, 1000);                
            }
            else if(recv_ptr != RT_NULL)
            {
							/*uint8_t dumy[size];
							for(uint32_t i = 0; i < size; i++)
							{
								dumy[i] = 0xff;
							}*/
							//HAL_SPI_TransmitReceive(&hspix,dumy,message->recv_buf,size, 1000);
               HAL_SPI_Receive(&hspix,message->recv_buf,size, 1000);
            }
            #else
            //__HAL_SPI_ENABLE(&hspi);
							/*while(1)
						{
							uint8_t recv[4];
							HAL_GPIO_WritePin(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin,GPIO_PIN_RESET);
							SPI_FLASH_SendByte(&hspi,0x9f);
							recv[0] = SPI_FLASH_SendByte(&hspi,0xFF);
							recv[1] = SPI_FLASH_SendByte(&hspi,0xFF);
							recv[2] = SPI_FLASH_SendByte(&hspi,0xFF);
							recv[3] = SPI_FLASH_SendByte(&hspi,0xFF);
							HAL_GPIO_WritePin(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin,GPIO_PIN_SET);
						}*/
            while(size--)
            {
               // rt_uint8_t data = 0;//0xFF;

                if(send_ptr != RT_NULL)
                {
                    //data = *send_ptr++;
									SPI_FLASH_SendByte(&hspix,*send_ptr++);
                }
                //Wait until the transmit buffer is empty
                //while (__HAL_SPI_GET_FLAG(&hspi, SPI_FLAG_TXE) == RESET);
                // Send the byte
                //(&hspi)->Instance->DR = data;

                //Wait until a data is received
                //while (__HAL_SPI_GET_FLAG(&hspi, SPI_FLAG_RXNE) == RESET);
                // Get the received data
                //data = (&hspi)->Instance->DR;

                if(recv_ptr != RT_NULL)
                {
                    //*recv_ptr++ = data;
									*recv_ptr++ = SPI_FLASH_ReadByte(&hspix);
                }
            }
					
            #endif
							
        }
        else if(config->data_width <= 16)
        {
            const rt_uint8_t * send_ptr = message->send_buf;
            rt_uint8_t * recv_ptr = message->recv_buf;
						
            if(send_ptr != RT_NULL)
            {
                HAL_SPI_Transmit(&hspix,(uint8_t*)send_ptr,size, 1000);
                //HAL_SPI_TransmitReceive(&hspi,(uint8_t*)send_ptr,recv,5, 1000);//size
            }
            if(recv_ptr != RT_NULL)
            {
                HAL_SPI_Receive(&hspix,recv_ptr,size, 1000);
            }
        }
    }

    /* release CS */
    if(message->cs_release)
    {
        //__HAL_SPI_DISABLE(&hspi);
        HAL_GPIO_WritePin(spi_cs->GPIOx, spi_cs->GPIO_Pin,GPIO_PIN_SET);        
    }

    return message->length;
};

/** \brief init and register stm32 spi bus.
 *
 * \param SPI: STM32 SPI, e.g: SPI1,SPI2,SPI3.
 * \param stm32_spi: stm32 spi bus struct.
 * \param spi_bus_name: spi bus name, e.g: "spi1"
 * \return
 *
 */
rt_err_t stm32_spi_register( struct rt_spi_bus * spi_bus, const char * spi_bus_name )
															
{
	
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	  //struct rt_spi_bus spi_device_bus ;
    //SPI_HandleTypeDef hspix;
	
#ifdef RT_USING_SPI1
    	//stm32_spi->spiHandle.Instance = SPI1;
#ifdef RT_USING_SPI_DMA
        /* Enable the DMA1 Clock */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

        hspi1->DMA_Channel_RX = DMA1_Channel2;
        spi_bus->DMA_Channel_TX = DMA1_Channel3;
        spi_bus->DMA_Channel_RX_FLAG_TC = DMA1_FLAG_TC2;
        spi_bus->DMA_Channel_RX_FLAG_TE = DMA1_FLAG_TE2;
        spi_bus->DMA_Channel_TX_FLAG_TC = DMA1_FLAG_TC3;
        spi_bus->DMA_Channel_TX_FLAG_TE = DMA1_FLAG_TE3;
#endif
#endif  
	
    return rt_spi_bus_register(spi_bus, spi_bus_name, &stm32_spi_ops);
}


static int rt_hw_spi_init(void)
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
				
				hspi1.Instance = SPI1;
		
        stm32_spi_register(&_spibus1, "spi1");
							
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
				//_spibus1.parent = spi_soft_bus;
			  spi_flash_device.bus = &_spibus1;
        rt_spi_bus_attach_device(&spi_flash_device, "spiflash", "spi1", (void*)&spi_flash_cs);
		#endif   		
    }
#endif /* RT_USING_SPI1 */
		return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_spi_init);

