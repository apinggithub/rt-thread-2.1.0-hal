
#include <rtthread.h>

#ifdef RT_USING_SPI_LCD

#include "drv_spi_lcd.h"

static void 	SPIx_Init(void);
static void 	SPIx_Error (void);
extern void 	HAL_Delay(__IO uint32_t Delay);


#ifdef HAL_SPI_MODULE_ENABLED
uint32_t SpixTimeout = ILI9341_SPIx_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */
static SPI_HandleTypeDef ili9431_Spi;
#endif
#ifdef HAL_SPI_MODULE_ENABLED

/**
  * @brief  Initializes SPI MSP.
  */
static void SPIx_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef  GPIO_InitStruct;  
  
  /*** Configure the GPIOs ***/ 
  ILI9341_SPIx_CLK_ENABLE();	
  /* Enable GPIO clock */
  ILI9341_SPIx_SCK_GPIO_CLK_ENABLE();
  ILI9341_SPIx_MISO_MOSI_GPIO_CLK_ENABLE();
  
  /* Configure SPI SCK */
  GPIO_InitStruct.Pin = ILI9341_SPIx_SCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  //GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  //GPIO_InitStruct.Alternate = ILI9341_SPIx_SCK_AF;
  HAL_GPIO_Init(ILI9341_SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);
  /* Configure SPI MISO and MOSI */ 
	
  GPIO_InitStruct.Pin = ILI9341_SPIx_MOSI_PIN;
  //GPIO_InitStruct.Alternate = ILI9341_SPIx_MISO_MOSI_AF;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;	
  //GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(ILI9341_SPIx_MISO_MOSI_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pin = ILI9341_SPIx_MISO_PIN;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(ILI9341_SPIx_MISO_MOSI_GPIO_PORT, &GPIO_InitStruct);

  /*** Configure the SPI peripheral ***/ 
  /* Enable SPI clock */
  //ILI9341_SPIx_CLK_ENABLE();
}

/**
  * @brief  Initializes SPI HAL.
  */
static void SPIx_Init(void)
{
  //if(HAL_SPI_GetState(&ili9431_Spi) == HAL_SPI_STATE_RESET)
  //{
    /* SPI Config */
    //ili9431_Spi.Instance = ILI9431_SPIx;
      /* SPI baudrate is set to 12,5 MHz maximum (APB1/SPI_BaudRatePrescaler = 100/8 = 12,5 MHz) 
       to verify these constraints:
          - ILI9431 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
            Since the provided driver doesn't use read capability from LCD, only constraint 
            on write baudrate is considered.
          - SD card SPI interface max baudrate is 25MHz for write/read
          - PCLK2 max frequency is 100 MHz 
       */ 
	#if 1
    ili9431_Spi.Instance = ILI9341_SPIx;
	ili9431_Spi.Init.Mode = SPI_MODE_MASTER;
	ili9431_Spi.Init.Direction = SPI_DIRECTION_2LINES;
	ili9431_Spi.Init.DataSize = SPI_DATASIZE_8BIT;
	ili9431_Spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
	ili9431_Spi.Init.CLKPhase = SPI_PHASE_1EDGE;
	ili9431_Spi.Init.NSS = SPI_NSS_SOFT;
	ili9431_Spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	ili9431_Spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	ili9431_Spi.Init.TIMode = SPI_TIMODE_DISABLED;
	ili9431_Spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
	ili9431_Spi.Init.CRCPolynomial = 10;
	SPIx_MspInit(&ili9431_Spi);
	HAL_SPI_Init(&ili9431_Spi);
	/*if (HAL_SPI_Init(&ili9431_Spi) != HAL_OK)
	{
		while(1) 
		{
		}
	}*/
	#endif
	/*ili9431_Spi.Instance = SPI2;
	ili9431_Spi.Init.Mode = SPI_MODE_MASTER;
	ili9431_Spi.Init.Direction = SPI_DIRECTION_2LINES;
	ili9431_Spi.Init.DataSize = SPI_DATASIZE_8BIT;
	ili9431_Spi.Init.CLKPolarity = SPI_POLARITY_LOW;
	ili9431_Spi.Init.CLKPhase = SPI_PHASE_1EDGE;
	ili9431_Spi.Init.NSS = SPI_NSS_SOFT;
	ili9431_Spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	ili9431_Spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	ili9431_Spi.Init.TIMode = SPI_TIMODE_DISABLE;
	ili9431_Spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	ili9431_Spi.Init.CRCPolynomial = 10;
	SPIx_MspInit(&ili9431_Spi);
	if (HAL_SPI_Init(&ili9431_Spi) != HAL_OK)
	{
		while(1) 
		{
		}
	}*/

    //SPIx_MspInit(&ili9431_Spi);
 // }
	
}
#if 0
static void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_TransmitReceive(&ili9431_Spi, (uint8_t*) DataIn, DataOut, DataLegnth, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}
#endif
/**
  * @brief  SPI Write a byte to device.
  * @param  Value: value to be written
  */
static void SPIx_Write(uint8_t Value)
{
	uint8_t recvdate=0xaa;
  HAL_StatusTypeDef status = HAL_OK;
  //uint8_t data;
#if 1
  status = HAL_SPI_Transmit(&ili9431_Spi, (uint8_t*) &Value, 1, SpixTimeout);  	
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
#endif
  #if 0
  while(((ili9431_Spi.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
  {
  }
  /* Need to invert bytes for LCD*/
  *((__IO uint8_t*)&ili9431_Spi.Instance->DR) = Value;
  #endif
  #if 0
  status   =HAL_SPI_TransmitReceive(&ili9431_Spi, (uint8_t*) &Value,(uint8_t*) &recvdate, 2, SpixTimeout);
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
  #endif
}

/**
  * @brief  SPI Read a byte from device.
  * @param  Value: value from read
  */
static uint8_t SPIx_Read(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t Value;
#if 1
  status = HAL_SPI_Receive(&ili9431_Spi, (uint8_t*) &Value, 1, SpixTimeout);
    
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
	return 0;
  }
  else
	return Value; 
#endif
#if 0
	while(((ili9431_Spi.Instance->SR) & SPI_FLAG_RXNE) != SPI_FLAG_RXNE)
	{
	}
	/* Need to invert bytes for LCD*/
	Value =  *((__IO uint8_t*)&ili9431_Spi.Instance->DR); 
	return Value;
#endif
}

/**
  * @brief  SPI error treatment function.
  */
static void SPIx_Error (void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&ili9431_Spi);
  
  /* Re-Initiaize the SPI communication BUS */
  SPIx_Init();
}

/********************************* LINK LCD ***********************************/
/**
  * @brief  Initializes the LCD.
  */
void hw_lcd_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
   
  /* LCD_CS_GPIO and LCD_DC_GPIO Periph clock enable */
  LCD_CS_GPIO_CLK_ENABLE();
  LCD_DC_GPIO_CLK_ENABLE();
  LCD_RST_GPIO_CLK_ENABLE();
  
  /* Configure LCD_CS_PIN pin: LCD CS pin */
  GPIO_InitStruct.Pin = LCD_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(LCD_CS_GPIO_PORT, &GPIO_InitStruct);

  /* Configure LCD_DC_PIN pin: LCD DC pin */
  GPIO_InitStruct.Pin = LCD_DC_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;	
  HAL_GPIO_Init(LCD_DC_GPIO_PORT, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = LCD_BL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LCD_BL_GPIO_PORT, &GPIO_InitStruct);
	
	
  GPIO_InitStruct.Pin = LCD_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LCD_RST_GPIO_PORT, &GPIO_InitStruct);
  
  /* LCD chip select high */
  LCD_CS_HIGH();
  
  /* LCD SPI Config */
  SPIx_Init();
}

/**
  * @brief  Writes command to select the LCD register.
  * @param  LCDReg: Address of the selected register.
  */
void hw_lcd_write_cmd(uint8_t reg)
{
  /* Reset LCD control line CS */
  LCD_CS_LOW();
  
  /* Set LCD data/command line DC to Low */
  LCD_DC_LOW();
    
  /* Send Command */
  SPIx_Write(reg);
  
  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}

/**
  * @brief  Writes data to select the LCD register.
  *         This function must be used after WriteReg() function
  * @param  Data: data to write to the selected register.
  */
void hw_lcd_write_data(uint8_t data)
{

 
  /* Reset LCD control line CS */
  LCD_CS_LOW();
  /* Set LCD data/command line DC to High */
  LCD_DC_HIGH();
  /* Send Data */
  SPIx_Write(data);
  
  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}
/**
  * @brief  Read data from select the LCD register.
  *         This function must be used after WriteReg() function
  * @param  Data: data to write to the selected register.
  */
uint8_t hw_lcd_read_data(void)
{
  uint8_t data;
  /* Reset LCD control line CS */
  LCD_CS_LOW();
  
  /* Set LCD data/command line DC to High */
  LCD_DC_HIGH();

  /* Send Data */
  data = SPIx_Read();
  
  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
		
  return data;	
}
/**
  * @brief  Writes register value.
  * @param  pdata: Pointer on the register value
  * @param  size: Size of byte to transmit to the register
  */
void hw_lcd_write_multidata(uint8_t *pdata, uint32_t size)
{
  uint32_t counter = 0;
  __IO uint32_t data = 0;
  
  /* Reset LCD control line CS */
  LCD_CS_LOW();
  
  /* Set LCD data/command line DC to High */
  LCD_DC_HIGH();

  if (size == 1)
  {
    /* Only 1 byte to be sent to LCD - general interface can be used */
    /* Send Data */
    SPIx_Write(*pdata);
  }
  else
  {
    /* Several data should be sent in a raw */
    /* Direct SPI accesses for optimization */
    for (counter = size; counter != 0; counter--)
    {
      while(((ili9431_Spi.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
      {
      }
      /* Need to invert bytes for LCD*/
      *((__IO uint8_t*)&ili9431_Spi.Instance->DR) = *(pdata+1);
      
      while(((ili9431_Spi.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
      {
      }
      *((__IO uint8_t*)&ili9431_Spi.Instance->DR) = *pdata;
      counter--;
      pdata += 2;
    }
  
    /* Wait until the bus is ready before releasing Chip select */ 
    while(((ili9431_Spi.Instance->SR) & SPI_FLAG_BSY) != RESET)
    {
    } 
  } 

  /* Empty the Rx fifo */
  data = *(&ili9431_Spi.Instance->DR);
  UNUSED(data);

  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}

/**
  * @brief  Writes register value.
  * @param  pdata: Pointer on the register value
  * @param  size: Size of byte to transmit to the register
  */
void hw_lcd_read_multidata(uint8_t *pdata, uint32_t size)
{
  uint32_t counter = 0;
  __IO uint32_t data = 0;
  
  /* Reset LCD control line CS */
  LCD_CS_LOW();
  
  /* Set LCD data/command line DC to High */
  LCD_DC_HIGH();

  if (size == 1)
  {
    /* Only 1 byte to be sent to LCD - general interface can be used */
    /* Send Data */
   *pdata = SPIx_Read();
  }
  else
  {
    /* Several data should be sent in a raw */
    /* Direct SPI accesses for optimization */
    for (counter = size; counter != 0; counter--)
    {
      while(((ili9431_Spi.Instance->SR) & SPI_FLAG_RXNE) != SPI_FLAG_RXNE)
      {
      }
      /* Need to invert bytes for LCD*/
      *(pdata+1) =  *((__IO uint8_t*)&ili9431_Spi.Instance->DR);
      
      while(((ili9431_Spi.Instance->SR) & SPI_FLAG_RXNE) != SPI_FLAG_RXNE)
      {
      }
      *pdata = *((__IO uint8_t*)&ili9431_Spi.Instance->DR);
      counter--;
      pdata += 2;
    }
  
    /* Wait until the bus is ready before releasing Chip select */ 
    while(((ili9431_Spi.Instance->SR) & SPI_FLAG_BSY) != RESET)
    {
    } 
  } 

  /* Empty the Rx fifo */
  data = *(&ili9431_Spi.Instance->DR);
  UNUSED(data);

  /* Deselect : Chip Select high */
  LCD_CS_HIGH();
}
/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  */
void hw_lcd_delay(uint32_t delay)
{
	//HAL_Delay(Delay);
	HAL_Delay(delay);
}

void hw_lcd_reset(void)
{
	LCD_RST_LOW();
	hw_lcd_delay(100);
	LCD_RST_HIGH();
	hw_lcd_delay(50);
}

void  hw_lcd_backlight_on(void)
{
	LCD_BL_HIGH();
}
void  hw_lcd_backlight_off(void)
{
	LCD_BL_LOW();
}

#endif /* HAL_SPI_MODULE_ENABLED */
#endif /* RT_USING_SPI_LCD */
