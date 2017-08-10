#include "stm32f1xx_hal.h"
#include <rtthread.h>
#include <rtdevice.h>

/**
  ******************************************************************************
  * @file    ft5216.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   This file provides a set of functions needed to manage the FT5216
  *          touch screen devices.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "drv_iic_touch_ft5216.h"

/* Touch screen driver structure initialization */

//uint8_t ft5216_rcv[8];

CTP_DrvTypeDef ft5216_ts_drv =
{
  ft5216_Init,
  ft5216_ReadID,
  ft5216_Reset,

  ft5216_CTP_Start,
  ft5216_CTP_DetectTouch,
  ft5216_CTP_GetXY,

  ft5216_CTP_EnableIT,
  ft5216_CTP_ClearIT,
  ft5216_CTP_ITStatus,
  ft5216_CTP_DisableIT

};

struct CTP_Msg
{ 
	uint8_t active;
	uint8_t TC_statue;
	uint16_t touchx;
	uint16_t touchy;
	uint8_t  touch_id;
};  


/* Global ft5216 handle */
ft5216_handle_TypeDef ft5216_handle = { FT5216_I2C_NOT_INITIALIZED, 0, 0};

/**
  * 
  */

/** @defgroup ft5216_Private_Function_Prototypes
  * 
  */

/* Private functions prototypes-----------------------------------------------*/






/**
  * @brief  Return the status of I2C was initialized or not.
  * @param  None.
  * @retval : I2C initialization status.
  */
uint8_t ft5216_Get_I2C_InitializedStatus(void);

/**
  * @brief  I2C initialize if needed.
  * @param  None.
  * @retval : None.
  */
void ft5216_I2C_InitializeIfRequired(void);

/**
  * @brief  Basic static configuration of TouchScreen
  * @param  DeviceAddr: FT5216 Device address for communication on I2C Bus.
  * @retval Status FT5216_STATUS_OK or FT5216_STATUS_NOT_OK.
  */
uint32_t ft5216_CTP_Configure(uint16_t DeviceAddr);


#ifdef FT5216_USING_SOFT_I2C
static void rt_hw_us_delay(int us)
{
    rt_uint32_t delta;
    rt_uint32_t current_delay;

    /* ge ticks to delay  */
    us = us * (SysTick->LOAD/(1000000/RT_TICK_PER_SECOND));

    /* get current time */
    delta = SysTick->VAL;

    /* get current time by cycles,until the given time to exit */
    do
    {
        if ( delta > SysTick->VAL )
            current_delay = delta - SysTick->VAL;
        else
        /* the delay crossing a OS tick critical */
        current_delay = SysTick->LOAD + delta - SysTick->VAL;
    } while( current_delay < us );
}

/****************************************************
* 函数名称 ：
* 功    能 ：单片机发送起始信号
* 入口参数 ：无
* 返回参数 ：无
* 注意事项 ：
*****************************************************/
void FT6236_Start(void)					
{
	SDA_OUT();     		//sda线输出
	rt_hw_us_delay(3);
	CTP_I2Cx_SDA_ON;	
	rt_hw_us_delay(3);	
	CTP_I2Cx_SCL_ON;		//SCL最小高电平脉宽:0.6us
	rt_hw_us_delay(2);		//起始信号的最小建立时间:0.6us
	CTP_I2Cx_SDA_OFF;		//SCL高电平期间，SDA的一个下降沿表示起始信号
	rt_hw_us_delay(2);		//起始信号的最小保持时间:0.6us
	CTP_I2Cx_SCL_OFF;		//箝住总线,为发送器件地址做准备;
	rt_hw_us_delay(2);		//SCL最小低电平脉宽:1.2us,由RET实现
}

/****************************************************
* 函数名称 ：
* 功    能 ：单片机发送停止信号
* 入口参数 ：无
* 返回参数 ：无
* 注意事项 ：
*****************************************************/
void FT6236_Stop(void)							
{
	SDA_OUT();     		//sda线输出	
	rt_hw_us_delay(3);	
	CTP_I2Cx_SCL_ON;		//SCL最小高电平脉宽:0.6us		
	rt_hw_us_delay(2);		//停止信号的最小建立时间:0.6us
	CTP_I2Cx_SDA_OFF;	
	rt_hw_us_delay(2);
	CTP_I2Cx_SDA_ON;		//SCL高电平期间，SDA的一个上升沿表示停止信号
	rt_hw_us_delay(2);						
}

/****************************************************
* 函数名称 ：
* 功    能 ：单片机发送应答信号
* 入口参数 ：无
* 返回参数 ：无
* 注意事项 ：单片机读1B数据后发送一个应答信号
*****************************************************/
void FT6236_McuACK(void)							
{
	CTP_I2Cx_SCL_OFF;	
	SDA_OUT();     		//sda线输出	
	rt_hw_us_delay(3);
	CTP_I2Cx_SDA_OFF;	
	rt_hw_us_delay(2);																	
	CTP_I2Cx_SCL_ON;		//SCL最小高电平脉宽:0.6us
	rt_hw_us_delay(2);
	CTP_I2Cx_SCL_OFF;		//SCL最小低电平脉宽:1.2us
}

/****************************************************
* 函数名称 ：
* 功    能 ：单片机发送非应答信号
* 入口参数 ：无
* 返回参数 ：无
* 注意事项 ：单片机读数据停止前发送一个非应答信号
*****************************************************/
void FT6236_McuNACK(void)
{
	CTP_I2Cx_SCL_OFF;	
	SDA_OUT();     				//sda线输出	
	rt_hw_us_delay(3);
	CTP_I2Cx_SDA_ON;	
	rt_hw_us_delay(2);																	
	CTP_I2Cx_SCL_ON;				//SCL最小高电平脉宽:0.6us
	rt_hw_us_delay(2);
	CTP_I2Cx_SCL_OFF;				//SCL最小低电平脉宽:1.2us
}

/****************************************************
* 函数名称 ：
* 功    能 ：单片机检查FT6236送来的应答信号
* 入口参数 ：无
* 返回参数 ：1，接收应答失败
			 0，接收应答成功
* 注意事项 ：单片机写1个地址/数据后检查
			 全局变量RevAckF:收到FT6236应答信号的标志位,为0表示收到
*****************************************************/
rt_uint8_t FT6236_CheckAck(void)							
{
	rt_uint8_t ucErrTime=0;
	CTP_I2Cx_SDA_ON;
	rt_hw_us_delay(2);
	SDA_IN(); //SDA设置为输入
	rt_hw_us_delay(1);
	CTP_I2Cx_SCL_ON;				//使SDA上数据有效;SCL最小高电平脉宽:0.6us
	rt_hw_us_delay(2);
	while(CTP_I2Cx_SDA_READ)
	{	
		ucErrTime++;
		if(ucErrTime>250)		//无应答
		{
			FT6236_Stop();
       rt_hw_us_delay(2);			
			return 1;
		}
		rt_hw_us_delay(2);
	}
	CTP_I2Cx_SCL_OFF;
	rt_hw_us_delay(2);
	return 0;
}

/****************************************************
* 函数名称 ：
* 功    能 ：单片机向IIC总线发送1B的地址/数据
* 入口参数 ：待发送的1B地址/数据
* 返回参数 ：无
* 注意事项 ：不是一个完整的数据发送过程;送数的顺序是从高到低
*****************************************************/
void FT6236_WrOneByte(rt_uint8_t dat)						
{
	rt_uint8_t i;						
	SDA_OUT();     				//sda线输出	
	CTP_I2Cx_SCL_OFF;				//拉低时钟开始数据传输
	rt_hw_us_delay(4);
	for(i = 8; i > 0; i--)		//8位1B地址/数据的长度
	{
		if(dat & 0x80) 		
			CTP_I2Cx_SDA_ON;		//发送"1"		
		else
			CTP_I2Cx_SDA_OFF;		//发送"0"
		rt_hw_us_delay(1);
		CTP_I2Cx_SCL_ON;			//使SDA上的数据有效
		rt_hw_us_delay(2);			//SCL最小高电平脉宽:0.6us							
		CTP_I2Cx_SCL_OFF;			//SCL最小低电平脉宽:1.2us
		rt_hw_us_delay(2);
		dat <<= 1;				//发送数据左移1位,为下位发送准备	
	}		
}

/****************************************************
* 函数名称 ：
* 功    能 ：单片机从IIC总线接收1B的数据
* 入口参数 ：无
* 返回参数 ：收到的1B数据
* 注意事项 ：不是一个完整的数据接收过程;从高到低的顺序接收数据
*****************************************************/
rt_uint8_t FT6236_RdOneByte(void)						
{
	rt_uint8_t i,dat = 0;				//接收数据位数和内容暂存单元
	CTP_I2Cx_SDA_ON;	
	SDA_IN();						//SDA设置为输入
	rt_hw_us_delay(2);	
	CTP_I2Cx_SDA_ON;			//使能上拉,准备读取数据
	rt_hw_us_delay(2);
	for(i = 8;i > 0;i--)
	{
		CTP_I2Cx_SCL_OFF;
		rt_hw_us_delay(2);
		CTP_I2Cx_SCL_ON;
		dat <<= 1;
		if(CTP_I2Cx_SDA_READ)
			dat++;
		rt_hw_us_delay(2);			//SCL最小低电平脉宽:1.2us
	}
	CTP_I2Cx_SDA_ON;		
	return(dat);				//返回1B的数据
}

//向FT6236写入一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:写数据长度
//返回值:0,成功;1,失败.
rt_uint8_t FT6236_WR_data(rt_uint16_t reg,rt_uint8_t *buf,rt_uint8_t len)
{
	rt_uint8_t i;
	rt_uint8_t ret=0;
	FT6236_Start();	 
	FT6236_WrOneByte(CTP_I2C_ADDRESS);	//发送写命令 	 
	FT6236_CheckAck(); 	 										  		   
	FT6236_WrOneByte(reg&0XFF);   	//发送低8位地址
	FT6236_CheckAck();  
	for(i=0;i<len;i++)
	{	   
    	FT6236_WrOneByte(buf[i]);  	//发数据
		ret=FT6236_CheckAck();
		if(ret)break;  
	}
    FT6236_Stop();					//产生一个停止条件	    
	return ret; 
}
//从FT6236读出一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:读数据长度			  
void FT6236_RD_data(rt_uint16_t reg,rt_uint8_t *buf,rt_uint8_t len)
{
	rt_uint8_t i; 
 	FT6236_Start();	
 	FT6236_WrOneByte(CTP_I2C_ADDRESS);   	//发送写命令 	 
	FT6236_CheckAck(); 	 										  		   
 	FT6236_WrOneByte(reg&0XFF);   	//发送低8位地址
	FT6236_CheckAck();
  //FT6236_Stop();	  
 	FT6236_Start();  	 	   
	FT6236_WrOneByte(CTP_I2C_ADDRESS|0x01);   	//发送读命令		   
	if(!FT6236_CheckAck())	
	{  
		for(i=0;i<len;i++)
		{	   
			*buf++ = FT6236_RdOneByte();		//读入1B数据到接收数据缓冲区中
			FT6236_McuACK();					//发送应答位	  
		} 
	}
	//FT6236_McuNACK();						//n个字节读完,发送非应答位
    FT6236_Stop();					//产生一个停止条件	  
} 
#endif


/******************************* I2C Routines *********************************/
/**
  * @brief  Initializes I2C MSP.
  * @param  i2c_handler : I2C handler
  * @retval None
  */
	

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2c_handler)
{
  GPIO_InitTypeDef  gpio_init_structure;
  
  if (i2c_handler == (I2C_HandleTypeDef*)(&hI2cCtpHandler))
  {
    /* AUDIO and LCD I2C MSP init */

    /*** Configure the GPIOs ***/
    /* Enable GPIO clock */
    CTP_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();
    #ifdef FT5216_USING_SOFT_I2C
		gpio_init_structure.Pin = CTP_I2Cx_SCL_PIN;
    gpio_init_structure.Mode = GPIO_MODE_AF_OD;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    //gpio_init_structure.Alternate = CTP_I2Cx_SCL_SDA_AF;
    HAL_GPIO_Init(CTP_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

    /* Configure I2C Rx as alternate function */
    gpio_init_structure.Pin = CTP_I2Cx_SDA_PIN;
    HAL_GPIO_Init(CTP_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);
		CTP_I2Cx_RST_OFF;
		rt_thread_delay( RT_TICK_PER_SECOND/20 );
		CTP_I2Cx_RST_ON;
		rt_thread_delay( RT_TICK_PER_SECOND/10 );
		CTP_I2Cx_SDA_ON;
		CTP_I2Cx_SCL_ON;
		rt_thread_delay( RT_TICK_PER_SECOND/100 );
		/*temp=0;
		FT6236_WR_Reg(FT_DEVIDE_MODE,&temp,1);	//进入正常操作模式 
		temp=22;								//触摸有效值，22，越小越灵敏	
		FT6236_WR_Reg(FT_ID_G_THGROUP,&temp,1);	//设置触摸有效值
		temp=12;								//激活周期，不能小于12，最大14
		FT6236_WR_Reg(FT_ID_G_PERIODACTIVE,&temp,1); */
		#else
    /* Configure I2C Tx as alternate function */
    gpio_init_structure.Pin = CTP_I2Cx_SCL_PIN;
    gpio_init_structure.Mode = GPIO_MODE_AF_OD;
   // gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    //gpio_init_structure.Alternate = CTP_I2Cx_SCL_SDA_AF;
    HAL_GPIO_Init(CTP_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

    /* Configure I2C Rx as alternate function */
    gpio_init_structure.Pin = CTP_I2Cx_SDA_PIN;
    HAL_GPIO_Init(CTP_I2Cx_SCL_SDA_GPIO_PORT, &gpio_init_structure);

    /*** Configure the I2C peripheral ***/
		 __HAL_AFIO_REMAP_I2C1_ENABLE();
		 CTP_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();
    /* Enable I2C clock */
    CTP_I2Cx_CLK_ENABLE();
    /* Force the I2C peripheral clock reset */
		#if 0
    CTP_I2Cx_FORCE_RESET();
    /* Release the I2C peripheral clock reset */
    CTP_I2Cx_RELEASE_RESET();
		

    /* Enable and set I2Cx Interrupt to a lower priority */
    HAL_NVIC_SetPriority(CTP_I2Cx_EV_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(CTP_I2Cx_EV_IRQn);

    /* Enable and set I2Cx Interrupt to a lower priority */
    HAL_NVIC_SetPriority(CTP_I2Cx_ER_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(CTP_I2Cx_ER_IRQn);
		CTP_I2Cx_CLK_ENABLE();
		#endif
		#endif
		
  }
	
}
  
/**
  * @brief  Initializes I2C HAL.
  * @param  i2c_handler : I2C handler
  * @retval None
  */
void I2Cx_Init(I2C_HandleTypeDef *i2c_handler)
{
    /* Audio and LCD I2C configuration */
	
	//CTP_I2Cx_CLK_ENABLE();
	//CTP_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();
	#ifdef FT5216_USING_SOFT_I2C
	I2Cx_MspInit(i2c_handler);
	#else
    i2c_handler->Instance = CTP_I2Cx;
    i2c_handler->Init.ClockSpeed = 100000;
		i2c_handler->Init.DutyCycle = I2C_DUTYCYCLE_2;
    i2c_handler->Init.OwnAddress1      = 0;
    i2c_handler->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    i2c_handler->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    i2c_handler->Init.OwnAddress2      = 0;
    i2c_handler->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    i2c_handler->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    /* Init the I2C */
		HAL_I2C_Init(i2c_handler);
    //I2Cx_MspInit(i2c_handler);
	#endif
}

void I2Cx_Error(I2C_HandleTypeDef *i2c_handler, uint8_t Addr)
{
  /* De-initialize the I2C communication bus */
  HAL_I2C_DeInit(i2c_handler);
  
  /* Re-Initialize the I2C communication bus */
  I2Cx_Init(i2c_handler);
}

void I2C_busy_ERROR_Handing(I2C_HandleTypeDef *i2c_handler)
{
	GPIO_InitTypeDef GPIO_InitStruct;
		CTP_I2Cx_SCL_SDA_GPIO_CLK_ENABLE();
    
    /**I2C1 GPIO Configuration    
    PB8    ------> I2C1_SCL
    PB9     ------> I2C1_SDA 
    */
		__HAL_RCC_I2C1_CLK_ENABLE();
    GPIO_InitStruct.Pin = CTP_I2Cx_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CTP_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);
	
		GPIO_InitStruct.Pin = CTP_I2Cx_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CTP_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);
		HAL_GPIO_WritePin(CTP_I2Cx_SCL_SDA_GPIO_PORT,CTP_I2Cx_SCL_PIN,GPIO_PIN_SET);
		HAL_GPIO_WritePin(CTP_I2Cx_SCL_SDA_GPIO_PORT,CTP_I2Cx_SDA_PIN,GPIO_PIN_SET);
		/*delay_us(10);*/
		SET_BIT(i2c_handler->Instance->CR1, I2C_CR1_SWRST);
		 i2c_handler->Instance->CR1= 0;
		 
		HAL_GPIO_WritePin(CTP_I2Cx_SCL_SDA_GPIO_PORT,CTP_I2Cx_SDA_PIN,GPIO_PIN_SET);
		GPIO_InitStruct.Pin = CTP_I2Cx_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CTP_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);
		
	
		GPIO_InitStruct.Pin = CTP_I2Cx_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CTP_I2Cx_SCL_SDA_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief  Reads multiple data.
  * @param  i2c_handler : I2C handler
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  MemAddress: Memory address 
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
HAL_StatusTypeDef I2Cx_ReadMultiple(I2C_HandleTypeDef *i2c_handler,
                                           uint8_t Addr,
                                           uint16_t Reg,
                                           uint16_t MemAddSize,
                                           uint8_t *Buffer,
                                           uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(i2c_handler, Addr, (uint16_t)Reg, MemAddSize, Buffer, Length, 300);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* I2C error occurred */
		if(status == HAL_BUSY)
		{
			I2C_busy_ERROR_Handing(i2c_handler);
			I2Cx_Error(i2c_handler, Addr);
		}
		else
		{
			I2Cx_Error(i2c_handler, Addr);
		}
  }
  return status;    
}

/**
  * @brief  Writes a value in a register of the device through BUS in using DMA mode.
  * @param  i2c_handler : I2C handler
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  MemAddress: Memory address 
  * @param  Buffer: The target register value to be written 
  * @param  Length: buffer size to be written
  * @retval HAL status
  */
HAL_StatusTypeDef I2Cx_WriteMultiple(I2C_HandleTypeDef *i2c_handler,
                                            uint8_t Addr,
                                            uint16_t Reg,
                                            uint16_t MemAddress,
                                            uint8_t *Buffer,
                                            uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(i2c_handler, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the I2C Bus */
    I2Cx_Error(i2c_handler, Addr);
  }
  return status;
}

/**
  * @brief  Checks if target device is ready for communication. 
  * @note   This function is used with Memory devices
  * @param  i2c_handler : I2C handler
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
HAL_StatusTypeDef I2Cx_IsDeviceReady(I2C_HandleTypeDef *i2c_handler, uint16_t DevAddress, uint32_t Trials)
{ 
  return (HAL_I2C_IsDeviceReady(i2c_handler, DevAddress, Trials, 1000));
}

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  i2c_handler : I2C handler
  * @param  Addr: I2C Address
  * @retval None
  */


/********************************* LINK CTPSCREEN *********************************/

/**
  * @brief  Initializes Touchscreen low level.
  * @retval None
  */
void CTP_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/*Configure GPIO pins :  CTP_RST_Pin */
	GPIO_InitStruct.Pin = CTP_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(CTP_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : CTP_INT_Pin */
	GPIO_InitStruct.Pin = CTP_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(CTP_INT_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  #ifdef FT5216_USING_SOFT_I2C
	#else
	I2Cx_Init(&hI2cCtpHandler);
	#endif
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @param  Value: Data to be written
  * @retval None
  */
void CTP_IO_Write(uint8_t Reg, uint8_t *Value,uint16_t len)
{
	#ifdef FT5216_USING_SOFT_I2C
	FT6236_WR_data(Reg,&Value,len);
	#else	
  I2Cx_WriteMultiple(&hI2cCtpHandler, CTP_I2C_ADDRESS, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT,(uint8_t*)&Value, len);
	#endif
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @retval Data to be read
  */
uint8_t CTP_IO_Read(uint8_t Reg,uint8_t *value,uint16_t len)
{
  uint8_t read_value = 0;
  #ifdef FT5216_USING_SOFT_I2C
	FT6236_RD_data(Reg,&read_value,len);
	#else
  I2Cx_ReadMultiple(&hI2cCtpHandler, CTP_I2C_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT,(uint8_t*)value, len);
  #endif
  return read_value;
}

/**
  * @brief  CTP delay
  * @param  Delay: Delay in ms
  * @retval None
  */
void CTP_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}


/**
  * @brief  Initialize the ft5216 communication bus
  *         from MCU to FT5216 : ie I2C channel initialization (if required).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5216).
  * @retval None
  */
void ft5216_Init(uint16_t DeviceAddr)
{
  /* Wait at least 200ms after power up before accessing registers
   * Trsi timing (Time of starting to report point after resetting) from FT5216GQQ datasheet */
  CTP_IO_Delay(200);

  /* Initialize I2C link if needed */
  ft5216_I2C_InitializeIfRequired();
}

/**
  * @brief  Software Reset the ft5216.
  *         @note : Not applicable to FT5216.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5216).
  * @retval None
  */
void ft5216_Reset(uint16_t DeviceAddr)
{
  /* Do nothing */
  /* No software reset sequence available in FT5216 IC */
}

/**
  * @brief  Read the ft5216 device ID, pre initialize I2C in case of need to be
  *         able to read the FT5216 device ID, and verify this is a FT5216.
  * @param  DeviceAddr: I2C FT5216 Slave address.
  * @retval The Device ID (two bytes).
  */
uint16_t ft5216_ReadID(void)
{
  volatile uint8_t ucReadId = 0;
	uint8_t data2;
  uint8_t nbReadAttempts = 0;
  uint8_t bFoundDevice = 0; /* Device not found by default */
  /* Initialize I2C link if needed */
  ft5216_I2C_InitializeIfRequired();

  /* At maximum 4 attempts to read ID : exit at first finding of the searched device ID */
  for(nbReadAttempts = 0; ((nbReadAttempts < 8) && !(bFoundDevice)); nbReadAttempts++)
  {
    /* Read register FT5216_CHIP_ID_REG as DeviceID detection */
    CTP_IO_Read(FT5216_CHIP_ID_REG,&data2,1);
		ucReadId = data2;
    /* Found the searched device ID ? */
    if(ucReadId == FT5216_ID_VALUE)
    {
      /* Set device as found */
      bFoundDevice = 1;
    }
  }

  /* Return the device ID value */
  return (ucReadId);
}

/**
  * @brief  Configures the touch Screen IC device to start detecting touches
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address).
  * @retval None.
  */
void ft5216_CTP_Start(uint16_t DeviceAddr)
{
  /* Minimum static configuration of FT5216 */
  FT5216_ASSERT(ft5216_CTP_Configure(DeviceAddr));

  /* By default set FT5216 IC in Polling mode : no INT generation on FT5216 for new touch available */
  /* Note CTP_INT is active low                                                                      */
  ft5216_CTP_DisableIT(DeviceAddr);
}

/**
  * @brief  Return if there is touches detected or not.
  *         Try to detect new touches and forget the old ones (reset internal global
  *         variables).
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval : Number of active touches detected (can be 0, 1 or 2).
  */
uint8_t ft5216_CTP_DetectTouch(uint16_t DeviceAddr)
{
  uint8_t nbTouch = 0;

  /* Read register FT5216_TD_STAT_REG to check number of touches detection */
  CTP_IO_Read(FT5216_TD_STAT_REG,&nbTouch,1);
  nbTouch &= FT5216_TD_STAT_MASK;

  if(nbTouch > FT5216_MAX_DETECTABLE_CTP)
  {
    /* If invalid number of touch detected, set it to zero */
    nbTouch = 0;
  }

  /* Update ft5216 driver internal global : current number of active touches */
  ft5216_handle.currActiveTouchNb = nbTouch;

  /* Reset current active touch index on which to work on */
  ft5216_handle.currActiveTouchIdx = 0;

  return(nbTouch);
}

/**
  * @brief  Get the touch screen X and Y positions values
  *         Manage multi touch thanks to touch Index global
  *         variable 'ft5216_handle.currActiveTouchIdx'.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  X: Pointer to X position value
  * @param  Y: Pointer to Y position value
  * @retval None.
  */
void ft5216_CTP_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
{
  uint8_t ucReadData = 0;
  static uint16_t coord;
  uint8_t regAddressXLow = 0;
  uint8_t regAddressXHigh = 0;
  uint8_t regAddressYLow = 0;
  uint8_t regAddressYHigh = 0;

  if(ft5216_handle.currActiveTouchIdx < ft5216_handle.currActiveTouchNb)
  {
    switch(ft5216_handle.currActiveTouchIdx)
    {
    case 0 :
      regAddressXLow  = FT5216_P1_XL_REG;
      regAddressXHigh = FT5216_P1_XH_REG;
      regAddressYLow  = FT5216_P1_YL_REG;
      regAddressYHigh = FT5216_P1_YH_REG;
      break;

    case 1 :
      regAddressXLow  = FT5216_P2_XL_REG;
      regAddressXHigh = FT5216_P2_XH_REG;
      regAddressYLow  = FT5216_P2_YL_REG;
      regAddressYHigh = FT5216_P2_YH_REG;
      break;

    case 2 :
      regAddressXLow  = FT5216_P3_XL_REG;
      regAddressXHigh = FT5216_P3_XH_REG;
      regAddressYLow  = FT5216_P3_YL_REG;
      regAddressYHigh = FT5216_P3_YH_REG;
      break;

    case 3 :
      regAddressXLow  = FT5216_P4_XL_REG;
      regAddressXHigh = FT5216_P4_XH_REG;
      regAddressYLow  = FT5216_P4_YL_REG;
      regAddressYHigh = FT5216_P4_YH_REG;
      break;

    case 4 :
      regAddressXLow  = FT5216_P5_XL_REG;
      regAddressXHigh = FT5216_P5_XH_REG;
      regAddressYLow  = FT5216_P5_YL_REG;
      regAddressYHigh = FT5216_P5_YH_REG;
      break;

    case 5 :
      regAddressXLow  = FT5216_P6_XL_REG;
      regAddressXHigh = FT5216_P6_XH_REG;
      regAddressYLow  = FT5216_P6_YL_REG;
      regAddressYHigh = FT5216_P6_YH_REG;
      break;

    case 6 :
      regAddressXLow  = FT5216_P7_XL_REG;
      regAddressXHigh = FT5216_P7_XH_REG;
      regAddressYLow  = FT5216_P7_YL_REG;
      regAddressYHigh = FT5216_P7_YH_REG;
      break;

    case 7 :
      regAddressXLow  = FT5216_P8_XL_REG;
      regAddressXHigh = FT5216_P8_XH_REG;
      regAddressYLow  = FT5216_P8_YL_REG;
      regAddressYHigh = FT5216_P8_YH_REG;
      break;

    case 8 :
      regAddressXLow  = FT5216_P9_XL_REG;
      regAddressXHigh = FT5216_P9_XH_REG;
      regAddressYLow  = FT5216_P9_YL_REG;
      regAddressYHigh = FT5216_P9_YH_REG;
      break;

    case 9 :
      regAddressXLow  = FT5216_P10_XL_REG;
      regAddressXHigh = FT5216_P10_XH_REG;
      regAddressYLow  = FT5216_P10_YL_REG;
      regAddressYHigh = FT5216_P10_YH_REG;
      break;

    default :
      break;

    } /* end switch(ft5216_handle.currActiveTouchIdx) */

    /* Read low part of X position */
    CTP_IO_Read(regAddressXLow,&ucReadData,1);
    coord = (ucReadData & FT5216_CTP_POS_LSB_MASK) >> FT5216_CTP_POS_LSB_SHIFT;

    /* Read high part of X position */
    CTP_IO_Read(regAddressXHigh,&ucReadData,1);
    coord |= ((ucReadData & FT5216_CTP_POS_MSB_MASK) >> FT5216_CTP_POS_MSB_SHIFT) << 8;

    /* Send back ready X position to caller */
    *X = coord;

    /* Read low part of Y position */
    CTP_IO_Read(regAddressYLow,&ucReadData,1);
    coord = (ucReadData & FT5216_CTP_POS_LSB_MASK) >> FT5216_CTP_POS_LSB_SHIFT;

    /* Read high part of Y position */
    CTP_IO_Read(regAddressYHigh,&ucReadData,1);
    coord |= ((ucReadData & FT5216_CTP_POS_MSB_MASK) >> FT5216_CTP_POS_MSB_SHIFT) << 8;

    /* Send back ready Y position to caller */
    *Y = coord;

    ft5216_handle.currActiveTouchIdx++; /* next call will work on next touch */

  } /* of if(ft5216_handle.currActiveTouchIdx < ft5216_handle.currActiveTouchNb) */
}

/**
  * @brief  Configure the FT5216 device to generate IT on given INT pin
  *         connected to MCU as EXTI.
  * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT5216).
  * @retval None
  */
void ft5216_CTP_EnableIT(uint16_t DeviceAddr)
{
   uint8_t regValue = 0;
	uint8_t Value = FT5216_GMODE_REG;
   regValue = (FT5216_G_MODE_INTERRUPT_TRIGGER & (FT5216_G_MODE_INTERRUPT_MASK >> FT5216_G_MODE_INTERRUPT_SHIFT)) << FT5216_G_MODE_INTERRUPT_SHIFT;

   /* Set interrupt trigger mode in FT5216_GMODE_REG */
   CTP_IO_Write(DeviceAddr, &Value, regValue);
}

/**
  * @brief  Configure the FT5216 device to stop generating IT on the given INT pin
  *         connected to MCU as EXTI.
  * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT5216).
  * @retval None
  */
void ft5216_CTP_DisableIT(uint16_t DeviceAddr)
{
  uint8_t regValue = 0;
	uint8_t Value = FT5216_GMODE_REG;
  regValue = (FT5216_G_MODE_INTERRUPT_POLLING & (FT5216_G_MODE_INTERRUPT_MASK >> FT5216_G_MODE_INTERRUPT_SHIFT)) << FT5216_G_MODE_INTERRUPT_SHIFT;

  /* Set interrupt polling mode in FT5216_GMODE_REG */
  CTP_IO_Write(DeviceAddr, &Value, regValue);
}

/**
  * @brief  Get IT status from FT5216 interrupt status registers
  *         Should be called Following an EXTI coming to the MCU to know the detailed
  *         reason of the interrupt.
  *         @note : This feature is not applicable to FT5216.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5216).
  * @retval CTP interrupts status : always return 0 here
  */
uint8_t ft5216_CTP_ITStatus(uint16_t DeviceAddr)
{
  /* Always return 0 as feature not applicable to FT5216 */
  return 0;
}

/**
  * @brief  Clear IT status in FT5216 interrupt status clear registers
  *         Should be called Following an EXTI coming to the MCU.
  *         @note : This feature is not applicable to FT5216.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5216).
  * @retval None
  */
void ft5216_CTP_ClearIT(uint16_t DeviceAddr)
{
  /* Nothing to be done here for FT5216 */
}
void ft5216_CTP_ID( uint32_t * pGestureId)
{
  uint8_t ucReadData = 0;

  ucReadData = CTP_IO_Read( FT5216_GEST_ID_REG,&ucReadData,1);

  * pGestureId = ucReadData;
}

/**** NEW FEATURES enabled when Multi-touch support is enabled ****/

#if (CTP_MULTI_CTP_SUPPORTED == 1)

/**
  * @brief  Get the last touch gesture identification (zoom, move up/down...).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5216).
  * @param  pGestureId : Pointer to get last touch gesture Identification.
  * @retval None.
  */
void ft5216_CTP_GetGestureID(uint16_t DeviceAddr, uint32_t * pGestureId)
{
  uint8_t ucReadData = 0;

  ucReadData = CTP_IO_Read(FT5216_GEST_ID_REG,&ucReadData,1);

  * pGestureId = ucReadData;
}

/**
  * @brief  Get the touch detailed informations on touch number 'touchIdx' (0..1)
  *         This touch detailed information contains :
  *         - weight that was applied to this touch
  *         - sub-area of the touch in the touch panel
  *         - event of linked to the touch (press down, lift up, ...)
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5216).
  * @param  touchIdx : Passed index of the touch (0..1) on which we want to get the
  *                    detailed information.
  * @param  pWeight : Pointer to to get the weight information of 'touchIdx'.
  * @param  pArea   : Pointer to to get the sub-area information of 'touchIdx'.
  * @param  pEvent  : Pointer to to get the event information of 'touchIdx'.

  * @retval None.
  */
void ft5216_CTP_GetTouchInfo(uint16_t   DeviceAddr,
                            uint32_t   touchIdx,
                            uint32_t * pWeight,
                            uint32_t * pArea,
                            uint32_t * pEvent)
{
  uint8_t ucReadData = 0;
  uint8_t regAddressXHigh = 0;
  uint8_t regAddressPWeight = 0;
  uint8_t regAddressPMisc = 0;

  if(touchIdx < ft5216_handle.currActiveTouchNb)
  {
    switch(touchIdx)
    {
    case 0 :
      regAddressXHigh   = FT5216_P1_XH_REG;
      regAddressPWeight = FT5216_P1_WEIGHT_REG;
      regAddressPMisc   = FT5216_P1_MISC_REG;
      break;

    case 1 :
      regAddressXHigh   = FT5216_P2_XH_REG;
      regAddressPWeight = FT5216_P2_WEIGHT_REG;
      regAddressPMisc   = FT5216_P2_MISC_REG;
      break;

    case 2 :
      regAddressXHigh   = FT5216_P3_XH_REG;
      regAddressPWeight = FT5216_P3_WEIGHT_REG;
      regAddressPMisc   = FT5216_P3_MISC_REG;
      break;

    case 3 :
      regAddressXHigh   = FT5216_P4_XH_REG;
      regAddressPWeight = FT5216_P4_WEIGHT_REG;
      regAddressPMisc   = FT5216_P4_MISC_REG;
      break;

    case 4 :
      regAddressXHigh   = FT5216_P5_XH_REG;
      regAddressPWeight = FT5216_P5_WEIGHT_REG;
      regAddressPMisc   = FT5216_P5_MISC_REG;
      break;

    case 5 :
      regAddressXHigh   = FT5216_P6_XH_REG;
      regAddressPWeight = FT5216_P6_WEIGHT_REG;
      regAddressPMisc   = FT5216_P6_MISC_REG;
      break;

    case 6 :
      regAddressXHigh   = FT5216_P7_XH_REG;
      regAddressPWeight = FT5216_P7_WEIGHT_REG;
      regAddressPMisc   = FT5216_P7_MISC_REG;
      break;

    case 7 :
      regAddressXHigh   = FT5216_P8_XH_REG;
      regAddressPWeight = FT5216_P8_WEIGHT_REG;
      regAddressPMisc   = FT5216_P8_MISC_REG;
      break;

    case 8 :
      regAddressXHigh   = FT5216_P9_XH_REG;
      regAddressPWeight = FT5216_P9_WEIGHT_REG;
      regAddressPMisc   = FT5216_P9_MISC_REG;
      break;

    case 9 :
      regAddressXHigh   = FT5216_P10_XH_REG;
      regAddressPWeight = FT5216_P10_WEIGHT_REG;
      regAddressPMisc   = FT5216_P10_MISC_REG;
      break;

    default :
      break;

    } /* end switch(touchIdx) */

    /* Read Event Id of touch index */
    CTP_IO_Read(regAddressXHigh,&ucReadData,1);
    * pEvent = (ucReadData & FT5216_CTP_EVT_FLAG_MASK) >> FT5216_CTP_EVT_FLAG_SHIFT;

    /* Read weight of touch index */
    CTP_IO_Read(regAddressPWeight,&ucReadData,1);
    * pWeight = (ucReadData & FT5216_CTP_WEIGHT_MASK) >> FT5216_CTP_WEIGHT_SHIFT;

    /* Read area of touch index */
    CTP_IO_Read(regAddressPMisc,&ucReadData,1);
    * pArea = (ucReadData & FT5216_CTP_AREA_MASK) >> FT5216_CTP_AREA_SHIFT;

  } /* of if(touchIdx < ft5216_handle.currActiveTouchNb) */
}

#endif /* CTP_MULTI_CTP_SUPPORTED == 1 */

/** @defgroup ft5216_Static_Function_Body
  * @{
  */

/* Static functions bodies-----------------------------------------------*/


/**
  * @brief  Return the status of I2C was initialized or not.
  * @param  None.
  * @retval : I2C initialization status.
  */
uint8_t ft5216_Get_I2C_InitializedStatus(void)
{
  return(ft5216_handle.i2cInitialized);
}

/**
  * @brief  I2C initialize if needed.
  * @param  None.
  * @retval : None.
  */
void ft5216_I2C_InitializeIfRequired(void)
{
  if(ft5216_Get_I2C_InitializedStatus() == FT5216_I2C_NOT_INITIALIZED)
  {
    /* Initialize CTP IO BUS layer (I2C) */
    CTP_IO_Init();
    I2Cx_Init(&hI2cCtpHandler);
    /* Set state to initialized */
    ft5216_handle.i2cInitialized = FT5216_I2C_INITIALIZED;
  }
}

/**
  * @brief  Basic static configuration of TouchScreen
  * @param  DeviceAddr: FT5216 Device address for communication on I2C Bus.
  * @retval Status FT5216_STATUS_OK or FT5216_STATUS_NOT_OK.
  */
uint32_t ft5216_CTP_Configure(uint16_t DeviceAddr)
{
	uint8_t temp;
  uint32_t status = FT5216_STATUS_OK;
		CTP_I2Cx_RST_OFF;
		rt_thread_delay( RT_TICK_PER_SECOND/20 );
		CTP_I2Cx_RST_ON;
		rt_thread_delay( RT_TICK_PER_SECOND/10 );
		//CTP_I2Cx_SDA_ON;
		//CTP_I2Cx_SCL_ON;
		//rt_thread_delay( RT_TICK_PER_SECOND/100 );
	  temp=1;
		CTP_IO_Write(FT5216_GMODE_REG,&temp,1);	//进入正常操作模式 
		temp=70;								//触摸有效值，22，越小越灵敏	
		CTP_IO_Write(FT5216_TH_GROUP_REG,&temp,1);	//设置触摸有效值
		temp=12;								//激活周期，不能小于12，最大14
		CTP_IO_Write(FT5216_PERIODACTIVE_REG,&temp,1); 
    /* Nothing special to be done for FT5216 */

  return(status);
}


struct ft5216_data ft5216_statue;
/*uint8_t ft5216_int_flag = 0;
rt_timer_t ft5216_timer;
void ft5216_timer_out_callback(void* parameter)
{
	if(ft5216_int_flag >1)
		ft5216_int_flag -= 1;
	else if(1 == ft5216_int_flag )
	{
		uint8_t ft5216_rcv[8] = {0};
		CTP_IO_Read(0,ft5216_rcv,8);
		
		if(0x40 == (ft5216_rcv[3]&0xc0))
			ft5216_statue.active = FT5216_PUT_UP;
		else if((0x00 == (ft5216_rcv[3]&0xc0))||(0x80 == (ft5216_rcv[3]&0xc0)))
			ft5216_statue.active = FT5216_PUT_DOWN;
		ft5216_statue.X_phys =((uint16_t)(ft5216_rcv[3]&0x3f)<<8) + (uint16_t)ft5216_rcv[4];
		ft5216_statue.Y_phys =((uint16_t)(ft5216_rcv[5]&0x3f)<<8) + (uint16_t)ft5216_rcv[6];
		ft5216_int_flag = 0;
	}
	if(0 == ft5216_int_flag )
	{
		rt_timer_delete(ft5216_timer);
	}
}
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if( GPIO_PIN_4 == GPIO_Pin)
	{
		/*if(0 == ft5216_int_flag)
		{
			ft5216_int_flag = 1;
			ft5216_timer = rt_timer_create("ft5216timer",
                           ft5216_timer_out_callback,
                           RT_NULL,
                           RT_TICK_PER_SECOND/5,
                           RT_TIMER_FLAG_PERIODIC);
			if(ft5216_timer != RT_NULL)
				rt_timer_start(ft5216_timer);
		}
		else
			ft5216_int_flag += 1;*/
		
		/*CTP_IO_Read(0,temp_rcvbuf+i,8);
		i += 8;
		if(i > 512)
			i = 0;*/
		
		uint8_t ft5216_rcv[8] = {0};
		CTP_IO_Read(0,ft5216_rcv,8);
		
		if(0x40 == (ft5216_rcv[3]&0xc0))
			ft5216_statue.active = FT5216_PUT_UP;
		else if((0x00 == (ft5216_rcv[3]&0xc0))||(0x80 == (ft5216_rcv[3]&0xc0)))
			ft5216_statue.active = FT5216_PUT_DOWN;
		ft5216_statue.X_phys =((uint16_t)(ft5216_rcv[3]&0x3f)<<8) + (uint16_t)ft5216_rcv[4];
		ft5216_statue.Y_phys =((uint16_t)(ft5216_rcv[5]&0x3f)<<8) + (uint16_t)ft5216_rcv[6];
	}
}




int ft5216_init(void)
{
	ft5216_I2C_InitializeIfRequired();
	ft5216_CTP_Configure(0);
	HAL_GPIO_EXTI_Callback(GPIO_PIN_4);
	return 0;
}	


INIT_DEVICE_EXPORT(ft5216_init);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
