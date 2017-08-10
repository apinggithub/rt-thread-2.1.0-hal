/**
  ******************************************************************************
  * @file    ili9341.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file includes the LCD driver for ILI9341 LCD.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/

#include <rtthread.h>
#include <rtdevice.h>

#ifdef RT_USING_LCD_ILI9341  
//#include "drv_spi_lcd.h"
#include "drv_lcd_ili9341.h"
#include "drv_fsmc_lcd.h"
//#include "bsp_lcd.h"
/** @addtogroup ILI9341
  * @brief This file provides a set of functions needed to drive the 
  *        ILI9341 LCD.
  * @{
  */
/*
LCD_DrvTypeDef   ili9341_drv = 
{
  ili9341_init,
  ili9341_read_id,
  ili9341_display_on,
  ili9341_display_off,
  0,
  0,
  0,
  0,
  0,
  0,
  ili9341_get_lcd_pixel_width,
  ili9341_get_lcd_pixel_height,
  0,
  0,    
};*/

void hw_lcd_delay(uint32_t delay)
{
	//HAL_Delay(Delay);
	HAL_Delay(delay);
}

/**
  * @brief  Writes  to the selected LCD register.
  * @param  reg: address of the selected register.
  * @retval None
  */
void ili9341_write_cmd(uint8_t reg)
{
	uint16_t cmd;
	cmd = (uint16_t)reg;
  LCD_WRITE_CMD(cmd);
}

/**
  * @brief  Writes data to the selected LCD register.
  * @param  data: data of the selected register.
  * @retval None
  */
void ili9341_write_data(uint8_t data)
{
  //hw_lcd_write_data(data);
	uint16_t  pdata;
	pdata = (uint16_t)data;
	LCD_WRITE_DATA(pdata);
}
void ili9341_write_data16(uint16_t data)
{
	LCD_WRITE_DATA(data);
}

/**
  * @brief  Reads the selected LCD Register.
  * @param  RegValue: Address of the register to read
  * @param  ReadSize: Number of bytes to read
  * @retval LCD Register Value.
  */
uint8_t ili9341_read_data(void)
{
  /* Read a max of 4 bytes */
	uint16_t read_data   = LCD_READ_DATA();
	uint8_t  read_data1  = (uint8_t)(read_data&0x00ff);
  return (read_data1);  	
}
/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void ili9341_backlight_on(void)
{
  /* Display backlignt On */
  //hw_lcd_backlight_on();
	#if 1
	rt_lcd_bl_on();
	#else
	LCD_BK_ON();
	#endif
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ili9341_backlight_off(void)
{
  /* Display backlight Off */
  //hw_lcd_backlight_off();
	#if 1
	rt_lcd_bl_off();
	#else
	LCD_BK_OFF();
	#endif
}
void ili9341_hw_init(void)
{
	
	
	//hw_lcd_reset();
	//ili9341_init();
	MX_FSMC_Init();
	//data2 =BSP_LCD_Init();
	
}
/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
uint16_t ili9341_read_id(void)
{
	uint16_t lcdid = 0;
	ili9341_hw_init();
	rt_thread_delay( RT_TICK_PER_SECOND/100 );
	ili9341_write_cmd(LCD_READ_ID4);
	//hw_lcd_delay(100);
	ili9341_read_data();
	ili9341_read_data();
	lcdid = ili9341_read_data()<< 8;
	lcdid |= ili9341_read_data();
	
	return (uint16_t)lcdid;
}

void ili9341_multi_write_data(uint8_t *pdata, uint32_t size)
{
	while(size>0)
	{
		LCD_WRITE_DATA(*pdata);
		size--;
		pdata++;
	}
}

/*void ili9341_multi_read_data(uint8_t *pdata, uint32_t size)
{*/
	/* Read a max of 4 bytes 
	return (LCD_IO_ReadData(RegValue, ReadSize));
	hw_lcd_read_multidata(pdata, size);	
}*/
  
/** @defgroup ILI9341_Private_Functions
  * @{
  */   
//void ili9341_screen_clear(uint16_t color);
/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
void ili9341_init(void)
{
  /* Initialize ILI9341 low level bus layer ----------------------------------*/
  //LCD_IO_Init();
  ili9341_hw_init();
#if 0
	
  /* Configure LCD */
  ili9341_write_cmd(0xCA);
  ili9341_write_data(0xC3);
  ili9341_write_data(0x08);
  ili9341_write_data(0x50);
	
  ili9341_write_cmd(LCD_POWERB);
  ili9341_write_data(0x00);
  ili9341_write_data(0xC1);
  ili9341_write_data(0x30);
  ili9341_write_cmd(LCD_POWER_SEQ);
  ili9341_write_data(0x64);
  ili9341_write_data(0x03);
  ili9341_write_data(0x12);
  ili9341_write_data(0x81);
  ili9341_write_cmd(LCD_DTCA);
  ili9341_write_data(0x85);
  ili9341_write_data(0x00);
  ili9341_write_data(0x78);
  ili9341_write_cmd(LCD_POWERA);
  ili9341_write_data(0x39);
  ili9341_write_data(0x2C);
  ili9341_write_data(0x00);
  ili9341_write_data(0x34);
  ili9341_write_data(0x02);
  ili9341_write_cmd(LCD_PRC);
  ili9341_write_data(0x20);
  ili9341_write_cmd(LCD_DTCB);
  ili9341_write_data(0x00);
  ili9341_write_data(0x00);
  
  ili9341_write_cmd(LCD_POWER1);
  ili9341_write_data(0x10);
  ili9341_write_cmd(LCD_POWER2);
  ili9341_write_data(0x10);
  ili9341_write_cmd(LCD_VCOM1);
  ili9341_write_data(0x45);
  ili9341_write_data(0x15);
  ili9341_write_cmd(LCD_VCOM2);
  ili9341_write_data(0x90);
  ili9341_write_cmd(LCD_MACTL);
  ili9341_write_data(0xC8);

  ili9341_write_cmd(LCD_RGB_INTERFACE);
  ili9341_write_data(0xC2);
  ili9341_write_cmd(LCD_DFC);
  ili9341_write_data(0x0A);
  ili9341_write_data(0xA7);
  ili9341_write_data(0x27);
  ili9341_write_data(0x04);
  
  /* Colomn address set */
  ili9341_write_cmd(LCD_COLUMN_ADDR_SET);
  ili9341_write_data(0x00);
  ili9341_write_data(0x00);
  ili9341_write_data(0x00);
  ili9341_write_data(0xEF);
  /* Page address set */
  ili9341_write_cmd(LCD_PAGE_ADDR_SET);
  ili9341_write_data(0x00);
  ili9341_write_data(0x00);
  ili9341_write_data(0x01);
  ili9341_write_data(0x3F);
  ili9341_write_cmd(LCD_INTERFACE);
  ili9341_write_data(0x01);
  ili9341_write_data(0x00);
  ili9341_write_data(0x06);
  
  ili9341_write_cmd(LCD_WR_MEM_CMD);
  hw_lcd_delay(10);//10ms
  
  ili9341_write_cmd(LCD_FRMCTR1);
  ili9341_write_data(0x00);
  ili9341_write_data(0x1B);
  ili9341_write_cmd(LCD_DFC);// Display Function Control 
  ili9341_write_data(0x0A);
  ili9341_write_data(0xA2);
  
  ili9341_write_cmd(LCD_3GAMMA_EN);
  ili9341_write_data(0x00);
  ili9341_write_cmd(LCD_GAMMA_SET);//Gamma curve selected 
  ili9341_write_data(0x01);
  
  ili9341_write_cmd(LCD_PGAMMA);
  ili9341_write_data(0x0F);
  ili9341_write_data(0x29);
  ili9341_write_data(0x24);
  ili9341_write_data(0x0C);
  ili9341_write_data(0x0E);
  ili9341_write_data(0x09);
  ili9341_write_data(0x4E);
  ili9341_write_data(0x78);
  ili9341_write_data(0x3C);
  ili9341_write_data(0x09);
  ili9341_write_data(0x13);
  ili9341_write_data(0x05);
  ili9341_write_data(0x17);
  ili9341_write_data(0x11);
  ili9341_write_data(0x00);
  ili9341_write_cmd(LCD_NGAMMA);
  ili9341_write_data(0x00);
  ili9341_write_data(0x16);
  ili9341_write_data(0x1B);
  ili9341_write_data(0x04);
  ili9341_write_data(0x11);
  ili9341_write_data(0x07);
  ili9341_write_data(0x31);
  ili9341_write_data(0x33);
  ili9341_write_data(0x42);
  ili9341_write_data(0x05);
  ili9341_write_data(0x0C);
  ili9341_write_data(0x0A);
  ili9341_write_data(0x28);
  ili9341_write_data(0x2F);
  ili9341_write_data(0x0F);
 #endif
  
   LCD_WRITE_CMD(0xCF);  
	LCD_WRITE_DATA(0x00); 
	LCD_WRITE_DATA(0xC1); 
	LCD_WRITE_DATA(0X30); 
	
	LCD_WRITE_CMD(0xED);  
	LCD_WRITE_DATA(0x64); 
	LCD_WRITE_DATA(0x03); 
	LCD_WRITE_DATA(0X12); 
	LCD_WRITE_DATA(0X81);
	
	LCD_WRITE_CMD(0xE8);  
	LCD_WRITE_DATA(0x85); 
	LCD_WRITE_DATA(0x10); 
	LCD_WRITE_DATA(0x7A); 
	
	LCD_WRITE_CMD(0xCB);  
	LCD_WRITE_DATA(0x39); 
	LCD_WRITE_DATA(0x2C); 
	LCD_WRITE_DATA(0x00); 
	LCD_WRITE_DATA(0x34); 
	LCD_WRITE_DATA(0x02);
	
	LCD_WRITE_CMD(0xF7);  
	LCD_WRITE_DATA(0x20); 
	LCD_WRITE_CMD(0xEA);  
	LCD_WRITE_DATA(0x00); 
	LCD_WRITE_DATA(0x00);
	
	LCD_WRITE_CMD(0xC0);    //Power control 
	LCD_WRITE_DATA(0x1B);   //VRH[5:0] 
	
	LCD_WRITE_CMD(0xC1);    //Power control 
	LCD_WRITE_DATA(0x01);   //SAP[2:0];BT[3:0] 
	
	LCD_WRITE_CMD(0xC5);    //VCM control 
	LCD_WRITE_DATA(0x30); 	 //3F
	LCD_WRITE_DATA(0x30); 	 //3C
	
	LCD_WRITE_CMD(0xC7);    //VCM control2 
	LCD_WRITE_DATA(0XB7); 
	
	LCD_WRITE_CMD(0x36);    // Memory Access Control 
	LCD_WRITE_DATA(0x08); 
	
	LCD_WRITE_CMD(0x3A);   
	LCD_WRITE_DATA(0x55); 
	
	LCD_WRITE_CMD(0xB1);   
	LCD_WRITE_DATA(0x00);   
	LCD_WRITE_DATA(0x1A); 
	
	LCD_WRITE_CMD(0xB6);    // Display Function Control 
	LCD_WRITE_DATA(0x0A); 
	LCD_WRITE_DATA(0xA2); 
	
	LCD_WRITE_CMD(0xF2);    // 3Gamma Function Disable 
	LCD_WRITE_DATA(0x00); 
	LCD_WRITE_CMD(0x26);    //Gamma curve selected 
	LCD_WRITE_DATA(0x01); 
	
	LCD_WRITE_CMD(0xE0);    //Set Gamma 
	LCD_WRITE_DATA(0x0F); 
	LCD_WRITE_DATA(0x2A); 
	LCD_WRITE_DATA(0x28); 
	LCD_WRITE_DATA(0x08); 
	LCD_WRITE_DATA(0x0E); 
	LCD_WRITE_DATA(0x08); 
	LCD_WRITE_DATA(0x54); 
	LCD_WRITE_DATA(0XA9); 
	LCD_WRITE_DATA(0x43); 
	LCD_WRITE_DATA(0x0A); 
	LCD_WRITE_DATA(0x0F); 
	LCD_WRITE_DATA(0x00); 
	LCD_WRITE_DATA(0x00); 
	LCD_WRITE_DATA(0x00); 
	LCD_WRITE_DATA(0x00); 		 
	
	LCD_WRITE_CMD(0XE1);    //Set Gamma 
	LCD_WRITE_DATA(0x00); 
	LCD_WRITE_DATA(0x15); 
	LCD_WRITE_DATA(0x17); 
	LCD_WRITE_DATA(0x07); 
	LCD_WRITE_DATA(0x11); 
	LCD_WRITE_DATA(0x06); 
	LCD_WRITE_DATA(0x2B); 
	LCD_WRITE_DATA(0x56); 
	LCD_WRITE_DATA(0x3C); 
	LCD_WRITE_DATA(0x05); 
	LCD_WRITE_DATA(0x10); 
	LCD_WRITE_DATA(0x0F); 
	LCD_WRITE_DATA(0x3F); 
	LCD_WRITE_DATA(0x3F); 
	LCD_WRITE_DATA(0x0F); 
	
	LCD_WRITE_CMD(0x2B); 
	LCD_WRITE_DATA(0x00);
	LCD_WRITE_DATA(0x00);
	LCD_WRITE_DATA(0x01);
	LCD_WRITE_DATA(0x3f);
	
	LCD_WRITE_CMD(0x2A); 
	LCD_WRITE_DATA(0x00);
	LCD_WRITE_DATA(0x00);
	LCD_WRITE_DATA(0x00);
	LCD_WRITE_DATA(0xef);	
  /* 设置屏幕方向和尺寸 */
  //LCD_SetDirection(LCD_DIRECTION);
  LCD_WRITE_CMD(LCD_SLEEP_OUT);
  //hw_lcd_delay(120);//10ms
   HAL_Delay(120);
  LCD_WRITE_CMD(LCD_DISPLAY_ON);
  /* GRAM start writing */
  LCD_WRITE_CMD(LCD_WR_MEM_CMD);
  ili9341_backlight_on();
	
	
}


void ili9341_set_cursor(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) 
{
	/* column address set */
	ili9341_write_cmd(LCD_COLUMN_ADDR_SET);
	ili9341_write_data(x1 >> 8);
	ili9341_write_data(x1 & 0xFF);
	ili9341_write_data(x2 >> 8);
	ili9341_write_data(x2 & 0xFF);
	
	/* Page address set */
	ili9341_write_cmd(LCD_PAGE_ADDR_SET);
	ili9341_write_data(y1 >> 8);
	ili9341_write_data(y1 & 0xFF);
	ili9341_write_data(y2 >> 8);
	ili9341_write_data(y2 & 0xFF);
	
}

void ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color) 
{
	ili9341_set_cursor(x, y, x, y);
	ili9341_write_cmd(LCD_WR_MEM_CMD);
	LCD_WRITE_DATA(color );
	//ili9341_write_data(color & 0xFF);
}

uint16_t ili9341_get_pixel(uint16_t x, uint16_t y) 
{
	uint16_t color;
	ili9341_set_cursor(x, y, x, y);
	ili9341_write_cmd(LCD_RD_MEM_CMD);
	color = LCD_READ_DATA();
	return color;
}

//区域大小:
// 
/**
  * @brief  在指定区域内 (xend-xstart)*(yend-ystart)填充指定颜色
  * @param  起止坐标和颜色值
  * @retval None
  */
void ili9341_fill(uint16_t xstart, uint16_t ystart, uint16_t xend, uint16_t yend, uint16_t color)
{          
	uint16_t i,j;
	uint16_t xlen = 0;

	xlen = xend - xstart + 1;	
	ili9341_set_cursor(xstart, ystart, xend, yend);      //设置光标位置 
	//ili9341_WriteRAM_Prepare();     //开始写入GRAM	
	ili9341_write_cmd(LCD_WR_MEM_CMD);
	for(i = ystart; i <= yend; i++)
	{	 			
		for(j = 0; j < xlen; j++)
		{
			//ili9341_write_data(color >> 8);
			//ili9341_write_data(color & 0xFF);
			ili9341_write_data16(color);
		}	    
	}					  	    
}

/**
  * @brief  Clear the screen.
  * @param  None
  * @retval None
  */
void ili9341_screen_clear(uint16_t color)
{
	ili9341_fill(0,0,ILI9341_LCD_PIXEL_WIDTH-1,ILI9341_LCD_PIXEL_HEIGHT-1,color);
}  

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void ili9341_display_on(void)
{
  /* Display On */
  ili9341_write_cmd(LCD_DISPLAY_ON);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ili9341_display_off(void)
{
  /* Display Off */
  ili9341_write_cmd(LCD_DISPLAY_OFF);
}

/**
  * @brief  Get LCD PIXEL WIDTH.
  * @param  None
  * @retval LCD PIXEL WIDTH.
  */
uint16_t ili9341_get_lcd_pixel_width(void)
{
  /* Return LCD PIXEL WIDTH */
  return ILI9341_LCD_PIXEL_WIDTH;
}

/**
  * @brief  Get LCD PIXEL HEIGHT.
  * @param  None
  * @retval LCD PIXEL HEIGHT.
  */
uint16_t ili9341_get_lcd_pixel_height(void)
{
  /* Return LCD PIXEL HEIGHT */
  return ILI9341_LCD_PIXEL_HEIGHT;
}



/**
  * @brief  Displays a bitmap picture loaded in the internal Flash.
  * @param  BmpAddress: Bmp picture address in the internal Flash.
  * @retval None
  */
void ili9341_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp)
{
  uint32_t index = 0, size = 0;
  
  /* Read bitmap size */
  size = *(volatile uint16_t *) (pbmp + 2);
  size |= (*(volatile uint16_t *) (pbmp + 4)) << 16;
  /* Get bitmap data address offset */
  index = *(volatile uint16_t *) (pbmp + 10);
  index |= (*(volatile uint16_t *) (pbmp + 12)) << 16;
  size = (size - index)/2;
  pbmp += index;
  
  /* Set GRAM write direction and BGR = 0 */
  /* Memory access control: MY = 0, MX = 1, MV = 0, ML = 0 */
  ili9341_write_cmd(LCD_MACTL);
  ili9341_write_data(0x40);

  /* Set Cursor */
  ili9341_set_cursor(Xpos, Ypos, Xpos, Ypos);  
 
  ili9341_multi_write_data((uint8_t*)pbmp, size*2);
 
  /* Set GRAM write direction and BGR = 0 */
  /* Memory access control: MY = 1, MX = 1, MV = 0, ML = 0 */
  ili9341_write_cmd(LCD_MACTL);
  ili9341_write_data(0xc0);
}

void ili9341_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint16_t t; 
	int xerr = 0,yerr = 0, delta_x, delta_y, distance; 
	int incx, incy, uRow, uCol; 

	delta_x = x2 - x1; //计算坐标增量 
	delta_y = y2 - y1; 
	uRow = x1; 
	uCol = y1; 
	if(delta_x > 0)
		incx = 1; //设置单步方向 
	else if(delta_x == 0)
		incx = 0;//垂直线 
	else 
	{
		incx = -1;
		delta_x = -delta_x;
	} 
	if(delta_y > 0)
		incy = 1; 
	else if(delta_y==0)
		incy = 0;//水平线 
	else
	{
		incy = -1;
		delta_y = -delta_y;
	} 
	if( delta_x > delta_y)
		distance = delta_x; //选取基本增量坐标轴 
	else 
		distance = delta_y; 
	for(t = 0; t <= distance + 1; t++ )//画线输出 
	{  
		ili9341_draw_pixel(uRow, uCol, color);//画点 
		xerr += delta_x ; 
		yerr += delta_y ; 
		if(xerr > distance) 
		{ 
			xerr -= distance; 
			uRow += incx; 
		} 
		if(yerr > distance) 
		{ 
			yerr -= distance; 
			uCol += incy; 
		} 
	}  
} 

/*  设置像素点 颜色,X,Y */
void rt_hw_lcd_set_pixel(const char *pixel, int x, int y)
{
	ili9341_draw_pixel((uint16_t)x, (uint16_t)y, *(int16_t*)pixel);  
}

/* 获取像素点颜色 */
void rt_hw_lcd_get_pixel(char *pixel, int x, int y)
{
	ili9341_set_cursor((uint16_t)x, (uint16_t)y, (uint16_t)x, (uint16_t)y);
	
	//lcd_read_ram_prepare();
	ili9341_write_cmd(LCD_MACTL);
    //*(rt_uint16_t*)pixel = read_data16(); //BGR2RGB( lcd_read_gram(x,y) ); 
		
}

/* 画水平线 */
void rt_hw_lcd_draw_hline(const char *pixel, int x1, int y1, int x2)
{	
	//ili9341_DrawHLine(*(rt_uint16_t*)pixel,(uint16_t)x1,(uint16_t)y1,(uint16_t)(x2 -x1));	
	ili9341_DrawLine((uint16_t)x1, (uint16_t)y1, (uint16_t)x2, (uint16_t)y1, *(uint16_t *)pixel);
}

/* 垂直线 */
void rt_hw_lcd_draw_vline(const char *pixel, int x1, int y1, int y2)
{
	//ili9341_DrawVLine(*(rt_uint16_t*)pixel,(uint16_t)x1,(uint16_t)y1,(uint16_t)(y2 -y1));
	ili9341_DrawLine((uint16_t)x1, (uint16_t)y1, (uint16_t)x1, (uint16_t)y2, *(uint16_t *)pixel);
}

struct rt_device_graphic_ops lcd_ili9341_ops =
{
    rt_hw_lcd_set_pixel,
    rt_hw_lcd_get_pixel,
    rt_hw_lcd_draw_hline,
    rt_hw_lcd_draw_vline,
    //rt_hw_lcd_draw_blit_dot
};

static rt_err_t lcd_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t lcd_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t lcd_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t lcd_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch (cmd)
    {
		case RTGRAPHIC_CTRL_GET_INFO:
		{
			struct rt_device_graphic_info *info;

			info = (struct rt_device_graphic_info*) args;
			RT_ASSERT(info != RT_NULL);

			info->bits_per_pixel = 16;
			info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565P;
			info->framebuffer = RT_NULL;
			info->width = ILI9341_LCD_PIXEL_WIDTH;
			info->height = ILI9341_LCD_PIXEL_HEIGHT;
		}
		break;

		case RTGRAPHIC_CTRL_RECT_UPDATE:
			/* nothong to be done */
			break;

		default:
			break;
    }

    return RT_EOK;
}
/*
static void lcd_lock(struct spi_lcd_device * lcd_device)
{
    rt_mutex_take(&lcd_device->lock, RT_WAITING_FOREVER);
}

static void lcd_unlock(struct spi_lcd_device * lcd_device)
{
    rt_mutex_release(&lcd_device->lock);
}*/

/**
  * @} 
  */ 
 
/**
  * @} define the gloab variable
  */ 
static struct rt_device _lcd_device; 
/**
  * @} Register the lcd ili9341 into the rtthread 
  */ 
int rt_hw_lcd_init(void)
{  
    ili9341_init();
	
	#if 1
	/* register lcd device */
	_lcd_device.type  = RT_Device_Class_Graphic;
	_lcd_device.init  = lcd_init;
	_lcd_device.open  = lcd_open;
	_lcd_device.close = lcd_close;
	_lcd_device.control = lcd_control;
	_lcd_device.read  = RT_NULL;
	_lcd_device.write = RT_NULL;

	_lcd_device.user_data = &lcd_ili9341_ops;   

    /* register graphic device driver */
	rt_device_register(&_lcd_device, "lcd",	RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);	
	#endif
	return 1;
}
INIT_DEVICE_EXPORT(rt_hw_lcd_init);

#endif /* RT_USING_LCD_ILI9341 */  

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
