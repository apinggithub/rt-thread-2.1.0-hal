#include "drv_lcd_ili9341.h"
#include "board.h"
#include <rtdevice.h>
#include <rthw.h>
#include <rtthread.h>
LCD_DrvTypeDef   ili9341_drv = 
{
  ili9341_Init,
  0,
  ili9341_DisplayOn,
  ili9341_DisplayOff,
  ili9341_SetCursor,
  ili9341_WritePixel,
  0,
  ili9341_SetDisplayWindow,
  ili9341_DrawHLine,
  ili9341_DrawVLine,
  ili9341_GetLcdPixelWidth,
  ili9341_GetLcdPixelHeight,
  ili9341_DrawBitmap,
};

static uint16_t ArrayRGB[320] = {0};
//static struct rt_device _lcd_device;

/**
* @}
*/ 

void Lcd_Clear(uint16_t Color)               
{	
   unsigned int i,m;
   ili9341_SetDisplayWindow(0,0,ST7735_LCD_PIXEL_WIDTH,ST7735_LCD_PIXEL_HEIGHT);
   LCD_IO_WriteReg(LCD_REG_44);/* Memory write: RAMWR */
   for(i=0;i<ST7735_LCD_PIXEL_WIDTH;i++)
    for(m=0;m<ST7735_LCD_PIXEL_HEIGHT;m++)
    {	
	  	LCD_IO_WriteData(Color>>8);
			LCD_IO_WriteData(Color);
    }   
}
void ili9341_FillRect(uint16_t Color, int x0, int y0, int x1, int y1)
{
	unsigned int i,m;
   ili9341_SetDisplayWindow(x0,y0,(x1-x0+1),(y1-y0+1));
  LCD_IO_WriteReg(LCD_REG_44);/* Memory write: RAMWR */
   for(i=0;i<(x1-x0+1);i++)
    for(m=0;m<(y1-y0+1);m++)
    {	
	  	LCD_IO_WriteData(Color>>8);
 			LCD_IO_WriteData(Color);
    }
}

/**
  * @brief  Initialize the ST7735 LCD Component.
  * @param  None
  * @retval None
  */
void ili9341_Init(void)
{    
  uint8_t data = 0;
  
  /* Initialize ili9341 low level bus layer -----------------------------------*/
  LCD_IO_Init();
  LCD_Reset();
  LCD_BL_light_On();
  	
  #if 0	
  /* Out of sleep mode, 0 args, no delay */
  ili9341_WriteReg(LCD_REG_17, 0x00); 
	LCD_Delay(50);
  /* Frame rate ctrl - normal mode, 3 args:Rate = fosc/(1x2+40) * (LINE+2C+2D)*/
  LCD_IO_WriteReg(LCD_REG_177);
  data = 0x01;
  LCD_IO_WriteMultipleData(&data, 1);
  data = 0x2C;
  LCD_IO_WriteMultipleData(&data, 1);
  data = 0x2D;
  LCD_IO_WriteMultipleData(&data, 1);
  /* Frame rate control - idle mode, 3 args:Rate = fosc/(1x2+40) * (LINE+2C+2D) */    
  ili9341_WriteReg(LCD_REG_178, 0x01);
  ili9341_WriteReg(LCD_REG_178, 0x2C);
  ili9341_WriteReg(LCD_REG_178, 0x2D);
  /* Frame rate ctrl - partial mode, 6 args: Dot inversion mode, Line inversion mode */ 
  ili9341_WriteReg(LCD_REG_179, 0x01);
  ili9341_WriteReg(LCD_REG_179, 0x2C);
  ili9341_WriteReg(LCD_REG_179, 0x2D);
  ili9341_WriteReg(LCD_REG_179, 0x01);
  ili9341_WriteReg(LCD_REG_179, 0x2C);
  ili9341_WriteReg(LCD_REG_179, 0x2D);
  /* Display inversion ctrl, 1 arg, no delay: No inversion */
  ili9341_WriteReg(LCD_REG_180, 0x07);
  /* Power control, 3 args, no delay: -4.6V , AUTO mode */
  ili9341_WriteReg(LCD_REG_192, 0xA2);
  ili9341_WriteReg(LCD_REG_192, 0x02);
  ili9341_WriteReg(LCD_REG_192, 0x84);
  /* Power control, 1 arg, no delay: VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD */
  ili9341_WriteReg(LCD_REG_193, 0xC5);
  /* Power control, 2 args, no delay: Opamp current small, Boost frequency */ 
  ili9341_WriteReg(LCD_REG_194, 0x0A);
  ili9341_WriteReg(LCD_REG_194, 0x00);
  /* Power control, 2 args, no delay: BCLK/2, Opamp current small & Medium low */  
  ili9341_WriteReg(LCD_REG_195, 0x8A);
  ili9341_WriteReg(LCD_REG_195, 0x2A);
  /* Power control, 2 args, no delay */
  ili9341_WriteReg(LCD_REG_196, 0x8A);
  ili9341_WriteReg(LCD_REG_196, 0xEE);
  /* Power control, 1 arg, no delay */
  ili9341_WriteReg(LCD_REG_197, 0x0E);
  /* Don't invert display, no args, no delay */
  LCD_IO_WriteReg(LCD_REG_32);
  /* Set color mode, 1 arg, no delay: 16-bit color */
  ili9341_WriteReg(LCD_REG_58, 0x05);
  /* Column addr set, 4 args, no delay: XSTART = 0, XEND = 127 */
  LCD_IO_WriteReg(LCD_REG_42);
  data = 0x00;
  LCD_IO_WriteMultipleData(&data, 1);
  LCD_IO_WriteMultipleData(&data, 1);
  LCD_IO_WriteMultipleData(&data, 1);
  data = 0x7F;
  LCD_IO_WriteMultipleData(&data, 1);
  /* Row addr set, 4 args, no delay: YSTART = 0, YEND = 159 */
  LCD_IO_WriteReg(LCD_REG_43);
  data = 0x00;
  LCD_IO_WriteMultipleData(&data, 1);
  LCD_IO_WriteMultipleData(&data, 1);
  LCD_IO_WriteMultipleData(&data, 1);
  data = 0x9F;
  LCD_IO_WriteMultipleData(&data, 1);
  /* Magical unicorn dust, 16 args, no delay */
  ili9341_WriteReg(LCD_REG_224, 0x02); 
  ili9341_WriteReg(LCD_REG_224, 0x1c);  
  ili9341_WriteReg(LCD_REG_224, 0x07); 
  ili9341_WriteReg(LCD_REG_224, 0x12);
  ili9341_WriteReg(LCD_REG_224, 0x37);  
  ili9341_WriteReg(LCD_REG_224, 0x32);  
  ili9341_WriteReg(LCD_REG_224, 0x29);  
  ili9341_WriteReg(LCD_REG_224, 0x2d);
  ili9341_WriteReg(LCD_REG_224, 0x29);  
  ili9341_WriteReg(LCD_REG_224, 0x25);  
  ili9341_WriteReg(LCD_REG_224, 0x2B);  
  ili9341_WriteReg(LCD_REG_224, 0x39);  
  ili9341_WriteReg(LCD_REG_224, 0x00);  
  ili9341_WriteReg(LCD_REG_224, 0x01);  
  ili9341_WriteReg(LCD_REG_224, 0x03);  
  ili9341_WriteReg(LCD_REG_224, 0x10);
  /* Sparkles and rainbows, 16 args, no delay */
  ili9341_WriteReg(LCD_REG_225, 0x03);
  ili9341_WriteReg(LCD_REG_225, 0x1d);  
  ili9341_WriteReg(LCD_REG_225, 0x07);  
  ili9341_WriteReg(LCD_REG_225, 0x06);
  ili9341_WriteReg(LCD_REG_225, 0x2E);  
  ili9341_WriteReg(LCD_REG_225, 0x2C);  
  ili9341_WriteReg(LCD_REG_225, 0x29);  
  ili9341_WriteReg(LCD_REG_225, 0x2D);
  ili9341_WriteReg(LCD_REG_225, 0x2E);  
  ili9341_WriteReg(LCD_REG_225, 0x2E);  
  ili9341_WriteReg(LCD_REG_225, 0x37);  
  ili9341_WriteReg(LCD_REG_225, 0x3F);  
  ili9341_WriteReg(LCD_REG_225, 0x00);  
  ili9341_WriteReg(LCD_REG_225, 0x00);  
  ili9341_WriteReg(LCD_REG_225, 0x02);  
  ili9341_WriteReg(LCD_REG_225, 0x10);
  /* Normal display on, no args, no delay */
  ili9341_WriteReg(LCD_REG_19, 0x00);
  /* Main screen turn on, no delay */
  ili9341_WriteReg(LCD_REG_41, 0x00);
  /* Memory access control: MY = 1, MX = 1, MV = 0, ML = 0 */
  ili9341_WriteReg(LCD_REG_54, 0xa0);
  #endif
  
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOn(void)
{
  uint8_t data = 0;
  LCD_IO_WriteReg(LCD_REG_19);
  LCD_Delay(10);
  LCD_IO_WriteReg(LCD_REG_41);
  LCD_Delay(10);
  LCD_IO_WriteReg(LCD_REG_54);
  data = 0xC0;
  LCD_IO_WriteMultipleData(&data, 1);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOff(void)
{
  uint8_t data = 0;
  LCD_IO_WriteReg(LCD_REG_19);
  LCD_Delay(10);
  LCD_IO_WriteReg(LCD_REG_40);
  LCD_Delay(10);
  LCD_IO_WriteReg(LCD_REG_54);
  data = 0xC0;
  LCD_IO_WriteMultipleData(&data, 1);
}

/**
  * @brief  Sets Cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @retval None
  */
void ili9341_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
  uint8_t data = 0;
  LCD_IO_WriteReg(LCD_REG_42);
  data = (Xpos) >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Xpos) & 0xFF;
  LCD_IO_WriteMultipleData(&data, 1);
  LCD_IO_WriteReg(LCD_REG_43); 
  data = (Ypos) >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Ypos) & 0xFF;
  LCD_IO_WriteMultipleData(&data, 1);
  LCD_IO_WriteReg(LCD_REG_44);/* Memory write: RAMWR */  
}

/**
  * @brief  Writes pixel.   
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  RGBCode: the RGB pixel color
  * @retval None
  */
void ili9341_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode)
{
  uint8_t data = 0;
  if((Xpos >= ILI9341_LCD_PIXEL_WIDTH) || (Ypos >= ILI9341_LCD_PIXEL_HEIGHT)) 
  {
    return;
  }
  
  /* Set Cursor */
  ili9341_SetCursor(Xpos, Ypos);
  
  data = RGBCode >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = RGBCode;
  LCD_IO_WriteMultipleData(&data, 1);
}  


/**
  * @brief  Writes to the selected LCD register.
  * @param  LCDReg: Address of the selected register.
  * @param  LCDRegValue: value to write to the selected register.
  * @retval None
  */
void ili9341_WriteReg(uint8_t LCDReg, uint8_t LCDRegValue)
{
  LCD_IO_WriteReg(LCDReg);
  LCD_IO_WriteMultipleData(&LCDRegValue, 1);
}

/**
  * @brief  Sets a display window
  * @param  Xpos:   specifies the X bottom left position.
  * @param  Ypos:   specifies the Y bottom left position.
  * @param  Height: display window height.
  * @param  Width:  display window width.
  * @retval None
  */
void ili9341_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  uint8_t data = 0;
  /* Column addr set, 4 args, no delay: XSTART = Xpos, XEND = (Xpos + Width - 1) */
  LCD_IO_WriteReg(LCD_REG_42);
  data = (Xpos) >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Xpos) & 0xFF;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Xpos + Width - 1) >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Xpos + Width - 1) & 0xFF;
  LCD_IO_WriteMultipleData(&data, 1);
  /* Row addr set, 4 args, no delay: YSTART = Ypos, YEND = (Ypos + Height - 1) */
  LCD_IO_WriteReg(LCD_REG_43);
  data = (Ypos) >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Ypos) & 0xFF;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Ypos + Height - 1) >> 8;
  LCD_IO_WriteMultipleData(&data, 1);
  data = (Ypos + Height - 1) & 0xFF;
  LCD_IO_WriteMultipleData(&data, 1);
}

/**
  * @brief  Draws horizontal line.
  * @param  RGBCode: Specifies the RGB color   
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Length: specifies the line length.  
  * @retval None
  */
void ili9341_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint8_t counter = 0;
  
  if(Xpos + Length > ST7735_LCD_PIXEL_WIDTH) return;
  
  /* Set Cursor */
  ili9341_SetCursor(Xpos, Ypos);
  
  for(counter = 0; counter < Length; counter++)
  {
    ArrayRGB[counter] = RGBCode;
  }
  LCD_IO_WriteMultipleData((uint8_t*)&ArrayRGB[0], Length * 2);
}

void ili9341_DrawHColorLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length,uint16_t *_pColor)
{
	uint8_t counter = 0;
  
  if(Xpos + Length > ST7735_LCD_PIXEL_WIDTH) return;
  
  /* Set Cursor */
  ili9341_SetCursor(Xpos, Ypos);
  
  for(counter = 0; counter < Length; counter++)
  {
    ArrayRGB[counter] = *_pColor++;
  }
  LCD_IO_WriteMultipleData((uint8_t*)&ArrayRGB[0], Length * 2);
	
}

/**
  * @brief  Draws vertical line.
  * @param  RGBCode: Specifies the RGB color   
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Length: specifies the line length.  
  * @retval None
  */
void ili9341_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint8_t counter = 0;
  
  if(Ypos + Length > ST7735_LCD_PIXEL_HEIGHT) return;
	
	ili9341_SetDisplayWindow(Xpos,Ypos,1,Length);
	/* Memory write: RAMWR */  
	LCD_IO_WriteReg(LCD_REG_44);
  for(counter = 0; counter < Length; counter++)
  {
    ArrayRGB[counter] = RGBCode;
  }
   LCD_IO_WriteMultipleData((uint8_t*)&ArrayRGB[0], Length * 2);	
}

/**
  * @brief  Gets the LCD pixel Width.
  * @param  None
  * @retval The Lcd Pixel Width
  */
uint16_t ili9341_GetLcdPixelWidth(void)
{
  return ST7735_LCD_PIXEL_WIDTH;
}

/**
  * @brief  Gets the LCD pixel Height.
  * @param  None
  * @retval The Lcd Pixel Height
  */
uint16_t ili9341_GetLcdPixelHeight(void)
{                          
  return ST7735_LCD_PIXEL_HEIGHT;
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
  ili9341_WriteReg(LCD_REG_54, 0xa0);

  /* Set Cursor */
  ili9341_SetCursor(Xpos, Ypos);  
 
  LCD_IO_WriteMultipleData((uint8_t*)pbmp, size*2);
 
  /* Set GRAM write direction and BGR = 0 */
  /* Memory access control: MY = 1, MX = 1, MV = 0, ML = 0 */
  ili9341_WriteReg(LCD_REG_54, 0xa0);
}
#if 0
/*  设置像素点 颜色,X,Y */
void rt_hw_lcd_set_pixel(const char *pixel, int x, int y)
{
  ili9341_WritePixel((uint16_t)x,(uint16_t)y,*(rt_uint16_t*)pixel);  
}

/* 获取像素点颜色 */
void rt_hw_lcd_get_pixel(char *pixel, int x, int y)
{
	//lcd_set_cursor(x,y,x+1,y+1);
	
	//lcd_read_ram_prepare();
	
    //*(rt_uint16_t*)pixel = read_data16(); //BGR2RGB( lcd_read_gram(x,y) ); 
		
}

/* 画水平线 */
void rt_hw_lcd_draw_hline( const char *pixel, int x1, int y1, int x2)
{
	
	ili9341_DrawHLine(*(rt_uint16_t*)pixel,(uint16_t)x1,(uint16_t)y1,(uint16_t)(x2 -x1));
	
}

/* 垂直线 */
void rt_hw_lcd_draw_vline(const char *pixel, int x1, int y1, int y2)
{
	ili9341_DrawVLine(*(rt_uint16_t*)pixel,(uint16_t)x1,(uint16_t)y1,(uint16_t)(y2 -y1));
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
			info->width = 160;
			info->height = 128;
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


void hw_lcd_init(void)
{

	/* register lcd device */
    _lcd_device.type  = RT_Device_Class_Graphic;
    _lcd_device.init  = lcd_init;
    _lcd_device.open  = lcd_open;
    _lcd_device.close = lcd_close;
    _lcd_device.control = lcd_control;
    _lcd_device.read  = RT_NULL;
    _lcd_device.write = RT_NULL;

    _lcd_device.user_data = &lcd_ili9341_ops;
		
    //ili9341_Init();
		/* init lcd */
    //Lcd_Clear(RED);
		

    /* register graphic device driver */
    rt_device_register(&_lcd_device, "lcd",
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
		
		//return RT_EOK;
}
//INIT_BOARD_EXPORT(hw_lcd_init);
#endif
