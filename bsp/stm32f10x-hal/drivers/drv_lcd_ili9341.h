/**
  ******************************************************************************
  * @file    ili9341.h
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file contains all the functions prototypes for the ili9341.c
  *          driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_LCD_ILI9341_H__
#define __DRV_LCD_ILI9341_H__

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
	 
#include <stdint.h>
	 
/** @addtogroup 画笔颜色
  * @{
  */ 

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	 0X841F //浅绿色
//#define LIGHTGRAY        0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)


/** 
  * @brief ILI9341 chip IDs  
  */ 
#define ILI9341_ID                  0x9341

/** 
  * @brief  ILI9341 Size  
  */  
#define  ILI9341_LCD_PIXEL_WIDTH    ((uint16_t)240)
#define  ILI9341_LCD_PIXEL_HEIGHT   ((uint16_t)320)

/** 
  * @brief  ILI9341 Timing  
  */     
/* Timing configuration  (Typical configuration from ILI9341 datasheet)
  HSYNC=10 (9+1)
  HBP=20 (29-10+1)
  ActiveW=240 (269-20-10+1)
  HFP=10 (279-240-20-10+1)

  VSYNC=2 (1+1)
  VBP=2 (3-2+1)
  ActiveH=320 (323-2-2+1)
  VFP=4 (327-320-2-2+1)
*/
#define  ILI9341_HSYNC            ((uint32_t)9)   /* Horizontal synchronization */
#define  ILI9341_HBP              ((uint32_t)29)    /* Horizontal back porch      */ 
#define  ILI9341_HFP              ((uint32_t)2)    /* Horizontal front porch     */
#define  ILI9341_VSYNC            ((uint32_t)1)   /* Vertical synchronization   */
#define  ILI9341_VBP              ((uint32_t)3)    /* Vertical back porch        */
#define  ILI9341_VFP              ((uint32_t)2)    /* Vertical front porch       */

/** 
  * @brief  ILI9341 Registers  
  */

/* Level 1 Commands */
#define LCD_SOFT_RESET             			0x01   /* Software Reset */
#define LCD_RD_DISP_ID     					0x04   /* Read display identification information */
#define LCD_RD_DISP_STAT     				0x09   /* Read Display Status */
#define LCD_RD_DISP_PWR_MOD          		0x0A   /* Read Display Power Mode */
#define LCD_RD_DISP_MADCTL           		0x0B   /* Read Display MADCTL */
#define LCD_RD_DISP_PIXEL_FMT           	0x0C   /* Read Display Pixel Format */
#define LCD_RD_DISP_IMG_FMT               	0x0D   /* Read Display Image Format */
#define LCD_RD_DISP_SIG_MOD               	0x0E   /* Read Display Signal Mode */
#define LCD_RD_DISP_SDGN_RSLT             	0x0F   /* Read Display Self-Diagnostic Result */
#define LCD_SLEEP_IN               			0x10   /* Enter Sleep Mode */
#define LCD_SLEEP_OUT           			0x11   /* Sleep out register */
#define LCD_PTL_ON               			0x12   /* Partial Mode ON */
#define LCD_NORMAL_MODE_ON      			0x13   /* Normal Display Mode ON */
#define LCD_DISP_INV_OFF             		0x20   /* Display Inversion OFF */
#define LCD_DISP_INV_ON              		0x21   /* Display Inversion ON */
#define LCD_GAMMA_SET           			0x26   /* Gamma Set register */
#define LCD_DISPLAY_OFF         			0x28   /* Display off register */
#define LCD_DISPLAY_ON          			0x29   /* Display on register */
#define LCD_COLUMN_ADDR_SET         		0x2A   /* Colomn address set register */ 
#define LCD_PAGE_ADDR_SET           		0x2B   /* Page address set register */ 
#define LCD_WR_MEM_CMD                		0x2C   /* Memory Write register, transfer data from MCU to frame memory.*/   
#define LCD_RGB_SET              			0x2D   /* Color SET */   
#define LCD_RD_MEM_CMD               		0x2E   /* Memory Read , transfers image data from ILI9341抯 frame memory to the host processor*/   
#define LCD_PLT_AREA               			0x30   /* Partial Area */   
#define LCD_VTCL_SCR_DEF             		0x33   /* Vertical Scrolling Definition */   
#define LCD_TEAR_EFT_OFF               		0x34   /* Tearing Effect Line OFF */   
#define LCD_TEAR_EFT_ON                		0x35   /* Tearing Effect Line ON */   
#define LCD_MACTL                 			0x36   /* Memory Access Control register*/
#define LCD_VTCL_SCRL_START_ADD            	0x37   /* Vertical Scrolling Start Address */   
#define LCD_IDL_MOD_OFF              		0x38   /* Idle Mode OFF */   
#define LCD_IDL_MOD_ON               		0x39   /* Idle Mode ON */   
#define LCD_PIXEL_FORMAT_SET        		0x3A   /* Pixel Format register */
#define LCD_WR_MEM_CONTINUE  				0x3C   /* Write Memory Continue */   
#define LCD_RD_MEM_CONTINUE   				0x3E   /* Read Memory Continue */   
#define LCD_SET_TEAR_SCANLINE   			0x44   /* Set Tear Scanline */   
#define LCD_GET_SCANLINE        			0x45   /* Get Scanline */   
#define LCD_WR_DISP_BRIGHT               	0x51   /* Write Brightness Display register */
#define LCD_RD_DISP_BRIGHT             		0x52   /* Read Display Brightness */   
#define LCD_WR_CTRL_DISP                 	0x53   /* Write Control Display register*/
#define LCD_RD_CTRL_DISP             		0x54   /* Read CTRL Display */   
#define LCD_WR_CABC              			0x55   /* Write Content Adaptive Brightness Control */   
#define LCD_RD_CABC              			0x56   /* Read Content Adaptive Brightness Control */   
#define LCD_WR_MCABC          				0x5E   /* Write CABC Minimum Brightness */   
#define LCD_RD_MCABC           				0x5F   /* Read CABC Minimum Brightness */   
#define LCD_READ_ID1            			0xDA   /* Read ID1 */
#define LCD_READ_ID2            			0xDB   /* Read ID2 */
#define LCD_READ_ID3            			0xDC   /* Read ID3 */

/* Level 2 Commands */
#define LCD_RGB_INTERFACE       0xB0   /* RGB Interface Signal Control */
#define LCD_FRMCTR1             0xB1   /* Frame Rate Control (In Normal Mode) */
#define LCD_FRMCTR2             0xB2   /* Frame Rate Control (In Idle Mode) */
#define LCD_FRMCTR3             0xB3   /* Frame Rate Control (In Partial Mode) */
#define LCD_INVTR               0xB4   /* Display Inversion Control */
#define LCD_BPC                 0xB5   /* Blanking Porch Control register */
#define LCD_DFC                 0xB6   /* Display Function Control register */
#define LCD_ETMOD               0xB7   /* Entry Mode Set */
#define LCD_BACKLIGHT1          0xB8   /* Backlight Control 1 */
#define LCD_BACKLIGHT2          0xB9   /* Backlight Control 2 */
#define LCD_BACKLIGHT3          0xBA   /* Backlight Control 3 */
#define LCD_BACKLIGHT4          0xBB   /* Backlight Control 4 */
#define LCD_BACKLIGHT5          0xBC   /* Backlight Control 5 */
#define LCD_BACKLIGHT7          0xBE   /* Backlight Control 7 */
#define LCD_BACKLIGHT8          0xBF   /* Backlight Control 8 */
#define LCD_POWER1              0xC0   /* Power Control 1 register */
#define LCD_POWER2              0xC1   /* Power Control 2 register */
#define LCD_VCOM1               0xC5   /* VCOM Control 1 register */
#define LCD_VCOM2               0xC7   /* VCOM Control 2 register */
#define LCD_NVMWR               0xD0   /* NV Memory Write */
#define LCD_NVMPKEY             0xD1   /* NV Memory Protection Key */
#define LCD_RDNVM               0xD2   /* NV Memory Status Read */
#define LCD_READ_ID4            0xD3   /* Read ID4 */
#define LCD_PGAMMA              0xE0   /* Positive Gamma Correction register */
#define LCD_NGAMMA              0xE1   /* Negative Gamma Correction register */
#define LCD_DGAMCTRL1           0xE2   /* Digital Gamma Control 1 */
#define LCD_DGAMCTRL2           0xE3   /* Digital Gamma Control 2 */
#define LCD_INTERFACE           0xF6   /* Interface control register */

/* Extend register commands */
#define LCD_POWERA               0xCB   /* Power control A register */
#define LCD_POWERB               0xCF   /* Power control B register */
#define LCD_DTCA                 0xE8   /* Driver timing control A */
#define LCD_DTCB                 0xEA   /* Driver timing control B */
#define LCD_POWER_SEQ            0xED   /* Power on sequence register */
#define LCD_3GAMMA_EN            0xF2   /* 3 Gamma enable register */
#define LCD_PRC                  0xF7   /* Pump ratio control register */

/* Size of read registers */
#define LCD_READ_ID4_SIZE        3      /* Size of Read ID4 */

/**
  * @}
  */
typedef struct
{
  void     (*Init)(void);
  uint16_t (*ReadID)(void);
  void     (*DisplayOn)(void);
  void     (*DisplayOff)(void);
  void     (*SetCursor)(uint16_t, uint16_t);
  void     (*WritePixel)(uint16_t, uint16_t, uint16_t);
  uint16_t (*ReadPixel)(uint16_t, uint16_t);
  
   /* Optimized operation */
  void     (*SetDisplayWindow)(uint16_t, uint16_t, uint16_t, uint16_t);
  void     (*DrawHLine)(uint16_t, uint16_t, uint16_t, uint16_t);
  void     (*DrawVLine)(uint16_t, uint16_t, uint16_t, uint16_t);
  
  uint16_t (*GetLcdPixelWidth)(void);
  uint16_t (*GetLcdPixelHeight)(void);
  void     (*DrawBitmap)(uint16_t, uint16_t, uint8_t*);
  void     (*DrawRGBImage)(uint16_t, uint16_t, uint16_t, uint16_t, uint8_t*);
}LCD_DrvTypeDef;    
  
/** @defgroup ILI9341_Exported_Functions
  * @{
  */ 

void     ili9341_init(void);
uint16_t ili9341_read_id(void);
void     ili9341_write_cmd(uint8_t reg);
void     ili9341_write_data(uint8_t data);
uint8_t  ili9341_read_data(void);
void     ili9341_display_on(void);
void     ili9341_display_off(void);
void ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
uint16_t ili9341_get_pixel(uint16_t x, uint16_t y); 

uint16_t ili9341_get_lcd_pixel_width(void);
uint16_t ili9341_get_lcd_pixel_height(void);

//int 	rt_hw_lcd_init(void);
void 	ili9341_screen_clear(uint16_t color);
  
#ifdef __cplusplus
}
#endif

#endif /* __DRV_LCD_ILI9341_H__ */


  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
