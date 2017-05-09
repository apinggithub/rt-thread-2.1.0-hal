#include "drv_lcd_xxx.h"
#include "stdlib.h"
#include <rtthread.h>
//#include "font.h" 
//#include "usart.h"
//#include "delay.h"	 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//2.4/2.8寸TFT液晶驱动	  
//支持驱动IC型号包括:ILI9341/ILI9325/RM68021/ILI9320/ILI9328/LGDP4531/LGDP4535/SPFD5408/SSD1289/1505/B505/C505等
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2011/1/13
//版本：V1.7
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//********************************************************************************
//V1.2修改说明
//支持了SPFD5408的驱动,另外把液晶ID直接打印成HEX格式.方便查看LCD驱动IC.
//V1.3
//加入了快速IO的支持
//修改了背光控制的极性（适用于V1.8及以后的开发板版本）
//对于1.8版本之前(不包括1.8)的液晶模块,请修改LCD_Init函数的LCD_LED=1;为LCD_LED=1;
//V1.4
//修改了LCD_ShowChar函数，使用画点功能画字符。
//加入了横竖屏显示的支持
//V1.5 20110730
//1,修改了B505液晶读颜色有误的bug.
//2,修改了快速IO及横竖屏的设置方式.
//V1.6 20111116
//1,加入对LGDP4535液晶的驱动支持
//V1.7 20120713
//1,增加LCD_RD_DATA函数
//2,增加对ILI9341的支持
//3,增加ILI9325的独立驱动代码
//4,增加LCD_Scan_Dir函数(慎重使用)	  
//6,另外修改了部分原来的函数,以适应9341的操作
//////////////////////////////////////////////////////////////////////////////////
	   					 
//画笔颜色,背景颜色
uint16_t POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
uint16_t DeviceCode = 0;//0x9341;	
//uint16_t DeviceState[5];

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

//写寄存器函数
void LCD_WR_REG(uint8_t data)
{ 
	LCD_RD_SET;
	LCD_RS_CLR;//写地址  
 	LCD_CS_CLR; 
	DATAOUT(data); 
	LCD_WR_CLR; 
	LCD_WR_SET; 
 	LCD_CS_SET;   
}
void LCD_WR_DATA(uint16_t data)
{
	
#ifdef LCD_USING_DATA_WTDTH_8BIT //使用8位并行数据总线模式
	LCD_RS_SET;
	LCD_CS_CLR;
	DATAOUT( data<<8 );
	LCD_WR_CLR;
	LCD_WR_SET;
	LCD_CS_SET;
	
#else//使用16位并行数据总线模式
	LCD_RD_SET;
	LCD_RS_SET;
	LCD_CS_CLR;	
	DATAOUT(data);	
	LCD_WR_CLR;
	LCD_WR_SET;
	LCD_CS_SET;
			
#endif
}

   
//读LCD数据
//返回值:读到的值
uint16_t LCD_RD_DATA(void)
{										   
	uint16_t t;
	
	#ifdef LCD_USING_DATA_WTDTH_8BIT
 	GPIOB->CRL = 0x88888888; //PB0-7  上拉输入
	GPIOB->ODR = 0x00;     //全部输出0
	#else
	GPIOB->CRL = 0x88888888; //PB0-7  上拉输入
	GPIOB->CRH = 0x88888888; //PB8-15 上拉输入	
	GPIOB->ODR = 0x0000;     //全部输出0
	#endif

	LCD_WR_SET;
	LCD_RS_SET;
	LCD_CS_CLR;
	//读取数据(读寄存器时,并不需要读2次)
	LCD_RD_CLR;			   
	LCD_RD_SET;
	t = DATAIN;  
	LCD_CS_SET; 

	#ifdef LCD_USING_DATA_WTDTH_8BIT
	GPIOB->CRL = 0x33333333; //PB0-7  上拉输出
	GPIOB->ODR = 0xFF;    //全部输出高
	#else
	GPIOB->CRL = 0x33333333; //PB0-7  上拉输出
	GPIOB->CRH = 0x33333333; //PB8-15 上拉输出
	GPIOB->ODR = 0xFFFF;    //全部输出高
	#endif
	
	return t;  
}	

//写寄存器
//LCD_Reg:寄存器编号
//LCD_RegValue:要写入的值
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}	   
//读寄存器
//LCD_Reg:寄存器编号
//返回值:读到的值
uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{										   
 	LCD_WR_REG(LCD_Reg);  //写入要读的寄存器号  
	return LCD_RD_DATA(); 
}  	 
//开始写GRAM
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(0x2C);//ILI9341开始写 RAM 指令是 0x2c

}
//开始写GRAM
void LCD_ReadRAM_Prepare(void)
{
	LCD_WR_REG(0x2E);//ILI9341开始读 RAM 指令是 0x2e

}	
//LCD写GRAM
//RGB_Code:颜色值
void LCD_WriteRAM(uint16_t RGB_Code)
{							    
	LCD_WR_DATA(RGB_Code);//写十六位GRAM
}
//从ILI93xx读出的数据为GBR格式，而我们写入的时候为RGB格式。
//通过该函数转换
//c:GBR格式的颜色值
//返回值：RGB格式的颜色值
uint16_t LCD_BGR2RGB(uint16_t c)
{
	uint16_t  r,g,b,rgb;   
	b=( c >> 0 )& 0x1f;
	g=( c >> 5 )& 0x3f;
	r=( c >> 11 )&0x1f;	 
	rgb=( b << 11 ) + ( g << 5 ) + (r << 0);		 
	return(rgb);
}
		 
//读取个某点的颜色值	 
//x:0~239
//y:0~319
//返回值:此点的颜色
uint16_t LCD_Read_Point(void)
{
 	uint16_t r;
   
	//LCD_SetCursor(x,y);	
	LCD_WR_REG(0x2E);	//ILI9341发送读GRAM指令
	#ifdef LCD_USING_DATA_WTDTH_8BIT
 	GPIOB->CRL = 0x88888888; //PB0-7  上拉输入
	GPIOB->ODR = 0xFF;      //全部输出0
	#else
	GPIOB->CRL = 0x88888888; //PB0-7  上拉输入
	GPIOB->CRH = 0x88888888; //PB8-15 上拉输入
	GPIOB->ODR = 0xFFFF;     //全部输出高
	#endif
	
	LCD_RS_SET;
	LCD_CS_CLR;
	//读取数据(读GRAM时,第一次为假读)	
	LCD_RD_CLR;	
	rt_hw_us_delay(1);	//delay_us(1);//延时1us					   
	LCD_RD_SET;
 	//dummy READ
	LCD_RD_CLR;					   
	rt_hw_us_delay(1); //delay_us(1);//延时1us					   
	LCD_RD_SET;
 	r = DATAIN;  
	LCD_CS_SET;

	#ifdef LCD_USING_DATA_WTDTH_8BIT
	GPIOB->CRL = 0x33333333; //PB0-7  上拉输出
	GPIOB->ODR = 0xFF;    //全部输出高
	#else
	GPIOB->CRL = 0x33333333; //PB0-7  上拉输出
	GPIOB->CRH = 0x33333333; //PB8-15 上拉输出
	GPIOB->ODR = 0xFFFF;    //全部输出高  
	#endif
	return r;						//ILI9341需要公式转换一下
}
//LCD开启显示
void LCD_DisplayOn(void)
{	
	LCD_WR_REG(0x29);	//开启显示
}	 
//LCD关闭显示
void LCD_DisplayOff(void)
{	   
	LCD_WR_REG(0x28);	//关闭显示
}   
//设置光标位置
//Xpos:横坐标
//Ypos:纵坐标
__inline void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
#ifdef LCD_USING_HORIZONTAL
	if(DeviceCode == 0x8989)
	{
		//Xpos = 319 - Xpos; LCD_HIGHT
		Xpos = LCD_HIGHT - Xpos; 
		LCD_WriteReg(0x4E, Ypos);
		LCD_WriteReg(0x4F, Xpos);
    }
	else if(DeviceCode == 0x9341)//9341,设置坐标
	{			 
		LCD_WR_REG(0x2B); 
		LCD_WR_DATA(Xpos>>8); 
		LCD_WR_DATA(Xpos&0xFF);	 
		LCD_WR_REG(0x2A); 
		LCD_WR_DATA(Ypos>>8); 
		LCD_WR_DATA(Ypos&0xFF);						
	}
	else
	{
		//Xpos = 319 - Xpos;
		Xpos = LCD_HIGHT - Xpos; 
		LCD_WriteReg(R32,Ypos);
		LCD_WriteReg(R33,Xpos);
	}							   
#else
	if(DeviceCode == 0x8989)
	{
		LCD_WriteReg(0x4E, Xpos);
		LCD_WriteReg(0x4F, Ypos);
    }
	else if(DeviceCode == 0x9341)
	{		    
		LCD_WR_REG(0x2A); 
		LCD_WR_DATA(Xpos>>8); 
		LCD_WR_DATA(Xpos&0xFF);	 
		LCD_WR_REG(0x2B); 
		LCD_WR_DATA(Ypos>>8); 
		LCD_WR_DATA(Ypos&0xFF);		
	}
	else
	{
		LCD_WriteReg(R32, Xpos);
		LCD_WriteReg(R33, Ypos);
	}						    
#endif
}

//设置LCD的自动扫描方向
//注意:其他函数可能会受到此函数设置的影响(尤其是9341这个奇葩),所以,
//一般设置为L2R_U2D即可,如果设置为其他扫描方式,可能导致显示不正常.
//0~7：代表8个方向(具体定义见lcd.h)
//9320/9325/9328/4531/4535/1505/b505/8989/5408/9341等IC已经实际测试	   
void LCD_Scan_Dir(uint8_t dir)
{
	uint16_t regval = 0;
	uint8_t dirreg = 0;
#ifdef LCD_USING_HORIZONTAL //使用横屏
	switch(dir)//方向转换
	{
		case 0:dir=6;break;
		case 1:dir=7;break;
		case 2:dir=4;break;
		case 3:dir=5;break;
		case 4:dir=1;break;
		case 5:dir=0;break;
		case 6:dir=3;break;
		case 7:dir=2;break;	     
	}
#endif
	if(DeviceCode==0x9341)//9341,很特殊
	{
		switch(dir)
		{
			case L2R_U2D://从左到右,从上到下
				regval|=(0<<7)|(0<<6)|(0<<5); 
				break;
			case L2R_D2U://从左到右,从下到上
				regval|=(1<<7)|(0<<6)|(0<<5); 
				break;
			case R2L_U2D://从右到左,从上到下
				regval|=(0<<7)|(1<<6)|(0<<5); 
				break;
			case R2L_D2U://从右到左,从下到上
				regval|=(1<<7)|(1<<6)|(0<<5); 
				break;	 
			case U2D_L2R://从上到下,从左到右
				regval|=(0<<7)|(0<<6)|(1<<5); 
				break;
			case U2D_R2L://从上到下,从右到左
				regval|=(0<<7)|(1<<6)|(1<<5); 
				break;
			case D2U_L2R://从下到上,从左到右
				regval|=(1<<7)|(0<<6)|(1<<5); 
				break;
			case D2U_R2L://从下到上,从右到左
				regval|=(1<<7)|(1<<6)|(1<<5); 
				break;	 
		}
		dirreg = 0x36;
 		regval |= 0x08;//BGR   
		LCD_WriteReg(dirreg,regval);
		if(regval&0x20)
		{
 	 		LCD_WR_REG(0x2A); 
			LCD_WR_DATA(0);
			LCD_WR_DATA(0);
			LCD_WR_DATA(319>>8);
			LCD_WR_DATA(319&0xFF);
	 		LCD_WR_REG(0x2B); 
			LCD_WR_DATA(0);
			LCD_WR_DATA(0);
			LCD_WR_DATA(239>>8);
			LCD_WR_DATA(239&0xFF);
		}
		else 
		{
 	 		LCD_WR_REG(0x2A); 
			LCD_WR_DATA(0);
			LCD_WR_DATA(0);
			LCD_WR_DATA(239>>8);
			LCD_WR_DATA(239&0xFF);
	 		LCD_WR_REG(0x2B); 
			LCD_WR_DATA(0);
			LCD_WR_DATA(0);
			LCD_WR_DATA(319>>8);
			LCD_WR_DATA(319&0xFF);
		}  
 	}else 
	{
		switch(dir)
		{
			case L2R_U2D://从左到右,从上到下
				regval|=(1<<5)|(1<<4)|(0<<3); 
				break;
			case L2R_D2U://从左到右,从下到上
				regval|=(0<<5)|(1<<4)|(0<<3); 
				break;
			case R2L_U2D://从右到左,从上到下
				regval|=(1<<5)|(0<<4)|(0<<3);
				break;
			case R2L_D2U://从右到左,从下到上
				regval|=(0<<5)|(0<<4)|(0<<3); 
				break;	 
			case U2D_L2R://从上到下,从左到右
				regval|=(1<<5)|(1<<4)|(1<<3); 
				break;
			case U2D_R2L://从上到下,从右到左
				regval|=(1<<5)|(0<<4)|(1<<3); 
				break;
			case D2U_L2R://从下到上,从左到右
				regval|=(0<<5)|(1<<4)|(1<<3); 
				break;
			case D2U_R2L://从下到上,从右到左
				regval|=(0<<5)|(0<<4)|(1<<3); 
				break;	 
		}
		if(DeviceCode == 0x8989)//8989 IC
		{
			dirreg = 0x11;
			regval |= 0x6040;	//65K   
	 	}else//其他驱动IC		  
		{
			dirreg = 0x03;
			regval |= 1<<12;  
		}
		LCD_WriteReg(dirreg,regval);
	}
} 
//画点
//x:0~239
//y:0~319
//POINT_COLOR:此点的颜色
void LCD_SetPointPixel(uint16_t x,uint16_t y, uint16_t color)
{
	LCD_SetCursor(x,y);//设置光标位置 
	LCD_WriteRAM_Prepare();	//开始写入GRAM
	LCD_WR_DATA(color); //POINT_COLOR
} 
//画点
//x:0~239
//y:0~319
//POINT_COLOR:此点的颜色
uint16_t LCD_GetPointPixel(uint16_t x,uint16_t y)
{
	LCD_SetCursor(x,y);//设置光标位置 
	LCD_ReadRAM_Prepare();//开始读GRAM
	return LCD_RD_DATA(); //POINT_COLOR
} 

//初始化lcd
//该初始化函数可以初始化各种ILI93XX液晶,但是其他函数是基于ILI9320的!!!
//在其他型号的驱动芯片上没有测试! 
void LCD_Device_Init(void)
{
	//delay_ms(50); //delay 50 ms 
	LCD_WriteReg(0x0000, 0x0001);
	//delay_ms(50); // delay 50 ms 
	//rt_thread_delay( RT_TICK_PER_SECOND*50/1000 );
	
	DeviceCode = LCD_ReadReg(0x0000);   
 	if(DeviceCode == 0 || DeviceCode == 0xFFFF)//读到ID不正确
	{	
		//可能是9341,尝试9341的ID读取		
		LCD_WR_REG(0xD3);		
		LCD_RD_DATA(); 			//dummy read 	
 		LCD_RD_DATA();   	    //读回0x00
  		DeviceCode = LCD_RD_DATA()&0xBF;//这里读回0xD3,实际是0x93才对.强制去掉第6位   								   
 		DeviceCode <<= 8;
		DeviceCode |= LCD_RD_DATA();	     
	}
	//printf(" LCD ID:%x\r\n",DeviceCode); //打印LCD ID */
	if(DeviceCode == 0x9341)	//9341初始化
	{
		LCD_WR_REG(0xCF);  
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0xC1); 
		LCD_WR_DATA(0x30); 
		LCD_WR_REG(0xED);  
		LCD_WR_DATA(0x64); 
		LCD_WR_DATA(0x03); 
		LCD_WR_DATA(0x12); 
		LCD_WR_DATA(0x81); 
		LCD_WR_REG(0xE8);  
		LCD_WR_DATA(0x85); 
		LCD_WR_DATA(0x10); 
		LCD_WR_DATA(0x7A); 
		LCD_WR_REG(0xCB);  
		LCD_WR_DATA(0x39); 
		LCD_WR_DATA(0x2C); 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x34); 
		LCD_WR_DATA(0x02); 
		LCD_WR_REG(0xF7);  
		LCD_WR_DATA(0x20); 
		LCD_WR_REG(0xEA);  
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x00); 
		LCD_WR_REG(0xC0);    //Power control 
		LCD_WR_DATA(0x1B);   //VRH[5:0] 
		LCD_WR_REG(0xC1);    //Power control 
		LCD_WR_DATA(0x01);   //SAP[2:0];BT[3:0] 
		LCD_WR_REG(0xC5);    //VCM control 
		LCD_WR_DATA(0x30); 	 //3F
		LCD_WR_DATA(0x30); 	 //3C
		LCD_WR_REG(0xC7);    //VCM control2 
		LCD_WR_DATA(0xB7); 
		LCD_WR_REG(0x36);    // Memory Access Control 
		LCD_WR_DATA(0x48); 
		LCD_WR_REG(0x3A);   
		LCD_WR_DATA(0x55); 
		LCD_WR_REG(0xB1);   
		LCD_WR_DATA(0x00);   
		LCD_WR_DATA(0x1A); 
		LCD_WR_REG(0xB6);    // Display Function Control 
		LCD_WR_DATA(0x0A); 
		LCD_WR_DATA(0xA2); 
		LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
		LCD_WR_DATA(0x00); 
		LCD_WR_REG(0x26);    //Gamma curve selected 
		LCD_WR_DATA(0x01); 
		LCD_WR_REG(0xE0);    //Set Gamma 
		LCD_WR_DATA(0x0F); 
		LCD_WR_DATA(0x2A); 
		LCD_WR_DATA(0x28); 
		LCD_WR_DATA(0x08); 
		LCD_WR_DATA(0x0E); 
		LCD_WR_DATA(0x08); 
		LCD_WR_DATA(0x54); 
		LCD_WR_DATA(0xA9); 
		LCD_WR_DATA(0x43); 
		LCD_WR_DATA(0x0A); 
		LCD_WR_DATA(0x0F); 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x00); 		 
		LCD_WR_REG(0xE1);    //Set Gamma 
		LCD_WR_DATA(0x00); 
		LCD_WR_DATA(0x15); 
		LCD_WR_DATA(0x17); 
		LCD_WR_DATA(0x07); 
		LCD_WR_DATA(0x11); 
		LCD_WR_DATA(0x06); 
		LCD_WR_DATA(0x2B); 
		LCD_WR_DATA(0x56); 
		LCD_WR_DATA(0x3C); 
		LCD_WR_DATA(0x05); 
		LCD_WR_DATA(0x10); 
		LCD_WR_DATA(0x0F); 
		LCD_WR_DATA(0x3F); 
		LCD_WR_DATA(0x3F); 
		LCD_WR_DATA(0x0F); 
		LCD_WR_REG(0x2B); 
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x01);
		LCD_WR_DATA(0x3f);
		LCD_WR_REG(0x2A); 
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0x00);
		LCD_WR_DATA(0xef);	 
		LCD_WR_REG(0x11); //Exit Sleep
		//delay_ms(120);
		//rt_thread_delay( RT_TICK_PER_SECOND/100 );
		LCD_WR_REG(0x29); //display on					
	}
	
	LCD_Scan_Dir(DFT_SCAN_DIR);		 
	LCD_bklight(1);//LCD_LED = 1;//点亮背光	 
	LCD_Clear(WHITE);//BLUE
}

//清屏函数
//Color:要清屏的填充色
void LCD_Clear(uint16_t Color)
{
	uint32_t index = 0;      
	LCD_SetCursor(0x00,0x0000);//设置光标位置 
	LCD_WriteRAM_Prepare();     //开始写入GRAM	 	  
	for(index = 0; index < 76800; index++ )
	{
		LCD_WR_DATA(Color);    
	}
}  
//在指定区域内填充指定颜色
//区域大小:(xend-xsta+1)*(yend-ysta+1)
//xsta
//color:要填充的颜色
void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color)
{          
	uint16_t i,j;
	uint16_t xlen=0;
	LCD_Scan_Dir(L2R_U2D);
#ifdef LCD_USING_HORIZONTAL
	xlen = ey - sy + 1;	   
	for(i = sx; i <= ex; i++)
	{
	 	LCD_SetCursor( i,sy );      			//设置光标位置 
		LCD_WriteRAM_Prepare();     			//开始写入GRAM	  
		for(j = 0; j < xlen; j++ )
		{
			LCD_WR_DATA(color);	//设置光标位置 	  
		}
	}
#else
	xlen = ex - sx + 1;	   
	for(i = sy; i <= ey; i++)
	{
	 	LCD_SetCursor(sx, i);      				//设置光标位置 
		LCD_WriteRAM_Prepare();     			//开始写入GRAM	  
		for(j = 0; j < xlen; j++ )
		{
			LCD_WR_DATA(color);	//设置光标位置 	  
		}			
	}
#endif
	LCD_Scan_Dir(DFT_SCAN_DIR);
}  

//画线
//x1,y1:起点坐标
//x2,y2:终点坐标  
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint16_t t; 
	int xerr = 0, yerr = 0, delta_x, delta_y, distance; 
	int incx, incy, uRow, uCol; 

	delta_x = x2 - x1; //计算坐标增量 
	delta_y = y2 - y1; 
	uRow = x1; 
	uCol = y1; 
	if(delta_x > 0)
	{
		incx = 1; //设置单步方向 
	}
	else if(delta_x == 0)
	{
		incx = 0;//垂直线 
	}
	else 
	{
		incx = -1;
		delta_x = -delta_x;
	} 
	if(delta_y > 0)
	{
		incy = 1;
	}		
	else if(delta_y == 0)
	{
		incy = 0;//水平线 
	}
	else
	{
		incy = -1;
		delta_y = -delta_y;
	} 
	if( delta_x > delta_y)
	{
		distance = delta_x; //选取基本增量坐标轴 
	}
	else 
	{
		distance = delta_y; 
	}
	for(t = 0; t <= distance + 1; t++ )//画线输出 
	{  
		LCD_SetPointPixel(uRow, uCol, color);//画点 
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
//画矩形
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	LCD_DrawLine(x1, y1, x2, y1, color);
	LCD_DrawLine(x1, y1, x1, y2, color);
	LCD_DrawLine(x1, y2, x2, y2, color);
	LCD_DrawLine(x2, y1, x2, y2, color);
}
//在指定位置画一个指定大小的圆
//(x,y):中心点
//r    :半径
void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
	int a,b;
	int di;
	a = 0;
	b = r;	  
	di = 3-(r << 1);             //判断下个点位置的标志
	while(a <= b)
	{
		LCD_SetPointPixel(x0 - b,y0 - a, color);             //3           
		LCD_SetPointPixel(x0 + b,y0 - a, color);             //0           
		LCD_SetPointPixel(x0 - a,y0 + b, color);             //1       
		LCD_SetPointPixel(x0 - b,y0 - a, color);             //7           
		LCD_SetPointPixel(x0 - a,y0 - b, color);             //2             
		LCD_SetPointPixel(x0 + b,y0 + a, color);             //4               
		LCD_SetPointPixel(x0 + a,y0 - b, color);             //5
		LCD_SetPointPixel(x0 + a,y0 + b, color);             //6 
		LCD_SetPointPixel(x0 - b,y0 + a, color);             
		a++;
		//使用Bresenham算法画圆     
		if(di < 0)
			di += 4 * a + 6;	  
		else
		{
			di += 10 + 4 * (a - b);   
			b--;
		} 
		LCD_SetPointPixel(x0 + a, y0 + b, color);
	}
} 

const unsigned char asc2_1206[1][12]={
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*" ",0*/
}; 
const unsigned char asc2_1608[1][16]={
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},/*" ",0*/
};      
//在指定位置显示一个字符
//x:0~234
//y:0~308
//num:要显示的字符:" "--->"~"
//size:字体大小 12/16
//mode:叠加方式(1)还是非叠加方式(0)
//在指定位置显示一个字符
//x:0~234
//y:0~308
//num:要显示的字符:" "--->"~"
//size:字体大小 12/16
//mode:叠加方式(1)还是非叠加方式(0)
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode)
{  
#ifdef LCD_USING_HORIZONTAL
#define MAX_CHAR_POSX 312
#define MAX_CHAR_POSY 232 
#else     
#define MAX_CHAR_POSX 232
#define MAX_CHAR_POSY 312
#endif 
    uint8_t temp;
    uint8_t pos,t;
	uint16_t x0 = x;
	uint16_t colortemp = POINT_COLOR;      
    if( x > MAX_CHAR_POSX || y > MAX_CHAR_POSY )
	{
		return;	    
	}
	//设置窗口		   
	num = num - ' '; //得到偏移后的值
	if(!mode) //非叠加方式
	{
		for(pos=0;pos<size;pos++)
		{
			if(size == 12)
			{
				temp = asc2_1206[num][pos];//调用1206字体
			}
			else 
			{
				temp = asc2_1608[num][pos];		 //调用1608字体
			}
			for(t = 0; t < size / 2; t++ )
			{                 
				if( temp&0x01 )
				{
					POINT_COLOR = colortemp;
				}
				else 
				{
					POINT_COLOR = BACK_COLOR;
				}
				LCD_SetPointPixel(x, y, POINT_COLOR);	
				temp >>= 1; 
				x++;
			}
			x = x0;
			y++;
		}	
	}
	else//叠加方式
	{
		for( pos = 0; pos < size; pos++)
		{
			if(size == 12)
			{
				temp = asc2_1206[num][pos];//调用1206字体
			}
			else 
			{
				temp = asc2_1608[num][pos];		 //调用1608字体
			}
			for(t = 0; t < size/2; t++)
		    {                 
		        if(temp&0x01)
				{
					LCD_SetPointPixel(x + t, y + pos, POINT_COLOR);//画一个点     
					temp >>= 1; 
				}
		    }
		}
	}
	POINT_COLOR = colortemp;	    	   	 	  
}   
//m^n函数
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result = 1;	 
	while(n--)
	{
		result *= m; 
	}		
	return result;
}			 
//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//color:颜色
//num:数值(0~4294967295);	 
void LCD_ShowNum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t size)
{         	
	uint8_t t,temp;
	uint8_t enshow = 0;						   
	for(t = 0; t < len; t++)
	{
		temp = ( num / mypow(10,len - t - 1) )%10;
		if( enshow == 0 && t < ( len - 1 ) )
		{
			if( temp == 0 )
			{
				LCD_ShowChar(x + (size / 2) * t, y, ' ', size, 0 );
				continue;
			}
			else 
			{
				enshow=1; 
			}
		 	 
		}
	 	LCD_ShowChar( x + ( size / 2 ) *t , y, temp + '0', size, 0); 
	}
} 
//显示2个数字
//x,y:起点坐标
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~99);	 
void LCD_Show2Num(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint8_t size, uint8_t mode)
{         	
	uint8_t t,temp;						   
	for(t = 0; t < len; t++)
	{
		temp = ( num / mypow(10,len - t - 1) ) % 10;
	 	LCD_ShowChar( x + ( size / 2) * t, y, temp + '0' , size, mode); 
	}
} 
//显示字符串
//x,y:起点坐标  
//*p:字符串起始地址
//用16字体
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p)
{         
    while( *p != '\0' )
    {       
        if(x>MAX_CHAR_POSX)
		{
			x = 0;
			y += 16;
		}
        if(y>MAX_CHAR_POSY)
		{
			y = x = 0;
			LCD_Clear(WHITE);
		}
        LCD_ShowChar(x, y, *p, 16, 0);
        x += 8;
        p++;
    }  
}

void LCD_GPIO_MspInit( void )
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_AFIO_CLK_ENABLE();
    /* GPIO Ports Clock Enable */
	RCC_GPIO_LCD_CTRL_CLK_ENABLE();   
	
	/**
  * @brief Enable the Serial wire JTAG configuration
  * @note  NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  * @retval None
  */
	__HAL_AFIO_REMAP_SWJ_NOJTAG();/* disable PB3-->JTDO ,PB4 -->JNTRST*/
	
    /* Configure GPIO data pin: GPIO_PIN_All */
    GPIO_InitStruct.Pin   = GPIO_PIN_LCD_CTRL;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIO_PORT_LCD_CTRL, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIO_PORT_LCD_CTRL, GPIO_PIN_LCD_CTRL, GPIO_PIN_SET);
	
	/* GPIO Ports Clock Enable */
    RCC_GPIO_LCD_DATA_CLK_ENABLE();	            
    /* Configure GPIO data pin: GPIO_PIN_All */
    GPIO_InitStruct.Pin   = GPIO_PIN_LCD_DATA;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIO_PORT_LCD_DATA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIO_PORT_LCD_DATA, GPIO_PIN_LCD_DATA, GPIO_PIN_SET);
	
	GPIO_InitStruct.Pin   = GPIO_PIN_LCD_BKLT;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    //GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIO_PORT_LCD_BKLT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIO_PORT_LCD_BKLT, GPIO_PIN_LCD_BKLT, GPIO_PIN_RESET);
	
}
#if 1
/*  set the pixel of x,x */
void lcd_set_pixel(const char *pixel, int x, int y)
{
	//st7735_WritePixel((uint16_t)x,(uint16_t)y,*(rt_uint16_t*)pixel); 
	LCD_SetPointPixel(x, y, *(uint16_t*)pixel);
}

/* get the pixel of x,y */
void lcd_get_pixel(char *pixel, int x, int y)
{
	//lcd_set_cursor(x,y,x+1,y+1);	
	//lcd_read_ram_prepare();	
    //*(rt_uint16_t*)pixel = read_data16(); //BGR2RGB( lcd_read_gram(x,y) ); 
	*(rt_uint16_t*)pixel = LCD_GetPointPixel(x, y);//read_data16();	
}

/* draw h line*/
void lcd_draw_hline( const char *pixel, int x1, int y1, int x2)
{	
	//st7735_DrawHLine(*(rt_uint16_t*)pixel,(uint16_t)x1,(uint16_t)y1,(uint16_t)(x2 -x1));
	LCD_DrawLine(x1, y1, x2-x1, y1, *(uint16_t*)pixel);
}

/* draw v line */
void lcd_draw_vline(const char *pixel, int x1, int y1, int y2)
{
	//st7735_DrawVLine(*(rt_uint16_t*)pixel,(uint16_t)x1,(uint16_t)y1,(uint16_t)(y2 -y1));
	LCD_DrawLine(x1, y1, x1, y2-y1, *(uint16_t*)pixel);
}
/* draw the*/
/*void lcd_blit_line(const char *pixel, int x, int y, rt_size_t size)
{
	//LCD_SetCursor(x, x);
	uint16_t *prt = (uint16_t*)pixel;
	while(size--)
	{
		LCD_SetPointPixel(x, y, *prt++);	
	}
}*/
/* lcd_blit_line */
struct rt_device _lcd_device;
struct rt_device_graphic_ops lcd_ops =
{
	lcd_set_pixel,
	lcd_get_pixel,
	lcd_draw_hline,
	lcd_draw_vline,
	//lcd_blit_line
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
			info->width = 240;
			info->height = 320;
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
#endif
int rt_hw_lcd_init(void)
{  
    LCD_GPIO_MspInit();										 	 
	LCD_Device_Init();
	
	#if 1
	/* register lcd device */
	_lcd_device.type  = RT_Device_Class_Graphic;
	_lcd_device.init  = lcd_init;
	_lcd_device.open  = lcd_open;
	_lcd_device.close = lcd_close;
	_lcd_device.control = lcd_control;
	_lcd_device.read  = RT_NULL;
	_lcd_device.write = RT_NULL;

	_lcd_device.user_data = &lcd_ops;   

    /* register graphic device driver */
	rt_device_register(&_lcd_device, "lcd",
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);	
	#endif
	return 1;
}
INIT_BOARD_EXPORT(rt_hw_lcd_init);
