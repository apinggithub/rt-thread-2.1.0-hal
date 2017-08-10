#include "GUI.h"
#include "drv_iic_touch_ft5216.h"
extern struct ft5216_data ft5216_statue;

int  GUI_TOUCH_GetState(GUI_PID_STATE * pState)
{
	//uint16_t x,y;
	//ft5216_CTP_GetXY(CTP_I2C_ADDRESS,&x,&y);
	//HAL_GPIO_EXTI_Callback(GPIO_PIN_4);
	pState->x = (int)ft5216_statue.X_phys;
	pState->y = (int)ft5216_statue.Y_phys;
	if(FT5216_PUT_DOWN == ft5216_statue.active)
		pState->Pressed = 1;
	else
		pState->Pressed = 0;
	//pState->Layer = 0;
}	

void GUI_TOUCH_GetUnstable  (int * px, int * py)  /* for diagnostics only */
{
	//uint16_t x,y;
	//ft5216_CTP_GetXY(CTP_I2C_ADDRESS,&x,&y);
	//HAL_GPIO_EXTI_Callback(GPIO_PIN_4);
	*px = (int)ft5216_statue.X_phys;
	*py = (int)ft5216_statue.Y_phys;
}

void GUI_TOUCH_X_ActivateX(void) 
{
 
 // XPT2046_WriteCMD(0x90);
 
}
 
 
void GUI_TOUCH_X_ActivateY(void)
{
 
  //XPT2046_WriteCMD(0xd0);
 
}
 
int  GUI_TOUCH_X_MeasureX(void) 
{
 
int32_t xvalue;
HAL_GPIO_EXTI_Callback(GPIO_PIN_4);
xvalue=(int32_t)ft5216_statue.X_phys;
return xvalue;
 
 
}
 
int  GUI_TOUCH_X_MeasureY(void) 
{
 
int32_t yvalue;
HAL_GPIO_EXTI_Callback(GPIO_PIN_4);
yvalue = (int32_t)ft5216_statue.Y_phys;
return yvalue;
 
}
/*void GUI_TOUCH_SetLayer     (int Layer);
void GUI_TOUCH_StoreState   (int x, int y);
void GUI_TOUCH_StoreStateEx (const GUI_PID_STATE * pState);
void GUI_TOUCH_StoreUnstable(int x, int y);*/


