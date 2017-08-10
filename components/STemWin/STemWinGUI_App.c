#include <rthw.h>
#include "GUI.h"
//#include "GUIDEMO.h"
//#include "MainTask.h"
//#include "drv_lcd_st7735.h"

#define RT_GUI_NUMBYTES 1024*5
static struct rt_thread emwin_thread;
static char emWin_thread_stack[RT_GUI_NUMBYTES];
char buff[40]="aswffdgfhgjhjhngfgjgfjh";

#if 1
void emWin_thread_entry(void *parameter)
{
	//WM_SetCreateFlags(WM_CF_MEMDEV);
	GUI_Init();	
	GUI_SetBkColor(GUI_BLUE);
	GUI_Clear();
		
	rt_kprintf("\nGUIINIT\n");

	GUI_SetColor(GUI_WHITE); 
	GUI_SetFont(GUI_FONT_8X16_ASCII);	
	
	//GUI_DispStringAt(buff,0,0);
	while(1)
	{
		GUI_PID_STATE zuobiao;
		//GUIDEMO_Main();
		//rt_thread_delay(1000);
		//MainTask();
		GUI_TOUCH_GetState(&zuobiao);
		if ( 1 == zuobiao.Pressed)
			GUI_DispStringAt("Hello World!", zuobiao.x,zuobiao.y);
		else
		{
			GUI_SetBkColor(GUI_BLUE);
			GUI_Clear();
		}
		rt_thread_delay(RT_TICK_PER_SECOND/10);
	}
}

int emwin_system_init(void)
{
	rt_err_t result;
	//rt_thread_t emwin_thread1;
	rt_device_t device = rt_device_find("lcd");
	if(device == RT_NULL)
	{
		rt_kprintf("no graphic device in the system. \n");
		
	}
	else
	{	
		result = rt_thread_init(&emwin_thread,
		"emwin",
		emWin_thread_entry, RT_NULL,
		&emWin_thread_stack[0], sizeof(emWin_thread_stack),
		21, 100);
				
		/*emwin_thread1 = rt_thread_create("emwin",
						emWin_thread_entry,
						 RT_NULL,
						 RT_GUI_NUMBYTES,
						 21,
						 30);*/

		if (result == RT_EOK)
			rt_thread_startup(&emwin_thread);
	}
    return 0;
}
INIT_COMPONENT_EXPORT(emwin_system_init);
#endif

