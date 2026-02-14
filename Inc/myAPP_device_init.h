#ifndef __MYAPP_DEVICE_INIT_H__
#define __MYAPP_DEVICE_INIT_H__

#include "ltx_app.h"

extern struct ltx_App_stu app_device_init;
extern struct st7305_stu myLCD;
extern struct ltx_Topic_stu topic_spi_tx_over;
extern struct ltx_Topic_stu topic_lcd_clear_over;
extern struct ltx_Script_stu script_lcd_clear;

#endif // __MYAPP_DEVICE_INIT_H__
