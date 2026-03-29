#ifndef __MYAPP_LED_H__
#define __MYAPP_LED_H__

#include "ltx_app.h"

extern struct ltx_App_stu app_led;

void led_set_blink_color(uint8_t r, uint8_t g, uint8_t b);

#endif // __MYAPP_LED_H__
