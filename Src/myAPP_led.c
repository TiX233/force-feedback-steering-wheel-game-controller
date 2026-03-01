#include "myApp_led.h"
#include "ltx.h"
#include "ltx_app.h"
#include "ltx_param.h"
#include "ltx_log.h"
#include "ltx_cmd.h"
#include "ltx_script.h"
#include "ws2812.h"
#include "myAPP_device_init.h"

struct ltx_Script_stu script_led_anime_blink;

void script_cb_led_anime_blink(struct ltx_Script_stu *script);

int myApp_led_init(struct ltx_App_stu *app){

    ltx_Script_init(&script_led_anime_blink, script_cb_led_anime_blink);

    return 0;
}

int myApp_led_pause(struct ltx_App_stu *app){

    ltx_Script_pause(&script_led_anime_blink);
    
    return 0;
}

int myApp_led_resume(struct ltx_App_stu *app){

    ltx_Script_resume(&script_led_anime_blink, 0);

    return 0;
}

int myApp_led_destroy(struct ltx_App_stu *app){

    ltx_Script_pause(&script_led_anime_blink);

    // free...

    return 0;
}


struct ltx_App_stu app_led = {
    .is_initialized = 0,
    .status = ltx_App_status_pause,
    .name = "led",

    .init = myApp_led_init,
    .pause = myApp_led_pause,
    .resume = myApp_led_resume,
    .destroy = myApp_led_destroy,

    .task_list = NULL,
    
    .next = NULL,
};

void script_cb_led_anime_blink(struct ltx_Script_stu *script){
    static uint8_t flag_led_status = 0;

    if(flag_led_status){
        ws2812_set_1_color(&my_led, 0, 0, 0, 20);
    }else {
        ws2812_set_1_color(&my_led, 0, 0, 0, 0);
    }
    flag_led_status = !flag_led_status;

    ws2812_refresh_dma(&my_led);
    ltx_Script_next_step_delay(script, 0, 500);
}
