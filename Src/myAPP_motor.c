#include "myAPP_motor.h"
#include "ltx.h"
#include "ltx_app.h"
#include "ltx_param.h"
#include "ltx_log.h"
#include "ltx_cmd.h"

// 方向盘电机对象
struct ltx_bldc_stu motor_wheel = {
    .id = 0,
    .current_u = 0.0f,
    .current_v = 0.0f,
    .current_w = 0.0f,
};


int myApp_motor_init(struct ltx_App_stu *app){

    return 0;
}

int myApp_motor_pause(struct ltx_App_stu *app){

    return 0;
}

int myApp_motor_resume(struct ltx_App_stu *app){

    return 0;
}

int myApp_motor_destroy(struct ltx_App_stu *app){

    // free...

    return 0;
}


struct ltx_App_stu app_motor = {
    .is_initialized = 0,
    .status = ltx_App_status_pause,
    .name = "motor",

    .init = myApp_motor_init,
    .pause = myApp_motor_pause,
    .resume = myApp_motor_resume,
    .destroy = myApp_motor_destroy,

    .task_list = NULL,
    
    .next = NULL,
};
