#include "myAPP_device_init.h"
#include "ltx.h"
#include "ltx_app.h"
#include "ltx_param.h"
#include "ltx_log.h"
#include "ltx_script.h"
#include "ltx_event_group.h"
#include "myAPP_system.h"

// 所需初始化外部硬件完成事件
#define EVENT_INIT_LED_OVER                 0x0001
#define EVENT_INIT_MAG_ENCODER_OVER         0x0002

// 函数声明
// led 初始化脚本回调
void script_cb_led_init(struct ltx_Script_stu *script);
// 外部硬件初始化完成事件组回调
void eventg_cb_device_init_over(struct ltx_Event_group_stu *event);

// 组件全局变量
// dma 发送完成话题
struct ltx_Topic_stu topic_spi_tx_over = _LTX_TOPIC_DEAFULT_CONFIG(topic_spi_tx_over);

// 所有外部硬件初始化完成事件组
struct ltx_Event_group_stu eventg_device_init_over;

// led 初始化脚本对象结构体
struct ltx_Script_stu script_led_init;


// app 相关
int myAPP_device_init_init(struct ltx_App_stu *app){
    // 创建显示屏初始脚本
    ltx_Script_init(&script_led_init, script_cb_led_init);

    // 创建所有外部硬件初始化完成事件组
    ltx_Event_group_init(&eventg_device_init_over, eventg_cb_device_init_over, EVENT_INIT_LED_OVER \
                                                                                \
                                                                                , 10000); // 10s 超时时间

    return 0;
}

int myAPP_device_init_pause(struct ltx_App_stu *app){

    ltx_Script_pause(&script_led_init);

    return 0;
}

int myAPP_device_init_resume(struct ltx_App_stu *app){

    ltx_Script_resume(&script_led_init, 0);

    return 0;
}

int myAPP_device_init_destroy(struct ltx_App_stu *app){

    ltx_Script_pause(&script_led_init);

    ltx_Event_group_cancel(&eventg_device_init_over);

    return 0;
}

struct ltx_App_stu app_device_init = {
    .is_initialized = 0,
    .status = ltx_App_status_pause,
    .name = "device_init",

    .init = myAPP_device_init_init,
    .pause = myAPP_device_init_pause,
    .resume = myAPP_device_init_resume,
    .destroy = myAPP_device_init_destroy,

    .task_list = NULL,
    
    .next = NULL,
};


// led 初始化脚本回调
void script_cb_led_init(struct ltx_Script_stu *script){
    // if(ltx_Script_get_triger_type(script) == SC_TRIGER_RESET){ // 外部要求此脚本重置，可在这里做释放资源等操作
    //     return ;
    // }
    switch(script->step_now){
        case 0:
        
            break;

        case 1:

            break;

        default:

            break;
    }

}


// 所有外部硬件初始化完成或者初始化超时回调
void eventg_cb_device_init_over(struct ltx_Event_group_stu *eventg){
    if(ltx_Event_group_is_timeout(eventg)){
        LTX_LOG_ERRO("Device init Timeout!\n");
        LTX_LOG_ERRO("events: 0x%08x\n", eventg->events);

        return ;
    }

    // 所有外部硬件均初始化完成
    LTX_LOG_INFO("All devices init over.\n");
    // 关闭 device_init app
    ltx_App_destroy(&app_device_init);

    // 启动业务 app
    // ltx_App_init(&app_xxx);
    // ltx_App_resume(&app_xxx);
}


// spi dma 发送完成回调
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    // ltx_Lock_off(&lock_spi);
    ltx_Topic_publish(&topic_spi_tx_over);
}
