#include "myAPP_system.h"
#include "ltx.h"
#include "ltx_app.h"
#include "ltx_param.h"
#include "ltx_log.h"
#include "ltx_cmd.h"

void task_func_heart_beat(struct ltx_Task_stu *task);
void task_func_cmd(struct ltx_Task_stu *task);
void subscriber_cb_sys_error(void *param);


// 心拍周期任务对象
struct ltx_Task_stu task_heart_beat;

// 命令处理周期任务对象
struct ltx_Task_stu task_cmd;


// 系统错误码
uint32_t SYS_ERROR_CODE = 0;
const char *SYS_ERROR_MSG = "Okay";

// 系统错误话题
struct ltx_Topic_stu topic_sys_error = _LTX_TOPIC_DEAFULT_CONFIG(topic_sys_error);

// 系统错误话题订阅者
struct ltx_Topic_subscriber_stu subscriber_sys_error = _LTX_SUBSCRIBER_DEAFULT_CONFIG(subscriber_cb_sys_error);

// 错误码周期打印任务
struct ltx_Task_stu task_error_code = {.is_initialized = 0};

int myApp_system_init(struct ltx_App_stu *app){
    // 创建心拍周期任务
    ltx_Task_init(&task_heart_beat, task_func_heart_beat, 1000, 0);
    ltx_Task_add_to_app(&task_heart_beat, app, "heart_beat");

    // 创建命令处理周期任务
    ltx_Task_init(&task_cmd, task_func_cmd, 200, 0);
    ltx_Task_add_to_app(&task_cmd, app, "cmd");

    // 订阅话题
    ltx_Topic_subscribe(&topic_sys_error, &subscriber_sys_error);

    return 0;
}

int myApp_system_pause(struct ltx_App_stu *app){

    return 0;
}

int myApp_system_resume(struct ltx_App_stu *app){

    return 0;
}

int myApp_system_destroy(struct ltx_App_stu *app){

    // free...

    return 0;
}


struct ltx_App_stu app_system = {
    .is_initialized = 0,
    .status = ltx_App_status_pause,
    .name = "system",

    .init = myApp_system_init,
    .pause = myApp_system_pause,
    .resume = myApp_system_resume,
    .destroy = myApp_system_destroy,

    .task_list = NULL,
    
    .next = NULL,
};


// 心拍任务
uint32_t heart_beat_count = 0;
void task_func_heart_beat(struct ltx_Task_stu *task){

    heart_beat_count ++;
    // LTX_LOG_DEBG("Heartbeat: %d\n", heart_beat_count);

}


uint8_t cmd_buffer[CMD_BUF_SIZE];
// 处理命令任务
void task_func_cmd(struct ltx_Task_stu *task){

    // 读取命令
    if(SEGGER_RTT_HasData(0)){
        int len = SEGGER_RTT_Read(0, cmd_buffer, CMD_BUF_SIZE - 1);
        if (len > 0) {
            cmd_buffer[len] = '\0'; // 添加字符串终止符
            for(uint8_t i = len - 1; i > 0; i --){ // 去除尾追回车
                if(cmd_buffer[i] == '\n' || cmd_buffer[i] == '\r'){
                    cmd_buffer[i] = 0;
                }else {
                    break;
                }
            }

            // LOG_FMT(PRINT_DEBUG"cmd len: %d\n", len);
            ltx_Cmd_process((char *)cmd_buffer); // 处理命令
        }
    }
}

// 系统错误码每秒打印周期任务
void task_func_error_code(struct ltx_Task_stu *task){

    LTX_LOG_ERRO("Error: 0x%08x, %s\n", SYS_ERROR_CODE, SYS_ERROR_MSG);
}

// 系统错误订阅回调
void subscriber_cb_sys_error(void *param){
    // 系统发生错误
    // 创建一个不断打印错误码的任务
    if(!task_error_code.is_initialized){
        ltx_Task_init(&task_error_code, task_func_error_code, 1000, 1000);
        ltx_Task_add_to_app(&task_error_code, &app_system, "error_code");
        // 运行
        ltx_Task_resume(&task_error_code);
    }

    // 关闭其他 app
    // todo

    // 蓝屏显示错误码
    #if 0
    extern struct gc9a01_stu myLCD;
    if(myLCD.is_initialized){
		// gc9a01_clear(&myLCD, RGB565_BLUE);
        HAL_SPI_DMAStop(&spi1_handler);
		uint8_t _color[2];
		_color[1] = RGB565_BLUE & 0xFF;
		_color[0] = RGB565_BLUE >> 8;
		gc9a01_set_window(&myLCD, 0, 0, 239, 239);
		myLCD.write_dc(GC9A01_PIN_LEVEL_DC_DATA);
		for(uint32_t i = 0; i < 240*240; i ++)
			myLCD.transmit_data(_color, 2);
		
        // 错误码 todo
    }
    #endif
}

// 发布系统错误 api
void _SYS_ERROR(uint32_t code, const char *msg){
    SYS_ERROR_CODE = code;
    SYS_ERROR_MSG = msg;

    ltx_Topic_publish(&topic_sys_error);
}
