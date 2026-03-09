#include "ltx_cmd.h"
#include <stdio.h>
#include "ltx_param.h"
#include "ltx.h"
#include "ltx_app.h"
#include "ltx_log.h"
#include "ltx_script.h"
#include "ltx_bldc.h"
#include "ltx_foc1.h"
#include "myAPP_system.h"
#include "myAPP_motor.h"
#include "myAPP_device_init.h"
#include "mt6701.h"
#include "ws2812.h"

typedef struct {
    const char *cmd_name;
    const char *brief;
    void (*cmd_cb)(uint8_t argc, char *argv[]);
} ltx_Cmd_item;

void cmd_cb_echo(uint8_t argc, char *argv[]);
void cmd_cb_help(uint8_t argc, char *argv[]);
void cmd_cb_hello(uint8_t argc, char *argv[]);
void cmd_cb_print(uint8_t argc, char *argv[]);
void cmd_cb_alarm(uint8_t argc, char *argv[]);
void cmd_cb_reboot(uint8_t argc, char *argv[]);
void cmd_cb_param(uint8_t argc, char *argv[]);
void cmd_cb_ltx_app(uint8_t argc, char *argv[]);

void cmd_cb_pwm(uint8_t argc, char *argv[]);
void cmd_cb_svpwm(uint8_t argc, char *argv[]);
void cmd_cb_mag(uint8_t argc, char *argv[]);
void cmd_cb_led(uint8_t argc, char *argv[]);
void cmd_cb_rotate(uint8_t argc, char *argv[]);
void cmd_cb_set_mag_rad_offset(uint8_t argc, char *argv[]);
void cmd_cb_zero_align(uint8_t argc, char *argv[]);

ltx_Cmd_item cmd_list[] = {
    {
        .cmd_name = "echo",
        .brief = "return 2nd param to test uart",
        .cmd_cb = cmd_cb_echo,
    },
    {
        .cmd_name = "hello",
        .brief = "hello",
        .cmd_cb = cmd_cb_hello,
    },
    {
        .cmd_name = "help",
        .brief = "print brief of commands",
        .cmd_cb = cmd_cb_help,
    },

    {
        .cmd_name = "print",
        .brief = "track a data item and print it whenever it is updated",
        .cmd_cb = cmd_cb_print,
    },

    {
        .cmd_name = "param",
        .brief = "read or write some param",
        .cmd_cb = cmd_cb_param,
    },

    {
        .cmd_name = "alarm",
        .brief = "set an alarm for test",
        .cmd_cb = cmd_cb_alarm,
    },

    {
        .cmd_name = "reboot",
        .brief = "reboot",
        .cmd_cb = cmd_cb_reboot,
    },

    {
        .cmd_name = "ltx_app",
        .brief = "manage ltx apps",
        .cmd_cb = cmd_cb_ltx_app,
    },


    {
        .cmd_name = "pwm",
        .brief = "set motor pwm duty",
        .cmd_cb = cmd_cb_pwm,
    },

    {
        .cmd_name = "svpwm",
        .brief = "test svpwm output",
        .cmd_cb = cmd_cb_svpwm,
    },

    {
        .cmd_name = "mag",
        .brief = "read mag encoder",
        .cmd_cb = cmd_cb_mag,
    },

    {
        .cmd_name = "led",
        .brief = "set led rgb",
        .cmd_cb = cmd_cb_led,
    },

    {
        .cmd_name = "rotate",
        .brief = "test svpwm algorithm",
        .cmd_cb = cmd_cb_rotate,
    },

    {
        .cmd_name = "set_mag_rad_offset",
        .brief = "set_mag_rad_offset",
        .cmd_cb = cmd_cb_set_mag_rad_offset,
    },

    {
        .cmd_name = "zero_align",
        .brief = "align mag and elec rad",
        .cmd_cb = cmd_cb_zero_align,
    },


    // end of list:
    {
        .cmd_name = " ",
        .cmd_cb = NULL,
    },
};

// 返回 0 代表匹配成功
int my_str_cmp(const char *str1, char *str2){
    uint32_t i;
    for(i = 0; i < CMD_BUF_SIZE - 1; i++){
        if(str1[i] == '\0' || str1[i] == ' '){
            if(str2[i] == '\0' || str2[i] == ' ' || str2[i] == '\n'){
                str2[i] = '\0';
                return 0;
            }else {
                return -1;
            }
        }
        if(str1[i] != str2[i]){
            return -1;
        }
    }

    return 0;
}


void cmd_cb_echo(uint8_t argc, char *argv[]){
    if(argc > 1){
        if(my_str_cmp("-h", argv[1]) == 0){
            goto Useage_echo;
        }
        LTX_LOG_INFO("%s\n", argv[1]);
    }else {
        LTX_LOG_WARN("Need param to echo!\n");
    }
    return ;
Useage_echo:
    LTX_LOG_INFO("Useage: This function will return the first param to test uart\n");
}

void cmd_cb_hello(uint8_t argc, char *argv[]){
    LTX_LOG_STR(LOG_INFO"Ciallo World~ (∠・ω< )⌒☆\n");
}

void cmd_cb_help(uint8_t argc, char *argv[]){
    uint8_t i;
    
    if(argc > 1){
        for(i = 0; cmd_list[i].cmd_name[0] != ' '; i ++){
            if(my_str_cmp(cmd_list[i].cmd_name, argv[1]) == 0){
                LTX_LOG_INFO("%s   -   %s\n", cmd_list[i].cmd_name, cmd_list[i].brief);
                return;
            }
        }
    }else {
        for(i = 0; cmd_list[i].cmd_name[0] != ' '; i ++){
            LTX_LOG_INFO("%s:\n\t%s\n", cmd_list[i].cmd_name, cmd_list[i].brief);
        }

        return ;
    }

    LTX_LOG_WARN("Unknown cmd: %s, Type /help to list all commands\n", argv[1]);
}

// V2 配个闹钟真麻烦，甚至不如直接创建个脚本来得方便，先这样吧
void alarm_cb_cmd_test_alarm(void *param){
    LTX_LOG_INFO("Alarm ring: %d\n", ltx_Sys_get_tick());
}
struct ltx_Topic_subscriber_stu alarm_cmd_test_subscriber;
struct ltx_Alarm_stu alarm_cmd_test = {
    .diff_tick = 0,
    .topic = {
        .flag_is_pending = 0,
        .subscriber_head = {
            .prev = NULL,
            .next = &alarm_cmd_test_subscriber
        },
        .subscriber_tail = &alarm_cmd_test_subscriber,
        .next = NULL
    },
    .prev = NULL,
    .next = NULL
};
struct ltx_Topic_subscriber_stu alarm_cmd_test_subscriber = {
    .callback_func = alarm_cb_cmd_test_alarm,
    .prev = &(alarm_cmd_test.topic.subscriber_head),
    .next = NULL,
};

void cmd_cb_alarm(uint8_t argc, char *argv[]){
    if(argc != 2){
        goto Useage_alarm;
    }
    uint32_t ticks_alarm;
    sscanf(argv[1], "%d", &ticks_alarm);

    // if(ticks_alarm < 1){
    //     alarm_cb_cmd_test_alarm(NULL);
    //     return ;
    // }

    ltx_Alarm_add(&alarm_cmd_test, ticks_alarm);

    LTX_LOG_INFO("Tick now: %d\n", ltx_Sys_get_tick()); // 这条语句可能会耗时，所以下一条语句的 get tick 可能不是这一次的，先不管
    LTX_LOG_INFO("Alarm will ring after %d ticks(%d)\n", ticks_alarm, ticks_alarm + ltx_Sys_get_tick());

    return ;
Useage_alarm:
    LTX_LOG_INFO("Useage: %s <ticks>\n", argv[0]);
}

void cmd_cb_reboot(uint8_t argc, char *argv[]){
    NVIC_SystemReset();
}

void cmd_cb_ltx_app(uint8_t argc, char *argv[]){
    enum ltx_app_option_e{
        RA_list_app = 0,
        RA_list_task,
        RA_kill_app,
        RA_pause_app,
        RA_resume_app,
        RA_kill_task,
        RA_pause_task,
        RA_resume_task,
    };

    const char *options[] = {
        [RA_list_app] = "-list_app",
        [RA_list_task] = "-list_task",

        [RA_kill_app] = "-kill_app",
        [RA_pause_app] = "-pause_app",
        [RA_resume_app] = "-resume_app",

        [RA_kill_task] = "-kill_task",
        [RA_pause_task] = "-pause_task",
        [RA_resume_task] = "-resume_task",
        " ",
    };

    if(argc < 2){
        goto Useage_ltx_app;
    }

    uint8_t i = 0;
    for(; options[i][0] != ' '; i ++){
        if(my_str_cmp(options[i], argv[1]) == 0){
            break;
        }
    }

    struct ltx_App_stu **pApp = &(ltx_sys_app_list.next);
    struct ltx_Task_stu **pTask;
    switch(i){
        case RA_list_app:
            LTX_LOG_STR(LOG_INFO"All apps:\n");
            while((*pApp) != NULL){
                LTX_LOG_FMT("\t%s: %s\n", (*pApp)->name, (*pApp)->status?"running":"pause");
                pApp = &((*pApp)->next);
            }
            break;
            
        case RA_list_task:
            if(argc < 3){
                LTX_LOG_STR(LOG_WARNNING"Please provide app name!\n");
                goto Useage_ltx_app;
            }
            while((*pApp) != NULL){
                if(my_str_cmp((*pApp)->name, argv[2]) == 0){
                    
                    pTask = &((*pApp)->task_list);
                    LTX_LOG_STR(LOG_INFO"All tasks:\n");
                    while((*pTask) != NULL){
                        LTX_LOG_FMT("\t%s: %s\n", (*pTask)->name, (*pTask)->status?"running":"pause");
                        pTask = &((*pTask)->next);
                    }
                    return ;
                }
                pApp = &((*pApp)->next);
            }
            LTX_LOG_WARN("App(%s) not found!\n", argv[2]);
            
            break;
            
        case RA_kill_app:
        case RA_pause_app:
        case RA_resume_app:
            if(argc < 3){
                goto Useage_ltx_app;
            }
            while((*pApp) != NULL){
                if(my_str_cmp((*pApp)->name, argv[2]) == 0){
                    switch(i){
                        case RA_kill_app:
                            ltx_App_destroy((*pApp));

                            break;

                        case RA_pause_app:
                            ltx_App_pause((*pApp));

                            break;
                            
                        case RA_resume_app:
                            ltx_App_resume((*pApp));

                            break;
                    }
                    LTX_LOG_INFO("App(%s) set okay\n", argv[2]);
                    return ;
                }
                pApp = &((*pApp)->next);
            }
            LTX_LOG_WARN("App(%s) not found!\n", argv[2]);

            break;
            
        case RA_kill_task:
        case RA_pause_task:
        case RA_resume_task:
            if(argc < 4){
                goto Useage_ltx_app;
            }
            while((*pApp) != NULL){
                if(my_str_cmp((*pApp)->name, argv[2]) == 0){
                    
                    pTask = &((*pApp)->task_list);
                    while((*pTask) != NULL){
                        if(my_str_cmp((*pTask)->name, argv[3]) == 0){

                            switch(i){
                                case RA_kill_task:
                                    ltx_Task_destroy((*pTask), *pApp);

                                    break;

                                case RA_pause_task:
                                    ltx_Task_pause((*pTask));
                                    
                                    break;

                                case RA_resume_task:
                                    ltx_Task_resume((*pTask));
                                    
                                    break;
                            }
                            LTX_LOG_INFO("Task(%s) set okay\n", argv[3]);
                            return ;
                        }
                        pTask = &((*pTask)->next);
                    }
                    LTX_LOG_WARN("Task(%s) not found!\n", argv[3]);
                    return ;
                }
                pApp = &((*pApp)->next);
            }
            LTX_LOG_WARN("App(%s) not found!\n", argv[2]);

            break;

        default:
            LTX_LOG_WARN("Unknown option:%s\n", argv[1]);
            goto ltx_app_print_all_option;
            break;
    }

    return ;

Useage_ltx_app:
    LTX_LOG_INFO("Useage: %s <option> [app_name] [task_name]\n", argv[0]);
ltx_app_print_all_option:
    LTX_LOG_STR(LOG_INFO"All options:\n");
    for(uint8_t j = 0; options[j][0] != ' '; j ++){
        LTX_LOG_FMT("\t%s\n", options[j]);
    }
}


// 数据更新追踪打印相关内容
// 心跳计数数据更新打印回调
void print_cb_heart_beat(void *param){
    extern uint32_t heart_beat_count;
    LTX_LOG_FMT("Heartbeat: %d\n", heart_beat_count);
}
// 磁编码角度更新打印回调
void print_cb_mag_angle(void *param){
    LTX_LOG_FMT("ma:%f\n", mag_angle);
}
// 磁编码弧度更新打印回调
void print_cb_mag_rad(void *param){
    LTX_LOG_FMT("mr:%f\n", motor_foc.rotater_rad);
}
// 三相电流原始值更新打印回调
// uint32_t _test_cnt = 0;
void print_cb_adc1(void *param){
    uint32_t adc1_print[3];
    adc1_print[0] = adc1_buffer[0];
    adc1_print[1] = adc1_buffer[1];
    adc1_print[2] = adc1_buffer[2];
    // _test_cnt ++;
    // GPIOA->BSRR = (uint32_t)GPIO_PIN_15;
    // LTX_LOG_FMT("a1:%d,%d,%d,%d\n", _test_cnt, adc1_print[0], adc1_print[1], adc1_print[2]);
    LTX_LOG_FMT("a1:%d,%d,%d\n", adc1_print[0], adc1_print[1], adc1_print[2]);
    // GPIOA->BRR = (uint32_t)GPIO_PIN_15;
}
// 电流弧度与机械弧度
void print_cb_am_rad(void *param){
    float print_mag_rad = motor_foc.rotater_rad;
    LTX_LOG_FMT("amr:%f,%f\n", motor_foc.vector_I_rad, print_mag_rad);
}
// 电流模长与弧度
void print_cb_alr(void *param){
    LTX_LOG_FMT("alr:%f,%f\n", motor_foc.vector_I_len, motor_foc.vector_I_rad);
}
// 三相电流
void print_cb_iabc(void *param){
    float print_ia, print_ib, print_ic;
    print_ia = motor_foc.i_A;
    print_ib = motor_foc.i_B;
    print_ic = motor_foc.i_C;
    LTX_LOG_FMT("i:%f,%f,%f\n", print_ia, print_ib, print_ic);
}

// 可追踪打印数据的参数信息，需要提供名字、话题指针以及打印回调
#define _P_DATA_INFO(name_str, topic_ptr, callback)     {.item_name = name_str,.topic = topic_ptr,\
                                                        .subscriber = {.callback_func = callback,.prev = NULL,.next = NULL,},}
// 可供打印的数据对象的列表
struct {
    const char *item_name;
    struct ltx_Topic_stu *topic;
    struct ltx_Topic_subscriber_stu subscriber;
} print_data_item_list[] = {
    // 心跳任务的心跳数值
    _P_DATA_INFO("heart_beat", &(task_heart_beat.alarm.topic), print_cb_heart_beat),
    // 磁编码器的角度
    _P_DATA_INFO("mag_angle", &topic_mag_read_over, print_cb_mag_angle),
    // 磁编码器的弧度
    _P_DATA_INFO("mag_rad", &topic_mag_read_over, print_cb_mag_rad),
    // 三相电流原始值
    _P_DATA_INFO("adc1", &topic_adc1_update, print_cb_adc1),
    // 电流弧度与机械弧度
    _P_DATA_INFO("am_rad", &topic_adc1_update, print_cb_am_rad),
    // 电流弧度与机械弧度
    _P_DATA_INFO("alr", &topic_adc1_update, print_cb_alr),
    // 三相电流
    _P_DATA_INFO("iabc", &topic_adc1_update, print_cb_iabc),

    // 列表结尾项
    {.item_name = " ",},
};

// 数据更新打印订阅设置命令。非阻塞，一旦某个数据的更新事件触发便会立即打印
void cmd_cb_print(uint8_t argc, char *argv[]){
    if(argc < 2){
        goto Useage_print;
    }

enum print_option_e{
    PO_START = 0,
    PO_STOP = 1,
    PO_LIST = 2,
};

    const char *print_option_list[] = {
        [PO_START] = "-start", // 启动某一数据的更新打印
        [PO_STOP] = "-stop", // 会直接关闭所有数据更新打印
        [PO_LIST] = "-list", // 列出所有可被追踪打印的数据

        " ",
    };

    uint8_t i;
    for(i = 0; print_option_list[i][0] != ' ' && my_str_cmp(print_option_list[i], argv[1]) != 0; i++);

    switch(i){
        case PO_START:
            if(argc != 3){
                LTX_LOG_STR(LOG_WARNNING"Please enter the data name correctly!\n");

                goto Useage_print;
            }

            // LTX_LOG_STR(LOG_DEBUG"going into PO_START\n");

            for(i = 0; print_data_item_list[i].item_name[0] != ' '; i ++){
                if(my_str_cmp(print_data_item_list[i].item_name, argv[2]) == 0){
                    LTX_LOG_INFO("Start track data \"%s\"...\n", argv[2]);

                    ltx_Topic_subscribe(print_data_item_list[i].topic, &(print_data_item_list[i].subscriber));
                    return ;
                }
            }

            LTX_LOG_WARN("Data name \"%s\" not matched! Use <-list> option to list all data name.\n", argv[2]);

            break;

        case PO_STOP:
            // LTX_LOG_STR(LOG_DEBUG"going into PO_STOP\n");

            for(i = 0; print_data_item_list[i].item_name[0] != ' '; i ++){
                ltx_Topic_unsubscribe(print_data_item_list[i].topic, &(print_data_item_list[i].subscriber));
            }
            LTX_LOG_STR(LOG_INFO"All data tracking has been stopped.\n");

            break;

        case PO_LIST:
            // LTX_LOG_STR(LOG_DEBUG"going into PO_LIST\n");

            LTX_LOG_STR(LOG_INFO"All data item name:\n");
            for(i = 0; print_data_item_list[i].item_name[0] != ' '; i ++){
                LTX_LOG_FMT("\t%s\n", print_data_item_list[i].item_name);
            }

            break;

        default:
            LTX_LOG_WARN("Unknown option: %s\n", argv[1]);

            goto Useage_print;
            break;
    }

    return ;

Useage_print:
    LTX_LOG_INFO("Useage: %s <-start/-stop/-list> [data_name]\n", argv[0]);
}

// 读写某些参数的命令
void cmd_cb_param(uint8_t argc, char *argv[]){
    if(argc < 2){
        goto Useage_param;
    }

    switch(argv[1][1]){
        case 'r':
            if(argc < 3){
                LTX_LOG_STR(LOG_WARNNING"Please enter param name! You can use -l option to list all param.\n");
                return ;
            }

            for(uint8_t i = 0; param_list[i].param_name[0] != ' '; i ++){
                if(my_str_cmp(param_list[i].param_name, argv[2]) == 0){
                    param_list[i].param_read(&param_list[i]);

                    return ;
                }
            }

            LTX_LOG_WARN("Param '%s' was not found, use -l to list all param\n", argv[2]);

            break;

        case 'w':
            if(argc < 4){
                LTX_LOG_STR(LOG_WARNNING"Please enter param name and new value! You can use -l option to list all param.\n");
                return ;
            }

            for(uint8_t i = 0; param_list[i].param_name[0] != ' '; i ++){
                if(my_str_cmp(param_list[i].param_name, argv[2]) == 0){
                    param_list[i].param_write(&param_list[i], argv[3]);

                    return ;
                }
            }

            LTX_LOG_WARN("Param '%s' was not found, use -l to list all param\n", argv[2]);

            break;

        case 'l':
            LTX_LOG_STR(LOG_INFO"All rw-able parameter:\n");
            for(uint8_t i = 0; param_list[i].param_name[0] != ' '; i ++){
                LTX_LOG_INFO("\t%s\n", param_list[i].param_name);
            }

            break;

        default:
            LTX_LOG_WARN("Unknown option: %s\n", argv[1]);
            goto Useage_param;
            break;
    }

    return ;
    
Useage_param:
    LTX_LOG_INFO("Useage: %s <-r/-s/-l> [<param_name> [new_value]]\n", argv[0]);
    LTX_LOG_STR(LOG_INFO"\t-r: read, -w: write, -l: list\n");
}


// 处理输入，执行命令
void ltx_Cmd_process(char *cmd){
    uint32_t i = 0;
    uint8_t index = 1;
    char *argv[CMD_MAX_ARG_COUNTS] = {
        [0] = cmd + 1,
    };

    if(!(cmd[0] == '/' || cmd[0] == '#')){ // 非指令
        LTX_LOG_WARN("Unknow format!\n");
        // LTX_LOG_DEBG("%s\n", cmd);
        return;
    }

    for(i = 1; (cmd[i] != '\0') && (i < CMD_BUF_SIZE - 1) && (index < CMD_MAX_ARG_COUNTS - 1); i ++){
        if((cmd[i-1] == ' ') && (cmd[i] != ' ')){
            argv[index] = &cmd[i];
            index ++;
            cmd[i-1] = '\0';
        }
    }

    for(i = 0; cmd_list[i].cmd_name[0] != ' '; i ++){
        if(my_str_cmp(cmd_list[i].cmd_name, argv[0]) == 0){
            if(cmd_list[i].cmd_cb != NULL){
                argv[0] = cmd;
                cmd_list[i].cmd_cb(index, argv);
            }else {
                LTX_LOG_ERRO("Callback function of \"%s\" is not define!\n", argv[0]);
            }
            return;
        }
    }

    LTX_LOG_WARN("Unknown cmd: %s\n", argv[0]);
    LTX_LOG_INFO("Type /help to list all commands\n");
}


// 直接设置 pwm 占空比命令，调试用
void cmd_cb_pwm(uint8_t argc, char *argv[]){
    if(argv[0][0] != '#'){
        LTX_LOG_WARN("PERMISSION DENIED!\n");
        return ;
    }

    if(argc < 4){
        goto Useage_pwm;
    }

    float duty_u, duty_v, duty_w;

    sscanf(argv[1], "%f", &duty_u);
    sscanf(argv[2], "%f", &duty_v);
    sscanf(argv[3], "%f", &duty_w);

    if(duty_u > 100.0f || duty_v > 100.0f || duty_w > 100.0f){
        LTX_LOG_WARN("duty not in range(0~100)!");
        return ;
    }

    LTX_LOG_INFO("Set pwm to %f%% %f%% %f%%\n", duty_u, duty_v, duty_w);
    duty_u /= 100.0f;
    duty_v /= 100.0f;
    duty_w /= 100.0f;

    ltx_bldc_set_duty_u(motor_wheel, duty_u);
    ltx_bldc_set_duty_v(motor_wheel, duty_v);
    ltx_bldc_set_duty_w(motor_wheel, duty_w);

    LTX_LOG_DEBG("1: %d\n", TIM1->CCR1);
    LTX_LOG_DEBG("2: %d\n", TIM1->CCR2);
    LTX_LOG_DEBG("3: %d\n", TIM1->CCR3);

    return ;
Useage_pwm:
    LTX_LOG_INFO("Useage: %s <u(0~100)> <v> <w>\n", argv[0]);
}

// 直接设置 svpwm 输出命令，调试用
void cmd_cb_svpwm(uint8_t argc, char *argv[]){
    if(argv[0][0] != '#'){
        LTX_LOG_WARN("PERMISSION DENIED!\n");
        return ;
    }

    if(argc < 4){
        goto Useage_svpwm;
    }

    if(argv[1][0] != '-'){
        goto Useage_svpwm;
    }

    float sv_v_rad, sv_v_len;
    sscanf(argv[2], "%f", &sv_v_len);
    sscanf(argv[3], "%f", &sv_v_rad);

    if(sv_v_len > 1.0f || sv_v_len < 0.0f){
        LTX_LOG_WARN("len out of range(0~1): %f\n", sv_v_len);
        goto Useage_svpwm;
    }

    if(sv_v_rad > 6.28f || sv_v_len < 0.0f){
        LTX_LOG_WARN("rad out of range(0~6.28): %f\n", sv_v_rad);
        goto Useage_svpwm;
    }

    float _test_output_abc[3];
    switch(argv[1][1]){
        case '1': // U0 和 U1 平分零向量时间
            ltx_foc1_svpwm_vec10(sv_v_len, sv_v_rad, _test_output_abc);

            break;

        case '0': // 使用全 U0 作为零向量
            ltx_foc1_svpwm_vec0(sv_v_len, sv_v_rad, _test_output_abc);

            break;

        default:
            goto Useage_svpwm;
    }

    ltx_bldc_set_duty_u(motor_wheel, _test_output_abc[0]);
    ltx_bldc_set_duty_v(motor_wheel, _test_output_abc[1]);
    ltx_bldc_set_duty_w(motor_wheel, _test_output_abc[2]);

    LTX_LOG_INFO("Set svpwm to: l:%f, r:%f\n", sv_v_len, sv_v_rad);

    return ;
Useage_svpwm:
    LTX_LOG_INFO("Useage: %s <-10/-00> <len(0~1)> <rad(0~6.28)>\n", argv[0]);
}

// 直接读取磁编码器命令，调试用
// 如果想要不影响磁编码器正常工作，则应该使用 print 命令跟踪磁编码更新来实时打印数据
void cmd_cb_mag(uint8_t argc, char *argv[]){
    if(argv[0][0] != '#'){
        LTX_LOG_WARN("PERMISSION DENIED!\n");
        return ;
    }

    // 如果有第二个参数且参数为 dma，则启动 dma 读取
    if(argc > 1){
        if(argv[1][0] == 'd'){
            mt6701_read_dma(&mag_encoder_wheel);
            LTX_LOG_INFO("Mag read dma start.\n");

            return ;
        }
    }
    // 阻塞读取
    mt6701_read(&mag_encoder_wheel);
    float angle = mt6701_trans_angle(&mag_encoder_wheel);
    float rad = mt6701_trans_rad(&mag_encoder_wheel);
    LTX_LOG_DEBG("Mag origin: %d, %d\n", mag_encoder_wheel.data_buffer[0], mag_encoder_wheel.data_buffer[1]);

    LTX_LOG_INFO("Mag angle: %f\n", angle);
    LTX_LOG_INFO("Mag rad: %f\n", rad);
}

// 直接设置 led 显示 rgb，阻塞发送
void cmd_cb_led(uint8_t argc, char *argv[]){
    if(argc < 4){
        goto Useage_led;
    }
    uint16_t r, g, b;
    sscanf(argv[1], "%hd", &r);
    sscanf(argv[2], "%hd", &g);
    sscanf(argv[3], "%hd", &b);

    ws2812_set_1_color(&my_led, 0, (uint8_t)r, (uint8_t)g, (uint8_t)b);
    ws2812_refresh(&my_led);

    LTX_LOG_INFO("Set led to: %d %d %d\n", (uint8_t)r, (uint8_t)g, (uint8_t)b);

    return ;
Useage_led:
    LTX_LOG_INFO("Useage: %s <r(0~255)> <g> <b>\n", argv[0]);
}

#ifndef PI
    #define PI 3.14159265358979f
#endif

typedef enum {
    _SVPWM_USE_10 = 0,
    _SVPWM_USE_00 = 1,
    _SVPWM_USE_C0 = 2,
} _svpwm_choose_e;
static uint8_t _svpwm_algorithm_choose = _SVPWM_USE_10;
static float _svpwm_voltage_output_pct = 0.0f;
static float _svpwm_rad_per_second = 0.0f;

// 测试电机 svpwm 旋转算法脚本
uint8_t flag_rotate_script_is_inited = 0;
uint8_t flag_svpwm_test_print = 0;
struct ltx_Script_stu script_test_svpwm_rotate;
void script_cb_test_svpwm_rotate(struct ltx_Script_stu *script){
    float _test_output_abc[3];
    static float _test_rad_now = 0.0f;

    ltx_Script_next_step_delay(script, 0, 1); // 1ms 后再次调用此脚本

    _test_rad_now += _svpwm_rad_per_second/1000.0f;
    if(_test_rad_now >= (2*PI)){
        _test_rad_now = fmodf(_test_rad_now, 2*PI);
    }

    switch(_svpwm_algorithm_choose){
        case _SVPWM_USE_10:
            ltx_foc1_svpwm_vec10(_svpwm_voltage_output_pct, _test_rad_now, _test_output_abc);
            break;
            
        case _SVPWM_USE_00:
            ltx_foc1_svpwm_vec0(_svpwm_voltage_output_pct, _test_rad_now, _test_output_abc);
            break;
            
        case _SVPWM_USE_C0:
            // 近似算法暂时有问题，会全功率输出
            // ltx_foc1_svpwm_vec0_close(_svpwm_voltage_output_pct, _test_rad_now, _test_output_abc);
            _test_output_abc[0] = 0.0f;
            _test_output_abc[1] = 0.0f;
            _test_output_abc[2] = 0.0f;
            break;

        default:
            _test_output_abc[0] = 0.0f;
            _test_output_abc[1] = 0.0f;
            _test_output_abc[2] = 0.0f;

            break;
    }
    ltx_bldc_set_duty_u(motor_wheel, _test_output_abc[0]);
    ltx_bldc_set_duty_v(motor_wheel, _test_output_abc[1]);
    ltx_bldc_set_duty_w(motor_wheel, _test_output_abc[2]);
    if(flag_svpwm_test_print){
        LTX_LOG_FMT("t:%d,%d,%d\n", TIM1->CCR1, TIM1->CCR2, TIM1->CCR3);
    }
}

// 测试电机 svpwm 旋转算法命令，会创建一个脚本来按照参数的速度和电压输出占比来旋转电机
void cmd_cb_rotate(uint8_t argc, char *argv[]){
    if(argv[0][0] != '#'){
        LTX_LOG_WARN("PERMISSION DENIED!\n");
        return ;
    }

    if(argc < 4){
        goto Useage_rotate;
    }

    if(!(argv[1][0] == 's' && argv[1][1] == 'v')){
        goto Useage_rotate;
    }

    switch(argv[1][2]){
        case '1': // 零向量平均分配给 U0 和 U1
            _svpwm_algorithm_choose = _SVPWM_USE_10;
            break;
            
        case '0': // 使用全 U0 作为零向量
            _svpwm_algorithm_choose = _SVPWM_USE_00;
            
            break;
            
        case 'c': // 使用近似算法版本的全 U0 作为零向量
            _svpwm_algorithm_choose = _SVPWM_USE_C0;

            break;
            
        case 's': // stop
            ltx_Script_pause(&script_test_svpwm_rotate);
            ltx_bldc_set_duty_u(motor_wheel, 0);
            ltx_bldc_set_duty_v(motor_wheel, 0);
            ltx_bldc_set_duty_w(motor_wheel, 0);

            flag_rotate_script_is_inited = 0;

            return ;

            break;
            
        case 't': // test
            flag_svpwm_test_print = !flag_svpwm_test_print;

            return ;

            break;

        default:
            goto Useage_rotate;
    }

    float _rad_per_s, _vol_output;
    sscanf(argv[2], "%f", &_rad_per_s);
    sscanf(argv[3], "%f", &_vol_output);

    if(_vol_output > 1.0f){
        LTX_LOG_WARN("v_pct out of range(0~1): %f\n", _vol_output);
        goto Useage_rotate;
    }

    LTX_LOG_INFO("Set motor to %f rad/s, %f pct voltage\n", _rad_per_s, _vol_output);
    _svpwm_rad_per_second = _rad_per_s;
    _svpwm_voltage_output_pct = _vol_output;

    // 脚本未初始化的话
    if(!flag_rotate_script_is_inited){
        flag_rotate_script_is_inited = 1;

        ltx_Script_init(&script_test_svpwm_rotate, script_cb_test_svpwm_rotate); // 初始化脚本
        ltx_Script_resume(&script_test_svpwm_rotate, 0); // 运行脚本
    }

    return ;
Useage_rotate:
    LTX_LOG_INFO("Useage: %s <sv10/sv00/svc0> <rad_per_s> <v_pct(0~1)>\n", argv[0]);
}


// 设置机械弧度偏置命令
void cmd_cb_set_mag_rad_offset(uint8_t argc, char *argv[]){
    if(argv[0][0] != '#'){
        LTX_LOG_WARN("PERMISSION DENIED!\n");
        return ;
    }

    if(argc < 2){
        goto Useage_set_mag_rad_offset;
    }

    float mag_rad_offset;
    sscanf(argv[1], "%f", &mag_rad_offset);

    mt6701_set_rad_offset(&mag_encoder_wheel, mag_rad_offset);
    LTX_LOG_INFO("Set mag rad offset to %f(%f)\n", mag_rad_offset, mag_encoder_wheel.rad_offset);

    return ;
Useage_set_mag_rad_offset:
    LTX_LOG_INFO("Useage: %s <rad_offset>\n", argv[0]);
}

float zero_align_list[7];
extern int16_t adc1_offset[3];
float adc1_offset_count[3];

// 计算每个极对平均偏差对齐电角度与机械角度的脚本
struct ltx_Script_stu script_zero_align;
void script_cb_zero_align(struct ltx_Script_stu *script){
    static uint8_t mag_encoder_error_count = 0;
    static uint8_t motor_stable_count = 0; // 机械角度稳定计数
    static uint8_t elec_stable_count = 0; // 电角度稳定计数
    static float last_mag_rad = 0;
    static float average_elec_rad = 0;
    uint8_t pole_now = 0; // 当前极对
    static uint8_t pole_flags = 0; // 已较准的极对
    static uint8_t mag_rad_in7 = 0; // 机械角度转换到电角度正在七个极对中的哪个
    static float mag_rad_times7mod2pi = 0; // 机械角度转换到电角度

    float _test_output_abc[3];
    float align_rad_min;
    float align_rad_max;
    float align_rad_average;


    if(ltx_Script_get_triger_type(script) == SC_TRIGER_RESET){ // 外部要求此脚本复位，在此处释放资源

        ltx_bldc_set_duty_u(motor_wheel, 0);
        ltx_bldc_set_duty_v(motor_wheel, 0);
        ltx_bldc_set_duty_w(motor_wheel, 0);
        return ;
    }

    switch(script->step_now){
        case 0: // 初始化
            LTX_LOG_INFO("Start init zero align...\n");
            
            motor_foc.flag_is_inited = 0;
            ltx_bldc_set_duty_u(motor_wheel, 0);
            ltx_bldc_set_duty_v(motor_wheel, 0);
            ltx_bldc_set_duty_w(motor_wheel, 0);
            motor_stable_count = 0;
            mag_encoder_error_count = 0;

            // 下一步等待磁编码器数据发布
            LTX_LOG_INFO("Try get mag encoder data...\n");
            ltx_Script_next_step_topic(script, 1, 5, &topic_mag_read_over);
            break;

        case 1: // 检测磁编码器通信是否正常
            if(ltx_Script_get_triger_type(script) == SC_TRIGER_TIMEOUT){ // 等待磁编码器数据超时，磁编码器 i2c 可能不正常
                if(mag_encoder_error_count++ > 100){ // 等待磁编码器数据超时次数过多，判定为磁编码器通信异常
                    LTX_LOG_ERRO("Mag encoder comunication Failed!\n");

                    // 结束此脚本
                    ltx_Script_next_step_over(script);
                    return ;
                }
                ltx_Script_next_step_topic(script, 1, 3, &topic_mag_read_over);
                return ;
            }
            LTX_LOG_INFO("Waitting for motor stable...\n");
            ltx_Script_next_step_delay(script, 2, 500);

            break;

        case 2: // 等待电机稳定
            if(motor_stable_count >= 100){ // 电机已经保持了 100ms 没有动作
                adc1_offset[0] = (int16_t)(adc1_offset_count[0] / 100.0f);
                adc1_offset[1] = (int16_t)(adc1_offset_count[1] / 100.0f);
                adc1_offset[2] = (int16_t)(adc1_offset_count[2] / 100.0f);
                LTX_LOG_INFO("ADC1 offset: %d, %d, %d\n", adc1_offset[0], adc1_offset[1], adc1_offset[2]);
                if((abs(adc1_offset[0]) > 100) || (abs(adc1_offset[1]) > 100) || (abs(adc1_offset[2]) > 100)){ // 电机 ADC 较准偏移值过大
                    LTX_LOG_ERRO("Motor adc offset too large!\n");
                    adc1_offset[0] = 0;
                    adc1_offset[1] = 0;
                    adc1_offset[2] = 0;
                    // 结束此脚本
                    ltx_Script_next_step_over(script);
                    return ;
                }
                // 进入下一步操作
                ltx_Script_next_step_topic(script, 3, 5, &topic_mag_read_over);
                LTX_LOG_INFO("Rotate motor to align zero...\n");
                return ;
            }
            if(((last_mag_rad - motor_foc.rotater_rad) > 0.001f) || ((last_mag_rad - motor_foc.rotater_rad) < -0.001f)){ // 电机有动作
                motor_stable_count = 0;
                adc1_offset_count[0] = 0;
                adc1_offset_count[1] = 0;
                adc1_offset_count[2] = 0;
            }else { // 电机未动
                // 计算 adc 较准偏置，取 100 次平均值
                adc1_offset_count[0] += 2048.0f - adc1_buffer[0];
                adc1_offset_count[1] += 2048.0f - adc1_buffer[1];
                adc1_offset_count[2] += 2048.0f - adc1_buffer[2];
                motor_stable_count ++;
            }
            last_mag_rad = motor_foc.rotater_rad;
            
            ltx_Script_next_step_delay(script, 2, 1); // 1ms 后再次检测

            break;

        case 3: // 准备开始进行零点较准算法
            mt6701_set_rad_offset(&mag_encoder_wheel, 0); // 清除原有机械角度偏置
            ltx_foc1_svpwm_vec0(0.2, 0, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_wheel, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_wheel, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_wheel, _test_output_abc[2]);
            motor_stable_count = 0; // 清除电机位置稳定计数器
            pole_flags = 0; // 清除极对位
            ltx_Script_next_step_delay(script, 9, 1000); // 等待电机稳定

            break;

        case 4: // 等待电机旋转到稳定的位置
            
            if(((last_mag_rad - motor_foc.rotater_rad) > 0.001f) || ((last_mag_rad - motor_foc.rotater_rad) < -0.001f)){ // 电机还没停住或者用户用手触碰了
                // 重新等待
                motor_stable_count = 0;
                average_elec_rad = 0;
            }else {
                motor_stable_count ++;
            }
            last_mag_rad = motor_foc.rotater_rad;

            if(motor_stable_count > 10){ // 电机已经稳定
                average_elec_rad += motor_foc.vector_I_rad; // 取 20 个电角度的平均值
                if(motor_stable_count > 30){ // 够 20 个了
                    average_elec_rad /= 20.0f;
                    // 计算是在哪个极对
                    // 应该加个超次数计数，不然如果磁编码器不在线的话就只会计算某个极对一直无法完成初始化
                    // 无所谓了，磁编码器要是不在线反正后续也用不了，电机一直转不结束让用户察觉也正好。
                    pole_now = (uint8_t)(motor_foc.rotater_rad*7 / (2*PI));
                    if(pole_now > 6) pole_now = 0;
                    // 将其存入列表
                    zero_align_list[pole_now] = average_elec_rad - fmodf(motor_foc.rotater_rad*7, (2*PI));
                    pole_flags |= 1<<pole_now;
                    LTX_LOG_INFO("Zero align[%d]: %f\n", pole_now, zero_align_list[pole_now]);

                    // 如果已经对所有极对校准过了，那么结束较准
                    if(pole_flags == 0x7F){
                        align_rad_min = zero_align_list[0];
                        align_rad_max = zero_align_list[0];
                        // 计算平均值，并且如果范围变化较大，那么报错
                        align_rad_average = 0;
                        for(uint8_t i = 0; i < 7; i ++){
                            align_rad_average += zero_align_list[i];
                            align_rad_min = zero_align_list[i] < align_rad_min ? zero_align_list[i] : align_rad_min;
                            align_rad_max = zero_align_list[i] > align_rad_max ? zero_align_list[i] : align_rad_max;
                        }
                        align_rad_average /= 7.0f;
                        LTX_LOG_INFO("Zero align average: %f\n", align_rad_average);
                        LTX_LOG_INFO("Range: %f\n", align_rad_max - align_rad_min);
                        if((align_rad_max - align_rad_min) > 0.07f){
                            LTX_LOG_WARN("Range of align error maybe too large!\n");
                        }
                        LTX_LOG_INFO("Align zero rad over.\n");
                        LTX_LOG_INFO("Cut off motor...\n");
                        // 应用到磁编码器偏置
                        mt6701_set_rad_offset(&mag_encoder_wheel, align_rad_average/7.0f);
                        // 准备逐渐减弱输出，结束脚本
                        ltx_Script_next_step_delay(script, 11, 0);

                        return ;
                    }
                    // 旋转到下一个极对对其进行较准
                    ltx_Script_next_step_delay(script, 10, 0);
                    return ;
                }
            }
            // 电机还没稳定或者还没取够 20 个算平均值，继续
            ltx_Script_next_step_delay(script, 4, 10);

            break;

        case 5: // 旋转极对，旋转 PI/2 弧度
            ltx_foc1_svpwm_vec0(0.2, PI/2, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_wheel, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_wheel, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_wheel, _test_output_abc[2]);

            ltx_Script_next_step_delay(script, 6, 200);

            break;

        case 6: // 旋转极对，旋转 PI 弧度
            ltx_foc1_svpwm_vec0(0.2, PI, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_wheel, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_wheel, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_wheel, _test_output_abc[2]);

            ltx_Script_next_step_delay(script, 7, 200);

            break;

        case 7: // 旋转极对，旋转 PI/2*3 弧度
            ltx_foc1_svpwm_vec0(0.2, PI/2*3, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_wheel, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_wheel, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_wheel, _test_output_abc[2]);

            ltx_Script_next_step_delay(script, 8, 200);

            break;

        case 8: // 旋转极对，旋转 2PI 弧度
            ltx_foc1_svpwm_vec0(0.2, 0, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_wheel, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_wheel, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_wheel, _test_output_abc[2]);

            // 进入新极对较准
            ltx_Script_next_step_delay(script, 9, 200);

            break;

        case 9: // 加强输出力
            ltx_foc1_svpwm_vec0(0.35, 0, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_wheel, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_wheel, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_wheel, _test_output_abc[2]);

            // 进入新极对较准
            ltx_Script_next_step_delay(script, 4, 600);

            break;

        case 10: // 减弱输出力
            ltx_foc1_svpwm_vec0(0.2, 0, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_wheel, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_wheel, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_wheel, _test_output_abc[2]);

            // 进入极对切换
            ltx_Script_next_step_delay(script, 5, 200);

            break;

        case 11: // 脚本结束，逐渐减弱驱动力
            ltx_foc1_svpwm_vec0(0.2, 0, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_wheel, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_wheel, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_wheel, _test_output_abc[2]);

            // 进入极对切换
            ltx_Script_next_step_delay(script, 12, 200);

            break;

        case 12: // 脚本结束，逐渐减弱驱动力
            ltx_foc1_svpwm_vec0(0.1, 0, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_wheel, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_wheel, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_wheel, _test_output_abc[2]);

            // 进入极对切换
            ltx_Script_next_step_delay(script, 13, 200);

            break;

        case 13: // 脚本结束，关闭输出
            ltx_bldc_set_duty_u(motor_wheel, 0);
            ltx_bldc_set_duty_v(motor_wheel, 0);
            ltx_bldc_set_duty_w(motor_wheel, 0);

            // 结束脚本
            ltx_Script_next_step_over(script);
            LTX_LOG_INFO("Init motor zero align success.\n");
            motor_foc.flag_is_inited = 1;

            break;

    }
}

// 计算每个极对平均偏差对齐电角度与机械角度的命令
void cmd_cb_zero_align(uint8_t argc, char *argv[]){
    if(argv[0][0] != '#'){
        LTX_LOG_WARN("PERMISSION DENIED!\n");
        return ;
    }

    static uint8_t flag_is_zero_align_script_inited = 0;

    if(flag_is_zero_align_script_inited == 0){
        ltx_Script_init(&script_zero_align, script_cb_zero_align);
        flag_is_zero_align_script_inited = 1;
    }
    ltx_Script_resume(&script_zero_align, 0);
    LTX_LOG_INFO("Start align elec_rad and mag_rad...\n");
    LTX_LOG_INFO("Do not touch motor!\n");
}
