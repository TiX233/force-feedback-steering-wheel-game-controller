#include "myAPP_device_init.h"
#include "ltx.h"
#include "ltx_app.h"
#include "ltx_param.h"
#include "ltx_log.h"
#include "ltx_script.h"
#include "ltx_event_group.h"
#include "myAPP_system.h"
#include "myAPP_led.h"
#include "ws2812.h"
#include "myAPP_motor.h"
#include "mt6701.h"
#include "myAPP_button.h"
#include "myAPP_ffb.h"

// 所需初始化外部硬件完成事件
#define EVENT_INIT_LED_OVER                 0x0001
#define EVENT_INIT_MOTOR_OVER               0x0002

// 函数声明
// led 初始化脚本回调
void script_cb_led_init(struct ltx_Script_stu *script);
// 外部硬件初始化完成事件组回调
void eventg_cb_device_init_over(struct ltx_Event_group_stu *event);

// ws2812 spi 发送回调
void my_led_cb_send_data(struct ws2812_stu *led, uint8_t *buffer, uint16_t size);
void my_led_cb_send_data_dma(struct ws2812_stu *led, uint8_t *buffer, uint16_t size);

// 零点对齐脚本回调
void script_cb_zero_align(struct ltx_Script_stu *script);

// 电机初始化脚本回调
void script_cb_motor_init(struct ltx_Script_stu *script);


// ws2812 对象
uint8_t my_led_display_buffer[WS2812_BUFFER_SIZE(1)];
struct ws2812_stu my_led = {
    .buffer = my_led_display_buffer,
    .lemp_num = 1,
    .send_data = my_led_cb_send_data,
    .send_data_dma = my_led_cb_send_data_dma,
};

// 组件全局变量
// dma 发送完成话题
struct ltx_Topic_stu topic_spi_tx_over = _LTX_TOPIC_DEAFULT_CONFIG(topic_spi_tx_over);

// 所有外部硬件初始化完成事件组
struct ltx_Event_group_stu eventg_device_init_over;

// led 初始化脚本对象结构体
struct ltx_Script_stu script_led_init;

// 计算每个极对平均偏差对齐电角度与机械角度的脚本
struct ltx_Script_stu script_zero_align;
// 零点对齐完成事件话题
struct ltx_Topic_stu topic_zero_align_over = _LTX_TOPIC_DEAFULT_CONFIG(topic_zero_align_over);

// 电机初始化脚本
struct ltx_Script_stu script_motor_init;


// app 相关
int myAPP_device_init_init(struct ltx_App_stu *app){
    // 创建指示灯初始脚本
    ltx_Script_init(&script_led_init, script_cb_led_init);
    // 初始化零点较准脚本
    ltx_Script_init(&script_zero_align, script_cb_zero_align);
    // 创建电机初始化脚本
    ltx_Script_init(&script_motor_init, script_cb_motor_init);

    // 创建所有外部硬件初始化完成事件组
    ltx_Event_group_init(&eventg_device_init_over, eventg_cb_device_init_over, EVENT_INIT_LED_OVER |\
                                                                               EVENT_INIT_MOTOR_OVER \
                                                                               , 0); // 超时时间设置为 TickType_t 最大值

    // 发起 adc1
    if(HAL_ADCEx_InjectedStart_IT(&hadc1_handler) != HAL_OK)
        while(1){ LTX_LOG_ERRO("ADC1 injected start IT Failed!\n"); HAL_Delay(1000); }

    return 0;
}

int myAPP_device_init_pause(struct ltx_App_stu *app){

    ltx_Script_pause(&script_led_init);
    ltx_Script_pause(&script_motor_init);

    return 0;
}

int myAPP_device_init_resume(struct ltx_App_stu *app){

    ltx_Script_resume(&script_led_init, 0);
    ltx_Script_resume(&script_motor_init, 0);

    return 0;
}

int myAPP_device_init_destroy(struct ltx_App_stu *app){

    ltx_Script_pause(&script_led_init);
    ltx_Script_pause(&script_motor_init);
    ltx_Script_pause(&script_zero_align);

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
        case 0: // 清除 led 显示
            ws2812_set_1_color(&my_led, 0, 20, 0, 0);
            ws2812_refresh_dma(&my_led);

            ltx_Script_next_step_topic(script, 1, 10, &topic_spi_tx_over); // spi 发送完成事件触发或者超时 10ms 后进入下一步
            break;

        case 1: // 检测发送是否完成
            if(ltx_Script_get_triger_type(script) != SC_TRIGER_TOPIC){ // 超时
                LTX_LOG_WARN("Init led Warning: spi dma timeout...rewait\n");
                ltx_Script_next_step_topic(script, 2, 10, &topic_spi_tx_over); // spi 发送完成事件触发或者超时 10ms 后进入下一步

                return ;
            }
            // 发送完成触发
            ltx_Event_group_publish(&eventg_device_init_over, EVENT_INIT_LED_OVER); // 发布初始化 led 完成事件
            ltx_Script_next_step_over(script); // 结束脚本
            // 开启 led 管理 app
            ltx_App_init(&app_led);
            ltx_App_resume(&app_led);

            break;

        case 2: // 未完成后的重新等待
            if(ltx_Script_get_triger_type(script) != SC_TRIGER_TOPIC){ // 超时
                LTX_LOG_ERRO("Init led Failed: spi dma timeout.\n");
                return ;
            }
            // 发送完成触发
            ltx_Event_group_publish(&eventg_device_init_over, EVENT_INIT_LED_OVER); // 发布初始化 led 完成事件
            ltx_Script_next_step_over(script); // 结束脚本
            // 开启 led 管理 app
            ltx_App_init(&app_led);
            ltx_App_resume(&app_led);

            break;

        default:

            break;
    }

}

extern uint32_t handle_adc_row_data[ADC_MAX_BIT];
uint32_t motor_sound_tickcount = 0;
// uint32_t motor_sound_tickreload = 20000/1000/2; // 1khz 频率声音
#define motor_sound_duty 0.3f
// 电机初始化脚本回调
void script_cb_motor_init(struct ltx_Script_stu *script){
    static uint8_t flag_ab = 0;
    static uint8_t beep_count = 0;

    switch(script->step_now){
        case 0:
            // 准备初始化电机
            led_set_blink_color(0, 20, 0); // 绿灯闪烁
            ltx_Script_next_step_delay(script, 1, 100);
            break;

        case 1:
            // 等待按下菜单键，按下后进入电机初始化
            if(HANDLE_IS_BTN_PRESS(BTN_MENU)){
                led_set_blink_color(20, 20, 0); // 黄灯闪烁
                ltx_Script_resume(&script_zero_align, 500); // 500ms 后开始执行对齐算法
                ltx_Script_next_step_topic(script, 2, 0, &topic_zero_align_over); // 以最大时间等待对齐完成
            }else {
                ltx_Script_next_step_delay(script, 1, 20);
            }

            break;

        case 2:
            if(ltx_Script_get_triger_type(script) == SC_TRIGER_TIMEOUT){ // 等待对齐完成事件超时
                led_set_blink_color(20, 0, 0); // 红灯闪烁
                LTX_LOG_ERRO("Zero align Failed!\n");
                ltx_Script_pause(&script_zero_align);
                ltx_Script_next_step_over(script);
                return ;
            }
            // 对齐完成，进入发声提示
            motor_foc.flag_is_inited = 0;
            ltx_bldc_set_duty_u(motor_foc, 0);
            ltx_bldc_set_duty_v(motor_foc, 0);
            ltx_bldc_set_duty_w(motor_foc, 0);
            ltx_Script_next_step_topic(script, 3, 0, &topic_adc1_update);
            led_set_blink_color(20, 0, 20); // 紫灯闪烁

            break;
#if 0
        case 3:
            if(motor_sound_tickcount++ > 20000/5){ // 播放声音超过 200ms
                ltx_bldc_set_duty_u(motor_foc, 0);
                ltx_bldc_set_duty_v(motor_foc, 0);
                ltx_bldc_set_duty_w(motor_foc, 0);
                if(beep_count ++){ // 响过两次
                    // 发起 adc 扫描
                    if(HAL_ADC_Start_DMA(&hadc2_handler, handle_adc_row_data, ADC_MAX_BIT) != HAL_OK){
                        led_set_blink_color(20, 0, 0); // 红灯闪烁
                        LTX_LOG_ERRO("ADC2 ERR\n");
                        ltx_Script_next_step_over(script);
                        return ;
                    }
                    // 进入方向盘中点和限位设置
                    motor_foc.flag_is_inited = 1;
                    ltx_Script_next_step_delay(script, 4, 20);
                }else {
                    ltx_Script_next_step_delay(script, 3, 200);
                    motor_sound_tickcount = 0;
                }
                return ;
            }

            if(motor_sound_tickcount % motor_sound_tickreload == 0){
                if(flag_ab){
                    ltx_bldc_set_duty_u(motor_foc, motor_sound_duty);
                    ltx_bldc_set_duty_v(motor_foc, 0);
                    flag_ab = 0;
                }else {
                    ltx_bldc_set_duty_u(motor_foc, 0);
                    ltx_bldc_set_duty_v(motor_foc, motor_sound_duty);
                    flag_ab = 1;
                }
            }

            ltx_Script_next_step_topic(script, 3, 1, &topic_adc1_update);
            break;
#endif
        case 3:
            if(motor_sound_tickcount++ > 400){ // 播放声音超过 200ms
                ltx_bldc_set_duty_u(motor_foc, 0);
                ltx_bldc_set_duty_v(motor_foc, 0);
                ltx_bldc_set_duty_w(motor_foc, 0);
                if(beep_count ++){ // 响过两次
                    // 发起 adc 扫描
                    if(HAL_ADC_Start_DMA(&hadc2_handler, handle_adc_row_data, ADC_MAX_BIT) != HAL_OK){
                        led_set_blink_color(20, 0, 0); // 红灯闪烁
                        LTX_LOG_ERRO("ADC2 ERR\n");
                        ltx_Script_next_step_over(script);
                        return ;
                    }
                    // 进入方向盘中点和限位设置
                    motor_foc.flag_is_inited = 1;
                    ltx_Script_next_step_delay(script, 4, 20);
                }else {
                    ltx_Script_next_step_delay(script, 3, 200);
                    motor_sound_tickcount = 0;
                }
                return ;
            }

            if(flag_ab){
                ltx_bldc_set_duty_u(motor_foc, motor_sound_duty);
                ltx_bldc_set_duty_v(motor_foc, 0);
                flag_ab = 0;
            }else {
                ltx_bldc_set_duty_u(motor_foc, 0);
                ltx_bldc_set_duty_v(motor_foc, motor_sound_duty);
                flag_ab = 1;
            }

            ltx_Script_next_step_delay(script, 3, 1);

            break;

        case 4:
            // 按下菜单键且挂入挡位才算
            if(HANDLE_IS_BTN_PRESS(BTN_MENU)){
                handle_gear_e gear_now = handle_gear_get();
                if(gear_now != GEAR_NONE){
                    handle_wheel_update(&handle_wheel_data, mag_encoder_wheel.data_row);
                    handle_wheel_set_zero(&handle_wheel_data); // 设置当前位置为方向盘中点
                    handle_wheel_set_max_turns(&handle_wheel_data, 25*gear_now); // 设置单边最大圈数，0.25*挡位
                    led_set_blink_color(0, 0, 20); // 蓝灯闪烁
                    // 初始化完成
                    ltx_Event_group_publish(&eventg_device_init_over, EVENT_INIT_MOTOR_OVER);
                    ltx_Script_next_step_over(script);
                    return ;
                }
            }
            HAL_ADC_Start_DMA(&hadc2_handler, handle_adc_row_data, ADC_MAX_BIT);
            ltx_Script_next_step_delay(script, 4, 20);

            break;
    }
}


// 根据三相电流计算电流向量的弧度
static float get_I_rad(float i_A, float i_B, float i_C){
    // 计算电流向量模长
    float vector_I_len = sqrtf(2.0f/3 * (i_A * i_A + i_B * i_B + i_C * i_C));
    float vector_I_rad;

    // 计算电流向量弧度
    // 钳位避免超越定义域 [-1, 1]
    float ratio = i_A / vector_I_len;
    // 范围超出那就直接赋值，不用额外算反余弦
    if(ratio < -1.0f){
        vector_I_rad = PI;
    }else if (ratio > 1.0f){
        if(i_B < i_C){
            vector_I_rad = 2*PI;
        }else {
            vector_I_rad = 0.0f;
        }
    }else {
        vector_I_rad = acosf(ratio);
        if(i_B < i_C) vector_I_rad = (2*PI) - vector_I_rad;
    }

    return vector_I_rad;
}

// 电机极对数
#define MOTOR_POLE_PAIRS   7

float zero_align_list[MOTOR_POLE_PAIRS];

// 计算每个极对平均偏差对齐电角度与机械角度的脚本回调
void script_cb_zero_align(struct ltx_Script_stu *script){
    #if 1
    static uint8_t mag_encoder_error_count = 0;
    static uint8_t motor_stable_count = 0; // 机械角度稳定计数
    static uint8_t elec_stable_count = 0; // 电角度稳定计数
    static float last_mag_rad = 0;
    static float average_elec_rad = 0;
    uint8_t pole_now = 0; // 当前极对
    static uint8_t pole_flags = 0; // 已较准的极对

    float _test_output_abc[3];
    float align_rad_min;
    float align_rad_max;
    float align_rad_average;


    if(ltx_Script_get_triger_type(script) == SC_TRIGER_RESET){ // 外部要求此脚本复位，在此处释放资源

        ltx_bldc_set_duty_u(motor_foc, 0);
        ltx_bldc_set_duty_v(motor_foc, 0);
        ltx_bldc_set_duty_w(motor_foc, 0);
        return ;
    }

    switch(script->step_now){
        case 0: // 初始化
            LTX_LOG_INFO("Start init zero align...\n");
            
            motor_foc.flag_is_inited = 0;
            ltx_bldc_set_duty_u(motor_foc, 0);
            ltx_bldc_set_duty_v(motor_foc, 0);
            ltx_bldc_set_duty_w(motor_foc, 0);
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
            adc1_offset[0] = 0;
            adc1_offset[1] = 0;
            // adc1_offset[2] = 0;
            ltx_Script_next_step_delay(script, 2, 500);

            break;

        case 2: // 等待电机稳定
            if(motor_stable_count >= 100){ // 电机已经保持了 100ms 没有动作
                adc1_offset[0] /= 100.0f;
                adc1_offset[1] /= 100.0f;
                adc1_offset[2] /= 100.0f;
                LTX_LOG_INFO("ADC1 offset: %f, %f, %f\n", adc1_offset[0], adc1_offset[1], adc1_offset[2]);
                // if((fabsf(adc1_offset[0] - 2048.0f) > 100) || (fabsf(adc1_offset[1] - 2048.0f) > 100) || (fabsf(adc1_offset[2] - 2048.0f) > 100)){ // 电机 ADC 较准偏移值过大
                if((fabsf(adc1_offset[0] - 2048.0f) > 100) || (fabsf(adc1_offset[2] - 2048.0f) > 100)){ // 电机 ADC 较准偏移值过大
                    LTX_LOG_ERRO("Motor adc offset too large!\n");
                    adc1_offset[0] = 2048.0f;
                    adc1_offset[1] = 2048.0f;
                    adc1_offset[2] = 2048.0f;
                    // 结束此脚本
                    ltx_Script_next_step_over(script);
                    return ;
                }
                // 进入下一步操作
                ltx_Script_next_step_topic(script, 3, 5, &topic_mag_read_over);
                LTX_LOG_INFO("Rotate motor to align zero...\n");
                return ;
            }
            if(((last_mag_rad - motor_foc.rotor_rad) > 0.001f) || ((last_mag_rad - motor_foc.rotor_rad) < -0.001f)){ // 电机有动作
                motor_stable_count = 0;
                adc1_offset[0] = 0;
                adc1_offset[1] = 0;
                adc1_offset[2] = 0;
            }else { // 电机未动
                // 计算 adc 较准偏置，取 100 次平均值
                adc1_offset[0] += adc1_buffer[0];
                adc1_offset[1] += adc1_buffer[1];
                adc1_offset[2] += adc1_buffer[2];
                motor_stable_count ++;
            }
            last_mag_rad = motor_foc.rotor_rad;
            
            ltx_Script_next_step_delay(script, 2, 1); // 1ms 后再次检测

            break;

        case 3: // 准备开始进行零点较准算法
            mt6701_set_rad_offset(&mag_encoder_wheel, 0); // 清除原有机械角度偏置
            ltx_foc1_svpwm_vec0(0.2, 0, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_foc, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_foc, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_foc, _test_output_abc[2]);
            motor_stable_count = 0; // 清除电机位置稳定计数器
            pole_flags = 0; // 清除极对位
            ltx_Script_next_step_delay(script, 9, 1000); // 等待电机稳定

            break;

        case 4: // 等待电机旋转到稳定的位置
            
            if(((last_mag_rad - motor_foc.rotor_rad) > 0.001f) || ((last_mag_rad - motor_foc.rotor_rad) < -0.001f)){ // 电机还没停住或者用户用手触碰了
                // 重新等待
                motor_stable_count = 0;
                average_elec_rad = 0;
            }else {
                motor_stable_count ++;
            }
            last_mag_rad = motor_foc.rotor_rad;

            if(motor_stable_count > 10){ // 电机已经稳定
                average_elec_rad += get_I_rad(-motor_foc.i_A, -motor_foc.i_B, -motor_foc.i_C); // 取 20 个电角度的平均值
                if(motor_stable_count > 30){ // 够 20 个了
                    average_elec_rad /= 20.0f;
                    // 计算是在哪个极对
                    // 应该加个超次数计数，不然如果磁编码器不在线的话就只会计算某个极对一直无法完成初始化
                    // 无所谓了，磁编码器要是不在线反正后续也用不了，电机一直转不结束让用户察觉也正好。
                    pole_now = (uint8_t)(motor_foc.rotor_rad*MOTOR_POLE_PAIRS / (2*PI));
                    if(pole_now > (MOTOR_POLE_PAIRS-1)) pole_now = 0;
                    // 将其存入列表
                    zero_align_list[pole_now] = average_elec_rad - fmodf(motor_foc.rotor_rad*MOTOR_POLE_PAIRS, (2*PI));
                    pole_flags |= 1<<pole_now;
                    LTX_LOG_INFO("Zero align[%d]: %f\n", pole_now, zero_align_list[pole_now]);

                    // 如果已经对所有极对校准过了，那么结束较准
                    if(pole_flags == 0x7F){
                        align_rad_min = zero_align_list[0];
                        align_rad_max = zero_align_list[0];
                        // 计算平均值，并且如果范围变化较大，那么报错
                        align_rad_average = 0;
                        for(uint8_t i = 0; i < MOTOR_POLE_PAIRS; i ++){
                            align_rad_average += zero_align_list[i];
                            align_rad_min = zero_align_list[i] < align_rad_min ? zero_align_list[i] : align_rad_min;
                            align_rad_max = zero_align_list[i] > align_rad_max ? zero_align_list[i] : align_rad_max;
                        }
                        align_rad_average /= MOTOR_POLE_PAIRS;
                        LTX_LOG_INFO("Zero align average: %f\n", align_rad_average);
                        LTX_LOG_INFO("Range: %f\n", align_rad_max - align_rad_min);
                        if((align_rad_max - align_rad_min) > 0.07f){
                            LTX_LOG_WARN("Range of align error maybe too large!\n");
                        }
                        LTX_LOG_INFO("Align zero rad over.\n");
                        LTX_LOG_INFO("Cut off motor...\n");
                        // 应用到磁编码器偏置
                        mt6701_set_rad_offset(&mag_encoder_wheel, align_rad_average/MOTOR_POLE_PAIRS);
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
            ltx_bldc_set_duty_u(motor_foc, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_foc, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_foc, _test_output_abc[2]);

            ltx_Script_next_step_delay(script, 6, 200);

            break;

        case 6: // 旋转极对，旋转 PI 弧度
            ltx_foc1_svpwm_vec0(0.2, PI, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_foc, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_foc, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_foc, _test_output_abc[2]);

            ltx_Script_next_step_delay(script, 7, 200);

            break;

        case 7: // 旋转极对，旋转 PI/2*3 弧度
            ltx_foc1_svpwm_vec0(0.2, PI/2*3, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_foc, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_foc, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_foc, _test_output_abc[2]);

            ltx_Script_next_step_delay(script, 8, 200);

            break;

        case 8: // 旋转极对，旋转 2PI 弧度
            ltx_foc1_svpwm_vec0(0.2, 0, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_foc, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_foc, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_foc, _test_output_abc[2]);

            // 进入新极对较准
            ltx_Script_next_step_delay(script, 9, 200);

            break;

        case 9: // 加强输出力
            ltx_foc1_svpwm_vec0(0.35, 0, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_foc, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_foc, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_foc, _test_output_abc[2]);

            // 进入新极对较准
            ltx_Script_next_step_delay(script, 4, 600);

            break;

        case 10: // 减弱输出力
            ltx_foc1_svpwm_vec0(0.2, 0, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_foc, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_foc, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_foc, _test_output_abc[2]);

            // 进入极对切换
            ltx_Script_next_step_delay(script, 5, 200);

            break;

        case 11: // 脚本结束，逐渐减弱驱动力
            ltx_foc1_svpwm_vec0(0.2, 0, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_foc, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_foc, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_foc, _test_output_abc[2]);

            // 进入极对切换
            ltx_Script_next_step_delay(script, 12, 200);

            break;

        case 12: // 脚本结束，逐渐减弱驱动力
            ltx_foc1_svpwm_vec0(0.1, 0, _test_output_abc); // 输出特定电压向量，让电机定在某个角度
            ltx_bldc_set_duty_u(motor_foc, _test_output_abc[0]);
            ltx_bldc_set_duty_v(motor_foc, _test_output_abc[1]);
            ltx_bldc_set_duty_w(motor_foc, _test_output_abc[2]);

            // 进入极对切换
            ltx_Script_next_step_delay(script, 13, 200);

            break;

        case 13: // 脚本结束，关闭输出
            ltx_bldc_set_duty_u(motor_foc, 0);
            ltx_bldc_set_duty_v(motor_foc, 0);
            ltx_bldc_set_duty_w(motor_foc, 0);

            // 结束脚本
            ltx_Script_next_step_over(script);
            LTX_LOG_INFO("Init motor zero align success.\n");
            ltx_Topic_publish(&topic_zero_align_over); // 发布对齐完成事件
            motor_foc.flag_is_inited = 1;

            break;

    }
    #else
        mt6701_set_rad_offset(&mag_encoder_wheel, -2.75f/MOTOR_POLE_PAIRS);
        motor_foc.flag_is_inited = 1;
    #endif
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
    // ltx_App_init(&app_led);
    // ltx_App_resume(&app_led);

    ltx_App_init(&app_motor);
    // ltx_App_resume(&app_motor);

    ltx_App_init(&app_button);
    ltx_App_resume(&app_button);

    ltx_App_init(&app_ffb);
    ltx_App_resume(&app_ffb);
}


// ws2812 spi 发送回调
void my_led_cb_send_data(struct ws2812_stu *led, uint8_t *buffer, uint16_t size){
    HAL_SPI_Transmit(&hspi2_handler, buffer, size, 1000);
}

// ws2812 spi dma 发送回调
void my_led_cb_send_data_dma(struct ws2812_stu *led, uint8_t *buffer, uint16_t size){
    HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi2_handler, buffer, size);
    if(status != HAL_OK){
        HAL_SPI_DMAStop(&hspi2_handler);
        LTX_LOG_WARN("SPI DMA WARNING: %d, %d, %d\n", status, hspi2_handler.ErrorCode, hspi2_handler.hdmatx->ErrorCode);
    }
}

// spi dma 发送完成回调
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    // ltx_Lock_off(&lock_spi);
    ltx_Topic_publish(&topic_spi_tx_over);
}
