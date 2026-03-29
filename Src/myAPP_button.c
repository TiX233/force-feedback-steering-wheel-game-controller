#include "myApp_button.h"
#include "ltx.h"
#include "ltx_app.h"
#include "ltx_log.h"
#include "ltx_cmd.h"
#include "ltx_script.h"
#include "myAPP_device_init.h"
#include "myAPP_motor.h"
#include "mt6701.h"
#include "usb_config.h"

struct ltx_Topic_stu topic_hid_upload_over = _LTX_TOPIC_DEAFULT_CONFIG(topic_hid_upload_over);

struct ltx_Script_stu script_button_send;

void script_cb_button_send(struct ltx_Script_stu *script);

int myApp_button_init(struct ltx_App_stu *app){

    ltx_Script_init(&script_button_send, script_cb_button_send);
    handle_wheel_set_max_turns(&handle_wheel_data, 1*100);

    return 0;
}

int myApp_button_pause(struct ltx_App_stu *app){

    ltx_Script_pause(&script_button_send);
    
    return 0;
}

int myApp_button_resume(struct ltx_App_stu *app){

    ltx_Script_resume(&script_button_send, 0);

    return 0;
}

int myApp_button_destroy(struct ltx_App_stu *app){

    ltx_Script_pause(&script_button_send);

    // free...

    return 0;
}


struct ltx_App_stu app_button = {
    .is_initialized = 0,
    .status = ltx_App_status_pause,
    .name = "button",

    .init = myApp_button_init,
    .pause = myApp_button_pause,
    .resume = myApp_button_resume,
    .destroy = myApp_button_destroy,

    .task_list = NULL,
    
    .next = NULL,
};


struct handle_pin_describe_stu handle_btns[] = {
    [BTN_LB] = {GPIOA, GPIO_PIN_15},
    [BTN_RB] = {GPIOB, GPIO_PIN_12},
    [BTN_VIEW] = {GPIOB, GPIO_PIN_14},
    [BTN_MENU] = {GPIOB, GPIO_PIN_13},
    [BTN_X] = {GPIOB, GPIO_PIN_6},
    [BTN_Y] = {GPIOB, GPIO_PIN_7},
    [BTN_A] = {GPIOB, GPIO_PIN_8},
    [BTN_B] = {GPIOB, GPIO_PIN_9},
    [BTN_LS] = {GPIOC, GPIO_PIN_13},
};

struct handle_pin_describe_stu handle_gear_ud[] = {
    // 板子里画反了，软件里对调一下
    [GEAR_PIN_U] = {GPIOB, GPIO_PIN_10},
    [GEAR_PIN_D] = {GPIOB, GPIO_PIN_11},
};

uint32_t handle_adc_row_data[ADC_MAX_BIT] = {[ADC_TRIGGER_L] = 0x7FF, [ADC_TRIGGER_R] = 0x7FF};
uint16_t trigger_left_offset = 50;
uint16_t trigger_left_max = 0x7FF;
uint16_t trigger_left_min = 0x7FF - 100;
uint16_t trigger_right_offset = 50;
uint16_t trigger_right_max = 0x7FF;
uint16_t trigger_right_min = 0x7FF - 100;

// 发送按键等等信息给电脑的脚本
void script_cb_button_send(struct ltx_Script_stu *script){
    
    // 填充手柄数据信息
    // 按键
    uint32_t button_scan = 0;
    for(uint8_t i = 0; i < BTN_MAX_BIT; i ++){
        if(HANDLE_IS_BTN_PRESS(i)){
            button_scan |= 1 << i;
        }        
    }
    // 挡杆
    handle_gear_e gear_now = handle_gear_get();
    if(gear_now != GEAR_NONE){ // 非空挡
        button_scan |= 0x80000000 >> (gear_now - GEAR_1);
    }
    handle_up.buttons = button_scan;
    // 摇杆
    handle_up.joystick_x = handle_adc_row_data[ADC_JSTK_X] >> 4;
    handle_up.joystick_y = 0xFF - (handle_adc_row_data[ADC_JSTK_Y] >> 4);
    // 左扳机
        // handle_up.trigger_left = 0xFF - (uint8_t)(handle_adc_row_data[ADC_TRIGGER_L] >> 1);
        // 较准
        if(handle_adc_row_data[ADC_TRIGGER_L] > trigger_left_max){
            trigger_left_max += handle_adc_row_data[ADC_TRIGGER_L];
            trigger_left_max >>= 1;
        }else if(handle_adc_row_data[ADC_TRIGGER_L] < trigger_left_min){
            trigger_left_min += handle_adc_row_data[ADC_TRIGGER_L];
            trigger_left_min >>= 1;
        }
        // 范围限定
        if(handle_adc_row_data[ADC_TRIGGER_L] > trigger_left_max - trigger_left_offset){
            handle_up.trigger_left = 0;
        }else if(handle_adc_row_data[ADC_TRIGGER_L] < trigger_left_min){
            handle_up.trigger_left = 0xFF;
        }else {
            handle_up.trigger_left = (uint8_t)((float)(trigger_left_max - trigger_left_offset - handle_adc_row_data[ADC_TRIGGER_L]) /
                                                         (trigger_left_max - trigger_left_offset - trigger_left_min) * 0xFF);
        }
    // 右扳机
        // handle_up.trigger_right = 0xFF - (uint8_t)(handle_adc_row_data[ADC_TRIGGER_R] >> 1);
        // 较准
        if(handle_adc_row_data[ADC_TRIGGER_R] > trigger_right_max){
            trigger_right_max += handle_adc_row_data[ADC_TRIGGER_R];
            trigger_right_max >>= 1;
        }else if(handle_adc_row_data[ADC_TRIGGER_R] < trigger_right_min){
            trigger_right_min += handle_adc_row_data[ADC_TRIGGER_R];
            trigger_right_min >>= 1;
        }
        // 范围限定
        if(handle_adc_row_data[ADC_TRIGGER_R] > trigger_right_max - trigger_right_offset){
            handle_up.trigger_right = 0;
        }else if(handle_adc_row_data[ADC_TRIGGER_R] < trigger_right_min){
            handle_up.trigger_right = 0xFF;
        }else {
            handle_up.trigger_right = (uint8_t)((float)(trigger_right_max - trigger_right_offset - handle_adc_row_data[ADC_TRIGGER_R]) /
                                                         (trigger_right_max - trigger_right_offset - trigger_right_min) * 0xFF);
        }
    // 方向盘
    handle_wheel_update(&handle_wheel_data, mag_encoder_wheel.data_row);
    handle_up.wheel = handle_wheel_get(&handle_wheel_data);

    // 发起 usb 发送
    int ret = handle_upload();

    // 发起下次 adc 扫描
    if(HAL_ADC_Start_DMA(&hadc2_handler, handle_adc_row_data, ADC_MAX_BIT) != HAL_OK){
        LTX_LOG_DEBG("ADC2 ERR\n");
    }
    // __HAL_DMA_DISABLE_IT(&hdma1ch1_handler, DMA_IT_HT);
    
    // usb 发送完成或者超时都会进入下次发起发送按键数据
    ltx_Script_next_step_topic(script, 0, 5, &topic_hid_upload_over); // 超时时间 5ms
}

// 获取挡位
handle_gear_e handle_gear_get(void){
    uint8_t gear_offset;
    if(!(handle_gear_ud[GEAR_PIN_U].GPIOx->IDR & handle_gear_ud[GEAR_PIN_U].pin)){ // 挡杆上推
        gear_offset = 0;
    }else if(!(handle_gear_ud[GEAR_PIN_D].GPIOx->IDR & handle_gear_ud[GEAR_PIN_D].pin)){ // 挡杆下推
        gear_offset = 1;
    }else { // 空挡
        return GEAR_NONE;
    }

    if(handle_adc_row_data[ADC_GEAR_LR] > 0xFFF/2){ // 右半
        if(handle_adc_row_data[ADC_GEAR_LR] > 0xFFF/4*3){ // 右半的第二个挡位
            return GEAR_7 + gear_offset;
        }else{ // 右半的第一个挡位
            return GEAR_5 + gear_offset;
        }
    }else{ // 左半
        if(handle_adc_row_data[ADC_GEAR_LR] < 0xFFF/4){ // 左半的第一个挡位
            return GEAR_1 + gear_offset;
        }else{ // 左半的第二个挡位
            return GEAR_3 + gear_offset;
        }
    }
}


struct handle_wheel_stu handle_wheel_data = {
    .encoder_max = 0x3FFF,
};
// 设置当前方向盘位置为中点
void handle_wheel_set_zero(struct handle_wheel_stu *handle_wheel){

    if(handle_wheel->encoder_max <= 0) handle_wheel->encoder_max = 0x3FFF; // 14-bit
    // 将当前编码器值作为零点，累计计数清零
    handle_wheel->zero_val = handle_wheel->encoder_last & handle_wheel->encoder_max;
    handle_wheel->cumulative = 0;
    handle_wheel->turn_now = 0;
}
// 设置方向盘单方向最大圈数
void handle_wheel_set_max_turns(struct handle_wheel_stu *handle_wheel, int32_t max_turns_x100){

    // 只接受正值，单位为 圈*100（例如 1.5 圈 -> 150）
    if(max_turns_x100 < 0) max_turns_x100 = -max_turns_x100;
    // 限定在 0~1000 圈
    if(max_turns_x100 > 1000 * 100) max_turns_x100 = 1000 * 100;

    int32_t counts_per_rev = handle_wheel->encoder_max + 1;
    // 计算单边最大计数：max_turns_x100/100 * counts_per_rev
    int64_t prod = (int64_t)max_turns_x100 * counts_per_rev;
    int32_t max_counts = (int32_t)((prod + 50) / 100); // 四舍五入

    handle_wheel->max_turns_x100 = max_turns_x100;
    handle_wheel->max_counts = max_counts;
}
// 传入编码器值更新方向盘值
void handle_wheel_update(struct handle_wheel_stu *handle_wheel, uint16_t encoder_val){

    int32_t mask_max = handle_wheel->encoder_max;
    int32_t counts_per_rev = mask_max + 1;
    int32_t half = counts_per_rev / 2;

    int32_t raw = encoder_val & mask_max;
    int32_t last = handle_wheel->encoder_last & mask_max;

    // 首次使用时，把 last 设为 raw（避免大跳变）
    if(handle_wheel->encoder_last < 0 || handle_wheel->encoder_last > mask_max){
        handle_wheel->encoder_last = raw;
        handle_wheel->zero_val = raw;
        handle_wheel->cumulative = 0;
        handle_wheel->turn_now = 0;
        return;
    }

    int32_t d = raw - last;
    if(d > half) d -= counts_per_rev;
    else if(d < -half) d += counts_per_rev;

    handle_wheel->cumulative += d;
    handle_wheel->turn_now = handle_wheel->cumulative;

    handle_wheel->encoder_last = raw;
}
 // 获取应该发送给电脑的方向值 (0~32767)
uint16_t handle_wheel_get(struct handle_wheel_stu *handle_wheel){

    const int32_t MID = 16383; // 32767/2

    int32_t max_counts = handle_wheel->max_counts;
    int32_t cum = handle_wheel->cumulative;

    if(max_counts <= 0){
        // 未设置最大旋转，返回中点
        return (uint16_t)MID;
    }

    // 限幅，映射到 [-max_counts, max_counts]
    if(cum > max_counts) cum = max_counts;
    if(cum < -max_counts) cum = -max_counts;

    // 使用 64-bit 中间量避免溢出： scaled in [-MID, MID]
    int64_t scaled = ((int64_t)cum * MID) / max_counts;
    int32_t out = (int32_t)scaled + MID;

    if(out < 0) out = 0;
    if(out > 32767) out = 32767;

    return (uint16_t)out;
}

// 返回超出范围的计数量（单位：原始编码器计数），正表示超过正方向（逆时针），负表示超过负方向（顺时针），未超出返回 0
int32_t handle_wheel_get_overrun(struct handle_wheel_stu *handle_wheel){

    int32_t max_counts = handle_wheel->max_counts;
    int32_t cum = handle_wheel->cumulative;
    if(max_counts <= 0) return 0;
    if(cum > max_counts) return cum - max_counts;
    if(cum < -max_counts) return cum + max_counts;
    return 0;
}
