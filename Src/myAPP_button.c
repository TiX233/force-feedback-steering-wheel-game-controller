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


struct {
    GPIO_TypeDef *GPIOx;
    uint16_t pin;
} handle_btns[] = {
    [BTN_A] = {GPIOB, GPIO_PIN_8},
    [BTN_B] = {GPIOB, GPIO_PIN_9},
    [BTN_X] = {GPIOB, GPIO_PIN_6},
    [BTN_Y] = {GPIOB, GPIO_PIN_7},
    [BTN_LB] = {GPIOA, GPIO_PIN_15},
    [BTN_RB] = {GPIOB, GPIO_PIN_12},
    [BTN_VIEW] = {GPIOB, GPIO_PIN_14},
    [BTN_MENU] = {GPIOB, GPIO_PIN_13},
    [BTN_LS] = {GPIOC, GPIO_PIN_13},
};

uint32_t handle_adc_row_data[5];

void script_cb_button_send(struct ltx_Script_stu *script){
    
    // 填充手柄数据信息
    // 按键
    uint32_t button_scan = 0;
    for(uint8_t i = 0; i < BTN_MAX_BIT; i ++){
        if(!(handle_btns[i].GPIOx->IDR & handle_btns[i].pin)){
            button_scan |= 1 << i;
        }        
    }
    // 挡杆
    // todo
    handle_up.buttons = button_scan;
    // 摇杆
    handle_up.joystick_x = handle_adc_row_data[ADC_JSTK_X] >> 4;
    handle_up.joystick_y = 0xFF - (handle_adc_row_data[ADC_JSTK_Y] >> 4);
    // 左扳机
    handle_up.trigger_left = 0xFF - (handle_adc_row_data[ADC_TRIGGER_L] >> 4);
    // 右扳机
    handle_up.trigger_right = 0xFF - (handle_adc_row_data[ADC_TRIGGER_R] >> 4);
    // 方向盘
    // handle_up.wheel = ((mag_encoder_wheel.data_buffer[0] & 0xFC)>>1) | (mag_encoder_wheel.data_buffer[1] << 7);
    handle_up.wheel = (((mag_encoder_wheel.data_buffer[0] & 0xFC)<<8) | (mag_encoder_wheel.data_buffer[1])) >> 1;

    // 发起 usb 发送
    int ret = handle_upload();

    // 发起下次 adc 扫描
    if(HAL_ADC_Start_DMA(&hadc2_handler, handle_adc_row_data, 5) != HAL_OK){
        LTX_LOG_DEBG("ADC2 ERR\n");
    }
    // __HAL_DMA_DISABLE_IT(&hdma1ch1_handler, DMA_IT_HT);
    
    // usb 发送完成或者超时都会进入下次发起发送按键数据
    ltx_Script_next_step_topic(script, 0, 10, &topic_hid_upload_over); // 超时时间 10ms
}

void handle_set_wheel_offset(struct handle_stu *handle, int16_t offset){

}

void handle_set_wheel_max_turns(struct handle_stu *handle, float max_turns){

}
