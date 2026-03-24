#ifndef __MYAPP_BUTTON_H__
#define __MYAPP_BUTTON_H__

#include "ltx_app.h"

extern struct ltx_App_stu app_button;
extern struct ltx_Topic_stu topic_hid_upload_over;

typedef enum {
    BTN_LB,         // 左肩键
    BTN_RB,         // 右肩键
    BTN_VIEW,       // led 左侧的按键
    BTN_MENU,       // led 右侧的按键
    BTN_X,          // x，右下侧第 1 个按键
    BTN_Y,          // y，右下侧第 2 个按键
    BTN_A,          // a，右下侧第 3 个按键
    BTN_B,          // b，右下侧第 4 个按键
    BTN_LS,         // 左摇杆下压按键

    BTN_MAX_BIT,
} handle_btn_e;

struct handle_pin_describe_stu {
    GPIO_TypeDef *GPIOx;
    uint16_t pin;
};

extern struct handle_pin_describe_stu handle_btns[];

// 判断按键是否按下，传入 handle_btn_e 成员
#define HANDLE_IS_BTN_PRESS(btn_e)      (!(handle_btns[btn_e].GPIOx->IDR & handle_btns[btn_e].pin))


typedef enum {
    ADC_JSTK_X,
    ADC_JSTK_Y,
    ADC_TRIGGER_L,
    ADC_TRIGGER_R,
    ADC_GEAR_LR,

    ADC_MAX_BIT,
} handle_adc_e;


typedef enum {
    GEAR_NONE = 0,  // 空挡
    GEAR_1 = 1,
    GEAR_2,
    GEAR_3,
    GEAR_4,
    GEAR_5,
    GEAR_6,
    GEAR_7,
    GEAR_R,         // 倒挡
} handle_gear_e;

typedef enum {
    GEAR_PIN_U,
    GEAR_PIN_D,
} handle_gear_pin_e;

extern struct handle_pin_describe_stu handle_gear_ud[];

handle_gear_e handle_gear_get(void); // 获取挡位


struct handle_wheel_stu {
    int32_t encoder_max;            // 编码器一圈值/最大值，逆时针增，对于 14bit 磁编，最大值 0x3FFF
    int32_t encoder_last;           // 磁编码器上次原始值 (0..encoder_max)
    int32_t zero_val;               // 零点处的编码器原始值 (raw)
    int32_t cumulative;             // 累计计数，相对于零点的编码器增量（可多圈，单位：原始计数）
    int32_t turn_now;               // 当前旋转计数（等于 cumulative）
    int32_t max_turns_x100;        // 单边最大圈数，乘以100（固定点，避免浮点）
    int32_t max_counts;            // 单边最大计数，对应 max_turns_x100
};

extern struct handle_wheel_stu handle_wheel_data;

void handle_wheel_set_zero(struct handle_wheel_stu *handle_wheel); // 设置当前方向盘位置为零点
// 设置方向盘单方向最大圈数，传入值为圈数*100（例如 1.5 圈 -> 150），避免浮点运算
void handle_wheel_set_max_turns(struct handle_wheel_stu *handle_wheel, int32_t max_turns_x100); // 设置方向盘单方向最大圈数
void handle_wheel_update(struct handle_wheel_stu *handle_wheel, uint16_t encoder_val); // 传入编码器原始值更新
uint16_t handle_wheel_get(struct handle_wheel_stu *handle_wheel); // 获取应该发送给电脑的方向值 (0~32767)
// 返回超出范围的计数量（单位：原始编码器计数），正值表示超过正方向，负值表示超过负方向，未超出返回 0
int32_t handle_wheel_get_overrun(struct handle_wheel_stu *handle_wheel);

#endif // __MYAPP_BUTTON_H__
