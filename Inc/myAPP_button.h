#ifndef __MYAPP_BUTTON_H__
#define __MYAPP_BUTTON_H__

#include "ltx_app.h"

extern struct ltx_App_stu app_button;
extern struct ltx_Topic_stu topic_hid_upload_over;

typedef enum {
    BTN_A,          // a，右下侧第 3 个按键
    BTN_B,          // b，右下侧第 4 个按键
    BTN_X,          // x，右下侧第 1 个按键
    BTN_Y,          // y，右下侧第 2 个按键
    BTN_LB,         // 左肩键
    BTN_RB,         // 右肩键
    BTN_VIEW,       // led 左边的按键
    BTN_MENU,       // led 右侧的按键
    BTN_LS,         // 左摇杆下压按键

    BTN_MAX_BIT,
} handle_btn_e;

typedef enum {
    ADC_JSTK_X,
    ADC_JSTK_Y,
    ADC_TRIGGER_L,
    ADC_TRIGGER_R,
    ADC_GEAR_LR,

    ADC_MAX_BIT,
} handle_adc_e;

typedef enum {
    TG_L = 1,       // 左扳机
    TG_R,           // 右扳机
} handle_trigger_e;

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

struct handle_stu {
    uint32_t btn_press;             // 每一位代表一个按键，1 表示按下
    uint8_t trigger_left_value;     // 左扳机
    uint8_t trigger_right_value;    // 右扳机
    uint8_t joystick_left_x;        // 左摇杆 x 轴，128 为中心
    uint8_t joystick_left_y;        // 左摇杆 y 轴，128 为中心
    handle_gear_e gear;             // 挡位
    uint16_t wheel_value_send;      // 发送给游戏的方向盘数据
    
    uint16_t wheel_value_row;       // 方向盘偏移后的数据
    int16_t wheel_offset;           // 方向盘中点偏移设置
    int8_t wheel_turns_now;         // 当前方向盘圈数的整数部分
    uint8_t wheel_max_turns_a;      // 方向盘最大圈数的整数部分
    uint8_t wheel_max_turns_b;      // 方向盘最大圈数的小数部分，0~99
};

void handle_set_wheel_offset(struct handle_stu *handle, int16_t offset);
void handle_set_wheel_max_turns(struct handle_stu *handle, float max_turns);

#endif // __MYAPP_BUTTON_H__
