#ifndef __MYAPP_MOTOR_H__
#define __MYAPP_MOTOR_H__

#include "ltx_app.h"

#include "ltx_foc1.h"
#include "ltx_bldc.h"

// 电机 foc 对象
extern struct ltx_foc1_stu motor_foc;

// adc 较准偏移值
extern int16_t adc1_offset[3];

// 配置设置电机 pwm 占空比内联回调
ltx_bldc_config_duty_u_cb(motor_foc, {
    TIM1->CCR1 = (uint32_t)((float)duty*3199); // 直接操作寄存器
})
ltx_bldc_config_duty_v_cb(motor_foc, {
    TIM1->CCR2 = (uint32_t)((float)duty*3199); // 直接操作寄存器
})
ltx_bldc_config_duty_w_cb(motor_foc, {
    TIM1->CCR3 = (uint32_t)((float)duty*3199); // 直接操作寄存器
})

// 配置转换电机 adc 值为电流值内联回调
ltx_bldc_config_trans_current_u_cb(motor_foc, {
    // i = adc*3.3伏/(50倍*0.02欧*2^12)
    // motor_foc.current_u = adc_val*(3.3f/50*0.02f*4095);
    // motor_foc.current_u = adc_val*(3.3f/4095);
    motor_foc.i_A = (adc_val + adc1_offset[0] - 2048.0f)*0.0008056640625f;
})
ltx_bldc_config_trans_current_v_cb(motor_foc, {
    motor_foc.i_B = (adc_val + adc1_offset[1] - 2048.0f)*0.0008056640625f;
})
ltx_bldc_config_trans_current_w_cb(motor_foc, {
    motor_foc.i_C = (adc_val + adc1_offset[2] - 2048.0f)*0.0008056640625f;
})


// 磁编码器对象
extern struct mt6701_stu mag_encoder_wheel;
// 磁编码器所读出来的角度与弧度
extern float mag_angle;
// extern float mag_rad;
// 磁编码器读取完成事件话题
extern struct ltx_Topic_stu topic_mag_read_over;

// 电流 adc 原始数据
extern uint32_t adc1_buffer[3];
// 电流 adc 更新事件话题
extern struct ltx_Topic_stu topic_adc1_update;
// 电压弧度
extern float vol_rad;

extern struct ltx_App_stu app_motor;

#endif // __MYAPP_MOTOR_H__
