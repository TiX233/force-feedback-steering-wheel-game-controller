#ifndef __MYAPP_MOTOR_H__
#define __MYAPP_MOTOR_H__

#include "ltx_app.h"

#include "ltx_bldc.h"

extern struct ltx_bldc_stu motor_wheel;
// 配置设置电机 pwm 占空比内联回调
ltx_bldc_config_duty_u_cb(motor_wheel, {
    TIM1->CCR1 = (uint32_t)((float)duty*3199); // 直接操作寄存器
})
ltx_bldc_config_duty_v_cb(motor_wheel, {
    TIM1->CCR2 = (uint32_t)((float)duty*3199); // 直接操作寄存器
})
ltx_bldc_config_duty_w_cb(motor_wheel, {
    TIM1->CCR3 = (uint32_t)((float)duty*3199); // 直接操作寄存器
})

// 配置转换电机 adc 值为电流值内联回调
ltx_bldc_config_trans_current_u_cb(motor_wheel, {
    // i = adc*3.3伏/(50倍*0.02欧*2^12)
    // motor_wheel.current_u = adc_val*(3.3f/50*0.02f*4095);
    // motor_wheel.current_u = adc_val*(3.3f/4095);
    motor_wheel.current_u = adc_val*((float)(8.058608E-4F));
})
ltx_bldc_config_trans_current_v_cb(motor_wheel, {
    motor_wheel.current_v = adc_val*((float)(8.058608E-4F));
})
ltx_bldc_config_trans_current_w_cb(motor_wheel, {
    motor_wheel.current_w = adc_val*((float)(8.058608E-4F));
})

// 磁编码器对象
extern struct mt6701_stu mag_encoder_wheel;
// 磁编码器所读出来的角度与弧度
extern float mag_angle;
extern float mag_rad;
// 磁编码器读取完成事件话题
extern struct ltx_Topic_stu topic_mag_read_over;

extern struct ltx_App_stu app_motor;

#endif // __MYAPP_MOTOR_H__
