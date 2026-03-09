#ifndef __LTX_FOC1_H__
#define __LTX_FOC1_H__

#include "ltx.h"
#include "ltx_pid.h"
#include "math.h"

#ifndef PI
    #define PI  3.14159265358979f
#endif

struct ltx_foc1_stu {
    uint8_t flag_is_inited; // 初始化标志位
    uint8_t pole_pairs; // 极对数
    float current_limit; // 电流限制，例如 0.5 代表 ±0.5A
    float voltage_limit; // 电压输出限制，0~1，例如 0.5 代表以母线电压的 50% 为上限，不可超过 1

    float vector_I_len; // 电流向量模长百分比，[0~1]
    float vector_I_rad; // 电流向量弧度，[0, 2pi)
    
    float vector_V_len; // 电压向量模长百分比，[0~1]
    float vector_V_rad; // 电压向量弧度，[0, 2pi)

    struct ltx_pid_pi_stu pi_theta; // 磁场相位环
    struct ltx_pid_pi_stu pi_amplitude; // 磁场强度环

    // 三相电压输出
    float v_outputABC[3];
    // 三相采集电流
    float i_A;
    float i_B;
    float i_C;

    float target_I_len; // 目标输出电流向量模长占比
    float target_I_rad; // 目标输出电流向量方向

    float rotater_rad; // 转子弧度，[0, 2pi)

    float dt; // foc 算法调用间隔，单位默认毫秒
};

// svpwm 生成算法，任选其一
void ltx_foc1_svpwm_vec10(float V_amplitude, float V_rad, float V_outputABC[]); // U1 和 U0 按照特定比例分配零向量，默认平均分配
void ltx_foc1_svpwm_vec0(float V_amplitude, float V_rad, float V_outputABC[]); // 全使用 U0 作为零向量
void ltx_foc1_svpwm_vec0_close(float V_amplitude, float V_rad, float V_outputABC[]); // 拟合全 U0 算法，无法正常使用

#define FOC_SVPWM_ALGORITHM     ltx_foc1_svpwm_vec10


// ================== 配置 设置 pwm 占空比 用户内联回调宏 ==================
#define ltx_foc1_config_duty_a_cb(motor, code)\
    ltx_inline void motor##_set_duty_a(float duty) code
#define ltx_foc1_config_duty_b_cb(motor, code)\
    ltx_inline void motor##_set_duty_b(float duty) code
#define ltx_foc1_config_duty_c_cb(motor, code)\
    ltx_inline void motor##_set_duty_c(float duty) code

// 样例：创建一个内联函数作为 motor1 的设置 a 路 pwm 占空比回调：
// ltx_foc1_config_duty_a_cb(motor1, {
//     TIM1->CCR1 = (uint32_t)(duty*1234); // 直接操作寄存器，响应更快
//     printf("set motor1 a duty to %d", duty); // 打印
// })
// 创建一个内联函数作为 motor2 的设置 c 路 pwm 占空比回调：
// ltx_foc1_config_duty_c_cb(motor2, {
//     __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty*5678); // 调用其他库
// })

// ================== 设置占空比 api ==================
#define ltx_foc1_set_duty_a(motor, duty)\
    motor##_set_duty_a(duty)
#define ltx_foc1_set_duty_b(motor, duty)\
    motor##_set_duty_b(duty)
#define ltx_foc1_set_duty_c(motor, duty)\
    motor##_set_duty_c(duty)

// 使用样例：
// ltx_foc1_set_duty_a(motor1, 0.5); // 设置电机 1 的 a 路 pwm 占空比为 50%
// ltx_foc1_set_duty_c(motor2, 0.72); // 设置电机 2 的 c 路 pwm 占空比为 72%

// ================== 配置 转换电流 用户内联回调宏 ==================
#define ltx_foc1_config_trans_current_a_cb(motor, code)\
    ltx_inline void motor##_trans_current_a(adc_type_t adc_val) code
#define ltx_foc1_config_trans_current_b_cb(motor, code)\
    ltx_inline void motor##_trans_current_b(adc_type_t adc_val) code
#define ltx_foc1_config_trans_current_c_cb(motor, code)\
    ltx_inline void motor##_trans_current_c(adc_type_t adc_val) code

// 样例：创建一个内联函数作为 motor1 的转换 a 路 adc 数值为电流回调：
// ltx_foc1_config_trans_current_a_cb(motor1, {
//     motor1.i_A = adc_val * 0.1234; // 转换好后可以存储在对象成员变量中
//     printf("motor1 u current now: %f", motor1.current_u); // 打印
// })
// 创建一个内联函数作为 motor2 的转换 c 路 adc 数值为电流回调：
// ltx_foc1_config_trans_current_c_cb(motor2, {
//     motor2.i_C = adc_val * 0.5678; // 转换好后可以存储在对象成员变量中
// })

// ================== 转换电流 api ==================
#define ltx_bldc_trans_current_a(motor, adc_val)\
    motor##_trans_current_a(adc_val)
#define ltx_bldc_trans_current_b(motor, adc_val)\
    motor##_trans_current_b(adc_val)
#define ltx_bldc_trans_current_c(motor, adc_val)\
    motor##_trans_current_c(adc_val)

// 使用样例：
// ltx_bldc_trans_current_a(motor1, adc1_buffer[0]); // 转换 adc 原始数据到电机 1 的 a 路电流
// ltx_bldc_trans_current_c(motor2, adc2_buffer[2]); // 转换 adc 原始数据到电机 2 的 c 路电流

// foc 算法，按 dt 定期调用，一般在 adc 采样完成回调调用，adc 一般配置为 pwm 周期触发
#define ltx_foc1_algorithm(foc) do{\
    float e_rad; /* 电角度 */ \
    float rad_diff; /* 角度偏差 */ \
\
    /* 转换转子角度匹配电角度：r*7 %2PI */ \
    e_rad = fmodf(foc.rotater_rad * foc.pole_pairs, (2*PI)); \
\
    /* 计算电流向量模长 */ \
    foc.vector_I_len = sqrtf(2.0f/3 * (foc.i_A * foc.i_A + foc.i_B * foc.i_B + foc.i_C * foc.i_C));\
\
    if(foc.vector_I_len > 0.0001f){ \
        /* 计算电流向量弧度 */ \
        foc.vector_I_rad = acosf(foc.i_A / foc.vector_I_len); \
        if(foc.i_B < foc.i_C) foc.vector_I_rad = (2*PI) - foc.vector_I_rad; \
        /* 对比电流向量与转子角度的偏差。保持角度差为 90 度就是追求 id = 0 控制 */ \
        rad_diff = foc.vector_I_rad - e_rad; \
        if(rad_diff < -PI){ \
            rad_diff += (2*PI); \
        }else if(rad_diff > PI){ \
            rad_diff -= (2*PI); \
        } \
\
        /* 将角度偏差值输入 pi 控制器，获取 svpwm 下次输出的电压角度 */ \
        /* foc.vector_V_rad = ltx_pid_pi_update(&(foc.pi_theta), rad_diff * foc.vector_I_len, foc.dt); */ \
        foc.vector_V_rad = ltx_pid_pi_update(&(foc.pi_theta), rad_diff, foc.dt); \
    }\
\
    /* 对比电流向量模长与设定输出模长的偏差。设定输出模长是由外层速度环/力矩环等 pid 控制器提供或者用户提供的 */ \
    /* 将模长偏差输入 pi 控制器，获取 svpwm 下次输出电压模长 */ \
    foc.vector_V_len = ltx_pid_pi_update(&(foc.pi_amplitude), target_I_len - foc.vector_I_len, foc.dt); \
\
    /* 计算 svpwm 输出 */ \
    FOC_SVPWM_ALGORITHM(foc.vector_V_len, foc.vector_V_rad, foc.v_outputABC); \
    /* 输出三相电压 */ \
    ltx_foc1_set_duty_a(foc, foc.v_outputABC[0]); \
    ltx_foc1_set_duty_b(foc, foc.v_outputABC[1]); \
    ltx_foc1_set_duty_c(foc, foc.v_outputABC[2]); \
}while(0)

// 设置转速为弧度每秒
void ltx_foc1_set_speed(struct ltx_foc1_stu *foc, float rad_per_s);
// 设置力，按输出能力百分比设置，0~1
void ltx_foc1_set_force(struct ltx_foc1_stu *foc, float f_pct);




#endif // __LTX_FOC1_H__
