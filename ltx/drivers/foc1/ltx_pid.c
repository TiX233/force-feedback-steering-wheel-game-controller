#include "ltx_pid.h"

/**
 * @brief               设置通用 pi 控制器参数
 * @param pi_stu        控制器指针
 * @param kp            比例
 * @param ki            积分
 */
void ltx_pid_pi_set_param(struct ltx_pid_pi_stu *pi_stu, float kp, float ki){
    pi_stu->kp = kp;
    pi_stu->ki = ki;
    // pi_stu->integral = 0;
    // pi_stu->limit = limit;
}

/**
 * @brief               设置角度 pi 控制器参数
 * @param pi_stu        控制器指针
 * @param kp            比例
 * @param ki            积分
 */
void ltx_pid_pi_angle_set_param(struct ltx_pid_pi_angle_stu *pi_stu, float kp, float ki){
    pi_stu->kp = kp;
    pi_stu->ki = ki;
    // pi->error_prev = 0.0f;
}
