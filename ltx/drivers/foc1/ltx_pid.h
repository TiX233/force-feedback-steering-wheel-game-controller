#ifndef __LTX_PID_H__
#define __LTX_PID_H__

#include "ltx.h"

// pi 控制器对象结构体
struct ltx_pid_pi_stu {
    float kp; // 比例
    float ki; // 积分

    float integral; // 积分量
    float limit; // 输出限幅
};

// 修改 pi 控制器参数
void ltx_pid_pi_set_param(struct ltx_pid_pi_stu *pi_stu, float kp, float ki, float limit);

// 更新 pi 控制器 api
ltx_inline float ltx_pid_pi_update(struct ltx_pid_pi_stu *pi_stu, float error, float dt){
    pi_stu->integral += error * dt;
    // 抗积分饱和
    if(pi_stu->integral > pi_stu->limit) pi_stu->integral = pi_stu->limit;
    if(pi_stu->integral < -pi_stu->limit) pi_stu->integral = -pi_stu->limit;
    
    float output = pi_stu->kp * error + pi_stu->ki * pi_stu->integral;
    // 输出限幅
    if(output > pi_stu->limit) output = pi_stu->limit;
    if(output < -pi_stu->limit) output = -pi_stu->limit;
    return output;
}

#endif // __LTX_PID_H__
