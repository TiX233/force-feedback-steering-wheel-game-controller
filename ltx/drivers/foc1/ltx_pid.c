#include "ltx_pid.h"

void ltx_pid_pi_set_param(struct ltx_pid_pi_stu *pi_stu, float kp, float ki, float limit){
    pi_stu->kp = kp;
    pi_stu->ki = ki;
    // pi_stu->integral = 0;
    pi_stu->limit = limit;
}
