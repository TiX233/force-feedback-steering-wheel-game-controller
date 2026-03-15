#include "ltx_param.h"
#include "ltx_app.h"
#include "myAPP_system.h"
#include "ltx_log.h"
#include "myAPP_motor.h"
#include "ltx_foc1.h"

#define PARAM_PRINT(_str_type, _data) LTX_LOG_FMT(#_data": "#_str_type"\n", _data)
#define PARAM_READ(_name, _str_type, _data) void param_read_##_name(struct param_stu *param){PARAM_PRINT(_str_type, _data);}
#define PARAM_WRITE(_name, _type, _str_type, _range_u, _range_d, _code) void param_write_##_name(struct param_stu *param, const char *new_val){_type new_VAL; \
                                                                sscanf(new_val, #_str_type, &new_VAL);\
                                                                if(new_VAL > _range_u || new_VAL < _range_d){\
                                                                    LTX_LOG_WARN("new val out of range: "#_str_type"\n", new_VAL);\
                                                                    return ;\
                                                                }\
                                                                LTX_LOG_INFO("Set %s to %f\n", #_name, new_VAL);\
                                                                _code}

// 心拍频率
uint16_t heart_beat_Hz = 1;
void param_read_heart_beat(struct param_stu *param){
    LTX_LOG_INFO("Heart beat: %d Hz\n", heart_beat_Hz);
}
void param_write_heart_beat(struct param_stu *param, const char *new_val){
    uint16_t new_Hz;

    sscanf(new_val, "%hd", &new_Hz);

    if(new_Hz > 1000 || new_Hz < 1){
        LTX_LOG_WARN("New value(%d) not in range(1~1000)!\n", new_Hz);
        return ;
    }

    heart_beat_Hz = new_Hz;
    
    task_heart_beat.tick_reload = 1000/heart_beat_Hz;

    LTX_LOG_INFO("Set heart beat to %d Hz\n", heart_beat_Hz);
}

#if 0
// 极坐标参数
// foc 目标 电流模长
PARAM_READ(target_len, %f, motor_foc.target_I_len)
PARAM_WRITE(target_len, float, %f, 0.5f, -0.5f, motor_foc.target_I_len = new_VAL;)
// foc 目标 电流向量与转子方向夹角
PARAM_READ(target_rad, %f, motor_foc.target_I_rad)
PARAM_WRITE(target_rad, float, %f, 3.14159f, 0, motor_foc.target_I_rad = new_VAL;)
// foc 目标 电流模长 kp
PARAM_READ(len_kp, %f, motor_foc.pi_amplitude.kp)
PARAM_WRITE(len_kp, float, %f, 100, 0, motor_foc.pi_amplitude.kp = new_VAL;)
// foc 目标 电流模长 ki
PARAM_READ(len_ki, %f, motor_foc.pi_amplitude.ki)
PARAM_WRITE(len_ki, float, %f, 100, 0, motor_foc.pi_amplitude.ki = new_VAL;)
// foc 目标 电流夹角 kp
PARAM_READ(rad_kp, %f, motor_foc.pi_theta.kp)
PARAM_WRITE(rad_kp, float, %f, 100, 0, motor_foc.pi_theta.kp = new_VAL;)
// foc 目标 电流夹角 ki
PARAM_READ(rad_ki, %f, motor_foc.pi_theta.ki)
PARAM_WRITE(rad_ki, float, %f, 100, 0, motor_foc.pi_theta.ki = new_VAL;)
#endif

// foc2 目标 iq
PARAM_READ(iq, %f, motor_foc.target_I_q)
PARAM_WRITE(iq, float, %f, 1, -1, motor_foc.target_I_q = new_VAL;)

// foc2 目标 id
PARAM_READ(id, %f, motor_foc.target_I_d)
PARAM_WRITE(id, float, %f, 1, -1, motor_foc.target_I_d = new_VAL;)

// foc2 q 轴 kp
PARAM_READ(kp_q, %f, motor_foc.pi_q.kp)
PARAM_WRITE(kp_q, float, %f, 100, 0.0f, motor_foc.pi_q.kp = new_VAL;)

// foc2 q 轴 ki
PARAM_READ(ki_q, %f, motor_foc.pi_q.ki)
PARAM_WRITE(ki_q, float, %f, 100, 0.0f, motor_foc.pi_q.ki = new_VAL;)

// foc2 d 轴 kp
PARAM_READ(kp_d, %f, motor_foc.pi_d.kp)
PARAM_WRITE(kp_d, float, %f, 100, 0.0f, motor_foc.pi_d.kp = new_VAL;)

// foc2 d 轴 ki
PARAM_READ(ki_d, %f, motor_foc.pi_d.ki)
PARAM_WRITE(ki_d, float, %f, 100, 0.0f, motor_foc.pi_d.ki = new_VAL;)

// 速度环目标速度
extern float rpm_target;
PARAM_READ(rpm_target, %f, rpm_target)
PARAM_WRITE(rpm_target, float, %f, 10000, -10000, rpm_target = new_VAL;)

// 速度环 kp
PARAM_READ(speed_kp, %f, pi_speed.kp)
PARAM_WRITE(speed_kp, float, %f, 100, 0, pi_speed.kp = new_VAL;)
// 速度环 ki
PARAM_READ(speed_ki, %f, pi_speed.ki)
PARAM_WRITE(speed_ki, float, %f, 100, 0, pi_speed.ki = new_VAL;)

// 速度环低通滤波系数
extern float rpm_lpf_coeff;
PARAM_READ(speed_lpf, %f, rpm_lpf_coeff)
PARAM_WRITE(speed_lpf, float, %f, 1, 0, rpm_lpf_coeff = new_VAL;)


#define PARAM_ITEM(_name)   {.param_name = #_name,.param_read = param_read_##_name,.param_write = param_write_##_name,}
struct param_stu param_list[] = {
    { // 心拍任务频率
        .param_name = "heart_beat_Hz",
        .param_read = param_read_heart_beat,
        .param_write = param_write_heart_beat,
    },

#if 0
    // foc1 可读写参数
    // 电流向量模长
    PARAM_ITEM(target_len),
    // 电流向量与转子的夹角
    PARAM_ITEM(target_rad),
    // 电流向量模长 kp
    PARAM_ITEM(len_kp),
    // 电流向量模长 ki
    PARAM_ITEM(len_ki),
    // 电流向量夹角 kp
    PARAM_ITEM(rad_kp),
    // 电流向量夹角 ki
    PARAM_ITEM(rad_ki),
#endif

    // foc2 可读写参数
    PARAM_ITEM(iq),
    PARAM_ITEM(id),
    PARAM_ITEM(kp_q),
    PARAM_ITEM(ki_q),
    PARAM_ITEM(kp_d),
    PARAM_ITEM(ki_d),

    // 速度环可读写参数
    // 目标转速
    PARAM_ITEM(rpm_target),
    // 速度环 kp
    PARAM_ITEM(speed_kp),
    // 速度环 ki
    PARAM_ITEM(speed_ki),
    // 速度滤波系数
    PARAM_ITEM(speed_lpf),


    // 末尾项
    {
        .param_name = " ",
    },
};
