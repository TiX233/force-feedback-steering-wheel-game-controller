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
// foc 目标 电流模长
void param_read_target_len(struct param_stu *param){
    LTX_LOG_INFO("foc target I len: %f\n", motor_foc.target_I_len);
}
void param_write_target_len(struct param_stu *param, const char *new_val){
    float new_len;

    sscanf(new_val, "%f", &new_len);

    if(new_len > 0.5f || new_len < -0.5f){
        LTX_LOG_WARN("new len out of range: %f\n", new_len);
        return ;
    }
    LTX_LOG_INFO("Set new len to %f\n", new_len);

    ltx_foc1_set_target_len(motor_foc, new_len);
}

// foc 目标 电流向量与转子方向夹角
void param_read_target_rad(struct param_stu *param){
    LTX_LOG_INFO("foc target I rad diff: %f\n", motor_foc.target_I_rad);
}
void param_write_target_rad(struct param_stu *param, const char *new_val){
    float new_rad;

    sscanf(new_val, "%f", &new_rad);

    if(new_rad > 3.1415926f || new_rad < 0.0f){
        LTX_LOG_WARN("new rad out of range: %f\n", new_rad);
        return ;
    }
    LTX_LOG_INFO("Set new rad to %f\n", new_rad);

    ltx_foc1_set_target_rad(motor_foc, new_rad);
}

// foc 目标 电流模长 kp
void param_read_len_kp(struct param_stu *param){
    LTX_LOG_INFO("foc I len kp: %f\n", motor_foc.pi_amplitude.kp);
}
void param_write_len_kp(struct param_stu *param, const char *new_val){
    float new_VAL;

    sscanf(new_val, "%f", &new_VAL);

    LTX_LOG_INFO("Set new len kp to %f\n", new_VAL);

    motor_foc.pi_amplitude.kp = new_VAL;
}
// foc 目标 电流模长 ki
void param_read_len_ki(struct param_stu *param){
    LTX_LOG_INFO("foc I len ki: %f\n", motor_foc.pi_amplitude.ki);
}
void param_write_len_ki(struct param_stu *param, const char *new_val){
    float new_VAL;

    sscanf(new_val, "%f", &new_VAL);

    LTX_LOG_INFO("Set new len ki to %f\n", new_VAL);

    motor_foc.pi_amplitude.ki = new_VAL;
}

// foc 目标 电流夹角 kp
void param_read_rad_kp(struct param_stu *param){
    LTX_LOG_INFO("foc I rad kp: %f\n", motor_foc.pi_theta.kp);
}
void param_write_rad_kp(struct param_stu *param, const char *new_val){
    float new_VAL;

    sscanf(new_val, "%f", &new_VAL);

    LTX_LOG_INFO("Set new rad kp to %f\n", new_VAL);

    motor_foc.pi_theta.kp = new_VAL;
}
// foc 目标 电流夹角 ki
void param_read_rad_ki(struct param_stu *param){
    LTX_LOG_INFO("foc I rad ki: %f\n", motor_foc.pi_theta.ki);
}
void param_write_rad_ki(struct param_stu *param, const char *new_val){
    float new_VAL;

    sscanf(new_val, "%f", &new_VAL);

    LTX_LOG_INFO("Set new rad ki to %f\n", new_VAL);

    motor_foc.pi_theta.ki = new_VAL;
}
#endif

// foc 目标 iq
PARAM_READ(iq, %f, motor_foc.target_I_q)
PARAM_WRITE(iq, float, %f, 0.5f, -0.5f, motor_foc.target_I_q = new_VAL;)

// foc 目标 id
PARAM_READ(id, %f, motor_foc.target_I_d)
PARAM_WRITE(id, float, %f, 0.5f, -0.5f, motor_foc.target_I_d = new_VAL;)

// foc q 轴 kp
PARAM_READ(kp_q, %f, motor_foc.pi_q.kp)
PARAM_WRITE(kp_q, float, %f, 100, 0.0f, motor_foc.pi_q.kp = new_VAL;)

// foc q 轴 ki
PARAM_READ(ki_q, %f, motor_foc.pi_q.ki)
PARAM_WRITE(ki_q, float, %f, 100, 0.0f, motor_foc.pi_q.ki = new_VAL;)

// foc d 轴 kp
PARAM_READ(kp_d, %f, motor_foc.pi_d.kp)
PARAM_WRITE(kp_d, float, %f, 100, 0.0f, motor_foc.pi_d.kp = new_VAL;)

// foc d 轴 ki
PARAM_READ(ki_d, %f, motor_foc.pi_d.ki)
PARAM_WRITE(ki_d, float, %f, 100, 0.0f, motor_foc.pi_d.ki = new_VAL;)

extern float rpm_target;
// 速度环目标速度
void param_read_rpm_target(struct param_stu *param){
    LTX_LOG_INFO("rpm target: %f\n", rpm_target);
}
void param_write_rpm_target(struct param_stu *param, const char *new_val){
    float new_VAL;

    sscanf(new_val, "%f", &new_VAL);

    LTX_LOG_INFO("Set new rpm target to %f\n", new_VAL);

    rpm_target = new_VAL;
}

// 速度环 kp
void param_read_speed_kp(struct param_stu *param){
    LTX_LOG_INFO("speed kp: %f\n", pi_speed.kp);
}
void param_write_speed_kp(struct param_stu *param, const char *new_val){
    float new_VAL;

    sscanf(new_val, "%f", &new_VAL);

    LTX_LOG_INFO("Set new speed kp to %f\n", new_VAL);

    pi_speed.kp = new_VAL;
}
// 速度环 ki
void param_read_speed_ki(struct param_stu *param){
    LTX_LOG_INFO("speed ki: %f\n", pi_speed.ki);
}
void param_write_speed_ki(struct param_stu *param, const char *new_val){
    float new_VAL;

    sscanf(new_val, "%f", &new_VAL);

    LTX_LOG_INFO("Set new speed ki to %f\n", new_VAL);

    pi_speed.ki = new_VAL;
}

// 速度环低通滤波系数
extern float rpm_lpf_coeff;
void param_read_speed_lpf(struct param_stu *param){
    LTX_LOG_INFO("speed rpm_lpf: %f\n", rpm_lpf_coeff);
}
void param_write_speed_lpf(struct param_stu *param, const char *new_val){
    float new_VAL;

    sscanf(new_val, "%f", &new_VAL);

    LTX_LOG_INFO("Set new speed rpm_lpf to %f\n", new_VAL);

    rpm_lpf_coeff = new_VAL;
}


#define PARAM_ITEM(_name)   {.param_name = #_name,.param_read = param_read_##_name,.param_write = param_write_##_name,}
struct param_stu param_list[] = {
    { // 心拍任务频率
        .param_name = "heart_beat_Hz",
        .param_read = param_read_heart_beat,
        .param_write = param_write_heart_beat,
    },

#if 0
    { // 电流向量模长
        .param_name = "target_len",
        .param_read = param_read_target_len,
        .param_write = param_write_target_len,
    },

    { // 电流向量与转子的夹角
        .param_name = "target_rad",
        .param_read = param_read_target_rad,
        .param_write = param_write_target_rad,
    },

    { // 电流向量模长 kp
        .param_name = "len_kp",
        .param_read = param_read_len_kp,
        .param_write = param_write_len_kp,
    },

    { // 电流向量模长 ki
        .param_name = "len_ki",
        .param_read = param_read_len_ki,
        .param_write = param_write_len_ki,
    },

    { // 电流向量夹角 kp
        .param_name = "rad_kp",
        .param_read = param_read_rad_kp,
        .param_write = param_write_rad_kp,
    },

    { // 电流向量夹角 ki
        .param_name = "rad_ki",
        .param_read = param_read_rad_ki,
        .param_write = param_write_rad_ki,
    },
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
