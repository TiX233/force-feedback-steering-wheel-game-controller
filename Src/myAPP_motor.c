#include "myAPP_motor.h"
#include "ltx.h"
#include "ltx_app.h"
#include "ltx_log.h"
#include "ltx_script.h"
#include "ltx_lock.h"
#include "math.h"
#include "mt6701.h"
#include "ltx_foc1.h"


// 测试六步换相脚本回调
void script_cb_six_step(struct ltx_Script_stu *script);
// 测试 spwm 脚本回调
void script_cb_spwm(struct ltx_Script_stu *script);

// 所选用的脚本回调
#define SCRIPT_CB_MOTOR     script_cb_six_step

// 磁编码器回调
void wheel_mag_e_read_reg(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num);
void wheel_mag_e_read_reg_dma(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num);

// 磁编码器对象
struct mt6701_stu mag_encoder_wheel = {
    .addr = MT6701_DEFAULT_ADDR,
    .read_reg = wheel_mag_e_read_reg,
    .read_reg_dma = wheel_mag_e_read_reg_dma,
};

// 磁编码器所读出来的角度与弧度
float mag_angle;
float mag_rad;

// 测试方向盘电机对象
struct ltx_bldc_stu motor_wheel = {
    .id = 0,
};

// 电机脚本
struct ltx_Script_stu script_motor;

// 磁编码器读取完成事件话题
struct ltx_Topic_stu topic_mag_read_over = _LTX_TOPIC_DEAFULT_CONFIG(topic_mag_read_over);

// 避免磁编码器读取出错导致无法更新数据，创建一个闹钟，会在超时时间后重新发起下次读取
// struct ltx_Lock_stu lock_mag_timeout;


int myApp_motor_init(struct ltx_App_stu *app){
    
    ltx_Script_init(&script_motor, SCRIPT_CB_MOTOR);

    // 发起 dma 读取磁编码器数据
    // mt6701_read_dma(&mag_encoder_wheel);

    return 0;
}

int myApp_motor_pause(struct ltx_App_stu *app){

    ltx_Script_pause(&script_motor);
    
    return 0;
}

int myApp_motor_resume(struct ltx_App_stu *app){

    ltx_Script_resume(&script_motor, 0);
    
    return 0;
}

int myApp_motor_destroy(struct ltx_App_stu *app){
    
    ltx_Script_pause(&script_motor);
    // free...

    return 0;
}


struct ltx_App_stu app_motor = {
    .is_initialized = 0,
    .status = ltx_App_status_pause,
    .name = "motor",

    .init = myApp_motor_init,
    .pause = myApp_motor_pause,
    .resume = myApp_motor_resume,
    .destroy = myApp_motor_destroy,

    .task_list = NULL,
    
    .next = NULL,
};


// 六步换相表
const uint8_t six_step_list[6][3] = {
    [0] = {0, 0, 1},
    [1] = {0, 1, 1},
    [2] = {0, 1, 0},
    [3] = {1, 1, 0},
    [4] = {1, 0, 0},
    [5] = {1, 0, 1},
};

#define TEST_DUTY_FOR_SIX_STEP  0.1f    // 10% 占空比

// 测试六步换相用脚本
void script_cb_six_step(struct ltx_Script_stu *script){
    // if(ltx_Script_get_triger_type(script) == SC_TRIGER_RESET){ // 外部要求此脚本重置，可在这里做释放资源等操作
    //     return ;
    // }

    ltx_bldc_set_duty_u(motor_wheel, (six_step_list[script->step_now][0] ? TEST_DUTY_FOR_SIX_STEP : 0));
    ltx_bldc_set_duty_v(motor_wheel, (six_step_list[script->step_now][1] ? TEST_DUTY_FOR_SIX_STEP : 0));
    ltx_bldc_set_duty_w(motor_wheel, (six_step_list[script->step_now][2] ? TEST_DUTY_FOR_SIX_STEP : 0));

    ltx_Script_next_step_delay(script, (script->step_now+1)%6, 100); // 100ms 换相一次
}


#define PI  3.1415926535f
// 正弦波每次增加的角度
#define SPWM_ANGLE_ADD      PI*2/360    // 每次增加一度 

// 正弦波输出幅值
#define SPWM_OUTPUT_PCT     0.1f        // 输出 10%

// 测试 spwm 用脚本
void script_cb_spwm(struct ltx_Script_stu *script){
    // 每根线的相位差为 120 度，这里用弧度表示
    static float s_angle_now_u = 0.0f;
    static float s_angle_now_v = PI*2/3;
    static float s_angle_now_w = PI*2/3*2;

    // 递增
    s_angle_now_u += SPWM_ANGLE_ADD;
    s_angle_now_v += SPWM_ANGLE_ADD;
    s_angle_now_w += SPWM_ANGLE_ADD;

    ltx_bldc_set_duty_u(motor_wheel, sinf(s_angle_now_u)*SPWM_OUTPUT_PCT);
    ltx_bldc_set_duty_v(motor_wheel, sinf(s_angle_now_v)*SPWM_OUTPUT_PCT);
    ltx_bldc_set_duty_w(motor_wheel, sinf(s_angle_now_w)*SPWM_OUTPUT_PCT);

    ltx_Script_next_step_delay(script, 1, 1); // 1ms 后再次切换
}



void wheel_mag_e_read_reg(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num){
    HAL_I2C_Mem_Read(&hi2c1_handler, mt->addr, reg_addr, 1, reg_buffer, reg_num, 1000);
}

void wheel_mag_e_read_reg_dma(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num){
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&hi2c1_handler, mt->addr, reg_addr, 1, reg_buffer, reg_num);
    // 发起 dma 读取失败
    if(status != HAL_OK){
        LTX_LOG_ERRO("mag dma read err: %d, %d\n", status, hi2c1_handler.ErrorCode);
        // 一般会在 dma 接收完成回调里面发起下次接收，所以肯定是上次收发完成调用这里，一般不会出错
        // 但是 py32f0 会有丢中断的情况，不知道 f4 会不会出现
        // 也就是有可能不会调用接收完成中断回调，进而不会发起下一次读取……
        // 但是引入超时闹钟又会有额外的开销，先就这样吧
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
    // 转换角度
    mag_angle = mt6701_trans_angle(&mag_encoder_wheel);
    mag_rad = mt6701_trans_rad(&mag_encoder_wheel);
    // 发起下次读取
    mt6701_read_dma(&mag_encoder_wheel);
    // 发布角度更新事件
    ltx_Topic_publish(&topic_mag_read_over);
}

