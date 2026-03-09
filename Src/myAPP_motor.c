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

// 电机所选用的脚本回调
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

struct ltx_foc1_stu motor_foc = {
    .flag_is_inited = 0,
    .pole_pairs = 7, // 极对数 7
    .current_limit = 0.5f, // 电流限制，例如 0.5 代表 ±0.5A
    .voltage_limit = 0.5f, // 电压输出限制，0~1，例如 0.5 代表以母线电压的 50% 为上限，不可超过 1

    .vector_I_len = 0, // 当前电流向量模长百分比，[0~1]
    .vector_I_rad = 0, // 当前电流向量弧度，[0, 2pi)

    .vector_V_len = 0, // 电压向量模长百分比，[0~1]
    .vector_V_rad = 0, // 电压向量弧度，[0, 2pi)

    .pi_theta = { // 磁场相位环
        .kp = 0,
        .ki = 0,
        .integral = 0,
        .limit = PI/2,
    },
    .pi_amplitude = { // 磁场强度环
        .kp = 0,
        .ki = 0,
        .integral = 0,
        .limit = 0.5,
    },

    // 三相电压输出
    .v_outputABC[0] = 0,
    .v_outputABC[1] = 0,
    .v_outputABC[2] = 0,
    // 三相采集电流
    .i_A = 0,
    .i_B = 0,
    .i_C = 0,

    .target_I_len = 0, // 目标输出电流向量模长占比

    .rotater_rad = 0, // 转子弧度，[0, 2pi)

    .dt = 0.05f, // foc 算法调用间隔，单位默认毫秒
};

// 磁编码器所读出来的机械角度与弧度
float mag_angle;
// float mag_rad; // 直接用 foc 对象里的成员变量
// adc1 原始值
uint32_t adc1_buffer[3];
// 测试方向盘电机对象
struct ltx_bldc_stu motor_wheel = {.id = 0,};

// 电机脚本
struct ltx_Script_stu script_motor;

// 磁编码器读取完成事件话题
struct ltx_Topic_stu topic_mag_read_over = _LTX_TOPIC_DEAFULT_CONFIG(topic_mag_read_over);
// 电流 adc 更新事件话题
struct ltx_Topic_stu topic_adc1_update = _LTX_TOPIC_DEAFULT_CONFIG(topic_adc1_update);

int myApp_motor_init(struct ltx_App_stu *app){
    
    // 电机脚本
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

#ifndef PI
	#define PI  3.1415926535f
#endif
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


// mt6701 用户平台自定义回调
void wheel_mag_e_read_reg(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num){
    HAL_I2C_Mem_Read(&hi2c1_handler, mt->addr, reg_addr, 1, reg_buffer, reg_num, 1000);
}

// 使用 hal 库内存读取函数，非常耗时，要占 63% 的 cpu 时间，估计写地址是阻塞的
#if 0
void wheel_mag_e_read_reg_dma(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num){
    
    // GPIOA->BSRR = (uint32_t)GPIO_PIN_15;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(&hi2c1_handler, mt->addr, reg_addr, 1, reg_buffer, reg_num);
    // GPIOA->BRR = (uint32_t)GPIO_PIN_15;

    // 发起 dma 读取失败
    if(status != HAL_OK){
        LTX_LOG_ERRO("mag dma read err: %d, %d\n", status, hi2c1_handler.ErrorCode);
    }
}
#else
// 拆分成发收，虽然有两次中断，但是发地址不阻塞
uint8_t reg_addr_for_tx = 0x03;
uint8_t *reg_read_buf;
volatile uint8_t flag_i2c_wdg = 0;
void wheel_mag_e_read_reg_dma(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num){
    
    // GPIOA->BSRR = (uint32_t)GPIO_PIN_15;
    reg_addr_for_tx = reg_addr;
    reg_read_buf = reg_buffer;

    // 开启看门狗
    flag_i2c_wdg = 3;

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(&hi2c1_handler, mt->addr, &reg_addr_for_tx, 1);

    // 发起 dma 读取失败
    if(status != HAL_OK){
        LTX_LOG_ERRO("mag dma read err: %d, %d\n", status, hi2c1_handler.ErrorCode);
    }
    // GPIOA->BRR = (uint32_t)GPIO_PIN_15;
}
#endif

// 使用 hal 库内存读取函数
#if 0
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
    // 转换角度
    mag_angle = mt6701_trans_angle(&mag_encoder_wheel);
    mag_rad = mt6701_trans_rad(&mag_encoder_wheel);
    // 发起下次读取
    mt6701_read_dma(&mag_encoder_wheel);
    // 发布角度更新事件
    ltx_Topic_publish(&topic_mag_read_over);
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
}
#else
// 拆分成两次中断
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
    // 看门狗标志位 ++
    flag_i2c_wdg ++;
    // GPIOA->BSRR = (uint32_t)GPIO_PIN_15;
    if(HAL_I2C_Master_Receive_DMA(&hi2c1_handler, MT6701_DEFAULT_ADDR, reg_read_buf, 2) != HAL_OK){
        // 强制生成停止位
        // SET_BIT(I2C1->CR1, I2C_CR1_STOP);
        // __HAL_UNLOCK(&hi2c1_handler);
        // hi2c1_handler.State = HAL_I2C_STATE_READY;
        LTX_LOG_DEBG("I2C e3\n");
    }
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
    // 转换角度
    mag_angle = mt6701_trans_angle(&mag_encoder_wheel);
    // mag_rad = mt6701_trans_rad(&mag_encoder_wheel);
    motor_foc.rotater_rad = mt6701_trans_rad(&mag_encoder_wheel);
    // 发起下次读取
    HAL_I2C_Master_Transmit_DMA(&hi2c1_handler, MT6701_DEFAULT_ADDR, &reg_addr_for_tx, 1);
    // 发布角度更新事件
    ltx_Topic_publish(&topic_mag_read_over);
    // GPIOA->BRR = (uint32_t)GPIO_PIN_15;
}
#endif

// 直接在中断中展开，不调用回调
#if 0
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    GPIOA->BSRR = (uint32_t)GPIO_PIN_15;
    // adc1_buffer[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    // adc1_buffer[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    // adc1_buffer[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
    adc1_buffer[1] = ADC1->JDR1;
    adc1_buffer[2] = ADC1->JDR2;
    adc1_buffer[0] = ADC1->JDR3;
    HAL_ADCEx_InjectedStart_IT(&hadc1_handler);
    GPIOA->BRR = (uint32_t)GPIO_PIN_15;

    int32_t mid_adc = adc1_buffer[0];

    i_A = (mid_adc - 2048)*0.8056640625f;
    mid_adc = adc1_buffer[1];
    i_B = (mid_adc - 2048)*0.8056640625f;
    mid_adc = adc1_buffer[2];
    i_C = (mid_adc - 2048)*0.8056640625f;

    /* 计算电流向量模长 */
    vector_I_len = sqrtf(2.0f/3 * (i_A * i_A + i_B * i_B + i_C * i_C));
    GPIOA->BSRR = (uint32_t)GPIO_PIN_15;

    if(vector_I_len > 0.00001f){
        /* 计算电流向量弧度 */
        elec_rad = acosf(vector_I_len / i_A);
        if(i_B < i_C) elec_rad = (2*PI) - elec_rad;
    }
    ltx_Topic_publish(&topic_adc1_update);
    GPIOA->BRR = (uint32_t)GPIO_PIN_15;
}
#endif
