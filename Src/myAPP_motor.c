#include "myAPP_motor.h"
#include "ltx.h"
#include "ltx_app.h"
#include "ltx_log.h"
#include "ltx_script.h"
#include "ltx_lock.h"
#include "math.h"
#include "mt6701.h"
#include "ltx_foc1.h"
#include "ltx_foc2.h"
#include "ltx_pid.h"


// 速度环脚本回调
void script_cb_speed(struct ltx_Script_stu *script);

// 磁编码器回调
void wheel_mag_e_read_reg(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num);
void wheel_mag_e_read_reg_dma(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num);

// 磁编码器对象
struct mt6701_stu mag_encoder_wheel = {
    .addr = MT6701_DEFAULT_ADDR,
    .read_reg = wheel_mag_e_read_reg,
    .read_reg_dma = wheel_mag_e_read_reg_dma,
};

// 电机 foc 对象
#if 0
struct ltx_foc1_stu motor_foc = {
    .flag_is_inited = 0,

    .pole_pairs = 7, // 极对数 7

    .vector_I_len = 0, // 当前输出电流向量模长
    .vector_I_rad = 0, // 当前输出电流向量弧度，[0, 2pi)

    .vector_V_len = 0, // 电压向量模长百分比，[0, 1]
    .vector_V_rad = 0, // 电压向量弧度，[0, 2pi)

    .pi_theta = { // 电流与转子夹角环
        .kp = 0.01f,
        .ki = 2.0f,
        .error_prev = 0,
        .theta = 0,
    },
    .pi_amplitude = { // 电流模长环
        .kp = 0.8f,
        .ki = 1.3f,
        .integral = 0,
        .limit_u = 0.5,
        .limit_d = 0,
    },

    // 三相电压输出
    .v_outputABC[0] = 0,
    .v_outputABC[1] = 0,
    .v_outputABC[2] = 0,
    // 三相采集电流
    .i_A = 0,
    .i_B = 0,
    .i_C = 0,

    .target_I_len = 0, // 目标 输出电流向量模长
    .target_I_rad = PI/2, // 目标 输出电流向量与转子夹角

    .diff_len = 0,
    .diff_rad = PI/2,

    .rotor_rad = 0, // 转子弧度，[0, 2pi)

    .dt = 0.05f, // foc 算法调用间隔，单位默认毫秒
};
#endif

struct ltx_foc2_stu motor_foc = {

    .flag_is_inited = 0,

    .pole_pairs = 7,

    .I_q = 0.0f,
    .I_d = 0.0f,
    
    .V_aplha = 0.0f,
    .V_beta = 0.0f,

    .pi_q = {
        .kp = 0.86f,
        .ki = 0.5f,
        .integral = 0.0f,
        // .limit_u = 3.0f,
        // .limit_d = -3.0f,
        .limit_u = 0.5f,
        .limit_d = -0.5f,
    },
    .pi_d = {
        .kp = 0.17f,
        .ki = 0.15f,
        .integral = 0.0f,
        // .limit_u = 3.0f,
        // .limit_d = -3.0f,
        .limit_u = 0.5f,
        .limit_d = -0.5f,
    },

    .v_outputABC[0] = 0,
    .v_outputABC[1] = 0,
    .v_outputABC[2] = 0,

    .i_A = 0.0f,
    .i_B = 0.0f,
    .i_C = 0.0f,

    .target_I_q = 0.0f,
    .target_I_d = 0.0f,

    .diff_I_q = 0.0f,
    .diff_I_d = 0.0f,

    .rotor_rad = 0.0f,

    .dt = 0.05f,
};

// 磁编码器所读出来的机械角度与弧度
float mag_angle;
// float mag_rad; // 直接用 foc 对象里的成员变量
// adc1 原始值
uint32_t adc1_buffer[3];

// 电机脚本
// struct ltx_Script_stu script_motor;
// 速度环脚本
struct ltx_Script_stu script_speed;
// 速度环 pi 对象
struct ltx_pid_pi_stu pi_speed = {
    .kp = 0.1f,
    .ki = 0.1f,
    .integral = 0,
    .limit_u = 0.5f, // 电流模长上限
    .limit_d = -0.5f, // 电流模长下限
};


// 磁编码器读取完成事件话题
struct ltx_Topic_stu topic_mag_read_over = _LTX_TOPIC_DEAFULT_CONFIG(topic_mag_read_over);
// 电流 adc 更新事件话题
struct ltx_Topic_stu topic_adc1_update = _LTX_TOPIC_DEAFULT_CONFIG(topic_adc1_update);

int myApp_motor_init(struct ltx_App_stu *app){
    
    // 电机脚本
    ltx_Script_init(&script_speed, script_cb_speed);

    // 发起 dma 读取磁编码器数据
    // mt6701_read_dma(&mag_encoder_wheel);

    return 0;
}

int myApp_motor_pause(struct ltx_App_stu *app){

    ltx_Script_pause(&script_speed);
    
    return 0;
}

int myApp_motor_resume(struct ltx_App_stu *app){

    ltx_Script_resume(&script_speed, 0);
    
    return 0;
}

int myApp_motor_destroy(struct ltx_App_stu *app){
    
    ltx_Script_pause(&script_speed);
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

// 逆时针为正转，正值
float rpm_target = 60.0f; // 目标转速
float rpm_real; // 实际转速
float last_mag_rad; // 上次的磁编码读数

float rpm_filtered = 0.0f;
float rpm_lpf_coeff = 0.1f; // 低通滤波系数

// 速度环脚本回调
void script_cb_speed(struct ltx_Script_stu *script){

    float speed_calculate;

    ltx_Script_next_step_delay(script, 0, 1); // 1ms 后再次执行此脚本

    // _LTX_IRQ_DISABLE();
    speed_calculate = motor_foc.rotor_rad - last_mag_rad;
    last_mag_rad = motor_foc.rotor_rad;
    // _LTX_IRQ_ENABLE();

    if(speed_calculate > PI){
        speed_calculate -= 2*PI;
    }else if(speed_calculate < -PI){
        speed_calculate += 2*PI;
    }
    rpm_real = speed_calculate / (2*PI) * 60000.0f;

    // 一阶低通滤波
    rpm_filtered = rpm_filtered * (1 - rpm_lpf_coeff) + rpm_real * rpm_lpf_coeff;

    // ltx_foc1_set_target_len(motor_foc, ltx_pid_pi_update(&pi_speed, rpm_target - rpm_filtered, 0.001f));
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

// 不使用 hal 库内存读取函数，因为写地址部分是阻塞的
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
    float rad_trans;
    // 转换角度
    // mag_angle = mt6701_trans_angle(&mag_encoder_wheel);
    rad_trans = mt6701_trans_rad(&mag_encoder_wheel);
    // 发起下次读取
    HAL_I2C_Master_Transmit_DMA(&hi2c1_handler, MT6701_DEFAULT_ADDR, &reg_addr_for_tx, 1);
    _LTX_IRQ_DISABLE();
    motor_foc.rotor_rad = rad_trans;
    _LTX_IRQ_ENABLE();
    // 发布角度更新事件
    ltx_Topic_publish(&topic_mag_read_over);
    // GPIOA->BRR = (uint32_t)GPIO_PIN_15;
}
#endif

// 直接在中断中展开，不调用回调
#if 0
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc){
    // ...
}
#endif
int16_t adc1_offset[3];
void ADC1_2_IRQHandler(void){
    // 检查是否为注入组转换结束中断（JEOC）
    if ((ADC1->SR & ADC_FLAG_JEOC) && (ADC1->CR1 & ADC_IT_JEOC)){

    GPIOA->BSRR = (uint32_t)GPIO_PIN_15;
        // 读 adc 并转换三相电流耗时 0.74us，怎么硬件浮点数还这么慢，离谱

        // 读取 adc 数值
        adc1_buffer[1] = ADC1->JDR1;
        adc1_buffer[2] = ADC1->JDR2;
        adc1_buffer[0] = ADC1->JDR3;

        // 换算 adc 值为电流
        ltx_bldc_trans_current_u(motor_foc, adc1_buffer[0]);
        ltx_bldc_trans_current_v(motor_foc, adc1_buffer[1]);
        ltx_bldc_trans_current_w(motor_foc, adc1_buffer[2]);

    GPIOA->BRR = (uint32_t)GPIO_PIN_15;
        // 运行 foc1 算法，耗时 8.7us
        // ltx_foc1_algorithm_1(&motor_foc);

        // 运行 foc2 算法，耗时 4.9us
        ltx_foc2_algorithm(&motor_foc);
        
    GPIOA->BSRR = (uint32_t)GPIO_PIN_15;
        // 计算输出以及发布事件耗时 0.56us

        // 根据是否初始化来决定是否要将 foc 算法得出的三相电压值输出
        if(motor_foc.flag_is_inited){
            ltx_bldc_set_duty_u(motor_foc, motor_foc.v_outputABC[0]);
            ltx_bldc_set_duty_v(motor_foc, motor_foc.v_outputABC[1]);
            ltx_bldc_set_duty_w(motor_foc, motor_foc.v_outputABC[2]);
        }

        // 发布采样完成事件
        ltx_Topic_publish(&topic_adc1_update);

    GPIOA->BRR = (uint32_t)GPIO_PIN_15;
        // 启动下一次注入组采样（直接寄存器操作）
        // 清除JEOC标志（写 1 清零，注意原代码在最后统一清除，但建议尽早清除避免重复触发）
        ADC1->SR = ~ADC_FLAG_JEOC; // 仅清除JEOC，其他位不受影响
        // 确保JEOC中断使能（若已使能可省略，但安全起见可再次使能）
        ADC1->CR1 |= ADC_IT_JEOC;
        // 软件触发注入组转换（需同时置位 JSWSTART 和 JEXTTRIG）
        ADC1->CR2 |= (ADC_CR2_JSWSTART | ADC_CR2_JEXTTRIG);
    }
}
