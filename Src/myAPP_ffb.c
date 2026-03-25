#include "myApp_ffb.h"
#include "ltx.h"
#include "ltx_app.h"
#include "ltx_param.h"
#include "ltx_log.h"
#include "ltx_cmd.h"
#include "ltx_script.h"
#include "myAPP_motor.h"
#include "myAPP_button.h"

struct ltx_Script_stu script_ffb;
void script_cb_ffb(struct ltx_Script_stu *script);

int myApp_ffb_init(struct ltx_App_stu *app){

    ltx_Script_init(&script_ffb, script_cb_ffb);

    return 0;
}

int myApp_ffb_pause(struct ltx_App_stu *app){

    ltx_Script_pause(&script_ffb);
    
    return 0;
}

int myApp_ffb_resume(struct ltx_App_stu *app){

    ltx_Script_resume(&script_ffb, 0);

    return 0;
}

int myApp_ffb_destroy(struct ltx_App_stu *app){

    ltx_Script_pause(&script_ffb);

    // free...

    return 0;
}


struct ltx_App_stu app_ffb = {
    .is_initialized = 0,
    .status = ltx_App_status_pause,
    .name = "ffb",

    .init = myApp_ffb_init,
    .pause = myApp_ffb_pause,
    .resume = myApp_ffb_resume,
    .destroy = myApp_ffb_destroy,

    .task_list = NULL,
    
    .next = NULL,
};

/* 
理论上应该是要设置一个效果池，支持多少个效果并行就设置多大，然后填在报告描述符中
hid 最多支持 40 个，所以最大的情况下要准备一个大小为 40 的效果池
每个效果块可以支持任意一种效果力，游戏通过配置效果块的各种参数来输出不同的效果
但是抓包欧卡2和神力科莎发现只会用到一个效果块，也只会发送一种力，常量力
那么应该是游戏直接输出总力矩给下位机
猜测可能性：
1、游戏比较老所以只支持流式控制，神力科莎不知道，欧卡2的路面颠簸增益条是灰色的，说明游戏是支持但认为设备不支持
2、游戏不认识这款设备所以用最保守的方式输出，或者游戏与某些商业方向盘签约独占某些效果
3、报告描述符有误
4、get report 没写完整

地平线5 只会开游戏的时候发几个 0xb 报文，不会输出力反馈数据，
而且如果不是商业方向盘的 vid/pid 它甚至都不接纳输入
*/

// 暂时只设置一个常量力效果块
struct {
    uint8_t flag_enable;
    ffb_effect_params_t effect;
} ffb_effect_pool[1];

uint8_t ffb_global_gain = 0; // 全局增益
uint8_t ffb_flag_output_state = 0; // 是否输出


// 解析 Set Effect Report (ID 3) 设置效果参数（仅存储，不立即执行）
// 数据格式（根据描述符）：
// 字节 0:      Report ID (3)
// 字节 1:      Effect Block Index
// 字节 2:      Effect Type
// 字节 3-4:    Duration (16位小端)
// 字节 5-6:    Trigger Repeat Interval
// 字节 7-8:    Sample Period
// 字节 9-10:   Start Delay
// 字节 11:     Gain
// 字节 12:     Trigger Button
// 字节 13:     Axes Enable
// 字节 14:     Direction Enable
// 字节 15-16:  Direction
// 字节 17-18:  Type Specific Block Offset
// 后续可能有填充字节，忽略
static void _ffb_parse_set_effect(const uint8_t *data, uint32_t len) {
    if (len < 18) return; // 确保数据长度足够
    ffb_set_effect_report_t report;
    report.effect_block_index = data[1];                     // 效果块索引
    report.effect_type = data[2];                            // 效果类型
    report.duration = (uint16_t)data[3] | ((uint16_t)data[4] << 8);           // 持续时间
    report.trigger_repeat_interval = (uint16_t)data[5] | ((uint16_t)data[6] << 8);
    report.sample_period = (uint16_t)data[7] | ((uint16_t)data[8] << 8);
    report.start_delay = (uint16_t)data[9] | ((uint16_t)data[10] << 8);
    report.gain = data[11];                                  // 增益
    report.trigger_button = data[12];                        // 触发按钮
    report.axes_enable = data[13];                           // 轴使能标志
#if 0
    report.direction_enable = data[14];                      // 方向使能标志
    report.direction = (uint16_t)data[15] | ((uint16_t)data[16] << 8);        // 方向
    report.type_specific_offset = (uint16_t)data[17] | ((uint16_t)data[18] << 8); // 类型特定偏移
#endif
    // ffb_set_effect(&report);

    // 设置效果参数（仅存储，不立即执行）
    // 根据 report.effect_block_index 选择效果槽（通常为1~40）
    // 根据 report.effect_type 确定效果类型
    // 将持续时间、增益、方向等保存到对应的效果槽结构体中
    // 此时效果尚未启动，需要等待 Effect Operation Report 的 START 命令

}

// 解析 Set Envelope Report (ID 4) 设置包络参数（起振/衰减）
// 数据格式：
// 字节 0:      Report ID (4)
// 字节 1:      Effect Block Index
// 字节 2-3:    Attack Level (16位小端)
// 字节 4-5:    Fade Level
// 字节 6-9:    Attack Time (32位小端)
// 字节 10-13:  Fade Time
static void _ffb_parse_set_envelope(const uint8_t *data, uint32_t len) {
    if (len < 13) return;
    ffb_set_envelope_report_t report;
    report.effect_block_index = data[1];
    report.attack_level = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    report.fade_level = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    report.attack_time = (uint32_t)data[6] | ((uint32_t)data[7] << 8) |
                         ((uint32_t)data[8] << 16) | ((uint32_t)data[9] << 24);
    report.fade_time = (uint32_t)data[10] | ((uint32_t)data[11] << 8) |
                       ((uint32_t)data[12] << 16) | ((uint32_t)data[13] << 24);
    // ffb_set_envelope(&report);

    // 设置包络参数（起振/衰减）
    // 与效果块关联，存储起振电平、时间等参数
}

// 解析 Set Condition Report (ID 5) 设置条件参数（弹簧、阻尼等）
// 数据格式：
// 字节0: Report ID (5)
// 字节 1:      Effect Block Index
// 字节 2:      Parameter Block Offset (低6位有效)
// 字节 3:      Type Specific Block Offset (低2位有效，高6位填充)
// 字节 4-5:    CP Offset (16位有符号)
// 字节 6-7:    Positive Coefficient
// 字节 8-9:    Negative Coefficient
// 字节 10-11:  Positive Saturation (16位无符号)
// 字节 12-13:  Negative Saturation
// 字节 14-15:  Dead Band
static void _ffb_parse_set_condition(const uint8_t *data, uint32_t len) {
    if (len < 15) return;
    ffb_set_condition_report_t report;
    report.effect_block_index = data[1];
    report.parameter_block_offset = data[2] & 0x3F;          // 取低6位
    report.type_specific_offset = (data[3] >> 0) & 0x03;     // 取低2位（实例选择）
    report.cp_offset = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
    report.positive_coefficient = (int16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
    report.negative_coefficient = (int16_t)((uint16_t)data[8] | ((uint16_t)data[9] << 8));
    report.positive_saturation = (uint16_t)data[10] | ((uint16_t)data[11] << 8);
    report.negative_saturation = (uint16_t)data[12] | ((uint16_t)data[13] << 8);
    report.dead_band = (uint16_t)data[14] | ((uint16_t)data[15] << 8);
    // ffb_set_condition(&report);

    // 设置条件参数（弹簧、阻尼等）
    // 根据参数块偏移（通常为0或1）选择是弹簧还是阻尼
    // 存储中心点偏移、正负系数、饱和、死区等
}

// 解析 Set Periodic Report (ID 6) 设置周期性波形参数
// 数据格式：
// 字节 0:    Report ID (6)
// 字节 1:    Effect Block Index
// 字节 2-3:  Magnitude (16位小端)
// 字节 4-5:  Offset (16位有符号)
// 字节 6-7:  Phase (16位无符号)
// 字节 8-11: Period (32位小端)
static void _ffb_parse_set_periodic(const uint8_t *data, uint32_t len) {
    if (len < 11) return;
    ffb_set_periodic_report_t report;
    report.effect_block_index = data[1];
    report.magnitude = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    report.offset = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
    report.phase = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
    report.period = (uint32_t)data[8] | ((uint32_t)data[9] << 8) |
                    ((uint32_t)data[10] << 16) | ((uint32_t)data[11] << 24);
    // ffb_set_periodic(&report);

    // 设置周期性波形参数
    // 存储幅值、偏移、相位、周期
}

// 解析 Set Constant Force Report (ID 7) 设置恒力参数
// 数据格式：
// 字节 0:   Report ID (7)
// 字节 1:   Effect Block Index
// 字节 2-3: Magnitude (16位有符号)
static void _ffb_parse_set_constant_force(const uint8_t *data, uint32_t len) {
    if (len < 4) return;
    ffb_set_constant_force_report_t report;
    report.effect_block_index = data[1];
    report.magnitude = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8)); // 正值表示产生逆时针力矩
    // ffb_set_constant_force(&report);

    // 设置恒力参数
    // 存储力的大小
    // LTX_LOG_FMT("F:%d\n", report.magnitude);
    ffb_effect_pool[0].effect.constant.magnitude = report.magnitude;
}

// 解析 Set Ramp Force Report (ID 8) 设置斜坡力参数
// 数据格式：
// 字节 0:      Report ID (8)
// 字节 1:      Effect Block Index
// 字节 2-3:    Ramp Start (16位有符号)
// 字节 4-5:    Ramp End (16位有符号)
static void _ffb_parse_set_ramp_force(const uint8_t *data, uint32_t len) {
    if (len < 5) return;
    ffb_set_ramp_force_report_t report;
    report.effect_block_index = data[1];
    report.ramp_start = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
    report.ramp_end = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
    // ffb_set_ramp_force(&report);

    // 设置斜坡力参数
    // 存储起始力和结束力
}

// 解析 Effect Operation Report (ID 9) 效果操作（启停）
// 数据格式：
// 字节 0: Report ID (9)
// 字节 1: Effect Block Index
// 字节 2: Operation (1:Start, 2:StartSolo, 3:Stop)
// 字节 3: Loop Count
static void _ffb_parse_effect_operation(const uint8_t *data, uint32_t len) {
    if (len < 4) return;
    ffb_effect_operation_report_t report;
    report.effect_block_index = data[1];
    report.operation = data[2];
    report.loop_count = data[3];
    // ffb_effect_operation(&report);

    // 效果操作（启停）
    switch(report.operation){
        case EFFECT_OP_START:
            // 启动 report->effect_block_index 对应的效果
            // 使用之前存储的参数开始输出力
            // LTX_LOG_STR("\t\tSTART\n");
            ffb_effect_pool[0].flag_enable = 1;
            break;
        case EFFECT_OP_START_SOLO:
            // 停止所有其他效果，然后启动该效果
            // LTX_LOG_STR("\t\tSOLO\n");
            ffb_effect_pool[0].flag_enable = 1;
            break;
        case EFFECT_OP_STOP:
            // 停止该效果
            // LTX_LOG_STR("\t\tSTOP\n");
            ffb_effect_pool[0].flag_enable = 0;
            break;
    }
}

// 解析 PID Block Free Report (ID 10, 0xa) 释放效果块（释放资源）
// 数据格式：
// 字节 0: Report ID (10)
// 字节 1: Effect Block Index
static void _ffb_parse_pid_block_free(const uint8_t *data, uint32_t len) {
    if (len < 2) return;
    ffb_pid_block_free_report_t report;
    report.effect_block_index = data[1];
    // ffb_pid_block_free(&report);

    // 释放效果块（释放资源）
    // 释放 report.effect_block_index 占用的资源
}

// 解析 PID Device Control Report (ID 11, 0xb) 设备级控制（如使能电机、复位等）
// 数据格式：
// 字节 0: Report ID (11)
// 字节 1: Control (1~6)
static void _ffb_parse_pid_device_control(const uint8_t *data, uint32_t len) {
    if (len < 2) return;
    ffb_pid_device_control_report_t report;
    report.control = data[1];
    // ffb_pid_device_control(&report);

    // 设备级控制（如使能电机、复位等）
    switch(report.control){
        case PID_CTRL_ENABLE_ACTUATORS:
            // 使能电机驱动
            // ffb_flag_output_state = 1;
            ffb_effect_pool[0].flag_enable = 1;
            break;
        case PID_CTRL_DISABLE_ACTUATORS:
            // 禁用电机驱动
            // ffb_flag_output_state = 0;
            ffb_effect_pool[0].flag_enable = 0;
            break;
        case PID_CTRL_STOP_ALL_EFFECTS:
            // 立即停止所有效果
            // ffb_flag_output_state = 0;
            ffb_effect_pool[0].flag_enable = 0;
            break;
        case PID_CTRL_DEVICE_RESET:
            // 复位设备，清空所有效果和状态
            // ffb_flag_output_state = 0;
            ffb_global_gain = 0;
            ffb_effect_pool[0].effect.constant.magnitude = 0;
            ffb_effect_pool[0].flag_enable = 0;
            break;
        case PID_CTRL_DEVICE_PAUSE:
            // 暂停所有效果（保留参数）
            // ffb_flag_output_state = 0;
            ffb_effect_pool[0].flag_enable = 0;
            break;
        case PID_CTRL_DEVICE_CONTINUE:
            // 继续效果
            ffb_flag_output_state = 1;
            ffb_effect_pool[0].flag_enable = 1;
            break;
    }
}

// 解析 Device Gain Report (ID 12, 0xc) 设置全局增益
// 数据格式：
// 字节 0: Report ID (12)
// 字节 1: Gain (0~255)
static void _ffb_parse_device_gain(const uint8_t *data, uint32_t len) {
    if (len < 2) return;
    ffb_device_gain_report_t report;
    report.gain = data[1];
    // ffb_device_gain(&report);

    // 设置全局增益
    // report.gain 范围 0~255，对应实际增益 0~10000
    // 计算实际增益比例：gain_percent = (report.gain * 10000) / 255
    // 将此比例应用到所有效果力的输出上
    
    // LTX_LOG_FMT("\t\tG:%d\n", report.gain);
    ffb_global_gain = report.gain;
}

// 解析 usb 数据
void ffb_parse_data(uint8_t *data, uint32_t len){
    switch(data[0]){
        case 3:  // Set Effect Report
            _ffb_parse_set_effect(data, len);
            break;
        case 4:  // Set Envelope Report
            _ffb_parse_set_envelope(data, len);
            break;
        case 5:  // Set Condition Report
            _ffb_parse_set_condition(data, len);
            break;
        case 6:  // Set Periodic Report
            _ffb_parse_set_periodic(data, len);
            break;
        case 7:  // Set Constant Force Report
            _ffb_parse_set_constant_force(data, len);
            break;
        case 8:  // Set Ramp Force Report
            _ffb_parse_set_ramp_force(data, len);
            break;
        case 9:  // Effect Operation Report
            _ffb_parse_effect_operation(data, len);
            break;
        case 10: // PID Block Free Report
            _ffb_parse_pid_block_free(data, len);
            break;
        case 11: // PID Device Control
            _ffb_parse_pid_device_control(data, len);
            break;
        case 12: // Device Gain Report
            _ffb_parse_device_gain(data, len);
            break;
    }
}

#define MAX_OUTPUT_IQ   0.7f

void script_cb_ffb(struct ltx_Script_stu *script){
    float output_Iq = 0;
    // if(ffb_flag_output_state){
        if(ffb_effect_pool[0].flag_enable){
            output_Iq = ((float)ffb_effect_pool[0].effect.constant.magnitude / 0xFFFF) * ((float)ffb_global_gain/0xff) * -MAX_OUTPUT_IQ;
        }
    // }
    
    int32_t wheel_overrun = handle_wheel_get_overrun(&handle_wheel_data);
#if 0
    if(wheel_overrun > 0){ // 逆时针范围超限
        // 产生顺时针方向的力矩
        float I_q = 0.001f * wheel_overrun;
        // I_q = I_q > 0.7f ? 0.7f : I_q;
        output_Iq += I_q;

    }else if(wheel_overrun < 0){ // 顺时针范围超限
        // 产生逆时针方向的力矩
        float I_q = 0.001f * wheel_overrun;
        // I_q = I_q < -0.7f ? -0.7f : I_q;
        output_Iq += I_q;
    }else {
        // 没有超限则关闭限位力矩
        // output_Iq += 0;
    }
#endif
    output_Iq += 0.001f * wheel_overrun;

    output_Iq = output_Iq > MAX_OUTPUT_IQ ? MAX_OUTPUT_IQ : (output_Iq < -MAX_OUTPUT_IQ ? -MAX_OUTPUT_IQ : output_Iq);

    motor_foc.target_I_q = output_Iq;

    ltx_Script_next_step_delay(script, 0, 1);
}

