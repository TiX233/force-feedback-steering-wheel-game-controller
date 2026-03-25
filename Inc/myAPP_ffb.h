#ifndef __MYAPP_FFB_H__
#define __MYAPP_FFB_H__

#include "ltx_app.h"

extern struct ltx_App_stu app_ffb;

// 效果类型枚举
typedef enum {
    EFFECT_TYPE_CONSTANT = 1,       // 恒力
    EFFECT_TYPE_RAMP = 2,           // 斜坡力
    EFFECT_TYPE_SQUARE = 3,         // 方波
    EFFECT_TYPE_SINE = 4,           // 正弦波
    EFFECT_TYPE_TRIANGLE = 5,       // 三角波
    EFFECT_TYPE_SAWTOOTH_UP = 6,    // 锯齿波上升
    EFFECT_TYPE_SAWTOOTH_DOWN = 7,  // 锯齿波下降
    EFFECT_TYPE_SPRING = 8,         // 弹簧效果
    EFFECT_TYPE_DAMPER = 9,         // 阻尼效果
    EFFECT_TYPE_INERTIA = 10,       // 惯性效果
    EFFECT_TYPE_FRICTION = 11       // 摩擦效果
} ffb_effect_type_t;

// 效果操作枚举
typedef enum {
    EFFECT_OP_START = 1,            // 开始效果
    EFFECT_OP_START_SOLO = 2,       // 独占开始（停止其他效果）
    EFFECT_OP_STOP = 3              // 停止效果
} ffb_effect_operation_t;

// PID 设备控制枚举
typedef enum {
    // 其实应该是位判断而不是值判断
    PID_CTRL_ENABLE_ACTUATORS = 0x01,   // 使能执行器（电机）
    PID_CTRL_DISABLE_ACTUATORS = 0x02,  // 禁用执行器
    PID_CTRL_STOP_ALL_EFFECTS = 0x04,   // 停止所有效果
    PID_CTRL_DEVICE_RESET = 0x08,       // 设备复位
    PID_CTRL_DEVICE_PAUSE = 0x10,       // 暂停所有效果
    PID_CTRL_DEVICE_CONTINUE = 0x20,    // 继续效果
} ffb_pid_control_t;

// Set Effect Report (ID 3) 结构体
typedef struct {
    uint8_t  effect_block_index;     // 效果块索引（1~40，由主机分配）
    uint8_t  effect_type;            // 效果类型（见 ffb_effect_type_t）
    uint16_t duration;               // 效果持续时间（毫秒）
    uint16_t trigger_repeat_interval;// 触发重复间隔（毫秒）
    uint16_t sample_period;          // 采样周期（毫秒）
    uint16_t start_delay;            // 启动延迟（毫秒）
    uint8_t  gain;                   // 效果增益（0~255，对应实际0~10000）
    uint8_t  trigger_button;         // 触发按钮编号（1~8）
    uint8_t  axes_enable;            // 轴使能（bit0=1 表示X轴有效）
#if 0
    // ai 写的
    uint8_t  direction_enable;       // 方向使能（通常为1）
    uint16_t direction;              // 方向（0~36000，对应0~360度，步长0.01度）
    uint16_t type_specific_offset;   // 类型特定偏移（用于条件效果）
#else
    // openffb 的
    // TODO axes are last bytes in struct if fewer axes are used. use different report if this is not enough anymore!
    uint16_t directionX; // angle (0=0 .. 36000=360deg)
    uint16_t directionY; // angle (0=0 .. 36000=360deg)
#endif
} ffb_set_effect_report_t;

// Set Envelope Report (ID 4) 结构体
typedef struct {
    uint8_t  effect_block_index;     // 效果块索引
    uint16_t attack_level;           // 起振电平（0~32767）
    uint16_t fade_level;             // 衰减电平（0~32767）
    uint32_t attack_time;            // 起振时间（毫秒）
    uint32_t fade_time;              // 衰减时间（毫秒）
} ffb_set_envelope_report_t;

// Set Condition Report (ID 5) 结构体（用于弹簧、阻尼、惯性、摩擦）
typedef struct {
    uint8_t  effect_block_index;     // 效果块索引
    uint8_t  parameter_block_offset; // 参数块偏移（0~3）
    uint8_t  type_specific_offset;   // 类型特定偏移（指定条件类型，通常为0或1）
    int16_t  cp_offset;              // 中心点偏移（-32767~32767）
    int16_t  positive_coefficient;   // 正系数（-32767~32767）
    int16_t  negative_coefficient;   // 负系数（-32767~32767）
    uint16_t positive_saturation;    // 正饱和（0~32767）
    uint16_t negative_saturation;    // 负饱和（0~32767）
    uint16_t dead_band;              // 死区（0~32767）
} ffb_set_condition_report_t;

// Set Periodic Report (ID 6) 结构体（周期性波形）
typedef struct {
    uint8_t  effect_block_index;     // 效果块索引
    uint16_t magnitude;              // 幅值（0~32767）
    int16_t  offset;                 // 偏移量（-32767~32767）
    uint16_t phase;                  // 相位（0~35999，对应0~360度，步长0.01度）
    uint32_t period;                 // 周期（毫秒）
} ffb_set_periodic_report_t;

// Set Constant Force Report (ID 7) 结构体（恒力）
typedef struct {
    uint8_t  effect_block_index;     // 效果块索引
    int16_t  magnitude;              // 力大小（-32767~32767）
} ffb_set_constant_force_report_t;

// Set Ramp Force Report (ID 8) 结构体（斜坡力）
typedef struct {
    uint8_t  effect_block_index;     // 效果块索引
    int16_t  ramp_start;             // 起始力（-32767~32767）
    int16_t  ramp_end;               // 结束力（-32767~32767）
} ffb_set_ramp_force_report_t;

// Effect Operation Report (ID 9) 结构体（控制效果启停）
typedef struct {
    uint8_t effect_block_index;      // 效果块索引
    uint8_t operation;               // 操作（见 ffb_effect_operation_t）
    uint8_t loop_count;              // 循环次数（0表示无限）
} ffb_effect_operation_report_t;

// PID Block Free Report (ID 10) 结构体（释放效果块）
typedef struct {
    uint8_t effect_block_index;      // 效果块索引
} ffb_pid_block_free_report_t;

// PID Device Control Report (ID 11) 结构体（设备级控制）
typedef struct {
    uint8_t control;                 // 控制命令（见 ffb_pid_control_t）
} ffb_pid_device_control_report_t;

// Device Gain Report (ID 12) 结构体（全局增益）
typedef struct {
    uint8_t gain;                    // 全局增益（0~255，对应实际0~10000）
} ffb_device_gain_report_t;

// 解析游戏下发来的力反馈数据
void ffb_parse_data(uint8_t *data, uint32_t len);



// 效果块最大数量
#define MAX_EFFECT_BLOCKS       8

// 效果状态
typedef enum {
    EFFECT_STATE_FREE = 0,      // 空闲，未使用
    EFFECT_STATE_CONFIGURED,    // 已配置参数，未启动
    EFFECT_STATE_PLAYING,       // 正在播放
    EFFECT_STATE_PAUSED         // 已暂停
} ffb_effect_state_t;

// 效果参数联合体（不同效果类型使用不同结构）
typedef union {
    // 恒力
    struct { int16_t magnitude; } constant;
    // 斜坡力
    struct { int16_t start; int16_t end; } ramp;
    // 周期波
    struct { uint16_t magnitude; int16_t offset; uint16_t phase; uint32_t period; } periodic;
    // 条件效果（弹簧、阻尼等）
    struct {
        int16_t cp_offset;
        int16_t positive_coeff;
        int16_t negative_coeff;
        uint16_t positive_sat;
        uint16_t negative_sat;
        uint16_t dead_band;
    } condition;
    // 通用参数（用于未实现的类型）
    struct { uint8_t raw[32]; } generic;
} ffb_effect_params_t;

// 效果块结构
typedef struct {
    ffb_effect_state_t state;           // 当前状态
    uint8_t type;                   // 效果类型
    uint16_t duration;              // 剩余持续时间（ms），0表示无限
    uint16_t gain;                  // 效果增益（0~255）
    uint8_t trigger_button;         // 触发按钮
    uint16_t direction;             // 方向（0~36000）
    ffb_effect_params_t params;         // 特定参数
    // 包络参数（可选）
    uint16_t attack_level;
    uint16_t fade_level;
    uint32_t attack_time;           // 起振时间（ms）
    uint32_t fade_time;             // 衰减时间（ms）
    uint32_t start_time;            // 开始时间戳（用于时间控制）
    uint32_t elapsed_time;          // 已运行时间
} ffb_effect_block_t;




#endif // __MYAPP_FFB_H__
