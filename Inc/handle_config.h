#ifndef __HANDLE_CONFIG_H__
#define __HANDLE_CONFIG_H__

#include "main.h"

#define _HCFG_start_addr    0x0805F800
#define _HCFG_flash_size    2048

#define _HCFG_sector        191

#define _HCFG_magic         0x82568256

#define _HCFG_default_size  10*4

// sec191，手柄配置，存储内容如下：
struct handle_config {
    uint32_t magic;
    uint32_t cfg_size; // size 不包括前三个变量
    // 异或校验值
    uint32_t check_value;

    uint32_t config_status;

    uint32_t trigger_left_adc_max;
    uint32_t trigger_left_adc_min;
    
    uint32_t trigger_right_adc_max;
    uint32_t trigger_right_adc_min;
    
    uint32_t trigger_mid_adc_max;
    uint32_t trigger_mid_adc_min;
    
    // 方向盘单边旋转最大圈数 x100
    uint32_t wheel_max_turn_x100;
    // 方向盘中点
    uint32_t wheel_encoder_mid;

    // 电机较准值
    union {
        float f;
        uint32_t u;
     }wheel_motor_accurate_value;
};

extern struct handle_config _cfg_of_handle;

void hcfg_read_from_flash(void);
uint8_t hcfg_check(void);
void hcfg_pack(void);
void hcfg_erase(void);
uint8_t hcfg_write_into_flash(struct handle_config *cfg);

#endif // __HANDLE_CONFIG_H__
