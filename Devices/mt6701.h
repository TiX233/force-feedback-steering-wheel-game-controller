/**
 * @file mt6701.h
 * @author realTiX
 * @brief 用于 mt6701 磁编码器
 * @version 0.1
 * @date 2026-02-25 (0.1, 初步完成)
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef __MT6701_H__
#define __MT6701_H__

#include "main.h"

// 默认设备地址
#define MT6701_DEFAULT_ADDR     (0x06 << 1)

struct mt6701_stu {
    uint8_t addr;
    uint8_t data_buffer[2];

    // void (*write_reg)(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num);
    void (*read_reg)(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num);
    void (*read_reg_dma)(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num);
};

// int mt6701_init(struct mt6701_stu *mt);
void mt6701_read(struct mt6701_stu *mt); // 阻塞方式读取角度寄存器
void mt6701_read_dma(struct mt6701_stu *mt); // dma 方式读取角度寄存器

float mt6701_trans_angle(struct mt6701_stu *mt); // 转换读出来的值为角度
float mt6701_trans_rad(struct mt6701_stu *mt); // 转换读出来的值为弧度

#endif // __MT6701_H__
