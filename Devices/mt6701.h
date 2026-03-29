/**
 * @file mt6701.h
 * @author realTiX
 * @brief 用于 mt6701 磁编码器，i2c 速度不能超过 1000k，spi 速度不能超过 15M
 * @version 0.3
 * @date 2026-02-25 (0.1, 初步完成)
 *       2026-03-08 (0.2, 增加弧度偏置功能)
 *       2026-03-18 (0.3, 适配 16bit spi 模式)
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef __MT6701_H__
#define __MT6701_H__

#include "main.h"

// 不是说 spi 只能使用 16bit 模式，而是使用 spi 情况下如果要用 16bit 就开这个宏
#define MT6701_USESPI_16BIT

// i2c 默认设备地址
#define MT6701_DEFAULT_ADDR     (0x06 << 1)

struct mt6701_stu {
    uint8_t addr;
    float rad_offset; // 对弧度设置偏置
    uint16_t data_row; // 原始数据
    uint8_t data_buffer[3];

    // 以读写寄存器的形式进行封装，兼顾 i2c 与 spi
    // void (*write_reg)(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num);
    void (*read_reg)(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num);
    void (*read_reg_dma)(struct mt6701_stu *mt, uint8_t reg_addr, uint8_t *reg_buffer, uint8_t reg_num);
};

// int mt6701_init(struct mt6701_stu *mt);
void mt6701_read(struct mt6701_stu *mt); // 阻塞方式读取角度寄存器
void mt6701_read_dma(struct mt6701_stu *mt); // dma 方式读取角度寄存器

float mt6701_trans_angle(struct mt6701_stu *mt); // 转换读出来的值为角度
float mt6701_trans_rad(struct mt6701_stu *mt); // 转换读出来的值为弧度，会叠加偏置值

void mt6701_set_rad_offset(struct mt6701_stu *mt, float offset); // 设置弧度换算偏置

#endif // __MT6701_H__
