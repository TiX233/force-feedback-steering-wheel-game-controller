/**
 * @file ws2812.h
 * @author realTiX
 * @brief ws2812 spi(dma) driver
 *        spi speed: 4 MBits/s
 *        spi first bit: MSB
 * @version 0.2
 * @date 2025-01-18 (0.1)
 *       2026-02-14 (0.2, 改进为平台无关)
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __WS2812_H__
#define __WS2812_H__

#include "main.h"

// 显存数组样例：
// 一个颜色 8 个码元(ws2812 比特)，3 种颜色，一个码元 4 个 spi 比特，一个字节 8 个比特，
// rst 信号可以不需要存储与发送，只需要将最后一位置为 0，spi 总线 mosi 会保持在 0，确保超过 rst 所需时间之前不要立即刷新下一帧就行
// uint8_t ws2812Buffer[灯珠数量 * 8 * 3 * 4 / 8 + WS2812_RST_BYTES];

// #define WS2812_RST_BYTES        158
#define WS2812_RST_BYTES        1

// 计算显存数组大小宏
#define WS2812_BUFFER_SIZE(lemp_num)    (lemp_num*12+WS2812_RST_BYTES)

#define WS2812_0_CODE           0x8     // 0b1000
#define WS2812_1_CODE           0xE     // 0b1110

#define WS2812_00_CODE          0x88
#define WS2812_01_CODE          0x8E
#define WS2812_10_CODE          0xE8
#define WS2812_11_CODE          0xEE

#define WS2812_0000_0000_CODE   0x88888888
#define WS2812_1111_1111_CODE   0xEEEEEEEE

typedef struct {
    uint32_t green;
    uint32_t red;
    uint32_t blue;
} ws2812_Color_stu;

/* 废弃
struct ws2812_Color_stu {
    // ws2812: GRB 顺序，高码先行
    // spi：4 Bits 作 wa2812 1 位，高位先行
    // 结构体：低地址先行，放高码
    uint32_t G0:4;
    uint32_t G1:4;
    uint32_t G2:4;
    uint32_t G3:4;
    uint32_t G4:4;
    uint32_t G5:4;
    uint32_t G6:4;
    uint32_t G7:4;
    
    uint32_t R0:4;
    uint32_t R1:4;
    uint32_t R2:4;
    uint32_t R3:4;
    uint32_t R4:4;
    uint32_t R5:4;
    uint32_t R6:4;
    uint32_t R7:4;

    uint32_t B0:4;
    uint32_t B1:4;
    uint32_t B2:4;
    uint32_t B3:4;
    uint32_t B4:4;
    uint32_t B5:4;
    uint32_t B6:4;
    uint32_t B7:4;
};
*/

struct ws2812_stu {
    uint16_t lemp_num; // 灯珠数量
    uint8_t *buffer; // 显存数组
    
    void (*send_data)(struct ws2812_stu *led, uint8_t *buffer, uint16_t size);
    void (*send_data_dma)(struct ws2812_stu *led, uint8_t *buffer, uint16_t size);
};

// 显存数组操作 api
void ws2812_set_1_color(struct ws2812_stu *led, uint16_t lampIndex, uint8_t red, uint8_t green, uint8_t blue);
void ws2812_set_all_color(struct ws2812_stu *led, uint8_t red, uint8_t green, uint8_t blue);
// 刷新 api
void ws2812_refresh(struct ws2812_stu *led);
void ws2812_refresh_dma(struct ws2812_stu *led);

#endif // __WS2812_H__
