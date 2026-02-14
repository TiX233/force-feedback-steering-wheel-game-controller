#include "ws2812.h"

/**
 * @brief 设置单颗灯珠颜色
 * 
 * @param led : ws2812 结构体
 * @param lampIndex : 灯珠索引，从 0 开始
 */
void ws2812_set_1_color(struct ws2812_stu *led, uint16_t lampIndex, uint8_t red, uint8_t green, uint8_t blue){
    uint32_t i;
    ws2812_Color_stu light = {
        .green = 0,
        .red = 0,
        .blue = 0,
    };

    if(lampIndex >= led->lemp_num){
        return ;
    }

    for(i = 0; i < 8; i ++){
        if((green >> i) & 0x1){
            light.green |= WS2812_1_CODE << (28 - i*4);
        }else {
            light.green |= WS2812_0_CODE << (28 - i*4);
        }
    }
    
    for(i = 0; i < 8; i ++){
        if((red >> i) & 0x1){
            light.red |= WS2812_1_CODE << (28 - i*4);
        }else {
            light.red |= WS2812_0_CODE << (28 - i*4);
        }
    }
    
    for(i = 0; i < 8; i ++){
        if((blue >> i) & 0x1){
            light.blue |= WS2812_1_CODE << (28 - i*4);
        }else {
            light.blue |= WS2812_0_CODE << (28 - i*4);
        }
    }

    *(((ws2812_Color_stu *)(led->buffer)) + lampIndex) = light;
}

/* 废弃
void ws2812_set_color(struct ws2812_stu *ws, uint32_t lampIndex, uint8_t red, uint8_t green, uint8_t blue){
    uint32_t i;
    struct ws2812_Color_stu color;

    for(i = 0; i < 4; i ++){
        switch((green >> (i*2)) && 0x03){
            case 0:
                *(((uint8_t *)&(color.G1)) - i) = WS2812_00_CODE;
                break;

            case 1:
                *(((uint8_t *)&(color.G1)) - i) = WS2812_01_CODE;
                break;

            case 2:
                *(((uint8_t *)&(color.G1)) - i) = WS2812_10_CODE;
                break;

            case 3:
                *(((uint8_t *)&(color.G1)) - i) = WS2812_11_CODE;
                break;
        }
    }

    for(i = 0; i < 4; i ++){
        switch((red >> (i*2)) && 0x03){
            case 0:
                *(((uint8_t *)&(color.R1)) - i) = WS2812_00_CODE;
                break;

            case 1:
                *(((uint8_t *)&(color.R1)) - i) = WS2812_01_CODE;
                break;

            case 2:
                *(((uint8_t *)&(color.R1)) - i) = WS2812_10_CODE;
                break;

            case 3:
                *(((uint8_t *)&(color.R1)) - i) = WS2812_11_CODE;
                break;
        }
    }

    for(i = 0; i < 4; i ++){
        switch((blue >> (i*2)) && 0x03){
            case 0:
                *(((uint8_t *)&(color.B1)) - i) = WS2812_00_CODE;
                break;

            case 1:
                *(((uint8_t *)&(color.B1)) - i) = WS2812_01_CODE;
                break;

            case 2:
                *(((uint8_t *)&(color.B1)) - i) = WS2812_10_CODE;
                break;

            case 3:
                *(((uint8_t *)&(color.B1)) - i) = WS2812_11_CODE;
                break;
        }
    }
    
    *((struct ws2812_Color_stu *)&(ws->buffer[(lampIndex - 1) * 8 * 3 * 4 / 8])) = color;
}
*/

/**
 * @brief 将所有灯珠设置为一种颜色
 * 
 * @param led 
 * @param red 
 * @param green 
 * @param blue 
 */
void ws2812_set_all_color(struct ws2812_stu *led, uint8_t red, uint8_t green, uint8_t blue){
    
    uint32_t i;
    ws2812_Color_stu light = {
        .green = 0,
        .red = 0,
        .blue = 0,
    };

        for(i = 0; i < 8; i ++){
            if((green >> i) & 0x1){
                light.green |= WS2812_1_CODE << (28 - i*4);
            }else {
                light.green |= WS2812_0_CODE << (28 - i*4);
            }
        }
        
        for(i = 0; i < 8; i ++){
            if((red >> i) & 0x1){
                light.red |= WS2812_1_CODE << (28 - i*4);
            }else {
                light.red |= WS2812_0_CODE << (28 - i*4);
            }
        }
        
        for(i = 0; i < 8; i ++){
            if((blue >> i) & 0x1){
                light.blue |= WS2812_1_CODE << (28 - i*4);
            }else {
                light.blue |= WS2812_0_CODE << (28 - i*4);
            }
        }

    for(i = 0; i < led->lemp_num; i ++){
        *(((ws2812_Color_stu *)(led->buffer)) + i) = light;
    }
}

/**
 * @brief 阻塞刷新一帧画面
 * 
 * @param led 
 */
void ws2812_refresh(struct ws2812_stu *led){
    // led->send_data(led, led->buffer, led->lemp_num * 8 * 3 * 4 / 8 + WS2812_RST_BYTES);
    led->send_data(led, led->buffer, led->lemp_num * 12 + WS2812_RST_BYTES);
}

/**
 * @brief DMA 方式刷新一帧画面
 * 
 * @param led 
 */
void ws2812_refresh_dma(struct ws2812_stu *led){
    // led->send_data_dma(led, led->buffer, led->lemp_num * 8 * 3 * 4 / 8 + WS2812_RST_BYTES);
    led->send_data_dma(led, led->buffer, led->lemp_num * 12 + WS2812_RST_BYTES);
}
