/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CHERRYUSB_CONFIG_H
#define CHERRYUSB_CONFIG_H

/* ================ USB common Configuration ================ */

#define CONFIG_USB_PRINTF(...) //printf(__VA_ARGS__)

#define usb_malloc(size) malloc(size)
#define usb_free(ptr)    free(ptr)

#ifndef CONFIG_USB_DBG_LEVEL
#define CONFIG_USB_DBG_LEVEL USB_DBG_ERROR
#endif

/* Enable print with color */
#define CONFIG_USB_PRINTF_COLOR_ENABLE

/* data align size when use dma */
#ifndef CONFIG_USB_ALIGN_SIZE
#define CONFIG_USB_ALIGN_SIZE 4
#endif

/* attribute data into no cache ram */
#define USB_NOCACHE_RAM_SECTION __attribute__((section(".noncacheable")))

/* ================= USB Device Stack Configuration ================ */

/* Ep0 max transfer buffer, specially for receiving data from ep0 out */
#define CONFIG_USBDEV_REQUEST_BUFFER_LEN 256

/* Setup packet log for debug */
// #define CONFIG_USBDEV_SETUP_LOG_PRINT

/* Check if the input descriptor is correct */
// #define CONFIG_USBDEV_DESC_CHECK

/* Enable test mode */
// #define CONFIG_USBDEV_TEST_MODE

#ifndef CONFIG_USBDEV_MSC_BLOCK_SIZE
#define CONFIG_USBDEV_MSC_BLOCK_SIZE 512
#endif

#ifndef CONFIG_USBDEV_MSC_MANUFACTURER_STRING
#define CONFIG_USBDEV_MSC_MANUFACTURER_STRING ""
#endif

#ifndef CONFIG_USBDEV_MSC_PRODUCT_STRING
#define CONFIG_USBDEV_MSC_PRODUCT_STRING ""
#endif

#ifndef CONFIG_USBDEV_MSC_VERSION_STRING
#define CONFIG_USBDEV_MSC_VERSION_STRING "0.01"
#endif

// #define CONFIG_USBDEV_MSC_THREAD

#ifdef CONFIG_USBDEV_MSC_THREAD
#ifndef CONFIG_USBDEV_MSC_STACKSIZE
#define CONFIG_USBDEV_MSC_STACKSIZE 2048
#endif

#ifndef CONFIG_USBDEV_MSC_PRIO
#define CONFIG_USBDEV_MSC_PRIO 4
#endif
#endif

#ifndef CONFIG_USBDEV_AUDIO_VERSION
#define CONFIG_USBDEV_AUDIO_VERSION 0x0100
#endif

#ifndef CONFIG_USBDEV_AUDIO_MAX_CHANNEL
#define CONFIG_USBDEV_AUDIO_MAX_CHANNEL 8
#endif


/* ================ USB Device Port Configuration ================*/
#include "py32f4xx_hal.h"

#define __HAL_USB_SOFT_RESET()     do { \
                                     __HAL_RCC_USB_CLK_DISABLE(); \
                                     HAL_Delay(10); \
                                     __HAL_RCC_USB_CLK_ENABLE();  \
                                   } while(0U)

#define USBD_IRQn       USB_IRQn

#define USBD_IRQHandler USBD_IRQHandler

// 数据结构定义
struct hid_handle_up {
    int16_t wheel;              // 方向盘 (0-32767)
    uint8_t joystick_x;         // 摇杆 X (0-255, 中心128)
    uint8_t joystick_y;         // 摇杆 Y (0-255, 中心128)
    uint8_t trigger_left;       // 左扳机 (0-255)
    uint8_t trigger_right;      // 右扳机 (0-255)
    uint32_t buttons;           // 32个按钮，每个bit代表一个按钮
} __attribute__((packed));

struct hid_handle_down {
    int16_t f_const;        // 常量力
    int16_t f_period;       // 周期力
    int16_t f_condition;    // 弹簧力、阻尼力
    int16_t gain;           // 增益
    int16_t on_off;
};

#if 0
struct hid_handle_feature {
    uint8_t config_id;       // 配置项 ID
    uint8_t data[7];         // 配置数据
} __attribute__((packed));
#endif

// 全局报告实例
extern struct hid_handle_up handle_up;
extern struct hid_handle_down handle_down;

void hid_handle_init(void);
int handle_upload(void);

#endif
