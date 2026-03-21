#include "usbd_core.h"
#include "usbd_hid.h"
#include "main.h"
#include "ltx.h"
#include "ltx_log.h"

#define HID_INT_EP          0x81
#define HID_INT_EP_SIZE     64      // 设为最大值，64
#define HID_INT_EP_INTERVAL 5       // 上行端口轮询间隔，单位毫秒

#define HID_OUT_EP          0x02
#define HID_OUT_EP_SIZE     64      // 设为最大值，64
#define HID_OUT_EP_INTERVAL 5       // 下行端口轮询间隔，单位毫秒

#define USBD_VID            0x36b7
#define USBD_PID            0x2568
#define USBD_MAX_POWER      100
#define USBD_LANGID_STRING  1033

/*!< config descriptor size */
#define USB_HID_CONFIG_DESC_SIZ 41
/*!< report descriptor size */
#define HID_HANDLE_REPORT_DESC_SIZE sizeof(hid_handle_report_desc)

#if 0
static const uint8_t hid_handle_report_desc[] = {
    // 1. 输入报告（ID=1）: 方向盘+摇杆+扳机+按钮
    0x05, 0x01,           // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,           // USAGE (Joystick)
    0xA1, 0x01,           // COLLECTION (Application)
    
    // 报告ID = 1
    0x85, 0x01,           // REPORT_ID (1)
    
    // 方向盘轴 (16位)
    0x05, 0x01,           //   USAGE_PAGE (Generic Desktop)
    0x09, 0x31,           //   USAGE (Steering) 或者 0x09,0xBA (Simulation Steering)
    0x15, 0x00,           //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,     //   LOGICAL_MAXIMUM (32767)  ; 16位有符号可调，这里用无符号0~65535也行
    0x75, 0x10,           //   REPORT_SIZE (16)
    0x95, 0x01,           //   REPORT_COUNT (1)
    0x81, 0x02,           //   INPUT (Data,Var,Abs)
    
    // 左摇杆 X (8位)
    0x05, 0x01,           //   USAGE_PAGE (Generic Desktop)
    0x09, 0x30,           //   USAGE (X)
    0x15, 0x00,           //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,     //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,           //   REPORT_SIZE (8)
    0x95, 0x01,           //   REPORT_COUNT (1)
    0x81, 0x02,           //   INPUT (Data,Var,Abs)
    
    // 左摇杆 Y (8位)
    0x05, 0x01,           //   USAGE_PAGE (Generic Desktop)
    0x09, 0x31,           //   USAGE (Y)
    0x15, 0x00,           //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,     //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,           //   REPORT_SIZE (8)
    0x95, 0x01,           //   REPORT_COUNT (1)
    0x81, 0x02,           //   INPUT (Data,Var,Abs)
    
    // 左扳机 (8位模拟量)
    0x05, 0x01,           //   USAGE_PAGE (Generic Desktop)
    0x09, 0x32,           //   USAGE (Z)  或者用 0x09,0x35 (Brake)
    0x15, 0x00,           //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,     //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,           //   REPORT_SIZE (8)
    0x95, 0x01,           //   REPORT_COUNT (1)
    0x81, 0x02,           //   INPUT (Data,Var,Abs)
    
    // 右扳机 (8位模拟量)
    0x05, 0x01,           //   USAGE_PAGE (Generic Desktop)
    0x09, 0x35,           //   USAGE (Brake) 或者 0x09,0x33 (RotZ)
    0x15, 0x00,           //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,     //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,           //   REPORT_SIZE (8)
    0x95, 0x01,           //   REPORT_COUNT (1)
    0x81, 0x02,           //   INPUT (Data,Var,Abs)
    
    // 按钮 (32个)
    0x05, 0x09,           //   USAGE_PAGE (Button)
    0x19, 0x01,           //   USAGE_MINIMUM (Button 1)
    0x29, 0x20,           //   USAGE_MAXIMUM (Button 32)  ; 最多32个
    0x15, 0x00,           //   LOGICAL_MINIMUM (0)
    0x25, 0x01,           //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,           //   REPORT_SIZE (1)
    0x95, 0x20,           //   REPORT_COUNT (32)
    0x81, 0x02,           //   INPUT (Data,Var,Abs)
    
    // 填充到整数个字节（可选，如果上面按钮正好占4字节，则已对齐）
    
    // 2. 输出报告（ID=2）: 接收力反馈力矩
    0x05, 0x0F,           //   USAGE_PAGE (Physical Interface Device)
    0x09, 0x00,           //   USAGE (Undefined)
    0xA1, 0x02,           //   COLLECTION (Logical)
    0x85, 0x02,           //     REPORT_ID (2)
    0x09, 0x01,           //     USAGE (Set Effect Report) 简化，直接用力矩值
    0x15, 0x00,           //     LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,     //     LOGICAL_MAXIMUM (32767)
    0x75, 0x10,           //     REPORT_SIZE (16)
    0x95, 0x01,           //     REPORT_COUNT (1)
    0x91, 0x02,           //     OUTPUT (Data,Var,Abs)
    0xC0,                 //   END_COLLECTION
    
    // 3. 特征报告（ID=3）: 配置参数（例如全局增益）
    0x05, 0x0F,           //   USAGE_PAGE (Physical Interface Device)
    0x09, 0x00,           //   USAGE (Undefined)
    0xA1, 0x02,           //   COLLECTION (Logical)
    0x85, 0x03,           //     REPORT_ID (3)
    0x09, 0x02,           //     USAGE (Effect Operation Report) 或自定义
    0x15, 0x00,           //     LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,     //     LOGICAL_MAXIMUM (255)
    0x75, 0x08,           //     REPORT_SIZE (8)
    0x95, 0x08,           //     REPORT_COUNT (8) ; 8字节配置数据
    0xB1, 0x02,           //     FEATURE (Data,Var,Abs)
    0xC0,                 //   END_COLLECTION
    
    0xC0                  // END_COLLECTION (Application)
};
#elif 0
static const uint8_t hid_handle_report_desc[] = {
    // ==================== 应用集合开始 ====================
    0x05, 0x01,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,              // USAGE (Joystick)
    0xA1, 0x01,              // COLLECTION (Application)

    // ---------- 1. 输入报告：上报方向盘状态 ----------
    // 报告ID使用0x01（通用输入报告，不与PID标准冲突）
    0x85, 0x01,              // REPORT_ID (1) - 输入报告
    
    // 方向盘轴 (16位，0-65535)
    0x05, 0x01,              //   USAGE_PAGE (Generic Desktop)
    0x09, 0x31,              //   USAGE (Steering) 或使用 0x09,0xBA (Simulation Steering)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        //   LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              //   REPORT_SIZE (16)
    0x95, 0x01,              //   REPORT_COUNT (1)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)
    
    // 左摇杆 X (8位)
    0x05, 0x01,              //   USAGE_PAGE (Generic Desktop)
    0x09, 0x30,              //   USAGE (X)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,              //   REPORT_SIZE (8)
    0x95, 0x01,              //   REPORT_COUNT (1)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)
    
    // 左摇杆 Y (8位)
    0x05, 0x01,              //   USAGE_PAGE (Generic Desktop)
    0x09, 0x31,              //   USAGE (Y)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,              //   REPORT_SIZE (8)
    0x95, 0x01,              //   REPORT_COUNT (1)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)
    
    // 左扳机 (8位)
    0x05, 0x01,              //   USAGE_PAGE (Generic Desktop)
    0x09, 0x32,              //   USAGE (Z)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,              //   REPORT_SIZE (8)
    0x95, 0x01,              //   REPORT_COUNT (1)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)
    
    // 右扳机 (8位)
    0x05, 0x01,              //   USAGE_PAGE (Generic Desktop)
    0x09, 0x35,              //   USAGE (Brake)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,              //   REPORT_SIZE (8)
    0x95, 0x01,              //   REPORT_COUNT (1)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)
    
    // 32个按钮
    0x05, 0x09,              //   USAGE_PAGE (Button)
    0x19, 0x01,              //   USAGE_MINIMUM (Button 1)
    0x29, 0x20,              //   USAGE_MAXIMUM (Button 32)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x25, 0x01,              //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,              //   REPORT_SIZE (1)
    0x95, 0x20,              //   REPORT_COUNT (32)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)
    
    // ========== 2. PID输出报告区域（游戏→设备）==========
    // 切换到PID Usage Page（关键！）
    0x05, 0x0F,              // USAGE_PAGE (Physical Interface Device)
    
    // ---------- PID_SET_EFFECT 报告 ----------
    0x09, 0x21,              // USAGE (Set Effect Report) - PID标准用法[citation:7]
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0x21,              // REPORT_ID (0x21) - 标准PID_SET_EFFECT ID
    0x09, 0x22,              // USAGE (Effect Block Index) - 效果块索引
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x3F,              // LOGICAL_MAXIMUM (63) - 最多64个效果
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    // 效果类型（常量力/周期性/弹簧等）
    0x09, 0x23,              // USAGE (Effect Type) - 效果类型
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x0A,              // LOGICAL_MAXIMUM (10) - 支持11种效果类型
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    // 持续时间
    0x09, 0x50,              // USAGE (Duration) - 持续时间[citation:7]
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (255)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    // 增益（强度）
    0x09, 0x52,              // USAGE (Gain) - 增益[citation:7]
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (10000)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    // 方向使能
    0x09, 0x56,              // USAGE (Enable Direction) - 方向使能[citation:7]
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x01,              // LOGICAL_MAXIMUM (1)
    0x75, 0x01,              // REPORT_SIZE (1)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION
    
    // ---------- PID_EFFECT_OPERATION 报告 ----------
    0x09, 0x77,              // USAGE (Effect Operation Report) - PID标准[citation:7]
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0x77,              // REPORT_ID (0x77) - 标准PID_EFFECT_OPERATION ID
    0x09, 0x22,              // USAGE (Effect Block Index)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x3F,              // LOGICAL_MAXIMUM (63)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    0x09, 0x7C,              // USAGE (Loop Count) - 循环次数[citation:7]
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0xFF,              // LOGICAL_MAXIMUM (255)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    0x09, 0x79,              // USAGE (Effect Start) - 开始效果[citation:7]
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x01,              // LOGICAL_MAXIMUM (1)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION
    
    // ---------- 常量力参数报告 ----------
    0x09, 0x73,              // USAGE (Set Constant Report) - 常量力参数[citation:7]
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0x73,              // REPORT_ID (0x73)
    0x09, 0x22,              // USAGE (Effect Block Index)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x3F,              // LOGICAL_MAXIMUM (63)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    0x09, 0x70,              // USAGE (Magnitude) - 常量力幅值[citation:7]
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        // LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION
    
    // ---------- 周期性力参数报告（振动）----------
    0x09, 0x6E,              // USAGE (Set Periodic Report) - 周期性力参数[citation:7]
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0x6E,              // REPORT_ID (0x6E)
    0x09, 0x22,              // USAGE (Effect Block Index)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x3F,              // LOGICAL_MAXIMUM (63)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    0x09, 0x70,              // USAGE (Magnitude) - 幅值
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        // LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    0x09, 0x71,              // USAGE (Period) - 周期[citation:7]
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (255)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    0x09, 0x6F,              // USAGE (Offset) - 偏移量[citation:7]
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (255)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    0x09, 0x72,              // USAGE (Phase) - 相位[citation:7]
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (36000)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    0x09, 0x6B,              // USAGE (Waveform) - 波形类型
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x06,              // LOGICAL_MAXIMUM (6) - 0=方波,1=正弦,2=三角...
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION
    
    // ---------- 条件力参数报告（弹簧/阻尼/摩擦）----------
    0x09, 0x5F,              // USAGE (Set Condition Report) - 条件力参数[citation:7]
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0x5F,              // REPORT_ID (0x5F)
    0x09, 0x22,              // USAGE (Effect Block Index)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x3F,              // LOGICAL_MAXIMUM (63)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    0x09, 0x60,              // USAGE (CP Offset) - 中心点偏移[citation:7]
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        // LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    0x09, 0x61,              // USAGE (Positive Coefficient) - 正方向系数[citation:7]
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        // LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    
    0x09, 0x62,              // USAGE (Negative Coefficient) - 负方向系数[citation:7]
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        // LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION
    
    // ---------- 设备增益报告 ----------
    0x09, 0x7E,              // USAGE (Device Gain Report) - 全局增益[citation:7]
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0x7E,              // REPORT_ID (0x7E)
    0x09, 0x7F,              // USAGE (Device Gain) - 设备增益
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (10000)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION
    
    // ========== 3. 自定义配置报告（保留给上位机）==========
    // 使用PID页面中的Reserved范围，避免与标准冲突
    0x05, 0x0F,              // USAGE_PAGE (Physical Interface Device)
    0x09, 0xFF,              // USAGE (Reserved for vendor use)
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0xF0,              // REPORT_ID (0xF0) - 避开0x00-0xEF标准ID
    0x09, 0xFF,              // USAGE (Reserved)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (255)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x08,              // REPORT_COUNT (8) - 8字节配置数据
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION
    
    0xC0                     // 应用集合结束
};
#elif 1
// 报告描述符
static const uint8_t hid_handle_report_desc[] = {
    // ========== 应用集合开始 ==========
    0x05, 0x01,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,              // USAGE (Joystick)
    0xA1, 0x01,              // COLLECTION (Application)

    // ---------- 1. 输入报告：上报方向盘状态 (ID=0x01) ----------
    0x85, 0x01,              // REPORT_ID (1)
    
    // 方向盘轴 (16位)
    0x05, 0x01,              //   USAGE_PAGE (Generic Desktop)
    0x09, 0x31,              //   USAGE (Steering)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        //   LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              //   REPORT_SIZE (16)
    0x95, 0x01,              //   REPORT_COUNT (1)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)
    
    // 左摇杆 X (8位)
    0x05, 0x01,              //   USAGE_PAGE (Generic Desktop)
    0x09, 0x30,              //   USAGE (X)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,              //   REPORT_SIZE (8)
    0x95, 0x01,              //   REPORT_COUNT (1)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)
    
    // 左摇杆 Y (8位)
    0x05, 0x01,              //   USAGE_PAGE (Generic Desktop)
    0x09, 0x31,              //   USAGE (Y)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,              //   REPORT_SIZE (8)
    0x95, 0x01,              //   REPORT_COUNT (1)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)
    
    // 左扳机 (8位)
    0x05, 0x01,              //   USAGE_PAGE (Generic Desktop)
    0x09, 0x32,              //   USAGE (Z)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,              //   REPORT_SIZE (8)
    0x95, 0x01,              //   REPORT_COUNT (1)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)
    
    // 右扳机 (8位)
    0x05, 0x01,              //   USAGE_PAGE (Generic Desktop)
    0x09, 0x35,              //   USAGE (Brake)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,              //   REPORT_SIZE (8)
    0x95, 0x01,              //   REPORT_COUNT (1)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)
    
    // 32个按钮
    0x05, 0x09,              //   USAGE_PAGE (Button)
    0x19, 0x01,              //   USAGE_MINIMUM (Button 1)
    0x29, 0x20,              //   USAGE_MAXIMUM (Button 32)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x25, 0x01,              //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,              //   REPORT_SIZE (1)
    0x95, 0x20,              //   REPORT_COUNT (32)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)

    // ========== 2. PID 标准输出报告区域 ==========
    0x05, 0x0F,              // USAGE_PAGE (Physical Interface Device)

    // ---------- PID_SET_EFFECT (ID=0x21) ----------
    0x09, 0x21,              // USAGE (Set Effect Report)
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0x21,              // REPORT_ID (0x21)
    0x09, 0x22,              // USAGE (Effect Block Index)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x3F,              // LOGICAL_MAXIMUM (63)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x23,              // USAGE (Effect Type)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x0A,              // LOGICAL_MAXIMUM (10)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x50,              // USAGE (Duration)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (255)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x52,              // USAGE (Gain)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (10000)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    // 修正：Enable Direction 从 1 位改为 8 位，避免未对齐
    0x09, 0x56,              // USAGE (Enable Direction)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x01,              // LOGICAL_MAXIMUM (1)
    0x75, 0x08,              // REPORT_SIZE (8)   ← 改为 8 位
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION

    // ---------- PID_EFFECT_OPERATION (ID=0x77) ----------
    0x09, 0x77,              // USAGE (Effect Operation Report)
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0x77,              // REPORT_ID (0x77)
    0x09, 0x22,              // USAGE (Effect Block Index)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x3F,              // LOGICAL_MAXIMUM (63)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x7C,              // USAGE (Loop Count)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0xFF,              // LOGICAL_MAXIMUM (255)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x79,              // USAGE (Effect Start)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x01,              // LOGICAL_MAXIMUM (1)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION

    // ---------- PID_SET_CONSTANT (ID=0x73) ----------
    0x09, 0x73,              // USAGE (Set Constant Report)
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0x73,              // REPORT_ID (0x73)
    0x09, 0x22,              // USAGE (Effect Block Index)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x3F,              // LOGICAL_MAXIMUM (63)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x70,              // USAGE (Magnitude)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        // LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION

    // ---------- PID_SET_PERIODIC (ID=0x6E) ----------
    0x09, 0x6E,              // USAGE (Set Periodic Report)
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0x6E,              // REPORT_ID (0x6E)
    0x09, 0x22,              // USAGE (Effect Block Index)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x3F,              // LOGICAL_MAXIMUM (63)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x70,              // USAGE (Magnitude)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        // LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x71,              // USAGE (Period)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (255)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x6F,              // USAGE (Offset)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (255)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x72,              // USAGE (Phase)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (36000)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x6B,              // USAGE (Waveform)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x06,              // LOGICAL_MAXIMUM (6)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION

    // ---------- PID_SET_CONDITION (ID=0x5F) ----------
    0x09, 0x5F,              // USAGE (Set Condition Report)
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0x5F,              // REPORT_ID (0x5F)
    0x09, 0x22,              // USAGE (Effect Block Index)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0x3F,              // LOGICAL_MAXIMUM (63)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x60,              // USAGE (CP Offset)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        // LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x61,              // USAGE (Positive Coefficient)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        // LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0x09, 0x62,              // USAGE (Negative Coefficient)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        // LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    // 添加填充字节，使总位数为8的倍数（原7字节，补1字节）
    0x09, 0x00,              // USAGE (Reserved)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x25, 0xFF,              // LOGICAL_MAXIMUM (255)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x03,              // OUTPUT (Constant,Var,Abs)  // 常量字段，设备忽略
    0xC0,                    // END_COLLECTION

    // ---------- PID_DEVICE_GAIN (ID=0x7E) ----------
    0x09, 0x7E,              // USAGE (Device Gain Report)
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0x7E,              // REPORT_ID (0x7E)
    0x09, 0x7F,              // USAGE (Device Gain)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (10000)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION

    // ========== 3. 自定义配置报告（上位机专用，ID=0xF0）==========
    0x05, 0x0F,              // USAGE_PAGE (Physical Interface Device)
    0x09, 0xFF,              // USAGE (Vendor Defined)
    0xA1, 0x02,              // COLLECTION (Logical)
    0x85, 0xF0,              // REPORT_ID (0xF0)
    0x09, 0xFF,              // USAGE (Vendor Defined)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (255)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x08,              // REPORT_COUNT (8)
    0x91, 0x02,              // OUTPUT (Data,Var,Abs)
    0xC0,                    // END_COLLECTION

    0xC0                     // END_COLLECTION (Application)
};
#endif

/*!< global descriptor */
const uint8_t hid_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0002, 0x01),
    // 配置描述符
    USB_CONFIG_DESCRIPTOR_INIT(USB_HID_CONFIG_DESC_SIZ, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),

    // 接口描述符
    /* 09 */
    0x09,                          /* bLength: Interface Descriptor size */
    USB_DESCRIPTOR_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
    0x00,                          /* bInterfaceNumber: Number of Interface */
    0x00,                          /* bAlternateSetting: Alternate setting */
    0x02,                          /* bNumEndpoints */
    0x03,                          /* bInterfaceClass: HID */
    0x00,                          /* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x00,                          /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0,                             /* iInterface: Index of string descriptor */
    // hid 描述符
    /* 18 */
    0x09,                    /* bLength: HID Descriptor size */
    HID_DESCRIPTOR_TYPE_HID, /* bDescriptorType: HID */
    0x11,                    /* bcdHID: HID Class Spec release number */
    0x01,
    0x00,                       /* bCountryCode: Hardware target country */
    0x01,                       /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,                       /* bDescriptorType */
    HID_HANDLE_REPORT_DESC_SIZE & 0xFF, /* wItemLength low */
    (HID_HANDLE_REPORT_DESC_SIZE >> 8) & 0xFF, /* high */
    /******************** IN Endpoint (用于上报输入) ********************/
    0x07,                         /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT, /* bDescriptorType */
    HID_INT_EP,                   /* bEndpointAddress: IN endpoint 1 */
    0x03,                         /* bmAttributes: Interrupt */
    HID_INT_EP_SIZE, 0x00,        /* wMaxPacketSize: 64 bytes */
    HID_INT_EP_INTERVAL,          /* bInterval: 轮询间隔，单位毫秒 */

    /******************** OUT Endpoint (用于接收力反馈) ********************/
    0x07,                         /* bLength */
    USB_DESCRIPTOR_TYPE_ENDPOINT, /* bDescriptorType */
    HID_OUT_EP,                   /* bEndpointAddress: OUT endpoint 2 */
    0x03,                         /* bmAttributes: Interrupt */
    HID_OUT_EP_SIZE, 0x00,        /* wMaxPacketSize: 64 bytes */
    HID_OUT_EP_INTERVAL,          /* bInterval: 轮询间隔，单位毫秒 */
    /* 34 */
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x0A,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '@', 0x00,                  /* wcChar0 */
    'T', 0x00,                  /* wcChar1 */
    'i', 0x00,                  /* wcChar2 */
    'X', 0x00,                  /* wcChar3 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x1C,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'T', 0x00,                  /* wcChar0 */
    'i', 0x00,                  /* wcChar1 */
    'X', 0x00,                  /* wcChar2 */
    ' ', 0x00,                  /* wcChar3 */
    'F', 0x00,                  /* wcChar4 */
    'F', 0x00,                  /* wcChar5 */
    'B', 0x00,                  /* wcChar6 */
    'H', 0x00,                  /* wcChar7 */
    'a', 0x00,                  /* wcChar8 */
    'n', 0x00,                  /* wcChar9 */
    'd', 0x00,                  /* wcChar10 */
    'l', 0x00,                  /* wcChar11 */
    'e', 0x00,                  /* wcChar12 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '6', 0x00,                  /* wcChar3 */
    '1', 0x00,                  /* wcChar4 */
    '2', 0x00,                  /* wcChar5 */
    '3', 0x00,                  /* wcChar6 */
    '4', 0x00,                  /* wcChar7 */
    '5', 0x00,                  /* wcChar8 */
    '6', 0x00,                  /* wcChar9 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};

#if 0
/*!< hid mouse report descriptor */
static const uint8_t hid_mouse_report_desc[HID_CUSTOM_REPORT_DESC_SIZE] = {
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x02, // USAGE (Mouse)
    0xA1, 0x01, // COLLECTION (Application)
    0x09, 0x01, //   USAGE (Pointer)

    0xA1, 0x00, //   COLLECTION (Physical)
    0x05, 0x09, //     USAGE_PAGE (Button)
    0x19, 0x01, //     USAGE_MINIMUM (Button 1)
    0x29, 0x03, //     USAGE_MAXIMUM (Button 3)

    0x15, 0x00, //     LOGICAL_MINIMUM (0)
    0x25, 0x01, //     LOGICAL_MAXIMUM (1)
    0x95, 0x03, //     REPORT_COUNT (3)
    0x75, 0x01, //     REPORT_SIZE (1)

    0x81, 0x02, //     INPUT (Data,Var,Abs)
    0x95, 0x01, //     REPORT_COUNT (1)
    0x75, 0x05, //     REPORT_SIZE (5)
    0x81, 0x01, //     INPUT (Cnst,Var,Abs)

    0x05, 0x01, //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30, //     USAGE (X)
    0x09, 0x31, //     USAGE (Y)
    0x09, 0x38,

    0x15, 0x81, //     LOGICAL_MINIMUM (-127)
    0x25, 0x7F, //     LOGICAL_MAXIMUM (127)
    0x75, 0x08, //     REPORT_SIZE (8)
    0x95, 0x03, //     REPORT_COUNT (2)

    0x81, 0x06, //     INPUT (Data,Var,Rel)
    0xC0, 0x09,
    0x3c, 0x05,
    0xff, 0x09,

    0x01, 0x15,
    0x00, 0x25,
    0x01, 0x75,
    0x01, 0x95,

    0x02, 0xb1,
    0x22, 0x75,
    0x06, 0x95,
    0x01, 0xb1,

    0x01, 0xc0 //   END_COLLECTION
};
#endif



#define HID_STATE_IDLE 0
#define HID_STATE_BUSY 1


// 全局报告实例
struct hid_handle_up handle_up;
struct hid_handle_down handle_down;
// static struct hid_handle_feature handle_feature;

// hid 上行状态标志
static volatile uint8_t hid_up_state = HID_STATE_IDLE;
// hid 下行数据 buffer
USB_MEM_ALIGNX uint8_t hid_down_buffer[64];

void usbd_configure_done_callback(void){
    // 开启 usb 接收
    usbd_ep_start_read(HID_OUT_EP, hid_down_buffer, HID_OUT_EP_SIZE);
}

// 上行数据端点传输完成回调
static void usbd_hid_up_callback(uint8_t ep, uint32_t nbytes){

    hid_up_state = HID_STATE_IDLE;
}

// 下行数据端点传输完成回调
static void usbd_hid_down_callback(uint8_t ep, uint32_t nbytes){

    if(nbytes > 0){
        switch(hid_down_buffer[0]){ // report id
            #if 0
            // PID标准：创建/修改效果
            case 0x21:  // PID_SET_EFFECT
                if (nbytes >= 3) {
                    uint8_t effect_idx = hid_down_buffer[1];
                    uint8_t effect_type = hid_down_buffer[2];  // 效果类型
                    // 存储效果参数，等待后续的参数报告
                    pending_effect_type[effect_idx] = effect_type;
                }
                break;
            
            // PID标准：设置常量力参数
            case 0x73:  // PID_SET_CONSTANT
                if (nbytes >= 4) {
                    uint8_t effect_idx = hid_down_buffer[1];
                    int16_t magnitude = (int16_t)((hid_down_buffer[2] << 8) | hid_down_buffer[3]);
                    // 存储常量力幅值
                    constant_magnitude[effect_idx] = magnitude;
                }
                break;
            
            // PID标准：设置周期性力参数（路面振动）
            case 0x6E:  // PID_SET_PERIODIC
                if (nbytes >= 8) {
                    uint8_t effect_idx = hid_down_buffer[1];
                    int16_t magnitude = (int16_t)((hid_down_buffer[2] << 8) | hid_down_buffer[3]);
                    uint16_t period = (uint16_t)((hid_down_buffer[4] << 8) | hid_down_buffer[5]);
                    uint8_t waveform = hid_down_buffer[7];  // 正弦/方波/三角
                    // 存储周期性力参数
                    periodic_magnitude[effect_idx] = magnitude;
                    periodic_period[effect_idx] = period;
                    periodic_waveform[effect_idx] = waveform;
                }
                break;
            
            // PID标准：设置条件力参数（弹簧力、阻尼力）
            case 0x5F:  // PID_SET_CONDITION
                if (nbytes >= 6) {
                    uint8_t effect_idx = hid_down_buffer[1];
                    int16_t cp_offset = (int16_t)((hid_down_buffer[2] << 8) | hid_down_buffer[3]);
                    int16_t pos_coeff = (int16_t)((hid_down_buffer[4] << 8) | hid_down_buffer[5]);
                    // 存储条件力参数
                    condition_offset[effect_idx] = cp_offset;
                    condition_pos_coeff[effect_idx] = pos_coeff;
                }
                break;
            
            // PID标准：启动/停止效果
            case 0x77:  // PID_EFFECT_OPERATION
                if (nbytes >= 4) {
                    uint8_t effect_idx = hid_down_buffer[1];
                    uint8_t loop_count = hid_down_buffer[2];  // 0=无限
                    uint8_t operation = hid_down_buffer[3];   // 0=停止, 1=开始
                    
                    if (operation == 1) {
                        start_effect(effect_idx, loop_count);
                    } else {
                        stop_effect(effect_idx);
                    }
                }
                break;
            
            // PID标准：设备增益
            case 0x7E:  // PID_DEVICE_GAIN
                if (nbytes >= 3) {
                    uint16_t gain = (uint16_t)((hid_down_buffer[1] << 8) | hid_down_buffer[2]);
                    ff_global_gain = gain;  // 0-10000范围
                }
                break;
            
            #endif
            // 自定义配置，上位机专用
            case 0xF0:
                if (nbytes >= 3) {
                    uint8_t config_id = hid_down_buffer[1];
                    // uint16_t config_value = (uint16_t)((hid_down_buffer[2] << 8) | hid_down_buffer[3]);
                    // handle_custom_config(config_id, config_value);
                }
                break;

            default:
                break;
        }
    }
    #if 0
    LTX_LOG_DEBG("HID: 0x%x %x %x %x %x %x, len: %d\n", hid_down_buffer[0],
                                                        hid_down_buffer[1],
                                                        hid_down_buffer[2],
                                                        hid_down_buffer[3],
                                                        hid_down_buffer[4],
                                                        hid_down_buffer[5],
                                                        nbytes);
    LTX_LOG_DEBG("%x %x %x %x %x %x\n", hid_down_buffer[6],
                                        hid_down_buffer[7],
                                        hid_down_buffer[8],
                                        hid_down_buffer[9],
                                        hid_down_buffer[10],
                                        hid_down_buffer[11]);
    #endif
    
    // 重新启动接收，准备下一次数据
    usbd_ep_start_read(ep, hid_down_buffer, HID_OUT_EP_SIZE);
}

// 端点描述符
static struct usbd_endpoint hid_in_ep = {
    .ep_cb = usbd_hid_up_callback,
    .ep_addr = HID_INT_EP,
};

static struct usbd_endpoint hid_out_ep = {
    .ep_cb = usbd_hid_down_callback,
    .ep_addr = HID_OUT_EP
};

#if 0
// py 提供的 cherryusb 版本没有 Feature Report
// HID 类回调
static int hid_get_report(uint8_t report_id, uint8_t report_type, uint8_t *buffer, uint32_t len){

    if(report_type == HID_REPORT_FEATURE){
        if(report_id == 0x03){
            buffer[0] = 0x03;
            buffer[1] = handle_feature.config_id;
            memcpy(&buffer[2], handle_feature.data, 7);
            return 8;
        }
    }
    return 0;
}
static int hid_set_report(uint8_t report_id, uint8_t report_type, uint8_t *buffer, uint32_t len){

    if(report_type == HID_REPORT_FEATURE){
        if(report_id == 0x03 && len >= 2){
            handle_feature.config_id = buffer[1];
            memcpy(handle_feature.data, &buffer[2], len - 2);
            // 根据配置更新设备行为（如增益、曲线等）
            return 0;
        }
    }
    return -1;
}
static struct usbd_hid_callback hid_cb = {
    .get_report = hid_get_report,
    .set_report = hid_set_report,
};
#endif

// 接口结构体
struct usbd_interface intf0;

// 初始化函数
void hid_handle_init(void){

    // 注册设备描述符（需包含修改后的配置描述符）
    usbd_desc_register(hid_descriptor);

    // 添加 hid 接口（传入自定义报告描述符和回调）
    usbd_add_interface(usbd_hid_init_intf(&intf0, hid_handle_report_desc, HID_HANDLE_REPORT_DESC_SIZE));

    // 添加两个端点
    usbd_add_endpoint(&hid_in_ep);
    usbd_add_endpoint(&hid_out_ep);

    // 初始化 USB 栈
    usbd_initialize();

    // 初始化输入数据
    handle_up.joystick_x = 128;
    handle_up.joystick_y = 128;
    handle_up.buttons = 0;
    // 其他置 0
}

// 发送输入报告
void handle_upload(void){
    if(hid_up_state == HID_STATE_IDLE){
        hid_up_state = HID_STATE_BUSY;

        uint8_t report[64] = {0};
        report[0] = 0x01; // Report ID

        // 填充数据（按描述符顺序）
        report[1] = (handle_up.angle >> 8) & 0xFF;
        report[2] = handle_up.angle & 0xFF;
        report[3] = handle_up.joystick_x;
        report[4] = handle_up.joystick_y;
        report[5] = handle_up.trigger_left;
        report[6] = handle_up.trigger_right;

        // 按钮占4字节
        report[7]  = (handle_up.buttons) & 0xFF;
        report[8]  = (handle_up.buttons >> 8) & 0xFF;
        report[9]  = (handle_up.buttons >> 16) & 0xFF;
        report[10] = (handle_up.buttons >> 24) & 0xFF;

        // 发送到 in 端点
        int ret = usbd_ep_start_write(HID_INT_EP, report, 11); // 实际有效字节数（不含填充）
        if(ret < 0){
            LTX_LOG_DEBG("USB Send Failed: %d\n", ret);
        }
    }else {
        LTX_LOG_DEBG("USB Busy\n");
    }
}
