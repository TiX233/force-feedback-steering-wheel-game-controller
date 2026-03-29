#include "usbd_core.h"
#include "usbd_hid.h"
#include "main.h"
#include "ltx.h"
#include "ltx_log.h"
#include "myAPP_button.h"
#include "myAPP_ffb.h"

#define HID_INT_EP          0x81
#define HID_INT_EP_SIZE     0x40    // 设为最大值，64
#define HID_INT_EP_INTERVAL 2       // 上行端口轮询间隔，单位毫秒

#define HID_OUT_EP          0x02
#define HID_OUT_EP_SIZE     0x40    // 设为最大值，64
#define HID_OUT_EP_INTERVAL 2       // 下行端口轮询间隔，单位毫秒

// 如果需要修改 hid_handle_report_desc 报告描述符，那么还需要变更 vid/pid 才能让 windows 重新发起识别，否则不会有效
// 填什么都行反正，只要不是别人的商用 vid/pid，以及不要是你电脑连接过的设备的 vid，不然 windows 会偷懒不匹配驱动，导致游戏不识别
#define USBD_VID            0x2568
#define USBD_PID            0x2333

#define USBD_MAX_POWER      100
#define USBD_LANGID_STRING  1033

/*!< config descriptor size */
#define USB_HID_CONFIG_DESC_SIZ 41
/*!< report descriptor size */
#define HID_HANDLE_REPORT_DESC_SIZE sizeof(hid_handle_report_desc)

// 报告描述符
#if 0
static const uint8_t hid_handle_report_desc[] = {
    // ==================== 应用集合开始 ====================
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,        // USAGE (Joystick)
    0xA1, 0x01,        // COLLECTION (Application)

    // ---------- 1. 输入报告 (ID=1) ----------
    0x85, 0x01,              // REPORT_ID (0x01)
    
    // 方向盘轴（使用 Simulation Controls 的 Steering）
    0x05, 0x02,              // USAGE_PAGE (Simulation Controls)
    0x09, 0xBA,              // USAGE (Steering)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,        // LOGICAL_MAXIMUM (32767)
    0x75, 0x10,              // REPORT_SIZE (16)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x81, 0x02,              // INPUT (Data,Var,Abs)
    
    // 左摇杆 X
    0x05, 0x01,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x30,              // USAGE (X)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (255)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x81, 0x02,              // INPUT (Data,Var,Abs)

    // 左摇杆 Y
    0x05, 0x01,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x31,              // USAGE (Y)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (255)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x81, 0x02,              // INPUT (Data,Var,Abs)
    
    // 左扳机（Z 轴）
    0x05, 0x01,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x32,              // USAGE (Z)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (255)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x81, 0x02,              // INPUT (Data,Var,Abs)

    // 右扳机（Rx）
    0x05, 0x01,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x33,              // USAGE (Rx)
    0x15, 0x00,              // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,        // LOGICAL_MAXIMUM (255)
    0x75, 0x08,              // REPORT_SIZE (8)
    0x95, 0x01,              // REPORT_COUNT (1)
    0x81, 0x02,              // INPUT (Data,Var,Abs)
    
    // 32个按钮
    0x05, 0x09,              //   USAGE_PAGE (Button)
    0x19, 0x01,              //   USAGE_MINIMUM (Button 1)
    0x29, 0x20,              //   USAGE_MAXIMUM (Button 32)
    0x15, 0x00,              //   LOGICAL_MINIMUM (0)
    0x25, 0x01,              //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,              //   REPORT_SIZE (1)
    0x95, 0x20,              //   REPORT_COUNT (32)
    0x81, 0x02,              //   INPUT (Data,Var,Abs)

    // ========== 2. PID Feature Reports (关键！Windows通过它们识别FFB能力) ==========
    // 切换到PID Usage Page
    0x05, 0x0F,        // USAGE_PAGE (Physical Interface Device)

    // 2.1 PID Pool Report (ID=1) - 告诉Windows支持多少力反馈效果
    // 0x09, 0x01,        // USAGE (PID Pool Report)
    // 0xA1, 0x02,        // COLLECTION (Logical)
    // 0x85, 0x01,        //   REPORT_ID (1)
    // 0x09, 0x02,        //   USAGE (Number of Effects Supported)
    // 0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    // 0x25, 0x3F,        //   LOGICAL_MAXIMUM (63)
    // 0x75, 0x08,        //   REPORT_SIZE (8)
    // 0x95, 0x01,        //   REPORT_COUNT (1)
    // 0xB1, 0x02,        //   FEATURE (Data,Var,Abs)
    // 0xC0,              // END_COLLECTION
    0x05, 0x0F,        // Usage Page (PID Page)
    0x09, 0x02,        // Usage (PID State Report)
    0xA1, 0x02,        // Collection (Logical)
    0x85, 0x01,        //   Report ID (1)
    0x09, 0x03,        //   Usage (Number of Effects Supported)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x3F,        //   Logical Maximum (63)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //   Feature (Data,Var,Abs)
    0xC0,              // END_COLLECTION

    // 2.2 PID Device Control (ID=2) - 启用/禁用执行器（最关键！）
    0x09, 0x03,        // USAGE (PID Device Control)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x02,        //   REPORT_ID (2)
    0x09, 0x04,        //   USAGE (PID Device Control Command)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x04,        //   LOGICAL_MAXIMUM (4)  // 支持的命令：0=停止,1=启用,2=禁用,3=停止所有,4=复位
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0xB1, 0x02,        //   FEATURE (Data,Var,Abs)
    0xC0,              // END_COLLECTION

    // 2.3 PID Block Load (ID=4) - 效果块加载状态
    0x09, 0x89,        // USAGE (PID Block Load)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x04,        //   REPORT_ID (4)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x00, 0x25, 0x3F, 0x75, 0x08, 0x95, 0x01, 0xB1, 0x02,
    0x09, 0x23,        //   USAGE (Effect Type)
    0x15, 0x00, 0x25, 0x0A, 0x75, 0x08, 0x95, 0x01, 0xB1, 0x02,
    0xC0,              // END_COLLECTION

    // 2.4 PID Device Gain (ID=0x7E) - 全局增益
    0x09, 0x7E,        // USAGE (Device Gain Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x7E,        //   REPORT_ID (0x7E)
    0x09, 0x7F,        //   USAGE (Device Gain)
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x10, 0x95, 0x01, 0xB1, 0x02,
    0xC0,              // END_COLLECTION

    // 2.5 PID New Effect Report (ID=0x7F) - 可选，增加兼容性
    0x09, 0x87,        // USAGE (New Effect Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x7F,        //   REPORT_ID (0x7F)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x00, 0x25, 0x3F, 0x75, 0x08, 0x95, 0x01, 0xB1, 0x02,
    0x09, 0x23,        //   USAGE (Effect Type)
    0x15, 0x00, 0x25, 0x0A, 0x75, 0x08, 0x95, 0x01, 0xB1, 0x02,
    0x09, 0x50,        //   USAGE (Duration)
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x10, 0x95, 0x01, 0xB1, 0x02,
    0x09, 0x52,        //   USAGE (Gain)
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x10, 0x95, 0x01, 0xB1, 0x02,
    0x09, 0x56,        //   USAGE (Enable Direction)
    0x15, 0x00, 0x25, 0x01, 0x75, 0x08, 0x95, 0x01, 0xB1, 0x02,
    0xC0,              // END_COLLECTION

    // ========== 3. PID Output Reports (游戏→设备) ==========
    // 3.1 PID_SET_EFFECT (ID=0x21)
    0x09, 0x21,        // USAGE (Set Effect Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x21,        //   REPORT_ID (0x21)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x00, 0x25, 0x3F, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x23,        //   USAGE (Effect Type)
    0x15, 0x00, 0x25, 0x0A, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x50,        //   USAGE (Duration)
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x52,        //   USAGE (Gain)
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x56,        //   USAGE (Enable Direction)
    0x15, 0x00, 0x25, 0x01, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
    0xC0,              // END_COLLECTION

    // 3.2 PID_EFFECT_OPERATION (ID=0x77)
    0x09, 0x77,        // USAGE (Effect Operation Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x77,        //   REPORT_ID (0x77)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x00, 0x25, 0x3F, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x7C,        //   USAGE (Loop Count)
    0x15, 0x00, 0x25, 0xFF, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x79,        //   USAGE (Effect Start)
    0x15, 0x00, 0x25, 0x01, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
    0xC0,              // END_COLLECTION

    // 3.3 PID_SET_CONSTANT (ID=0x73) - 常量力
    0x09, 0x73,        // USAGE (Set Constant Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x73,        //   REPORT_ID (0x73)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x00, 0x25, 0x3F, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x70,        //   USAGE (Magnitude)
    0x15, 0x00, 0x26, 0xFF, 0x7F, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
    0xC0,              // END_COLLECTION

    // 3.4 PID_SET_PERIODIC (ID=0x6E) - 周期性力（振动）
    0x09, 0x6E,        // USAGE (Set Periodic Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x6E,        //   REPORT_ID (0x6E)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x00, 0x25, 0x3F, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x70,        //   USAGE (Magnitude)
    0x15, 0x00, 0x26, 0xFF, 0x7F, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x71,        //   USAGE (Period)
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x6F,        //   USAGE (Offset)
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x72,        //   USAGE (Phase)
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x6B,        //   USAGE (Waveform)
    0x15, 0x00, 0x25, 0x06, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
    0xC0,              // END_COLLECTION

    // 3.5 PID_SET_CONDITION (ID=0x5F) - 条件力（弹簧/阻尼/摩擦）
    0x09, 0x5F,        // USAGE (Set Condition Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x5F,        //   REPORT_ID (0x5F)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x00, 0x25, 0x3F, 0x75, 0x08, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x60,        //   USAGE (CP Offset)
    0x15, 0x00, 0x26, 0xFF, 0x7F, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x61,        //   USAGE (Positive Coefficient)
    0x15, 0x00, 0x26, 0xFF, 0x7F, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
    0x09, 0x62,        //   USAGE (Negative Coefficient)
    0x15, 0x00, 0x26, 0xFF, 0x7F, 0x75, 0x10, 0x95, 0x01, 0x91, 0x02,
    0xC0,              // END_COLLECTION

    // 添加在Output Reports区域（0xF0报告之前即可）
    // 0x05, 0x0F,        // USAGE_PAGE (Physical Interface Device)
    // 0x09, 0x21,        // USAGE (Set Effect Report) — 复用标准Usage
    // 0xA1, 0x02,        // COLLECTION (Logical)
    // 0x85, 0x02,        // REPORT_ID (2)
    // 0x09, 0x70,        // USAGE (Magnitude)  // 力矩值
    // 0x15, 0x00,        // LOGICAL_MINIMUM (0)
    // 0x26, 0xFF, 0x7F,  // LOGICAL_MAXIMUM (32767)
    // 0x75, 0x10,        // REPORT_SIZE (16)
    // 0x95, 0x01,        // REPORT_COUNT (1)
    // 0x91, 0x02,        // OUTPUT (Data,Var,Abs)
    // 0xC0,              // END_COLLECTION

    // ========== 4. 自定义配置报告（上位机专用，ID=0xF0）==========
    0x05, 0x0F,        // USAGE_PAGE (Physical Interface Device)
    0x09, 0xFF,        // USAGE (Vendor Defined)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0xF0,        //   REPORT_ID (0xF0)
    0x09, 0xFF,        //   USAGE (Vendor Defined)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,  //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x08,        //   REPORT_COUNT (8)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)
    0xC0,              // END_COLLECTION

    0xC0               // END_COLLECTION (Application)
};
#endif

#if 1
static const uint8_t hid_handle_report_desc[] = {
    // ========== 应用集合 ==========
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,        // USAGE (Joystick)
    0xA1, 0x01,        // COLLECTION (Application)

    // ---------- 物理集合（输入报告） ----------
    0xA1, 0x00,        //   COLLECTION (Physical)

    // 输入报告 ID=1 (方向盘轴、其他轴、按钮)
    0x85, 0x01,        //     REPORT_ID (1)

    // 方向盘轴 (作为X轴)
    0x05, 0x01,        //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,        //     USAGE (X)
    0x15, 0x00,        //     LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,  //     LOGICAL_MAXIMUM (32767)
    0x75, 0x10,        //     REPORT_SIZE (16)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0x81, 0x02,        //     INPUT (Data,Var,Abs)

    // 其他轴 (Y, Z, Rx, Ry)
    0x09, 0x31,        //     USAGE (Y)
    0x09, 0x32,        //     USAGE (Z)
    0x09, 0x33,        //     USAGE (Rx)
    0x09, 0x34,        //     USAGE (Ry)
    0x15, 0x00,        //     LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,  //     LOGICAL_MAXIMUM (255)
    0x75, 0x08,        //     REPORT_SIZE (8)
    0x95, 0x04,        //     REPORT_COUNT (4)
    0x81, 0x02,        //     INPUT (Data,Var,Abs)

    // 32个按钮
    0x05, 0x09,        //     USAGE_PAGE (Button)
    0x19, 0x01,        //     USAGE_MINIMUM (Button 1)
    0x29, 0x20,        //     USAGE_MAXIMUM (Button 32)
    0x15, 0x00,        //     LOGICAL_MINIMUM (0)
    0x25, 0x01,        //     LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //     REPORT_SIZE (1)
    0x95, 0x20,        //     REPORT_COUNT (32)
    0x81, 0x02,        //     INPUT (Data,Var,Abs)

    0xC0,              //   END_COLLECTION

    // ---------- PID 状态报告 (输入, ID=2) ----------
    0x05, 0x0F,        // USAGE_PAGE (Physical Interface)
    0x09, 0x92,        // USAGE (PID State report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x02,        //   REPORT_ID (2)
    0x09, 0x9F,        //   USAGE (Device is Pause)
    0x09, 0xA0,        //   USAGE (Actuators Enabled)
    0x09, 0xA4,        //   USAGE (Safety Switch)
    0x09, 0xA6,        //   USAGE (Actuator Power)
    0x09, 0x94,        //   USAGE (Effect Playing)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
    0x35, 0x00,        //   PHYSICAL_MINIMUM (0)
    0x45, 0x01,        //   PHYSICAL_MAXIMUM (1)
    0x75, 0x01,        //   REPORT_SIZE (1)
    0x95, 0x05,        //   REPORT_COUNT (5)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)
    0x95, 0x03,        //   REPORT_COUNT (3)
    0x81, 0x03,        //   INPUT (Constant,Var)
    0xC0,              //   END_COLLECTION

    // ---------- Set Effect Report (输出, ID=3) ----------
    0x09, 0x21,        // USAGE (Set Effect Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x03,        //   REPORT_ID (3)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x01,        //   LOGICAL_MINIMUM (1)
    0x25, 0x28,        //   LOGICAL_MAXIMUM (40)
    0x35, 0x01,        //   PHYSICAL_MINIMUM (1)
    0x45, 0x28,        //   PHYSICAL_MAXIMUM (40)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x25,        //   USAGE (Effect Type)
    0xA1, 0x02,        //   COLLECTION (Logical)
    0x09, 0x26,        //     USAGE (ET Constant Force)
    0x09, 0x27,        //     USAGE (ET Ramp)
    0x09, 0x28,        //     USAGE (ET Square)
    0x09, 0x29,        //     USAGE (ET Sine)
    0x09, 0x2A,        //     USAGE (ET Triangle)
    0x09, 0x2B,        //     USAGE (ET Sawtooth Up)
    0x09, 0x2C,        //     USAGE (ET Sawtooth Down)
    0x09, 0x2D,        //     USAGE (ET Spring)
    0x09, 0x2E,        //     USAGE (ET Damper)
    0x09, 0x2F,        //     USAGE (ET Inertia)
    0x09, 0x30,        //     USAGE (ET Friction)
    0x25, 0x0B,        //     LOGICAL_MAXIMUM (11)
    0x15, 0x01,        //     LOGICAL_MINIMUM (1)
    0x35, 0x01,        //     PHYSICAL_MINIMUM (1)
    0x45, 0x0B,        //     PHYSICAL_MAXIMUM (11)
    0x75, 0x08,        //     REPORT_SIZE (8)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0x91, 0x00,        //     OUTPUT (Data,Var,Abs)
    0xC0,              //   END_COLLECTION

    0x09, 0x50,        //   USAGE (Duration)
    0x09, 0x54,        //   USAGE (Trigger Repeat Interval)
    0x09, 0x51,        //   USAGE (Sample Period)
    0x09, 0xA7,        //   USAGE (Start Delay)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,  //   LOGICAL_MAXIMUM (32767)
    0x35, 0x00,        //   PHYSICAL_MINIMUM (0)
    0x46, 0xFF, 0x7F,  //   PHYSICAL_MAXIMUM (32767)
    0x66, 0x03, 0x10,  //   UNIT (1003h)
    0x55, 0xFD,        //   UNIT_EXPONENT (-3)
    0x75, 0x10,        //   REPORT_SIZE (16)
    0x95, 0x04,        //   REPORT_COUNT (4)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x55, 0x00,        //   UNIT_EXPONENT (0)
    0x66, 0x00, 0x00,  //   UNIT (None)
    0x09, 0x52,        //   USAGE (Gain)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,  //   LOGICAL_MAXIMUM (255)
    0x35, 0x00,        //   PHYSICAL_MINIMUM (0)
    0x46, 0x10, 0x27,  //   PHYSICAL_MAXIMUM (10000)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x53,        //   USAGE (Trigger Button)
    0x15, 0x01,        //   LOGICAL_MINIMUM (1)
    0x25, 0x08,        //   LOGICAL_MAXIMUM (8)
    0x35, 0x01,        //   PHYSICAL_MINIMUM (1)
    0x45, 0x08,        //   PHYSICAL_MAXIMUM (8)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x55,        //   USAGE (Axes Enable)
    0xA1, 0x02,        //   COLLECTION (Logical)
    0x05, 0x01,        //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,        //     USAGE (X)
    0x15, 0x00,        //     LOGICAL_MINIMUM (0)
    0x25, 0x00,        //     LOGICAL_MAXIMUM (0)
    0x75, 0x01,        //     REPORT_SIZE (1)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0x91, 0x02,        //     OUTPUT (Data,Var,Abs)
    0xC0,              //   END_COLLECTION

    0x05, 0x0F,        //   USAGE_PAGE (Physical Interface)
    0x09, 0x56,        //   USAGE (Direction Enable)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x95, 0x06,        //   REPORT_COUNT (6)
    0x91, 0x03,        //   OUTPUT (Constant,Var)

    0x09, 0x57,        //   USAGE (Direction)
    0xA1, 0x02,        //   COLLECTION (Logical)
    0x0B, 0x01, 0x00, 0x0A, 0x00, //     USAGE (Ordinals:Instance 1)
    0x15, 0x00,        //     LOGICAL_MINIMUM (0)
    0x27, 0xA0, 0x8C, 0x00, 0x00, //     LOGICAL_MAXIMUM (36000)
    0x35, 0x00,        //     PHYSICAL_MINIMUM (0)
    0x47, 0xA0, 0x8C, 0x00, 0x00, //     PHYSICAL_MAXIMUM (36000)
    0x66, 0x00, 0x00,  //     UNIT (None)
    0x75, 0x10,        //     REPORT_SIZE (16)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0x91, 0x02,        //     OUTPUT (Data,Var,Abs)
    0xC0,              //   END_COLLECTION

    0x09, 0x58,        //   USAGE (Type Specific Block Offset)
    0xA1, 0x02,        //   COLLECTION (Logical)
    0x0B, 0x01, 0x00, 0x0A, 0x00, //     USAGE (Ordinals:Instance 1)
    0x26, 0xFD, 0x7F,  //     LOGICAL_MAXIMUM (32765)
    0x75, 0x10,        //     REPORT_SIZE (16)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0x91, 0x02,        //     OUTPUT (Data,Var,Abs)
    0xC0,              //   END_COLLECTION
    0xC0,              // END_COLLECTION

    // ---------- Set Envelope Report (输出, ID=4) ----------
    0x09, 0x5A,        // USAGE (Set Envelope Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x04,        //   REPORT_ID (4)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x01,        //   LOGICAL_MINIMUM (1)
    0x25, 0x28,        //   LOGICAL_MAXIMUM (40)
    0x35, 0x01,        //   PHYSICAL_MINIMUM (1)
    0x45, 0x28,        //   PHYSICAL_MAXIMUM (40)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x5B,        //   USAGE (Attack Level)
    0x09, 0x5D,        //   USAGE (Fade Level)
    0x16, 0x00, 0x00,  //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,  //   LOGICAL_MAXIMUM (32767)
    0x36, 0x00, 0x00,  //   PHYSICAL_MINIMUM (0)
    0x46, 0xFF, 0x7F,  //   PHYSICAL_MAXIMUM (32767)
    0x75, 0x10,        //   REPORT_SIZE (16)
    0x95, 0x02,        //   REPORT_COUNT (2)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x5C,        //   USAGE (Attack Time)
    0x09, 0x5E,        //   USAGE (Fade Time)
    0x66, 0x03, 0x10,  //   UNIT (1003h)
    0x55, 0xFD,        //   UNIT_EXPONENT (-3)
    0x27, 0xFF, 0x7F, 0x00, 0x00, //   LOGICAL_MAXIMUM (32767)
    0x47, 0xFF, 0x7F, 0x00, 0x00, //   PHYSICAL_MAXIMUM (32767)
    0x75, 0x20,        //   REPORT_SIZE (32)
    0x95, 0x02,        //   REPORT_COUNT (2)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x66, 0x00, 0x00,  //   UNIT (None)
    0x55, 0x00,        //   UNIT_EXPONENT (0)
    0xC0,              // END_COLLECTION

    // ---------- Set Condition Report (输出, ID=5) ----------
    0x09, 0x5F,        // USAGE (Set Condition Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x05,        //   REPORT_ID (5)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x01,        //   LOGICAL_MINIMUM (1)
    0x25, 0x28,        //   LOGICAL_MAXIMUM (40)
    0x35, 0x01,        //   PHYSICAL_MINIMUM (1)
    0x45, 0x28,        //   PHYSICAL_MAXIMUM (40)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x23,        //   USAGE (Parameter Block Offset)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x03,        //   LOGICAL_MAXIMUM (3)
    0x35, 0x00,        //   PHYSICAL_MINIMUM (0)
    0x45, 0x03,        //   PHYSICAL_MAXIMUM (3)
    0x75, 0x06,        //   REPORT_SIZE (6)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x58,        //   USAGE (Type Specific Block Offset)
    0xA1, 0x02,        //   COLLECTION (Logical)
    0x0B, 0x01, 0x00, 0x0A, 0x00, //     USAGE (Ordinals:Instance 1)
    0x75, 0x02,        //     REPORT_SIZE (2)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0x91, 0x02,        //     OUTPUT (Data,Var,Abs)
    0xC0,              //   END_COLLECTION

    0x16, 0x00, 0x80,  //   LOGICAL_MINIMUM (-32767)
    0x26, 0xFF, 0x7F,  //   LOGICAL_MAXIMUM (32767)
    0x36, 0x00, 0x80,  //   PHYSICAL_MINIMUM (-32767)
    0x46, 0xFF, 0x7F,  //   PHYSICAL_MAXIMUM (32767)

    0x09, 0x60,        //   USAGE (CP Offset)
    0x75, 0x10,        //   REPORT_SIZE (16)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x61,        //   USAGE (Positive Coefficient)
    0x09, 0x62,        //   USAGE (Negative Coefficient)
    0x95, 0x02,        //   REPORT_COUNT (2)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x16, 0x00, 0x00,  //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,  //   LOGICAL_MAXIMUM (32767)
    0x36, 0x00, 0x00,  //   PHYSICAL_MINIMUM (0)
    0x46, 0xFF, 0x7F,  //   PHYSICAL_MAXIMUM (32767)

    0x09, 0x63,        //   USAGE (Positive Saturation)
    0x09, 0x64,        //   USAGE (Negative Saturation)
    0x95, 0x02,        //   REPORT_COUNT (2)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x65,        //   USAGE (Dead Band)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)
    0xC0,              // END_COLLECTION

    // ---------- Set Periodic Report (输出, ID=6) ----------
    0x09, 0x6E,        // USAGE (Set Periodic Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x06,        //   REPORT_ID (6)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x01,        //   LOGICAL_MINIMUM (1)
    0x25, 0x28,        //   LOGICAL_MAXIMUM (40)
    0x35, 0x01,        //   PHYSICAL_MINIMUM (1)
    0x45, 0x28,        //   PHYSICAL_MAXIMUM (40)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x70,        //   USAGE (Magnitude)
    0x16, 0x00, 0x00,  //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x7F,  //   LOGICAL_MAXIMUM (32767)
    0x36, 0x00, 0x00,  //   PHYSICAL_MINIMUM (0)
    0x46, 0xFF, 0x7F,  //   PHYSICAL_MAXIMUM (32767)
    0x75, 0x10,        //   REPORT_SIZE (16)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x6F,        //   USAGE (Offset)
    0x16, 0x00, 0x80,  //   LOGICAL_MINIMUM (-32767)
    0x26, 0xFF, 0x7F,  //   LOGICAL_MAXIMUM (32767)
    0x36, 0x00, 0x80,  //   PHYSICAL_MINIMUM (-32767)
    0x46, 0xFF, 0x7F,  //   PHYSICAL_MAXIMUM (32767)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x71,        //   USAGE (Phase)
    0x66, 0x14, 0x00,  //   UNIT (14h)
    0x55, 0xFE,        //   UNIT_EXPONENT (-2)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x27, 0x9F, 0x8C, 0x00, 0x00, //   LOGICAL_MAXIMUM (35999)
    0x35, 0x00,        //   PHYSICAL_MINIMUM (0)
    0x47, 0x9F, 0x8C, 0x00, 0x00, //   PHYSICAL_MAXIMUM (35999)
    0x75, 0x10,        //   REPORT_SIZE (16)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x72,        //   USAGE (Period)
    0x15, 0x01,        //   LOGICAL_MINIMUM (1)
    0x27, 0xFF, 0x7F, 0x00, 0x00, //   LOGICAL_MAXIMUM (32767)
    0x35, 0x01,        //   PHYSICAL_MINIMUM (1)
    0x47, 0xFF, 0x7F, 0x00, 0x00, //   PHYSICAL_MAXIMUM (32767)
    0x66, 0x03, 0x10,  //   UNIT (1003h)
    0x55, 0xFD,        //   UNIT_EXPONENT (-3)
    0x75, 0x20,        //   REPORT_SIZE (32)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x66, 0x00, 0x00,  //   UNIT (None)
    0x55, 0x00,        //   UNIT_EXPONENT (0)
    0xC0,              // END_COLLECTION

    // ---------- Set Constant Force Report (输出, ID=7) ----------
    0x09, 0x73,        // USAGE (Set Constant Force Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x07,        //   REPORT_ID (7)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x01,        //   LOGICAL_MINIMUM (1)
    0x25, 0x28,        //   LOGICAL_MAXIMUM (40)
    0x35, 0x01,        //   PHYSICAL_MINIMUM (1)
    0x45, 0x28,        //   PHYSICAL_MAXIMUM (40)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x70,        //   USAGE (Magnitude)
    0x16, 0x00, 0x80,  //   LOGICAL_MINIMUM (-32767)
    0x26, 0xFF, 0x7F,  //   LOGICAL_MAXIMUM (32767)
    0x36, 0x00, 0x80,  //   PHYSICAL_MINIMUM (-32767)
    0x46, 0xFF, 0x7F,  //   PHYSICAL_MAXIMUM (32767)
    0x75, 0x10,        //   REPORT_SIZE (16)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)
    0xC0,              // END_COLLECTION

    // ---------- Set Ramp Force Report (输出, ID=8) ----------
    0x09, 0x74,        // USAGE (Set Ramp Force Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x08,        //   REPORT_ID (8)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x01,        //   LOGICAL_MINIMUM (1)
    0x25, 0x28,        //   LOGICAL_MAXIMUM (40)
    0x35, 0x01,        //   PHYSICAL_MINIMUM (1)
    0x45, 0x28,        //   PHYSICAL_MAXIMUM (40)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x75,        //   USAGE (Ramp Start)
    0x09, 0x76,        //   USAGE (Ramp End)
    0x16, 0x00, 0x80,  //   LOGICAL_MINIMUM (-32767)
    0x26, 0xFF, 0x7F,  //   LOGICAL_MAXIMUM (32767)
    0x36, 0x00, 0x80,  //   PHYSICAL_MINIMUM (-32767)
    0x46, 0xFF, 0x7F,  //   PHYSICAL_MAXIMUM (32767)
    0x75, 0x10,        //   REPORT_SIZE (16)
    0x95, 0x02,        //   REPORT_COUNT (2)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)
    0xC0,              // END_COLLECTION

    // ---------- Effect Operation Report (输出, ID=9) ----------
    0x09, 0x77,        // USAGE (Effect Operation Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x09,        //   REPORT_ID (9)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x01,        //   LOGICAL_MINIMUM (1)
    0x25, 0x28,        //   LOGICAL_MAXIMUM (40)
    0x35, 0x01,        //   PHYSICAL_MINIMUM (1)
    0x45, 0x28,        //   PHYSICAL_MAXIMUM (40)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    0x09, 0x78,        //   USAGE (Effect Operation)
    0xA1, 0x02,        //   COLLECTION (Logical)
    0x09, 0x79,        //     USAGE (Op Effect Start)
    0x09, 0x7A,        //     USAGE (Op Effect Start Solo)
    0x09, 0x7B,        //     USAGE (Op Effect Stop)
    0x15, 0x01,        //     LOGICAL_MINIMUM (1)
    0x25, 0x03,        //     LOGICAL_MAXIMUM (3)
    0x75, 0x08,        //     REPORT_SIZE (8)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0x91, 0x00,        //     OUTPUT
    0xC0,              //   END_COLLECTION

    0x09, 0x7C,        //   USAGE (Loop Count)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,  //   LOGICAL_MAXIMUM (255)
    0x35, 0x00,        //   PHYSICAL_MINIMUM (0)
    0x46, 0xFF, 0x00,  //   PHYSICAL_MAXIMUM (255)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)
    0xC0,              // END_COLLECTION

    // ---------- PID Block Free Report (输出, ID=10) ----------
    0x09, 0x90,        // USAGE (PID Block Free Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x0A,        //   REPORT_ID (10)
    0x09, 0x22,        //   USAGE (Effect Block Index)
    0x15, 0x01,        //   LOGICAL_MINIMUM (1)
    0x25, 0x28,        //   LOGICAL_MAXIMUM (40)
    0x35, 0x01,        //   PHYSICAL_MINIMUM (1)
    0x45, 0x28,        //   PHYSICAL_MAXIMUM (40)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)
    0xC0,              // END_COLLECTION

    // ---------- PID Device Control (输出, ID=11) ----------
    0x09, 0x95,        // USAGE (PID Device Control)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x0B,        //   REPORT_ID (11)
    0x09, 0x96,        //   USAGE (PID Device Control)
    0xA1, 0x02,        //   COLLECTION (Logical)
    0x09, 0x97,        //     USAGE (DC Enable Actuators)
    0x09, 0x98,        //     USAGE (DC Disable Actuators)
    0x09, 0x99,        //     USAGE (DC Stop All Effects)
    0x09, 0x9A,        //     USAGE (DC Device Reset)
    0x09, 0x9B,        //     USAGE (DC Device Pause)
    0x09, 0x9C,        //     USAGE (DC Device Continue)
    0x15, 0x01,        //     LOGICAL_MINIMUM (1)
    0x25, 0x06,        //     LOGICAL_MAXIMUM (6)
    0x75, 0x01,        //     REPORT_SIZE (1)
    0x95, 0x08,        //     REPORT_COUNT (8)
    0x91, 0x02,        //     OUTPUT
    0xC0,              //   END_COLLECTION
    0xC0,              // END_COLLECTION

    // ---------- Device Gain Report (输出, ID=12) ----------
    0x09, 0x7D,        // USAGE (Device Gain Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x0C,        //   REPORT_ID (12)
    0x09, 0x7E,        //   USAGE (Device Gain)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,  //   LOGICAL_MAXIMUM (255)
    0x35, 0x00,        //   PHYSICAL_MINIMUM (0)
    0x46, 0x10, 0x27,  //   PHYSICAL_MAXIMUM (10000)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)
    0xC0,              // END_COLLECTION

    // ---------- Create New Effect Report (特征, ID=13) ----------
    0x09, 0xAB,        // USAGE (Create New Effect Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x0D,        //   REPORT_ID (13)
    0x09, 0x25,        //   USAGE (Effect Type)
    0xA1, 0x02,        //   COLLECTION (Logical)
    0x09, 0x26,        //     USAGE (ET Constant Force)
    0x09, 0x27,        //     USAGE (ET Ramp)
    0x09, 0x28,        //     USAGE (ET Square)
    0x09, 0x29,        //     USAGE (ET Sine)
    0x09, 0x2A,        //     USAGE (ET Triangle)
    0x09, 0x2B,        //     USAGE (ET Sawtooth Up)
    0x09, 0x2C,        //     USAGE (ET Sawtooth Down)
    0x09, 0x2D,        //     USAGE (ET Spring)
    0x09, 0x2E,        //     USAGE (ET Damper)
    0x09, 0x2F,        //     USAGE (ET Inertia)
    0x09, 0x30,        //     USAGE (ET Friction)
    0x25, 0x0B,        //     LOGICAL_MAXIMUM (11)
    0x15, 0x01,        //     LOGICAL_MINIMUM (1)
    0x35, 0x01,        //     PHYSICAL_MINIMUM (1)
    0x45, 0x0B,        //     PHYSICAL_MAXIMUM (11)
    0x75, 0x08,        //     REPORT_SIZE (8)
    0x95, 0x01,        //     REPORT_COUNT (1)
    0xB1, 0x00,        //     FEATURE (Data,Var,Abs)
    0xC0,              //   END_COLLECTION

    0x05, 0x01,        //   USAGE_PAGE (Generic Desktop)
    0x09, 0x3B,        //   USAGE (Byte Count)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x01,  //   LOGICAL_MAXIMUM (511)
    0x35, 0x00,        //   PHYSICAL_MINIMUM (0)
    0x46, 0xFF, 0x01,  //   PHYSICAL_MAXIMUM (511)
    0x75, 0x0A,        //   REPORT_SIZE (10)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0xB1, 0x02,        //   FEATURE (Data,Var,Abs)
    0x75, 0x06,        //   REPORT_SIZE (6)
    0xB1, 0x01,        //   FEATURE (Constant)
    0xC0,              // END_COLLECTION

    // ---------- PID Pool Report (特征, ID=14) ----------
    0x09, 0x7F,        // USAGE (PID Pool Report)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0x0E,        //   REPORT_ID (14)
    0x09, 0x80,        //   USAGE (RAM Pool size)
    0x75, 0x10,        //   REPORT_SIZE (16)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x35, 0x00,        //   PHYSICAL_MINIMUM (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00, //   LOGICAL_MAXIMUM (65535)
    0x47, 0xFF, 0xFF, 0x00, 0x00, //   PHYSICAL_MAXIMUM (65535)
    0xB1, 0x02,        //   FEATURE (Data,Var,Abs)

    0x09, 0x83,        //   USAGE (Simultaneous Effects Max)
    0x26, 0xFF, 0x00,  //   LOGICAL_MAXIMUM (255)
    0x46, 0xFF, 0x00,  //   PHYSICAL_MAXIMUM (255)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0xB1, 0x02,        //   FEATURE (Data,Var,Abs)

    0x09, 0xA9,        //   USAGE (Device Managed Pool)
    0x09, 0xAA,        //   USAGE (Shared Parameter Blocks)
    0x75, 0x01,        //   REPORT_SIZE (1)
    0x95, 0x02,        //   REPORT_COUNT (2)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
    0x35, 0x00,        //   PHYSICAL_MINIMUM (0)
    0x45, 0x01,        //   PHYSICAL_MAXIMUM (1)
    0xB1, 0x02,        //   FEATURE (Data,Var,Abs)

    0x75, 0x06,        //   REPORT_SIZE (6)
    0x95, 0x01,        //   REPORT_COUNT (1)
    0xB1, 0x03,        //   FEATURE (Constant)
    0xC0,              // END_COLLECTION
#if 1
    // ========== 自定义配置报告（上位机专用，ID=0xF0）==========
    0x05, 0x0F,        // USAGE_PAGE (Physical Interface Device)
    0x09, 0xFF,        // USAGE (Vendor Defined)
    0xA1, 0x02,        // COLLECTION (Logical)
    0x85, 0xF0,        //   REPORT_ID (0xF0)
    0x09, 0xFF,        //   USAGE (Vendor Defined)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,  //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x95, 0x08,        //   REPORT_COUNT (8)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)
    0xC0,              // END_COLLECTION
#endif

    // ---------- 应用集合结束 ----------
    0xC0               // END_COLLECTION (Application)
};
#endif


/*!< global descriptor */
uint8_t hid_descriptor[] = {
    // https://www.usb.org/defined-class-codes
    // bDeviceClass, bDeviceSubClass, bDeviceProtocol，openffb 填的是 0，2，1，但是似乎如果 class 是 0 后面都不会有效
    // openffb 标注的是 misc，那么照理他们应该填 ef，2，1，不知道为什么
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
    0x1E,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'T', 0x00,                  /* wcChar0 */
    'i', 0x00,                  /* wcChar1 */
    'X', 0x00,                  /* wcChar2 */
    ' ', 0x00,                  /* wcChar3 */
    'F', 0x00,                  /* wcChar4 */
    'F', 0x00,                  /* wcChar5 */
    'B', 0x00,                  /* wcChar6 */
    ' ', 0x00,                  /* wcChar7 */
    'H', 0x00,                  /* wcChar8 */
    'a', 0x00,                  /* wcChar9 */
    'n', 0x00,                  /* wcChar10 */
    'd', 0x00,                  /* wcChar11 */
    'l', 0x00,                  /* wcChar12 */
    'e', 0x00,                  /* wcChar13 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x12,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '6', 0x00,                  /* wcChar3 */
    '0', 0x00,                  /* wcChar4 */
    '3', 0x00,                  /* wcChar5 */
    '2', 0x00,                  /* wcChar6 */
    '6', 0x00,                  /* wcChar7 */
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


// 全局报告实例
struct hid_handle_up handle_up;

// hid 下行数据 buffer
USB_MEM_ALIGNX uint8_t hid_down_buffer[64];

// usb 初始化完成回调
void usbd_configure_done_callback(void){
    // 开启 usb 接收
    usbd_ep_start_read(HID_OUT_EP, hid_down_buffer, HID_OUT_EP_SIZE);
}

// 上行数据端点传输完成回调
static void usbd_hid_up_callback(uint8_t ep, uint32_t nbytes){

    // 发布上传完成事件
    ltx_Topic_publish(&topic_hid_upload_over);
}

// 下行数据端点传输完成回调
static void usbd_hid_down_callback(uint8_t ep, uint32_t nbytes){
    if(nbytes){
        if(hid_down_buffer[0] >= 3 && hid_down_buffer[0] <= 12){
            ffb_parse_data(hid_down_buffer, nbytes);
        }
        switch(hid_down_buffer[0]){ // report id
            // case 3:
            // case 4:
            // case 5:
            // case 6:
            // case 7:
            // case 8:
            // case 9:
            // case 10:
            // case 11:
            // case 12:
            //     ffb_prase_data(hid_down_buffer, nbytes);
            //     break;
            default:
                // 未知报告ID，忽略
                break;
            // 自定义配置，预留给上位机用
            case 0xF0:
                switch(hid_down_buffer[1]){
                    case 0x01: // 设置当前方向盘位置为中点
                        LTX_LOG_DEBG("USB Set mid.\n");
                        handle_wheel_set_zero(&handle_wheel_data);
                        break;
                        
                    case 0x02: // 设置单边最大圈数
                        if(hid_down_buffer[2] == 0 && hid_down_buffer[3] == 0){
                            hid_down_buffer[3] = 10;
                        }
                        LTX_LOG_DEBG("USB Set max turns: %d.%02d\n", hid_down_buffer[2], hid_down_buffer[3]);
                        int32_t usb_set_max_turns = hid_down_buffer[2]*100 + hid_down_buffer[3];
                        handle_wheel_set_max_turns(&handle_wheel_data, usb_set_max_turns);
                        
                        break;

                    default:
                        LTX_LOG_WARN("Unknown USB Head: 0x%x\n", hid_down_buffer[1]);
                        break;
                }
                break;
        }
    }
    #if 0
    static uint8_t print_count = 0;
    if(print_count < 100){
    print_count ++;
    LTX_LOG_FMT("L:%d,0x%x,%x,%x,%x,%x,%x,", nbytes,
                                                hid_down_buffer[0],
                                                hid_down_buffer[1],
                                                hid_down_buffer[2],
                                                hid_down_buffer[3],
                                                hid_down_buffer[4],
                                                hid_down_buffer[5]);
    LTX_LOG_FMT("%x,%x,%x,%x,%x,%x\n", hid_down_buffer[6],
                                                hid_down_buffer[7],
                                                hid_down_buffer[8],
                                                hid_down_buffer[9],
                                                hid_down_buffer[10],
                                                hid_down_buffer[11]);
    }
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
void usbd_hid_get_report(uint8_t busid, uint8_t intf, uint8_t report_id, uint8_t report_type, uint8_t **data, uint32_t *len)
{
    (void)busid;
    LTX_LOG_DEBG("GET report intf=%d id=0x%02x type=%d\n", intf, report_id, report_type);
    if (report_type == HID_REPORT_FEATURE && data && *data && len && *len > 0) {
        switch (report_id) {
            case 0x01: // PID_POOL_REPORT - 返回支持的效果数量（示例返回 1）
                (*data)[0] = 1;
                *len = 1;
                break;
            case 0x08: // PID Pool Report (新描述符中 ID=8) - 返回 RAM Pool Size(16bit), Simultaneous Effects Max(8bit), 2 bits flags
                if (*len >= 5) {
                    uint16_t ram_pool_size = 1; // 主机期望非0值以识别能力，设置为1示例
                    uint8_t simultaneous_max = 1;
                    uint8_t flags = 0x00; // bit0: Device Managed Pool, bit1: Shared Parameter Blocks

                    (*data)[0] = 0x08; // report id
                    (*data)[1] = (uint8_t)(ram_pool_size & 0xFF);
                    (*data)[2] = (uint8_t)((ram_pool_size >> 8) & 0xFF);
                    (*data)[3] = simultaneous_max;
                    (*data)[4] = flags; // low bits + padding to fill to byte
                    *len = 5;
                } else {
                    *len = 0;
                }
                break;
            case 0x02: // PID_DEVICE_CONTROL
                (*data)[0] = 0;
                *len = 1;
                break;
            default:
                *len = 0;
                break;
        }
    } else {
        if (len) *len = 0;
    }
}
void usbd_hid_set_report(uint8_t busid, uint8_t intf, uint8_t report_id, uint8_t report_type, uint8_t *report, uint32_t report_len)
{
    (void)busid;
    // 记录日志，便于调试主机是否走 control SET_REPORT
    LTX_LOG_DEBG("HID SET_REPORT: intf=%d id=0x%02x type=%d len=%lu\n", intf, report_id, report_type, (unsigned long)report_len);

    if (report_type == HID_REPORT_FEATURE) {
        switch (report_id) {
            case 0x7E: // PID_DEVICE_GAIN (Device Gain Report)
                if (report_len >= 3) {
                    uint16_t gain = (uint16_t)(report[1] | (report[2] << 8));
                    LTX_LOG_DEBG("HID: set device gain=%u\n", gain);
                }
                break;
            case 0x04: // PID_BLOCK_LOAD - host 发送效果数据到设备
                LTX_LOG_DEBG("HID: PID_BLOCK_LOAD received, len=%lu\n", (unsigned long)report_len);
                break;
            case 0x01: // PID_POOL_REPORT - 主机查询设备支持多少效果（设置场景很少）
                break;
            default:
                break;
        }
    } else if (report_type == HID_REPORT_OUTPUT) {
        LTX_LOG_DEBG("HID: SET_REPORT OUTPUT id=0x%02x len=%lu\n", report_id, (unsigned long)report_len);
        if (report_len > 0 && report) {
            switch (report[0]) {
                case 0x21: // PID_SET_EFFECT
                    LTX_LOG_DEBG("HID: PID_SET_EFFECT idx=%d type=%d\n", report[1], report[2]);
                    break;
                case 0x73: // PID_SET_CONSTANT
                    if (report_len >= 4) {
                        uint8_t idx = report[1];
                        int16_t magnitude = (int16_t)((report[2] << 8) | report[3]);
                        LTX_LOG_DEBG("HID: PID_SET_CONSTANT idx=%d mag=%d\n", idx, magnitude);
                    }
                    break;
                case 0x6E: // PID_SET_PERIODIC
                    LTX_LOG_DEBG("HID: PID_SET_PERIODIC\n");
                    break;
                case 0x77: // PID_EFFECT_OPERATION
                    if (report_len >= 4) {
                        uint8_t idx = report[1];
                        uint8_t loop = report[2];
                        uint8_t op = report[3];
                        LTX_LOG_DEBG("HID: PID_EFFECT_OPERATION idx=%d op=%d loop=%d\n", idx, op, loop);
                    }
                    break;
                case 0x5F: // PID_SET_CONDITION
                    LTX_LOG_DEBG("HID: PID_SET_CONDITION\n");
                    break;
                case 0xF0: // 自定义报告ID
                    LTX_LOG_DEBG("HID: vendor custom report F0\n");
                    break;
                default:
                    break;
            }
        }
    }
}
#else
// 另一个 ai 写的
// 全局设备状态
static struct {
    uint8_t actuators_enabled;          // 执行器使能标志（1使能）
    uint8_t device_paused;              // 设备暂停标志
    uint8_t safety_switch;              // 安全开关（始终为1表示正常）
    uint8_t actuator_power;             // 执行器供电（1正常）
    uint8_t effect_playing;             // 是否有任何效果在播放（用于状态报告）
    uint16_t global_gain;               // 全局增益（0~10000）
    uint32_t effect_pool_size;          // RAM 池大小（返回给主机）
    uint8_t max_simultaneous;           // 最大同时效果数
    uint8_t device_managed_pool;        // 设备管理池标志（0）
    uint8_t shared_parameter_blocks;    // 共享参数块标志（0）
} g_ffb_state = {
    .actuators_enabled = 1,             // 默认使能
    .device_paused = 0,
    .safety_switch = 1,
    .actuator_power = 1,
    .effect_playing = 0,
    .global_gain = 10000,               // 最大增益
    .effect_pool_size = 4096,           // 随便给一个值
    .max_simultaneous = MAX_EFFECT_BLOCKS,
    .device_managed_pool = 0,
    .shared_parameter_blocks = 0,
};

// 效果块数组
ffb_effect_block_t g_effect_blocks[MAX_EFFECT_BLOCKS];

// 查找空闲效果块索引（1~MAX_EFFECT_BLOCKS），返回0表示无空闲
static uint8_t find_free_effect_block(void) {
    for (uint8_t i = 0; i < MAX_EFFECT_BLOCKS; i++) {
        if (g_effect_blocks[i].state == EFFECT_STATE_FREE) {
            return i + 1;  // 效果块索引从1开始
        }
    }
    return 0;
}

// 根据索引获取效果块指针（索引1~MAX_EFFECT_BLOCKS）
static ffb_effect_block_t *get_effect_block(uint8_t index) {
    if (index == 0 || index > MAX_EFFECT_BLOCKS) return NULL;
    return &g_effect_blocks[index - 1];
}

// 释放效果块
static void free_effect_block(uint8_t index) {
    ffb_effect_block_t *block = get_effect_block(index);
    if (block) {
        memset(block, 0, sizeof(ffb_effect_block_t));
        block->state = EFFECT_STATE_FREE;
    }
    // 更新全局效果播放标志
    g_ffb_state.effect_playing = 0;
    for (int i = 0; i < MAX_EFFECT_BLOCKS; i++) {
        if (g_effect_blocks[i].state == EFFECT_STATE_PLAYING) {
            g_ffb_state.effect_playing = 1;
            break;
        }
    }
}

void usbd_hid_get_report(uint8_t busid, uint8_t intf, uint8_t report_id, uint8_t report_type, uint8_t **data, uint32_t *len){
    
    static uint8_t report_buffer[64];  // 静态缓冲区，返回给主机

    LTX_LOG_DEBG("GET report intf=%d id=0x%02x type=%d\n", intf, report_id, report_type);

    // 只处理 Feature 报告和 Input 报告
    if (report_type == HID_REPORT_INPUT || report_type == HID_REPORT_FEATURE) {
        memset(report_buffer, 0, sizeof(report_buffer));

        switch (report_id) {
            case 2:  // PID State Report (Input, ID=2)
                if (report_type == HID_REPORT_INPUT) {
                    // 构建状态报告
                    report_buffer[0] = 0x02;  // Report ID
                    // 位域：bit0: Device Pause, bit1: Actuators Enabled, bit2: Safety Switch, bit3: Actuator Power, bit4: Effect Playing
                    report_buffer[1] = (g_ffb_state.device_paused << 0) |
                                       (g_ffb_state.actuators_enabled << 1) |
                                       (g_ffb_state.safety_switch << 2) |
                                       (g_ffb_state.actuator_power << 3) |
                                       (g_ffb_state.effect_playing << 4);
                    // 其余3位常量填充（描述符中有3位常量）
                    report_buffer[1] |= 0xE0;  // 高3位为1（常量）
                    *data = report_buffer;
                    *len = 2;  // 报告长度（ID+1字节）
                }
                break;

            case 13: // Create New Effect Report (Feature, ID=13)
                if (report_type == HID_REPORT_FEATURE) {
                    // 主机请求创建新效果时，需要返回一个空闲的效果块索引
                    uint8_t effect_index = find_free_effect_block();
                    if (effect_index == 0) {
                        // 无空闲块，返回0表示失败
                        effect_index = 0;
                    }
                    // 构建特征报告数据
                    report_buffer[0] = 0x0D;  // Report ID
                    // 效果类型（这里简单返回0，实际应根据主机请求的用法值确定，但CherryUSB中此回调无参数）
                    // 实际上主机在Set Report中会发送类型，Get Report只是返回索引，这里简化处理
                    report_buffer[1] = 0;      // 效果类型占位（主机通常忽略）
                    // 字节计数（返回2个字节）
                    report_buffer[2] = 0x02;   // 低8位
                    report_buffer[3] = 0x00;   // 高8位
                    *data = report_buffer;
                    *len = 4;  // ID + 类型(1) + 计数(2)
                }
                break;

            case 14: // PID Pool Report (Feature, ID=14)
                if (report_type == HID_REPORT_FEATURE) {
                    // 返回设备能力
                    report_buffer[0] = 0x0E;  // Report ID
                    // RAM Pool size (32位)
                    report_buffer[1] = (g_ffb_state.effect_pool_size >> 0) & 0xFF;
                    report_buffer[2] = (g_ffb_state.effect_pool_size >> 8) & 0xFF;
                    report_buffer[3] = (g_ffb_state.effect_pool_size >> 16) & 0xFF;
                    report_buffer[4] = (g_ffb_state.effect_pool_size >> 24) & 0xFF;
                    // Simultaneous Effects Max (8位)
                    report_buffer[5] = g_ffb_state.max_simultaneous;
                    // Device Managed Pool (1位) 和 Shared Parameter Blocks (1位)
                    report_buffer[6] = (g_ffb_state.device_managed_pool << 0) |
                                       (g_ffb_state.shared_parameter_blocks << 1);
                    // 剩余6位常量填充（描述符中定义为常量）
                    report_buffer[6] |= 0xFC;
                    *data = report_buffer;
                    *len = 7;  // ID + 4 + 1 + 1
                }
                break;

            default:
                // 其他报告ID不支持，返回空
                *data = NULL;
                *len = 0;
                break;
        }
    } else {
        *data = NULL;
        *len = 0;
    }
}
void usbd_hid_set_report(uint8_t busid, uint8_t intf, uint8_t report_id, uint8_t report_type, uint8_t *report, uint32_t report_len){

    LTX_LOG_DEBG("HID SET_REPORT: intf=%d id=0x%02x type=%d len=%d\n", intf, report_id, report_type, report_len);

    // 只处理 Feature 报告
    if (report_type != HID_REPORT_FEATURE) {
        return;
    }

    switch (report_id) {
        case 13: // Create New Effect Report (Feature)
            // 主机请求创建一个新效果块，并指定效果类型
            if (report_len >= 2) {
                uint8_t effect_type = report[1];  // 报告第二个字节是效果类型
                uint8_t effect_index = find_free_effect_block();
                if (effect_index) {
                    ffb_effect_block_t *block = get_effect_block(effect_index);
                    block->state = EFFECT_STATE_CONFIGURED;
                    block->type = effect_type;
                    // 清空参数
                    memset(&block->params, 0, sizeof(block->params));
                    // 注意：主机不会在此报告里发送完整参数，后续会通过其他报告（Set Effect等）设置参数
                }
                // 如果无空闲块，主机会收到0索引（在后续Get Report中），这里不做返回
            }
            break;

        case 14: // PID Pool Report (Feature)
            // 主机可能会设置池参数（通常由设备返回，主机一般不写）
            // 可忽略或更新设备能力
            break;

        case 0x12: // Block Load Report (Feature, ID=0x12)
            // 主机请求加载效果块参数（实际上在Set Effect Report中已经处理，这里可选）
            // 可根据需要实现，通常不需要额外处理
            break;

        default:
            // 其他报告可能用于控制，例如PID Device Control等，但这些通常通过Output报告发送
            // 如果主机通过Feature报告发送，可以在这里解析
            break;
    }
}
#endif

// 接口结构体
struct usbd_interface intf0;

// 初始化函数
void hid_handle_init(void){

    // 注册设备描述符（需包含修改后的配置描述符）
    usbd_desc_register((const uint8_t *)hid_descriptor);

    // 添加 hid 接口（传入自定义报告描述符和回调），busid 使用 0
    usbd_add_interface(usbd_hid_init_intf(0, &intf0, (const uint8_t *)hid_handle_report_desc, HID_HANDLE_REPORT_DESC_SIZE));

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
int handle_upload(void){

    uint8_t report[64] = {0};
    report[0] = 0x01; // Report ID (与描述符一致)

        // 填充数据（按描述符顺序）
        report[1] = handle_up.wheel & 0xFF;
        report[2] = (handle_up.wheel >> 8) & 0xFF;
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
    return usbd_ep_start_write(HID_INT_EP, report, 11); // 实际有效字节数（不含填充）
}
