#ifndef __MYAPP_SYSTEM_H__
#define __MYAPP_SYSTEM_H__

#include "ltx_app.h"

#define SYS_ERROR_SPI                   0x0100
    #define SYS_ERROR_SPI_DEINIT    0x01
    #define SYS_ERROR_SPI_INIT      0x02
    #define SYS_ERROR_SPI_TX        0x04
#define SYS_ERROR_DMA                   0x0200
    #define SYS_ERROR_DMA_INIT      0x01
    #define SYS_ERROR_DMA_TX        0x02
#define SYS_ERROR_LCD                   0x0400
    #define SYS_ERROR_LCD_INIT      0x01

extern uint32_t SYS_ERROR_CODE;
extern const char *SYS_ERROR_MSG;
extern struct ltx_Topic_stu topic_sys_error;
void _SYS_ERROR(uint32_t code, const char *msg);

extern struct ltx_App_stu app_system;
extern struct ltx_Task_stu task_heart_beat;

#endif // __MYAPP_SYSTEM_H__
