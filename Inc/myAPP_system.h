#ifndef __MYAPP_SYSTEM_H__
#define __MYAPP_SYSTEM_H__

#include "ltx_app.h"

extern uint32_t SYS_ERROR_CODE;
extern const char *SYS_ERROR_MSG;
extern struct ltx_Topic_stu topic_sys_error;
void _SYS_ERROR(uint32_t code, const char *msg);

extern struct ltx_App_stu app_system;
extern struct ltx_Task_stu task_heart_beat;

#endif // __MYAPP_SYSTEM_H__
