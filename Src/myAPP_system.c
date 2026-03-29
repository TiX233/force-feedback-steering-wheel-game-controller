#include "myAPP_system.h"
#include "ltx.h"
#include "ltx_app.h"
#include "ltx_param.h"
#include "ltx_log.h"
#include "ltx_cmd.h"
#include "ltx_log_config.h"
#include "ltx_mtbuf.h"

void task_func_heart_beat(struct ltx_Task_stu *task);
void subscriber_cb_sys_error(void *param);


// 心拍周期任务对象
struct ltx_Task_stu task_heart_beat;

#if(_LTX_LOG_CHOOSE == _LTX_LOG_USE_RTT)
// 使用 rtt 接收命令
    // 命令处理周期任务
    void task_func_cmd(struct ltx_Task_stu *task);
    struct ltx_Task_stu task_cmd;
#elif(_LTX_LOG_CHOOSE == _LTX_LOG_USE_DMA)
// 使用 dma 外设接收命令
    #define CMD_BUF_NUM     3

    // 命令处理任务，dma 接收完成事件触发
    void task_func_cmd(void *param);
    // dma 接收完成事件
    struct ltx_Topic_stu topic_cmd_get = _LTX_TOPIC_DEAFULT_CONFIG(topic_cmd_get);
    struct ltx_Topic_subscriber_stu subscriber_cmd_get = _LTX_SUBSCRIBER_DEAFULT_CONFIG(task_func_cmd);

    uint8_t _cmd_bufs[CMD_BUF_NUM][CMD_BUF_SIZE];
    struct ltx_mtbuf_stu _cmd_bufs_buf[CMD_BUF_NUM];
    struct ltx_mtbuf_manager_stu _cmd_buf_manager;
    struct ltx_mtbuf_stu *_cmd_buffer_reciving;
#endif


// 系统错误码
uint32_t SYS_ERROR_CODE = 0;
const char *SYS_ERROR_MSG = "Okay";

// 系统错误话题
struct ltx_Topic_stu topic_sys_error = _LTX_TOPIC_DEAFULT_CONFIG(topic_sys_error);

// 系统错误话题订阅者
struct ltx_Topic_subscriber_stu subscriber_sys_error = _LTX_SUBSCRIBER_DEAFULT_CONFIG(subscriber_cb_sys_error);

// 错误码周期打印任务
struct ltx_Task_stu task_error_code = {.is_initialized = 0};

int myApp_system_init(struct ltx_App_stu *app){
    // 创建心拍周期任务
    ltx_Task_init(&task_heart_beat, task_func_heart_beat, 1000, 0);
    // 加入到 app 进行管理，这样就不用在 app pause 等操作内部显式操作 task，做 app pause 等操作时，会顺便操作下属 task
    ltx_Task_add_to_app(&task_heart_beat, app, "heart_beat");

    #if(_LTX_LOG_CHOOSE == _LTX_LOG_USE_RTT)
    // 使用 rtt 接收命令
        // 创建命令处理周期任务
        ltx_Task_init(&task_cmd, task_func_cmd, 200, 0);
        ltx_Task_add_to_app(&task_cmd, app, "cmd");
    #elif(_LTX_LOG_CHOOSE == _LTX_LOG_USE_DMA)
    // 使用 dma 外设接收命令
        for(uint32_t i = 0; i < CMD_BUF_NUM; i ++){
            ltx_mtbuf_buf_init(&_cmd_bufs_buf[i], CMD_BUF_SIZE, _cmd_bufs[i]);
        }
        ltx_mtbuf_manager_init(&_cmd_buf_manager, CMD_BUF_NUM, _cmd_bufs_buf);
        ltx_Topic_subscribe(&topic_cmd_get, &subscriber_cmd_get);
        _cmd_buffer_reciving = ltx_mtbuf_write_get(&_cmd_buf_manager);
        // 开启接收
        if(HAL_UART_Receive_DMA(&huart2_handler, (uint8_t *)_cmd_buffer_reciving->buf_ptr, _cmd_buffer_reciving->buf_size) != HAL_OK){
            while(1);
        }
        // 关闭串口 DMA 接收传输过半中断
        __HAL_DMA_DISABLE_IT(&hdma1ch4_handler, DMA_IT_HT);
    #endif

    // 订阅系统错误事件话题
    ltx_Topic_subscribe(&topic_sys_error, &subscriber_sys_error);

    return 0;
}

int myApp_system_pause(struct ltx_App_stu *app){

    return 0;
}

int myApp_system_resume(struct ltx_App_stu *app){

    return 0;
}

int myApp_system_destroy(struct ltx_App_stu *app){

    // free...

    return 0;
}


struct ltx_App_stu app_system = {
    .is_initialized = 0,
    .status = ltx_App_status_pause,
    .name = "system",

    .init = myApp_system_init,
    .pause = myApp_system_pause,
    .resume = myApp_system_resume,
    .destroy = myApp_system_destroy,

    .task_list = NULL,
    
    .next = NULL,
};


// 心拍任务
uint32_t heart_beat_count = 0;
void task_func_heart_beat(struct ltx_Task_stu *task){

    heart_beat_count ++;
    // LTX_LOG_DEBG("Heartbeat: %d\n", heart_beat_count);

}


#if(_LTX_LOG_CHOOSE == _LTX_LOG_USE_RTT)
// 使用 rtt 接收命令
uint8_t cmd_buffer[CMD_BUF_SIZE];
// 处理命令任务，定期轮询共享内存
void task_func_cmd(struct ltx_Task_stu *task){

    // 读取命令
    if(SEGGER_RTT_HasData(0)){
        int len = SEGGER_RTT_Read(0, cmd_buffer, CMD_BUF_SIZE - 1);
        if (len > 0) {
            cmd_buffer[len] = '\0'; // 添加字符串终止符
            for(uint8_t i = len - 1; i > 0; i --){ // 去除尾追回车
                if(cmd_buffer[i] == '\n' || cmd_buffer[i] == '\r'){
                    cmd_buffer[i] = 0;
                }else {
                    break;
                }
            }

            // LOG_FMT(PRINT_DEBUG"cmd len: %d\n", len);
            ltx_Cmd_process((char *)cmd_buffer); // 处理命令
        }
    }
}
#elif(_LTX_LOG_CHOOSE == _LTX_LOG_USE_DMA)
// 使用 dma 外设接收命令
// 处理命令任务，dma 接收完成事件触发
void task_func_cmd(void *param){
    // 读取命令
    struct ltx_mtbuf_stu *cmd_read_buf = ltx_mtbuf_read_get(&_cmd_buf_manager);
    if(cmd_read_buf != NULL){
        // LTX_LOG_DEBG("%s", cmd_read_buf->buf_ptr);
        if(cmd_read_buf->data_size > 0){
            if(cmd_read_buf->data_size >= cmd_read_buf->buf_size){
                cmd_read_buf->buf_ptr[cmd_read_buf->data_size - 1] = '\0'; // 添加字符串终止符
                cmd_read_buf->data_size = cmd_read_buf->buf_size;
            }else {
                cmd_read_buf->buf_ptr[cmd_read_buf->data_size] = '\0'; // 添加字符串终止符
            }
            for(uint32_t i = cmd_read_buf->data_size; i > 0; i --){ // 去除尾追回车
                if(cmd_read_buf->buf_ptr[i] == '\n' || cmd_read_buf->buf_ptr[i] == '\r' || cmd_read_buf->buf_ptr[i] == 0){
                    cmd_read_buf->buf_ptr[i] = 0;
                }else {
                    break;
                }
            }
            ltx_Cmd_process((char *)cmd_read_buf->buf_ptr); // 处理命令
        }
        ltx_mtbuf_read_over(&_cmd_buf_manager, cmd_read_buf);
    }
}
#endif

// 系统错误码每秒打印周期任务
void task_func_error_code(struct ltx_Task_stu *task){

    LTX_LOG_ERRO("Error: 0x%08x, %s\n", SYS_ERROR_CODE, SYS_ERROR_MSG);
}

// 系统错误订阅回调
void subscriber_cb_sys_error(void *param){
    // 系统发生错误
    // 创建一个不断打印错误码的任务
    if(!task_error_code.is_initialized){
        ltx_Task_init(&task_error_code, task_func_error_code, 1000, 1000);
        ltx_Task_add_to_app(&task_error_code, &app_system, "error_code");
        // 运行
        ltx_Task_resume(&task_error_code);
    }

    // 关闭其他 app
    // todo

    // 蓝屏显示错误码
    #if 0
    extern struct gc9a01_stu myLCD;
    if(myLCD.is_initialized){
		// gc9a01_clear(&myLCD, RGB565_BLUE);
        HAL_SPI_DMAStop(&spi1_handler);
		uint8_t _color[2];
		_color[1] = RGB565_BLUE & 0xFF;
		_color[0] = RGB565_BLUE >> 8;
		gc9a01_set_window(&myLCD, 0, 0, 239, 239);
		myLCD.write_dc(GC9A01_PIN_LEVEL_DC_DATA);
		for(uint32_t i = 0; i < 240*240; i ++)
			myLCD.transmit_data(_color, 2);
		
        // 错误码 todo
    }
    #endif
}

// 发布系统错误 api
void _SYS_ERROR(uint32_t code, const char *msg){
    SYS_ERROR_CODE = code;
    SYS_ERROR_MSG = msg;

    ltx_Topic_publish(&topic_sys_error);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    ltx_Log_dma_send_over_handler();
}
void HAL_UART_IdleFrameDetectCpltCallback(UART_HandleTypeDef *huart){
    // LTX_LOG_STR("T2\n");

    struct ltx_mtbuf_stu *buf_next_recive = ltx_mtbuf_write_get(&_cmd_buf_manager);
    if(buf_next_recive != NULL){ // 能分配新的 buffer 给下次接收
        ltx_mtbuf_write_over(&_cmd_buf_manager, _cmd_buffer_reciving); // 完成此次接收
        _cmd_buffer_reciving = buf_next_recive;
        _cmd_buffer_reciving->data_size = huart->RxXferSize - huart->RxXferCount;
    }else { // 如果没有新的 buffer 能够分配给下次接收，那么此次接收的数据作废，用此次的 buf 准备接收新数据
        buf_next_recive = _cmd_buffer_reciving;
        // buf_next_recive->data_size = 0;
    }
    if ((huart->RxState == HAL_UART_STATE_BUSY_RX) && HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR))
    {
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        HAL_DMA_Abort(huart->hdmarx);
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_IDLEIE));
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        huart->RxState = HAL_UART_STATE_READY;
    }
    // 开启下次接收
    HAL_UART_Receive_DMA(&huart2_handler, (uint8_t *)buf_next_recive->buf_ptr, buf_next_recive->buf_size);
    // 关闭串口 DMA 接收传输过半中断
    __HAL_DMA_DISABLE_IT(&hdma1ch4_handler, DMA_IT_HT);

    // 发布命令接收完成事件
    ltx_Topic_publish(&topic_cmd_get);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    // 
    // LTX_LOG_STR("T1\n");
}
