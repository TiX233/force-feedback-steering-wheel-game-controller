#include "mt6701.h"

#define PI  3.14159265358979f

// 阻塞读取
void mt6701_read(struct mt6701_stu *mt){
    mt->read_reg(mt, 0x03, mt->data_buffer, 2);
}

// 非阻塞读取
void mt6701_read_dma(struct mt6701_stu *mt){
    mt->read_reg_dma(mt, 0x03, mt->data_buffer, 2);
}

// 转换读出来的值为角度
float mt6701_trans_angle(struct mt6701_stu *mt){
    uint16_t data = (mt->data_buffer[1] >> 2) + (mt->data_buffer[0] << 6);
    
    return data*(360.0f/16384.0f);
}

// 转换读出来的值为弧度
float mt6701_trans_rad(struct mt6701_stu *mt){
    uint16_t data = (mt->data_buffer[1] >> 2) + (mt->data_buffer[0] << 6);
    
    return data*(2*PI/16384.0f);
}
