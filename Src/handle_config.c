#include "handle_config.h"
#include "ltx_log.h"

struct handle_config _cfg_of_handle;

void hcfg_read_from_flash(void){
    uint8_t *_data = (uint8_t *)&_cfg_of_handle;
    // uint8_t size = ((struct handle_config *)_HCFG_start_addr)->cfg_size;
    uint8_t size = _HCFG_default_size;

    for(uint8_t i = 0; i < _HCFG_default_size + 12; i ++){
        _data[i] = ((volatile uint8_t *)_HCFG_start_addr)[i];
    }
}

struct handle_config *hcfg_get_handle(void){
    return &_cfg_of_handle;
}

uint8_t hcfg_check(void){
    if(_cfg_of_handle.magic != _HCFG_magic){
        return 1;
    }
    if(_cfg_of_handle.cfg_size != _HCFG_default_size){
        return 2;
    }
    uint8_t check_val = 0;
    uint32_t data_addr = (_HCFG_start_addr+12);
    for(uint8_t i = 0; i < _cfg_of_handle.cfg_size; i ++){
        check_val ^= *(uint8_t *)data_addr;
        data_addr ++;
    }
    if(check_val != _cfg_of_handle.check_value){
        return 3;
    }

    return 0;
}

// 处理，包括设置大小和计算异或值等操作
void hcfg_pack(void){
    _cfg_of_handle.magic = _HCFG_magic;
    _cfg_of_handle.cfg_size = _HCFG_default_size;

    uint8_t check_val = 0;
    uint32_t data_addr = (uint32_t)(&_cfg_of_handle);
    data_addr += 12;
    for(uint8_t i = 0; i < _cfg_of_handle.cfg_size; i ++){
        check_val ^= *(uint8_t *)data_addr;
        data_addr ++;
    }
    _cfg_of_handle.check_value = check_val;

    _cfg_of_handle.config_status = 6;
}

uint32_t _flash_page_error = 0;
FLASH_EraseInitTypeDef _flash_erase_stu;

void hcfg_erase(void){
    HAL_FLASH_Unlock();

    _flash_erase_stu.TypeErase   = FLASH_TYPEERASE_SECTORERASE;        /* Sector Erase */
    _flash_erase_stu.SectorAddress = _HCFG_start_addr;
    _flash_erase_stu.NbSectors  = 1;                                   /* Sector Erase Numbers */

    if(HAL_FLASHEx_Erase(&_flash_erase_stu, &_flash_page_error) != HAL_OK){
        HAL_FLASH_Lock();
        LTX_LOG_FMT("!Erase sec Failed:%d\n", _flash_page_error);
        return;
    }

    HAL_FLASH_Lock();
    LTX_LOG_STR("Erase sec ok\n");
}

// 一个 page 的容量（256Byte），不用完整的 2KB
volatile uint8_t _sec_data[256];

// 返回 0 代表写入成功
uint8_t hcfg_write_into_flash(struct handle_config *cfg){
    if(cfg == NULL){
        return 1;
    }

    // 将配置写入页缓存
    uint8_t *_data = (uint8_t *)cfg;
    uint8_t size = cfg->cfg_size;

    for(uint8_t i = 0; i < size + 12; i ++){
        _sec_data[i] = _data[i];
    }

    // 将页缓存写入 flash
    HAL_FLASH_Unlock();
    
    _flash_erase_stu.TypeErase   = FLASH_TYPEERASE_SECTORERASE;        /* Sector Erase */
    _flash_erase_stu.SectorAddress = _HCFG_start_addr;
    _flash_erase_stu.NbSectors  = 1;                                   /* Sector Erase Numbers */

    if(HAL_FLASHEx_Erase(&_flash_erase_stu, &_flash_page_error) != HAL_OK){
        HAL_FLASH_Lock();
        LTX_LOG_FMT("!Erase sec Failed:%d\n", _flash_page_error);
        return 2;
    }

    uint8_t time_out = 0;

    while(HAL_FLASH_Program(FLASH_TYPEPROGRAM_PAGE, _HCFG_start_addr, (uint32_t *)_sec_data) != HAL_OK){
        time_out ++;
        if(time_out ++ > 10){
            HAL_FLASH_Lock();
            LTX_LOG_ERRO("Write cfg Failed!\n");
            return 2;
        }
        HAL_Delay(2);
    }

    HAL_FLASH_Lock();

    return 0;
}
