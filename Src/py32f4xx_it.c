/**
 ******************************************************************************
 * @file    py32f4xx_it.c
 * @author  MCU Application Team
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by Puya under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32f4xx_it.h"
#include "usb_py32_reg.h"

/* Private includes ----------------------------------------------------------*/
#include "ltx.h"
#include "ltx_log.h"
#include "mt6701.h"
#include "myAPP_motor.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*          Cortex-M4 Processor Interruption and Exception Handlers           */
/******************************************************************************/
/**
 * @brief   This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
    LTX_LOG_STR("\n\n?!HF!?\n\n");
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void)
{
    ltx_Sys_scheduler();
}

#if 0
// i2c 看门狗计数器
extern volatile uint8_t flag_i2c_wdg;
uint8_t mag_reg_addr = 0x03;
#endif
/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
    HAL_IncTick();

#if 0
    if(flag_i2c_wdg){ // i2c 看门狗开启
        if(flag_i2c_wdg == 1){ // i2c 未更新
            // 修复 i2c
            // 强制停止
            HAL_I2C_Master_Abort_IT(&hi2c1_handler, MT6701_DEFAULT_ADDR);
            // 重新发起 i2c 读取
            HAL_I2C_Master_Transmit_DMA(&hi2c1_handler, MT6701_DEFAULT_ADDR, &mag_reg_addr, 1);
        }
        // 重置计数器
        flag_i2c_wdg = 1;
    }
#endif

    ltx_Sys_tick_tack();
}

/******************************************************************************/
/* PY32F4xx Peripheral Interrupt Handlers                                     */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/
void USB_IRQHandler(void)
{
    USBD_IRQHandler();
}


// void SPI2_IRQHandler(void){
//     HAL_SPI_IRQHandler(&hspi2_handler);
// }
void DMA1_Channel2_IRQHandler(void){
    HAL_DMA_IRQHandler(&hdma1ch2_handler);
}

void SPI1_IRQHandler(void){
    HAL_SPI_IRQHandler(&hspi1_handler);
}
void DMA1_Channel1_IRQHandler(void){
    HAL_DMA_IRQHandler(&hdma1ch1_handler);
}


#if 0
void I2C1_EV_IRQHandler(void){
    HAL_I2C_EV_IRQHandler(&hi2c1_handler);
}
void I2C1_ER_IRQHandler(void){
    HAL_I2C_ER_IRQHandler(&hi2c1_handler);
}
#endif

void USART2_IRQHandler(void){
    HAL_UART_IRQHandler(&huart2_handler);
}
void DMA1_Channel3_IRQHandler(void){
    // HAL_DMA_IRQHandler(hi2c1_handler.hdmatx);
    HAL_DMA_IRQHandler(huart2_handler.hdmatx);
}
void DMA1_Channel4_IRQHandler(void){
    // HAL_DMA_IRQHandler(hi2c1_handler.hdmarx);
    HAL_DMA_IRQHandler(huart2_handler.hdmarx);
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
