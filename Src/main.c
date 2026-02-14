/**
 ******************************************************************************
 * @file    main.c
 * @author  MCU Application Team
 * @brief   Main program body
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
#include "usb_config.h"

#include "ltx.h"
#include "ltx_log.h"
#include "ltx_app.h"
#include "myAPP_system.h"
#include "myAPP_device_init.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void sys_init_clock(void);
static void sys_init_usb(void);
/**
 * @brief  Main program.
 * @retval int
 */
int main(void)
{
    /* Reset of all peripherals, Initializes the Systick */
    HAL_Init();

    ltx_Log_init();
    LTX_LOG_STR("\n\nSYSTEM START\n\n");

    /* System clock configuration */
    sys_init_clock();

    #ifdef ltx_cfg_USE_IDLE_TASK
    // 如果需要空闲任务能力，那么需要将软中断设置为最低优先级，并且确保 systick 中断优先级比它更高
    HAL_NVIC_SetPriority(SysTick_IRQn, 6, 0U);
    HAL_NVIC_SetPriority(PendSV_IRQn, 7, 0U);
    #endif

    // 创建系统 app 并运行
    ltx_App_init(&app_system);
    ltx_App_resume(&app_system);
    
    // 创建外部硬件初始化 app 并运行
    // ltx_App_init(&app_device_init);
    // ltx_App_resume(&app_device_init);
    
    // 启动调度器
    #ifndef ltx_cfg_USE_IDLE_TASK
    // 不开启空闲任务功能，则直接在主循环运行调度器
    LTX_LOG_INFO("Start scheduler...\n");
    ltx_Sys_scheduler();
    #endif
    // 开启空闲休眠，调度器需要放到 pendsv

    // 运行空闲任务
    LTX_LOG_INFO("Start idle task...\n");
    while (1){
        // 进入休眠
        // __DSB();
        __WFI();
    }



    /* Initialize USB peripheral */
    sys_init_usb();

    /* Infinite loop */
    while (1)
    {
        /* Delay for 1s */
        HAL_Delay(1000);

        /* Call the test function to send data to the USB host */
        cdc_acm_data_send_with_dtr_test();
    }
}

/**
 * @brief  USB peripheral initialization function
 * @param  None
 * @retval None
 */
static void sys_init_usb(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    SET_BIT(RCC->CFGR1, RCC_CFGR1_USBSELHSI48_Msk);
    __HAL_RCC_USB_CLK_ENABLE();

    cdc_acm_init();

    /* Enable USB interrupt */
    NVIC_EnableIRQ(USBD_IRQn);
}

/**
 * @brief  System clock configuration function
 * @param  None
 * @retval None
 */
static void sys_init_clock(void)
{
    RCC_OscInitTypeDef OscInitstruct = {0};
    RCC_ClkInitTypeDef ClkInitstruct = {0};

    OscInitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE |
                                   RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSI48M;
    OscInitstruct.HSEState = RCC_HSE_ON;             /* Enable HSE */
    OscInitstruct.HSEFreq = RCC_HSE_16_32MHz;        /* HSE working frequency range: 16M~32M */
    OscInitstruct.HSI48MState = RCC_HSI48M_ON;       /* Enable HSI48M */
    OscInitstruct.HSIState = RCC_HSI_ON;             /* Enable HSI */
    OscInitstruct.LSEState = RCC_LSE_OFF;            /* Disable LSE */
    OscInitstruct.LSEDriver = RCC_LSEDRIVE_HIGH;     /* Drive capability level: High */
    OscInitstruct.LSIState = RCC_LSI_OFF;            /* Disable LSI */
    OscInitstruct.PLL.PLLState = RCC_PLL_ON;         /* Enable PLL */
    OscInitstruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; /* PLL clock source: HSE */
    OscInitstruct.PLL.PLLMUL = RCC_PLL_MUL5;         /* PLL multiplication factor is 5 */
    /* Configure Oscillators */
    if (HAL_RCC_OscConfig(&OscInitstruct) != HAL_OK)
    {
        APP_ErrorHandler();
    }

    ClkInitstruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    ClkInitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; /* SYSCLK source select as PLL */
    ClkInitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        /* AHB clock not divided */
    ClkInitstruct.APB1CLKDivider = RCC_HCLK_DIV1;         /* APB1 clock not divided */
    ClkInitstruct.APB2CLKDivider = RCC_HCLK_DIV1;         /* APB2 clock not divided */
    /* Configure Clocks */
    if (HAL_RCC_ClockConfig(&ClkInitstruct, FLASH_LATENCY_4) != HAL_OK)
    {
        APP_ErrorHandler();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void APP_ErrorHandler(void)
{
    /* Infinite loop */
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file：pointer to the source file name
 * @param  line：assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* Infinite loop */
    while (1)
    {
    }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
