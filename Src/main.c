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
SPI_HandleTypeDef hspi2_handler;
DMA_HandleTypeDef hdma1ch1_handler;

ADC_HandleTypeDef hadc1_handler;
// DMA_HandleTypeDef hdma1ch2_handler;

ADC_HandleTypeDef hadc2_handler;
DMA_HandleTypeDef hdma1ch2_handler;

TIM_HandleTypeDef htim1_handler;

I2C_HandleTypeDef hi2c1_handler;
DMA_HandleTypeDef hdma1ch3;
DMA_HandleTypeDef hdma1ch4;

/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void mcu_init_clock(void);
static void mcu_init_usb(void);
static void mcu_init_spi2(void);
static void mcu_init_i2c1(void);
static void mcu_init_tim1(void);
static void mcu_init_adc1(void);
static void mcu_init_adc2(void);
static void mcu_init_btn_pin(void);

/**
 * @brief  Main program.
 * @retval int
 */
int main(void){

    /* Reset of all peripherals, Initializes the Systick */
    HAL_Init();
    
    #ifdef ltx_cfg_USE_IDLE_TASK
        // 如果需要空闲任务能力，那么需要将软中断设置为最低优先级，并且确保 systick 中断优先级比它更高
        HAL_NVIC_SetPriority(SysTick_IRQn, 2, 1);
        HAL_NVIC_SetPriority(PendSV_IRQn, 3, 1);
    #else
        // 设置 systick 为最低优先级
        HAL_NVIC_SetPriority(SysTick_IRQn, 3, 1);
    #endif

    ltx_Log_init();
    LTX_LOG_STR("\n\nSYSTEM START\n\n");

    // 初始化外设
    mcu_init_clock();
    mcu_init_btn_pin();
    mcu_init_adc1();
    // adc1 较准
    if(HAL_ADCEx_Calibration_Start(&hadc1_handler) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC calibration Failed!\n");
            HAL_Delay(1000);
        }
    }
    // trgo 才能使用 dma，而且只有 ch1 才能触发，并且不能调整采样时间点且只有 ch1 输出非 0% 或 100% 才能触发采样
    // HAL_ADC_Start_DMA(&hadc1_handler, (uint32_t*)adc1_buffer, 3);
    // ch4 只能注入组中断触发采样，用不了 dma，无所谓了，dma 还要产生两次中断搞些判断，注入组直接后台采好一次中断就好了，比 dma 还 dma
    if(HAL_ADCEx_InjectedStart_IT(&hadc1_handler) != HAL_OK){
        while(1){ LTX_LOG_ERRO("ADC1 injected start IT Failed!\n"); HAL_Delay(1000); }
    }
    mcu_init_adc2();
    // adc 较准
    if(HAL_ADCEx_Calibration_Start(&hadc2_handler) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC calibration Failed!\n");
            HAL_Delay(1000);
        }
    }
    mcu_init_tim1();
    mcu_init_spi2();
    mcu_init_i2c1();
    // mcu_init_usb();

    LTX_LOG_INFO("MCU init over at %dms\n", ltx_Sys_get_tick());

    // 创建系统 app 并运行
    ltx_App_init(&app_system);
    ltx_App_resume(&app_system);
    
    // 创建外部硬件初始化 app 并运行
    ltx_App_init(&app_device_init);
    ltx_App_resume(&app_device_init);
    
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



    /* Infinite loop */
    while (1)
    {
        /* Delay for 1s */
        HAL_Delay(1000);

        /* Call the test function to send data to the USB host */
        cdc_acm_data_send_with_dtr_test();
    }
}


static void mcu_init_spi2(void){
    hspi2_handler.Instance                  = SPI2;
    hspi2_handler.Init.BaudRatePrescaler    = SPI_BAUDRATEPRESCALER_32; // 确保速度为 4Mbits/s
    hspi2_handler.Init.Direction            = SPI_DIRECTION_1LINE;
    hspi2_handler.Init.CLKPolarity          = SPI_POLARITY_LOW;
    hspi2_handler.Init.CLKPhase             = SPI_PHASE_1EDGE ;
    hspi2_handler.Init.DataSize             = SPI_DATASIZE_8BIT;
    hspi2_handler.Init.FirstBit             = SPI_FIRSTBIT_MSB;
    hspi2_handler.Init.NSS                  = SPI_NSS_SOFT;
    hspi2_handler.Init.Mode                 = SPI_MODE_MASTER;
    hspi2_handler.Init.CRCCalculation       = SPI_CRCCALCULATION_DISABLE;
    /* hspi2_handler.Init.CRCPolynomial = 1; */
    if (HAL_SPI_DeInit(&hspi2_handler) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("SPI2 Deinit Failed!\n");
            HAL_Delay(1000);
        }
    }
    
    /* Initialize SPI peripheral */
    if (HAL_SPI_Init(&hspi2_handler) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("SPI2 init Failed!\n");
            HAL_Delay(1000);
        }
    }
}

static void mcu_init_i2c1(void){
    hi2c1_handler.Instance             = I2C1;
    hi2c1_handler.Init.ClockSpeed      = 400000;
    hi2c1_handler.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
    hi2c1_handler.Init.OwnAddress1     = 0x00;
    hi2c1_handler.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1_handler.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;   /* Do not enable dual address */
    /* hi2c1_handler.Init.OwnAddress2     = I2C_ADDRESS; */         /* Second address */
    hi2c1_handler.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;  /* Disable general call */
    // hi2c1_handler.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;    /* Enable clock stretching */
    hi2c1_handler.Init.NoStretchMode   = I2C_NOSTRETCH_ENABLE; // 禁用时钟延展，避免 scl 受到干扰时主机误以为从机要求缓一缓，从而两边都在干等对方
    if (HAL_I2C_Init(&hi2c1_handler) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("I2C1 init Failed!\n");
            HAL_Delay(1000);
        }
    }
}

static void mcu_init_tim1(void){
    TIM_OC_InitTypeDef tim_channel_config;

    htim1_handler.Instance = TIM1;
    htim1_handler.Init.Period            = 3199;                            // 20kHz，中心对齐模式下先递增再递减，所以频率要除以 2
    htim1_handler.Init.Prescaler         = 0;                               // 不分频
    htim1_handler.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;          // 不分频
    htim1_handler.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;  // 中心对齐
    htim1_handler.Init.RepetitionCounter = 1 - 1;                                   /* repetition counter value:1-1 */
    htim1_handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;          /* TIM1_ARR register is not buffered */
    /* Initializes the TIM PWM Time Base */
    if (HAL_TIM_PWM_Init(&htim1_handler) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("Tim1 init Failed!\n");
            HAL_Delay(1000);
        }
    }
    
    tim_channel_config.OCMode       = TIM_OCMODE_PWM1;                                     /* Set as PWM1 mode */
    tim_channel_config.OCPolarity   = TIM_OCPOLARITY_HIGH;                                 /* OC channel active high */
    tim_channel_config.OCFastMode   = TIM_OCFAST_DISABLE;                                  /* Output Compare fast disable */
    tim_channel_config.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                                /* OCN channel active high */
    tim_channel_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;                              /* OC1N channel idle state is low level */
    tim_channel_config.OCIdleState  = TIM_OCIDLESTATE_RESET;                               /* OC1 channel idle state is low level */
    tim_channel_config.Pulse        = 0;

    if (HAL_TIM_PWM_ConfigChannel(&htim1_handler, &tim_channel_config, TIM_CHANNEL_1) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("Tim1 ch 1 init Failed!\n");
            HAL_Delay(1000);
        }
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1_handler, &tim_channel_config, TIM_CHANNEL_2) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("Tim1 ch 2 init Failed!\n");
            HAL_Delay(1000);
        }
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1_handler, &tim_channel_config, TIM_CHANNEL_3) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("Tim1 ch 3 init Failed!\n");
            HAL_Delay(1000);
        }
    }

    HAL_TIM_PWM_Start(&htim1_handler, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1_handler, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1_handler, TIM_CHANNEL_3);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;   // 或 TIM_OCMODE_TOGGLE，根据需求
    sConfigOC.Pulse = 3198; // 比较值，用于调整采样的时机
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                                /* OCN channel active high */
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;                              /* OC1N channel idle state is low level */
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;                               /* OC1 channel idle state is low level */
    if(HAL_TIM_OC_ConfigChannel(&htim1_handler, &sConfigOC, TIM_CHANNEL_4) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("Tim1 ch4 cfg Failed!\n");
            HAL_Delay(1000);
        }
    }
    HAL_TIM_OC_Start(&htim1_handler, TIM_CHANNEL_4);

    TIM_MasterConfigTypeDef  sMasterConfig;

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1_handler, &sMasterConfig);
    if (HAL_TIM_Base_Start(&htim1_handler) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("Tim1 trgo cfg Failed!\n");
            HAL_Delay(1000);
        }
    }
}

static void mcu_init_adc1(void){
    ADC_ChannelConfTypeDef   adc_channel_config={0};
    RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInit={0};
    
    __HAL_RCC_ADC1_CLK_ENABLE();
    
    RCC_PeriphCLKInit.PeriphClockSelection= RCC_PERIPHCLK_ADC;
    RCC_PeriphCLKInit.AdcClockSelection   = RCC_ADCPCLK2_DIV8; // 16Mhz
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInit);
    
    hadc1_handler.Instance = ADC1;
    
    hadc1_handler.Init.Resolution            = ADC_RESOLUTION_12B;             /* 12-bit resolution for converted data  */
    hadc1_handler.Init.DataAlign             = ADC_DATAALIGN_RIGHT;            /* Right-alignment for converted data */
    hadc1_handler.Init.ScanConvMode          = ADC_SCAN_ENABLE;                /* Scan Mode Enable */
    hadc1_handler.Init.ContinuousConvMode    = DISABLE;                        /* Single Conversion */
    hadc1_handler.Init.NbrOfConversion       = 3;                              /* Conversion Number */
    hadc1_handler.Init.DiscontinuousConvMode = DISABLE;                        /* Discontinuous Mode Disable */
    hadc1_handler.Init.NbrOfDiscConversion   = 1;                              /* Discontinuous Conversion Number 1 */
    /* regular group not used for hardware trigger here */
    hadc1_handler.Init.ExternalTrigConv      = ADC_SOFTWARE_START; /* regular triggered by software (unused) */

    if (HAL_ADC_Init(&hadc1_handler) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC1 init Failed!\n");
            HAL_Delay(1000);
        }
    }
    
    adc_channel_config.Channel      = ADC_CHANNEL_5;
    adc_channel_config.Rank         = ADC_REGULAR_RANK_1;
    adc_channel_config.SamplingTime = ADC_SAMPLETIME_3CYCLES_5; // 16Mhz，采样时间为 3.5+12.5=16周期，1us
    
    if (HAL_ADC_ConfigChannel(&hadc1_handler, &adc_channel_config) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC1 ch %d init Failed!\n", adc_channel_config.Channel);
            HAL_Delay(1000);
        }
    }

    /* 配置注入组：使用 TIM1 CC4 触发注入采样，3 个 injected rank 对应三相电流 */
    {
        ADC_InjectionConfTypeDef injcfg = {0};

        injcfg.InjectedChannel = ADC_CHANNEL_5;
        injcfg.InjectedRank = ADC_INJECTED_RANK_1;
        injcfg.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES_5;
        injcfg.InjectedOffset = 0;

        injcfg.InjectedNbrOfConversion = 3;
        injcfg.InjectedDiscontinuousConvMode = DISABLE;
        injcfg.AutoInjectedConv = DISABLE;
        injcfg.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4; /* TIM1 CC4 */
        // injcfg.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;

        if (HAL_ADCEx_InjectedConfigChannel(&hadc1_handler, &injcfg) != HAL_OK){
            while(1){ LTX_LOG_ERRO("ADC1 injected cfg Failed!\n"); HAL_Delay(1000); }
        }

        injcfg.InjectedChannel = ADC_CHANNEL_6;
        injcfg.InjectedRank = ADC_INJECTED_RANK_2;
        if (HAL_ADCEx_InjectedConfigChannel(&hadc1_handler, &injcfg) != HAL_OK){
            while(1){ LTX_LOG_ERRO("ADC1 injected cfg Failed!\n"); HAL_Delay(1000); }
        }

        injcfg.InjectedChannel = ADC_CHANNEL_7;
        injcfg.InjectedRank = ADC_INJECTED_RANK_3;
        if (HAL_ADCEx_InjectedConfigChannel(&hadc1_handler, &injcfg) != HAL_OK){
            while(1){ LTX_LOG_ERRO("ADC1 injected cfg Failed!\n"); HAL_Delay(1000); }
        }
    }
    
    adc_channel_config.Channel      = ADC_CHANNEL_6;
    adc_channel_config.Rank         = ADC_REGULAR_RANK_2;
    adc_channel_config.SamplingTime = ADC_SAMPLETIME_3CYCLES_5;
    
    if (HAL_ADC_ConfigChannel(&hadc1_handler, &adc_channel_config) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC1 ch %d init Failed!\n", adc_channel_config.Channel);
            HAL_Delay(1000);
        }
    }
    
    adc_channel_config.Channel      = ADC_CHANNEL_7;
    adc_channel_config.Rank         = ADC_REGULAR_RANK_3;
    adc_channel_config.SamplingTime = ADC_SAMPLETIME_3CYCLES_5;
    
    if (HAL_ADC_ConfigChannel(&hadc1_handler, &adc_channel_config) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC1 ch %d init Failed!\n", adc_channel_config.Channel);
            HAL_Delay(1000);
        }
    }
}

static void mcu_init_adc2(void){
    ADC_ChannelConfTypeDef   adc_channel_config={0};
    // RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInit={0};
    
    __HAL_RCC_ADC2_CLK_ENABLE();
    
    // RCC_PeriphCLKInit.PeriphClockSelection= RCC_PERIPHCLK_ADC;
    // RCC_PeriphCLKInit.AdcClockSelection   = RCC_ADCPCLK2_DIV8;
    // HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInit);
    
    hadc2_handler.Instance = ADC2;
    
    hadc2_handler.Init.Resolution            = ADC_RESOLUTION_12B;             /* 12-bit resolution for converted data  */
    hadc2_handler.Init.DataAlign             = ADC_DATAALIGN_RIGHT;            /* Right-alignment for converted data */
    hadc2_handler.Init.ScanConvMode          = ADC_SCAN_ENABLE;                /* Scan Mode Enable */
    hadc2_handler.Init.ContinuousConvMode    = DISABLE;                        /* Single Conversion */
    hadc2_handler.Init.NbrOfConversion       = 7;                              /* Conversion Number */
    hadc2_handler.Init.DiscontinuousConvMode = DISABLE;                        /* Discontinuous Mode Disable */
    hadc2_handler.Init.NbrOfDiscConversion   = 1;                              /* Discontinuous Conversion Number 1 */
    hadc2_handler.Init.ExternalTrigConv      = ADC_SOFTWARE_START;             /* Software Trigger */

    if (HAL_ADC_Init(&hadc2_handler) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC2 init Failed!\n");
            HAL_Delay(1000);
        }
    }

    adc_channel_config.Channel      = ADC_CHANNEL_0;
    adc_channel_config.Rank         = ADC_REGULAR_RANK_1;
    adc_channel_config.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    
    if (HAL_ADC_ConfigChannel(&hadc2_handler, &adc_channel_config) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC2 ch %d init Failed!\n", adc_channel_config.Channel);
            HAL_Delay(1000);
        }
    }
    
    adc_channel_config.Channel      = ADC_CHANNEL_1;
    adc_channel_config.Rank         = ADC_REGULAR_RANK_2;
    adc_channel_config.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    
    if (HAL_ADC_ConfigChannel(&hadc2_handler, &adc_channel_config) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC2 ch %d init Failed!\n", adc_channel_config.Channel);
            HAL_Delay(1000);
        }
    }
    
    adc_channel_config.Channel      = ADC_CHANNEL_2;
    adc_channel_config.Rank         = ADC_REGULAR_RANK_3;
    adc_channel_config.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    
    if (HAL_ADC_ConfigChannel(&hadc2_handler, &adc_channel_config) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC2 ch %d init Failed!\n", adc_channel_config.Channel);
            HAL_Delay(1000);
        }
    }
    
    adc_channel_config.Channel      = ADC_CHANNEL_3;
    adc_channel_config.Rank         = ADC_REGULAR_RANK_4;
    adc_channel_config.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    
    if (HAL_ADC_ConfigChannel(&hadc2_handler, &adc_channel_config) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC2 ch %d init Failed!\n", adc_channel_config.Channel);
            HAL_Delay(1000);
        }
    }
    
    adc_channel_config.Channel      = ADC_CHANNEL_4;
    adc_channel_config.Rank         = ADC_REGULAR_RANK_5;
    adc_channel_config.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    
    if (HAL_ADC_ConfigChannel(&hadc2_handler, &adc_channel_config) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC2 ch %d init Failed!\n", adc_channel_config.Channel);
            HAL_Delay(1000);
        }
    }
    
    adc_channel_config.Channel      = ADC_CHANNEL_8;
    adc_channel_config.Rank         = ADC_REGULAR_RANK_6;
    adc_channel_config.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    
    if (HAL_ADC_ConfigChannel(&hadc2_handler, &adc_channel_config) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC2 ch %d init Failed!\n", adc_channel_config.Channel);
            HAL_Delay(1000);
        }
    }
    
    adc_channel_config.Channel      = ADC_CHANNEL_9;
    adc_channel_config.Rank         = ADC_REGULAR_RANK_7;
    adc_channel_config.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
    
    if (HAL_ADC_ConfigChannel(&hadc2_handler, &adc_channel_config) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("ADC2 ch %d init Failed!\n", adc_channel_config.Channel);
            HAL_Delay(1000);
        }
    }
}

static void mcu_init_btn_pin(void){
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
#if 0
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
#else
    // 测试期将按键引脚作为输出引脚测试部分功能
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
#endif
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
 * @brief  USB peripheral initialization function
 * @param  None
 * @retval None
 */
static void mcu_init_usb(void)
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
static void mcu_init_clock(void)
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
    OscInitstruct.PLL.PLLMUL = RCC_PLL_MUL8;         // 128Mhz
    /* Configure Oscillators */
    if (HAL_RCC_OscConfig(&OscInitstruct) != HAL_OK){
        while(1){
            LTX_LOG_ERRO("RCC init Failed!\n");
            HAL_Delay(1000);
        }
    }

    ClkInitstruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    ClkInitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; /* SYSCLK source select as PLL */
    ClkInitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        /* AHB clock not divided */
    ClkInitstruct.APB1CLKDivider = RCC_HCLK_DIV1;         /* APB1 clock not divided */
    ClkInitstruct.APB2CLKDivider = RCC_HCLK_DIV1;         /* APB2 clock not divided */
    /* Configure Clocks */
    if (HAL_RCC_ClockConfig(&ClkInitstruct, FLASH_LATENCY_5) != HAL_OK){ // 被坑了，芯片是便宜，但是 flash 要开到 5 等待，希望他的 art 最好能像手册里说的一样“相当于 0 等待”
        while(1){
            LTX_LOG_ERRO("CLK init Failed!\n");
            HAL_Delay(1000);
        }
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
