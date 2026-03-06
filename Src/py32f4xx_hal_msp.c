/**
 ******************************************************************************
 * @file    py32f4xx_hal_msp.c
 * @author  MCU Application Team
 * @brief   This file provides code for the MSP Initialization
 *          and de-Initialization codes.
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* External functions --------------------------------------------------------*/

/**
 * @brief Initialize global MSP
 */
void HAL_MspInit(void){

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief Initialize SPI related MSP
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi){

    GPIO_InitTypeDef  GPIO_InitStruct;
    /* Initialize SPI2 */
    if (hspi->Instance == SPI2){

        __HAL_RCC_GPIOB_CLK_ENABLE();                   /* Enable GPIOB clock */
        __HAL_RCC_SYSCFG_CLK_ENABLE();                  /* Enable SYSCFG clock */
        __HAL_RCC_SPI2_CLK_ENABLE();                    /* Enable SPI2 clock */
        __HAL_RCC_DMA1_CLK_ENABLE();                    /* Enable DMA clock */

        /* GPIO configured as SPI：MOSI*/
        GPIO_InitStruct.Pin       = GPIO_PIN_15;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        /* Interrupt configuration */
        HAL_NVIC_SetPriority(SPI2_IRQn, 2, 1);
        HAL_NVIC_EnableIRQ(SPI2_IRQn);

        /* DMA_CH1 configuration */
        hdma1ch1_handler.Instance                 = DMA1_Channel1;
        hdma1ch1_handler.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma1ch1_handler.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma1ch1_handler.Init.MemInc              = DMA_MINC_ENABLE;
        if (hspi->Init.DataSize <= SPI_DATASIZE_8BIT){
            hdma1ch1_handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            hdma1ch1_handler.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        }else {
            hdma1ch1_handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
            hdma1ch1_handler.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        }

        hdma1ch1_handler.Init.Mode                = DMA_NORMAL;
        hdma1ch1_handler.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        /* Initialize DMA */
        HAL_DMA_Init(&hdma1ch1_handler);
        /* DMA handle is associated with SPI handle */
        __HAL_LINKDMA(hspi, hdmatx, hdma1ch1_handler);
        
        /* Set DMA channel map. */
        HAL_DMA_ChannelMap(&hdma1ch1_handler, DMA_CHANNEL_MAP_SPI2_WR); /* SPI2_TX DMA1_CH1 */
        
        /* DMA interrupt configuration*/
        HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 1);
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    }
}

/**
  * @brief Deinit SPI MSP
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi){

    if (hspi->Instance == SPI2){

        /* Reset SPI peripheral */
        __HAL_RCC_SPI2_FORCE_RESET();
        __HAL_RCC_SPI2_RELEASE_RESET();

        /* Disable SPI and GPIO clock */
        /* Deinit SPI SCK */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15);

        HAL_NVIC_DisableIRQ(SPI2_IRQn);

        HAL_DMA_DeInit(&hdma1ch1_handler);
        HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);

    }
}


/**
  * @brief Initialize ADC MSP.
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct={0};

    __HAL_RCC_SYSCFG_CLK_ENABLE();                              /* Enable SYSCFG clock */
    __HAL_RCC_DMA1_CLK_ENABLE();                                /* Enable DMA clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();                               /* Enable GPIOA clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();                               /* Enable GPIOB clock */

    // 电流采样
    if (hadc->Instance == ADC1){
    
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 |GPIO_PIN_7 ;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        #ifdef USE_ADC1_IRQ
        HAL_NVIC_SetPriority(ADC1_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(ADC1_IRQn);
        #endif
        
        hdma1ch2_handler.Instance                 = DMA1_Channel2;
        hdma1ch2_handler.Init.Direction           = DMA_PERIPH_TO_MEMORY;    /* Transfer mode Periph to Memory */
        hdma1ch2_handler.Init.PeriphInc           = DMA_PINC_DISABLE;        /* Peripheral increment mode Disable */
        hdma1ch2_handler.Init.MemInc              = DMA_MINC_ENABLE;         /* Memory increment mode Enable */
        hdma1ch2_handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;     /* Peripheral data alignment : Word  */
        hdma1ch2_handler.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;     /* Memory data alignment : Word  */
        hdma1ch2_handler.Init.Mode                = DMA_CIRCULAR;            /* Circular DMA mode */
        hdma1ch2_handler.Init.Priority            = DMA_PRIORITY_HIGH;  /* Priority level : high  */

        HAL_DMA_DeInit(&hdma1ch2_handler);
        HAL_DMA_Init(&hdma1ch2_handler);
        
        HAL_DMA_ChannelMap(&hdma1ch2_handler, DMA_CHANNEL_MAP_ADC1);          /* DMA Channel Remap */
        __HAL_LINKDMA(hadc, DMA_Handle, hdma1ch2_handler);
        
        #ifdef USE_ADC1_IRQ
        // HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2, 0);
        // HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
        #endif
    }

    // 摇杆等
    if (hadc->Instance == ADC2){
    
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        // HAL_NVIC_SetPriority(ADC1_IRQn, 1, 0);
        // HAL_NVIC_EnableIRQ(ADC1_IRQn);
        
        // hdma1ch2_handler.Instance                 = DMA1_Channel2;
        // hdma1ch2_handler.Init.Direction           = DMA_PERIPH_TO_MEMORY;    /* Transfer mode Periph to Memory */
        // hdma1ch2_handler.Init.PeriphInc           = DMA_PINC_DISABLE;        /* Peripheral increment mode Disable */
        // hdma1ch2_handler.Init.MemInc              = DMA_MINC_ENABLE;         /* Memory increment mode Enable */
        // hdma1ch2_handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;     /* Peripheral data alignment : Word  */
        // hdma1ch2_handler.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;     /* Memory data alignment : Word  */
        // hdma1ch2_handler.Init.Mode                = DMA_CIRCULAR;            /* Circular DMA mode */
        // hdma1ch2_handler.Init.Priority            = DMA_PRIORITY_VERY_HIGH;  /* Priority level : high  */

        // HAL_DMA_DeInit(&hdma1ch2_handler);
        // HAL_DMA_Init(&hdma1ch2_handler);
        
        // HAL_DMA_ChannelMap(&hdma1ch2_handler, DMA_CHANNEL_MAP_ADC1);          /* DMA Channel Remap */
        // __HAL_LINKDMA(hadc, DMA_Handle, hdma1ch2_handler);
        
        // HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
        // HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    }

}

/**
  * @brief Initialize TIM1 related MSP
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef   GPIO_InitStruct;
    /* Enable TIM1 clock */
    __HAL_RCC_TIM1_CLK_ENABLE();
    /* Enable GPIOA clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;        /* Alternate Function Push Pull Mode */
    // GPIO_InitStruct.Pull = GPIO_PULLUP;            /* Pull up */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    
    /* Initialize GPIOA8 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* Initialize GPIOA9 */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* Initialize GPIOA10 */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief Initialize I2C MSP
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_SYSCFG_CLK_ENABLE();                              /* Enable SYSCFG clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();                               /* Enable GPIOB clock */
    __HAL_RCC_I2C1_CLK_ENABLE();                                /* Enable I2C clock */
    __HAL_RCC_DMA1_CLK_ENABLE();                                 /* Enable DMA clock */

    // SCL
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // 开 pp 没用，示波器抓到的还是弱上拉，所以 scl 很容易被干扰，导致主机可能误认为有其他主机在操作总线，或者认为从机在请求等待
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;                  /* Alternate as I2C */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);                     /* Initialize GPIO */

    // SDA
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;                     /* Open-drain mode */
    GPIO_InitStruct.Pull = GPIO_PULLUP;                         /* Pull-up */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;                  /* Alternate as I2C */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);                     /* Initialize GPIO */
    /* Reset I2C */
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    /* I2C1 interrupt initialization */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);                     /* Set interrupt priority */
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);                             /* Enable I2C interrupt */
    
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);                     /* Set interrupt priority */
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);                             /* Enable I2C interrupt */

    /* Configure DMA */
    /* Configure DMA handle for transmission */
    hdma1ch3.Instance                 = DMA1_Channel3;           /* Select DMA channel 1 */
    hdma1ch3.Init.Direction           = DMA_MEMORY_TO_PERIPH;    /* Memory to peripheral direction */
    hdma1ch3.Init.PeriphInc           = DMA_PINC_DISABLE;        /* Disable peripheral address increment */
    hdma1ch3.Init.MemInc              = DMA_MINC_ENABLE;         /* Enable memory address increment */
    hdma1ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;     /* Peripheral data width is 8 bits */
    hdma1ch3.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;     /* Memory data width is 8 bits */
    hdma1ch3.Init.Mode                = DMA_NORMAL;              /* Disable circular mode */
    hdma1ch3.Init.Priority            = DMA_PRIORITY_VERY_HIGH;  /* Channel priority is very high */

    HAL_DMA_Init(&hdma1ch3);                                     /* Initialize DMA channel 1 */
    __HAL_LINKDMA(hi2c, hdmatx, hdma1ch3);                        /* Link DMA1 with IIC_TX */

    /* Configure DMA handle for reception */
    hdma1ch4.Instance                 = DMA1_Channel4;           /* Select DMA channel 2 */
    hdma1ch4.Init.Direction           = DMA_PERIPH_TO_MEMORY;    /* Direction : peripheral to memory */
    hdma1ch4.Init.PeriphInc           = DMA_PINC_DISABLE;        /* Disable peripheral address increment */
    hdma1ch4.Init.MemInc              = DMA_MINC_ENABLE;         /* Enable memory address increment */
    hdma1ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;     /* Peripheral data width is 8 bits */
    hdma1ch4.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;     /* Memory data width is 8 bits */
    hdma1ch4.Init.Mode                = DMA_NORMAL;              /* Disable circular mode */
    hdma1ch4.Init.Priority            = DMA_PRIORITY_HIGH;       /* Channel priority is high */

    HAL_DMA_Init(&hdma1ch4);                                     /* Initialize DMA channel 1 */
    __HAL_LINKDMA(hi2c, hdmarx, hdma1ch4);                        /* Link DMA1 with IIC_RX */
    
    /* DMA configuration request image */
    HAL_DMA_ChannelMap(&hdma1ch3, DMA_CHANNEL_MAP_I2C1_WR); /* DMA3_MAP选择为IIC_TX */
    HAL_DMA_ChannelMap(&hdma1ch4, DMA_CHANNEL_MAP_I2C1_RD); /* DMA4_MAP选择为IIC_RX */
    
    /* NVIC interrupt enabling DMA */ 
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 1);             /* Set interrupt priority */
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);                     /* Enable DMA channel 1 interrupt */

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 1);           /* Set interrupt priority */
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);                   /* Enable DMA channel 2 interrupt */
}

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
