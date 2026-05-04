/**
 * @file    main.c
 * @brief   系统主入口（重构版）
 * 
 * 启动流程：
 *   1. HAL 库初始化
 *   2. 系统时钟配置
 *   3. 外设初始化（GPIO、TIM、ADC、UART 等）
 *   4. 系统状态初始化（I2C 总线、全局状态）
 *   5. 启动所有 RT-Thread 线程
 *   6. 启动调度器
 */

#include "main.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "spi.h"
#include "i2c.h"
#include "rng.h"

#include "system_state.h"
#include "thread_manager_v2.h"
#include "uart_manager.h"

#include <rtthread.h>
#include <stdio.h>

/* ========================== 外设句柄（extern 引用，定义在 CubeMX 生成的 HAL 文件中） ========================== */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;

extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;
extern TIM_HandleTypeDef  htim4;
extern TIM_HandleTypeDef  htim6;

extern ADC_HandleTypeDef  hadc1;

/* ========================== 系统初始化（函数声明来自 HAL 头文件） ========================== */

void SystemClock_Config(void);

/**
 * @brief  系统上电入口
 *         注意：使用 RT_USING_USER_MAIN 时，RT-Thread 会在自己的 main 线程中调用此函数
 */
int main(void)
{
    /* HAL 库初始化 */
    HAL_Init();

    /* 系统时钟配置 */
    SystemClock_Config();

    /* 外设初始化 */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_UART4_Init();
    MX_UART5_Init();
    MX_UART7_Init();
#ifdef HAL_SPI_MODULE_ENABLED
    MX_SPI4_Init();
#endif
#ifdef HAL_RNG_MODULE_ENABLED
    MX_RNG_Init();
#endif

    rt_kprintf("\r\n========================================\r\n");
    rt_kprintf("  ROV Control System v2.0\r\n");
    rt_kprintf("  MCU: STM32H750VBT6 @ 480MHz\r\n");
    rt_kprintf("  RTOS: RT-Thread 3.1.5\r\n");
    rt_kprintf("========================================\r\n\r\n");

    /* 初始化系统状态（包括 I2C 总线） */
    rt_kprintf("[SYS] Initializing system state...\r\n");
    system_state_init();

    /* 启动所有线程 */
    rt_kprintf("[SYS] Starting threads...\r\n");
    thread_manager_start_all();

    rt_kprintf("[SYS] System ready.\r\n\r\n");

    /* RT-Thread 调度器已自动启动，此处不再返回 */
    return 0;
}

/* ========================== HAL UART 中断回调（转发到 uart_manager） ========================== */

/**
 * 注意：此回调替代了原有的 HAL_UART_RxCpltCallback 逻辑
 * 所有 UART 接收由 uart_manager 统一管理
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_rx_callback(huart);
}

/* ========================== 系统时钟配置（从 CubeMX 生成） ========================== */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ========================== 错误处理 ========================== */

void Error_Handler(void)
{
    /* 关闭所有电机 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);

    __disable_irq();

    while (1)
    {
        /* LED 快速闪烁指示错误 */
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
        for (volatile uint32_t i = 0; i < 1000000; i++) { __NOP(); }
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    rt_kprintf("Assert failed: %s, line %lu\r\n", file, line);
}
#endif
