/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "arm_pwm.h"       /* T1: arm_init() / arm_set_deg() */
#include "arm_joint.h"     /* T2: arm_joint_init() / 关节执行层（底座闭环） */
#include "arm_cfg.h"       /* T1: 标定表 g_arm_joint[] 与时基宏 */
#include "proto_app.h"     /* T5a: proto_app_init() / proto_app_poll() */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FW_NAME          "rm_arm"
#define FW_VERSION       "0.2.0"

/* 注意：脉宽 / 角度 / 时基这些数字**不在这里**，
 * 按任务书 T1 的要求全部集中在 Core/Inc/arm_cfg.h。 */
#define KEY_DEBOUNCE_MS  20u                       /* 按键消抖时间 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* main.c 刻意**不**声明 hUsbDeviceFS：
 * CDC 端点归协议层独占（proto_link.c），这里连指针都拿不到，
 * 一旦手滑写了 CDC 调用就变成编译错误，而不是运行时的隐性污染。 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---------------- 日志输出：**只走 USART1** ----------------
 * ⚠️ 绝对不要把日志写进 USB CDC。
 *
 * 原因：CDC 是给上位机的**协议字节流**，上位机按帧头 0xA5 同步、按长度字段切包。
 * 一旦掺进任何文本：
 *     A5 10 13 84 03 ... 5B 54 31 5D ...     ← "[T1]" 混进帧中间
 *     长度字段声明 19 字节，凑齐后 CRC 必然不符
 *     → 上位机丢 1 字节重新同步 → 帧视图标红 → 丢帧统计暴涨
 *
 * 规则：**一个端点只能有一个主人**。
 *   CDC 端点由协议层独占（proto_link.c），日志只走 USART1 —— 物理上是另一根线，
 *   怎么打都不会碰到协议字节流。
 *
 * 代价：手上没有 USB-TTL 时看不到这些输出。这是有意的取舍：
 *       调试信息看不到可以忍，协议被污染不行。
 */

#define LOG_BUF_SIZE   160u

static char     s_log_buf[LOG_BUF_SIZE];
static uint16_t s_log_len = 0;

static void log_flush(void)
{
  if (s_log_len == 0u)
  {
    return;
  }

  /* 50ms 超时：115200 下一行 100 字节约 8.7ms；没接线也只是空等，不会死锁 */
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)s_log_buf, s_log_len, 50);

  s_log_len = 0u;
}

/* newlib 的 printf 最终会一个字符一个字符调到这里
 * （链路：printf → _write() in syscalls.c → __io_putchar()） */
int __io_putchar(int ch)
{
  if (s_log_len >= (LOG_BUF_SIZE - 1u))
  {
    log_flush();                     /* 缓冲区满，先冲出去 */
  }

  s_log_buf[s_log_len++] = (char)ch;

  if (ch == '\n')
  {
    log_flush();                     /* 一行结束立刻发，保证及时可见 */
  }

  return ch;
}

/* ---------------- 本次复位原因 ---------------- */
static const char *reset_cause_str(void)
{
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)  != RESET) return "POR/PDR 上电复位";
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)  != RESET) return "NRST 外部复位脚";
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)  != RESET) return "SW 软件复位";
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) return "IWDG 独立看门狗";
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET) return "WWDG 窗口看门狗";
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET) return "LPWR 低功耗复位";
  return "未识别";
}

/* ---------------- 启动横幅 ----------------
 * 打两次：① 上电时（此时 USB 可能还没枚举完，走 CDC 的那份会丢）
 *         ② USB 被电脑配置好之后补发一次（见 log_usb_poll）
 * 所以"复位原因"必须缓存：清标志发生在第一次打印之后，
 * 第二次再读就变成"未识别"了。
 */
static const char *s_reset_cause = NULL;

static void print_banner(void)
{
  if (s_reset_cause == NULL)
  {
    s_reset_cause = reset_cause_str();   /* 只在第一次真正读标志位 */
  }

  printf("\r\n==============================\r\n");
  printf(" 工程名 : %s  v%s\r\n", FW_NAME, FW_VERSION);
  printf(" 芯片   : STM32F103C8T6 (64K Flash / 20K RAM)\r\n");
  printf(" SYSCLK : %lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
  printf(" HCLK   : %lu Hz\r\n", (unsigned long)HAL_RCC_GetHCLKFreq());
  printf(" PCLK1  : %lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK1Freq());
  printf(" PCLK2  : %lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK2Freq());
  printf(" USB    : 48 MHz\r\n");
  printf(" 复位源 : %s\r\n", s_reset_cause);
  printf(" PWM    : TIM2 4ch @ 50Hz, tick=%u Hz, CCR=%u~%u (中位 %u)\r\n",
         (unsigned)ARM_PWM_TICK_HZ,
         (unsigned)ARM_PWM_US_TO_CCR(SERVO_US_NOMINAL_MIN),
         (unsigned)ARM_PWM_US_TO_CCR(SERVO_US_NOMINAL_MAX),
         (unsigned)ARM_PWM_US_TO_CCR(SERVO_US_HOME));

  /* 把标定表打出来，改完 arm_cfg.c 一眼能确认烧进去的是不是新值 */
  printf(" 关节标定表 (改 Core/Src/arm_cfg.c):\r\n");
  {
    uint8_t j;
    for (j = 0; j < ARM_JOINT_NUM; j++)
    {
      printf("   [%u] %s  %d~%d deg  home %d  %u~%u us\r\n",
             (unsigned)j,
             g_arm_joint[j].name,
             (int)g_arm_joint[j].deg_min,
             (int)g_arm_joint[j].deg_max,
             (int)g_arm_joint[j].deg_home,
             (unsigned)g_arm_joint[j].us_at_deg_min,
             (unsigned)g_arm_joint[j].us_at_deg_max);
    }
  }

  printf("==============================\r\n");
  printf("[T1] 就绪: 四路 PWM 已输出中位；标定见 docs/T1_角度脉宽映射_说明.md\r\n");
}

/* ---------------- T0 上电自检 ---------------- */
static void t0_setup(void)
{
  /* LED 先灭（PC13 低电平点亮） */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  setvbuf(stdout, NULL, _IONBF, 0);   /* 关掉缓冲，插上串口就能看到 */

  /* 四路 PWM：交给 T1 模块，脉宽/角度参数全在 arm_cfg.h */
  arm_init();

  /* 关节执行层：决定哪个关节走闭环（底座 360°）、哪个直通。
   * 必须在 arm_init() 之后 —— 它内部要往 CCR 写零位脉宽。 */
  arm_joint_init();

  print_banner();

  /* 打印完再清标志，否则下次读到的是被清掉的状态 */
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

/* ---------------- USB 配置完成检测 ----------------
 * 说明：这里**不**碰 CDC。日志只走 USART1（见文件头 log_flush 的注释），
 *       所以不存在"USB 连上后补发横幅"这回事 —— 想看得接 USB-TTL。
 */

/* ---------------- 20ms 消抖按键：返回 1 = 刚按下 ---------------- */
static uint8_t key_scan(void)
{
  static uint8_t  stable   = 0;   /* 稳定态 0=松开 1=按下 */
  static uint8_t  raw_prev = 0;
  static uint32_t t_edge   = 0;

  uint8_t  raw = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
  uint32_t now = HAL_GetTick();

  if (raw != raw_prev)          /* 电平刚变，重新计时 */
  {
    raw_prev = raw;
    t_edge   = now;
    return 0;
  }

  if ((raw != stable) && ((now - t_edge) >= KEY_DEBOUNCE_MS))
  {
    stable = raw;               /* 稳定了才认 */
    if (stable)
    {
      return 1u;                /* 只在按下沿给一次事件 */
    }
  }
  return 0;
}

/* ---------------- T0 主循环体 ---------------- */
static void t0_loop(void)
{
  static uint8_t led_on = 0;    /* 逻辑亮灭；PC13 低电平点亮 */

  if (key_scan())
  {
    led_on = (uint8_t)!led_on;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, led_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    printf("[T0] PB0 按下 -> LED %s\r\n", led_on ? "ON" : "OFF");
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals *///初始化外设
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  t0_setup();
  proto_app_init();          /* T5a: 协议层就绪（先 arm_init 再初始化协议，顺序不能反） */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    t0_loop();
    proto_app_poll();        /* T5a: 收支协议帧、按 50Hz 上报状态、1Hz 心跳 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
/*
  24–75	  Includes / 宏 / 全局	                                              我们
  77–240	log_flush __io_putchar reset_cause_str t0_setup key_scan t0_loop	 我们
  242–290	main()	                                            CubeMX 骨架 + 我们两行
  292–556	MX_GPIO_Init 等 6 个初始化函数	                                CubeMX 生成
  557–591	USER CODE 4 + Error_Handler	                                      混
*/