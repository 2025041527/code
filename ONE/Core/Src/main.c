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
#include "gpio.h"
#include "bsp_dwt.h"
#include "motor_simulation.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_OUTPUT_MAX 24.0f 
#define PID_OUTPUT_MIN -24.0f 
#define PID_INTEGRAL_MAX 24.0f 
#define PID_INTEGRAL_MIN -24.0f 

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float velocityRef;
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
	  float Input;
	  float Current;
} pid_t;

motorObject_t Motor;
pid_t PID;
uint32_t DWT_CNT;
float dt;
float t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PID_Init(pid_t *p);
float PID_Calculate(float setpoint, float dt, pid_t *PID, float current);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PID_Init(pid_t *p)
{
    if (!p) return;
    p->kp = 1.0f;
    p->ki = 0.0f;
    p->kd = 0.0f;
    p->integral = 0.0f;
    p->prev_error = 0.0f;
}

float PID_Calculate(float setpoint, float dt, pid_t *PID, float current)
{
    if (dt <= 0.0f) return 0.0f;
    float error = setpoint - current;
    
    PID->integral += error * dt;
    PID->integral = (PID->integral > PID_INTEGRAL_MAX) ? PID_INTEGRAL_MAX : 
                   (PID->integral < PID_INTEGRAL_MIN) ? PID_INTEGRAL_MIN : PID->integral;
    
    float derivative = (error - PID->prev_error) / dt;
    
    float out = PID->kp * error + PID->ki * PID->integral + PID->kd * derivative;
    out = (out > PID_OUTPUT_MAX) ? PID_OUTPUT_MAX : 
          (out < PID_OUTPUT_MIN) ? PID_OUTPUT_MIN : out;
    
    PID->prev_error = error;
    return out;
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

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  
  /* USER CODE BEGIN 2 */
  DWT_Init(72);
  Motor_Object_Init(&Motor);
  PID_Init(&PID);
  DWT_CNT = 0;
  dt = 0.0f;
  t = 0.0f;
  velocityRef = 0.0f;
  /* USER CODE END 2 */

  while (1)
  {   
      dt = DWT_GetDeltaT(&DWT_CNT); 
      t += dt;

      PID.Current = Get_Motor_Current(&Motor);
      Motor.Velocity = Get_Motor_Velocity(&Motor);
      Motor.Angle = Get_Motor_Angle(&Motor);
      velocityRef = 10.0f;
      PID.Input = PID_Calculate(velocityRef, dt, &PID, Motor.Velocity);

      Motor_Simulation(&Motor,PID.Input, dt);

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
      HAL_Delay(1);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
