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
#define PID_OUTPUT_MAX 10000.0f 
#define PID_OUTPUT_MIN -10000.0f 
#define PID_INTEGRAL_MAX 10000.0f 
#define PID_INTEGRAL_MIN -10000.0f 
#define PI 3.1415926f
#define ANGLE_REF_STEP 2*PI
#define LOW_PASS_ALPHA 0.2f
#define VELOCITY_FF_K 0.5f   


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Velocity_Filtered = 0.0f; 
float Angle_Filtered = 0.0f;    
float omega = 5.0f;
float VelocityRef;
float Velocity;
float Angle;
float AngleRef;
typedef struct {
    float kp1;
    float kp2;
    float ki1;
    float ki2;
    float kd1;
    float kd2;
    float integral;
    float prev_error;
	  float error;
	  float Current;
	  float derivative;
    float out;
} pid_t;
motorObject_t Motor;
pid_t PID;
uint32_t DWT_CNT;
float dt;
float t;
float Input;
int CONTROL_MODE = 2;
float disturb = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PID_Init(pid_t *p);
float PID_Calculate(float setpoint, float dt, pid_t *PID,float current);
float LowPassFilter(float input, float prev_output, float alpha);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float LowPassFilter(float input, float prev_output, float alpha)
{
    if (alpha < 0.0f || alpha > 1.0f) alpha = 0.2f; 
    return alpha * input + (1.0f - alpha) * prev_output;
}
void PID_Init(pid_t *p)
{
    if (!p) return;
    p->kp1 = 50.0f;
    p->kp2 = 50.0f;
    p->ki1 = 0.1f;
    p->ki2 = 0.1f;
    p->kd1 = 0.0f;
    p->kd2 = 0.0f;
    p->integral = 0.0f;
    p->prev_error = 0.0f;
	  p->error = 0.0f;
    p->Current = 0.0f;
    p->derivative = 0.0f;
}

float PID_Calculate1(float setpoint, float dt, pid_t *PID,float current)
{
    if (dt <= 0.0f) return 0.0f;
    PID->error = setpoint - current;
    PID->integral += PID->error * dt;
    if (PID->integral > PID_INTEGRAL_MAX) PID->integral = PID_INTEGRAL_MAX;
    else if (PID->integral < PID_INTEGRAL_MIN) PID->integral = PID_INTEGRAL_MIN;
    PID->derivative = (PID->error - PID->prev_error) / dt;
    PID->out = PID->kp1 * PID->error + PID->ki1 * PID->integral + PID->kd1 * PID->derivative;
    if (PID->out > PID_OUTPUT_MAX) PID->out = PID_OUTPUT_MAX;
    else if (PID->out < PID_OUTPUT_MIN) PID->out = PID_OUTPUT_MIN;
    PID->prev_error = PID->error;
    return PID->out;
}
float PID_Calculate2(float setpoint, float dt, pid_t *PID,float current)
{
    if (dt <= 0.0f) return 0.0f;
    PID->error = setpoint - current;
    PID->integral += PID->error * dt;
    if (PID->integral > PID_INTEGRAL_MAX) PID->integral = PID_INTEGRAL_MAX;
    else if (PID->integral < PID_INTEGRAL_MIN) PID->integral = PID_INTEGRAL_MIN;
    PID->derivative = (PID->error - PID->prev_error) / dt;
    PID->out = PID->kp2 * PID->error + PID->ki2 * PID->integral + PID->kd2 * PID->derivative;
    if (PID->out > PID_OUTPUT_MAX) PID->out = PID_OUTPUT_MAX;
    else if (PID->out < PID_OUTPUT_MIN) PID->out = PID_OUTPUT_MIN;
    PID->prev_error = PID->error;
    return PID->out;
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
	Velocity_Filtered = Get_Motor_Velocity(&Motor);
  Angle_Filtered = Get_Motor_Angle(&Motor);
  /* USER CODE END 2 */

while (1)
  {   
      dt = DWT_GetDeltaT(&DWT_CNT); 
      t += dt;
      PID.Current = Get_Motor_Current(&Motor);
      Velocity = Get_Motor_Velocity(&Motor);
      Angle = Get_Motor_Angle(&Motor) ;
		  Velocity_Filtered = LowPassFilter(Velocity, Velocity_Filtered, LOW_PASS_ALPHA);
      Angle_Filtered = LowPassFilter(Angle, Angle_Filtered, LOW_PASS_ALPHA);
      if (CONTROL_MODE == 1)
      { AngleRef = 0.0f;
        Input = PID_Calculate1(AngleRef, dt, &PID, Angle_Filtered);
      }
      else if (CONTROL_MODE == 2)
      {
        AngleRef = 0.0f;
        VelocityRef = PID_Calculate1(AngleRef, dt, &PID, Angle_Filtered);
        float Velocity_FF = VELOCITY_FF_K * VelocityRef;
        Input = PID_Calculate2(VelocityRef, dt, &PID, Velocity_Filtered) + Velocity_FF;
      }
      
      Motor_Simulation(&Motor,Input, dt);

      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
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
