/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* MPU6050数据 */
float pitch,roll,yaw; 			/* 欧拉角 */
short aacx,aacy,aacz;				/* 加速度传感器原始数据 */
short gyrox,gyroy,gyroz;		/* 陀螺仪原始数据 */
short temperature;					/* 温度 */

/* 编码器的读数 */
int Left_encoder = 0;
int Right_encoder = 0;

/* 输出给电机的pwm值 */
int Moto1 = 0;
int Moto2 = 0;

/* 定义PID结构体 */
PID wPID;  /* 角速度环 */
PID vPID;  /* 速度环 */
PID aPID;  /* 角度环 */

/* 控制环输出 */
float vPIDOutput = 0;
float aPIDOutput = 0;
float wPIDOutput = 0;

float mechanicalZero = 0.0;

float Encoder_Least = 0;  /* 最新速度偏差 */
float Encoder = 0;  /* 速度值 */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

void PIDInit(PID *pp);

double PID_W(PID *pp, double NextPoint);
double PID_V(PID *pp, double NextPoint);
double PID_A(PID *pp, double NextPoint);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* printf重定向 */
int fputc(int ch, FILE *f)
{
	uint8_t temp[1] = {ch};
	HAL_UART_Transmit(&huart1, temp, 1, 2);/* huart1需要根据你的配置修改 */
	return ch;
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

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	/* ************************************************初始化******************************************** */
	
	delay_init(72);  /* 初始化延时函数 */
	
	/* 初始化OLED */
	OLED_Init();
	OLED_Clear();
	oled_first_show();
	
	HAL_Delay(1);
	
	/* 初始化MPU6050 */
	printf("mpu6050\n");
	while(MPU_Init());	
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  /* 启动编码器模式 */
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  /* 启动编码器模式 */
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* 使能PWM */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  /* 使能PWM */
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);  /* 设定初始占空比 */
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);  /* 设定初始占空比 */
	
	/* 初始化PID */
	PIDInit(&aPID);  /* 角度环 PD控制 */
	aPID.Kp = 300;
	aPID.Kd = 1;
	aPID.SetPoint = 0;      
	
	PIDInit(&wPID);  /* 角速度环 PI控制 */
	wPID.Kp = 1.5;
	wPID.Ki = wPID.Kp/200;
	wPID.SetPoint = 0;
	
	PIDInit(&vPID);  /* 速度环 PI控制 */
	vPID.Kp = 0;//0.05;
	vPID.Ki = vPID.Kp/200;
	vPID.SetPoint = 0;
	
	
	
	HAL_TIM_Base_Start_IT(&htim3);   /* 定时器3使能 */
	
	
	//Set_Pwm(1000, 1000);
	
	
	printf("init finished\n");
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		oled_show();
		delay_ms(1);

		
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
}

/* USER CODE BEGIN 4 */

/* 时间标志 */
uint16_t tim_cnt_5 = 0;
uint16_t tim_cnt_10 = 0;
uint16_t tim_cnt_20 = 0;

/* 定时器中断回调函数 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == (&htim3)){
		tim_cnt_5++;
		tim_cnt_10++;
		tim_cnt_20++;
		
		/* 5ms 角速度环控制*/
		if(tim_cnt_5 >= 5) {
			tim_cnt_5 = 0;
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	/* 得到陀螺仪数据 */
			//printf("%d\n", gyroy);
			
			wPIDOutput = PID_W(&wPID, gyroy);
			Set_Pwm(wPIDOutput, wPIDOutput);  /* 将PWM输出给电机 */
			
			//printf("%f\n", wPIDOutput);
		}
		
		/* 10ms 角度环控制*/
		if(tim_cnt_10 >= 10) {
			tim_cnt_10 = 0;
			/* 获取欧拉角 */
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	/* 得到加速度传感器数据 */
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	/* 得到陀螺仪数据 */
			IMU_quaterToEulerianAngles_mpu6050();  /* 获取欧拉角 */
			printf("%.2f,%.2f,%.2f\n", eulerAngle_mpu6050.pitch, eulerAngle_mpu6050.roll, eulerAngle_mpu6050.yaw);
			
			aPIDOutput = PID_A(&aPID, eulerAngle_mpu6050.pitch);
			
		}
		
		/* 20ms 速度环控制*/
		if(tim_cnt_20 >= 20) {
			tim_cnt_20 = 0;
			
			/* 获取转速 */
			Left_encoder = -Read_Encoder(4);
			Right_encoder = Read_Encoder(2);
			//printf("%d,%d\n", Left_encoder, Right_encoder);
			
			/* 对速度进行低通滤波 */
			Encoder_Least =(Left_encoder +  Right_encoder) - 0;   /* 获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） */
			Encoder *= 0.8;  /* 一阶低通滤波器 */
			Encoder += Encoder_Least*0.2;
			
			vPIDOutput = PID_V(&vPID, Encoder);
			
		}
	}
}


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
