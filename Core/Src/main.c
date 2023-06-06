#include "main.h"
#include "i2c-lcd.h"
#include "math.h"

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);

int init_sensor(void);
void getAccelerate(void);
float fact(float a, float b);
float getMagnitude(float a, float b, float c);
void calibrate(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);	

float true_AccelX, true_AccelY, true_AccelZ;;
float referenceMag;
char bufstring[100];
int16_t raw_GyroX, raw_GyroY, raw_GyroZ;
int16_t raw_AccelX, raw_AccelY, raw_AccelZ;
				
uint8_t pauseflag = 0;	
uint8_t resetflag = 0;	
uint8_t juststartedflag = 1;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
	
		
  lcd_init();
	lcd_clear();
	lcd_send_cmd (0x80|0x00);
	lcd_send_string("ELT3240-2EoS project");
	lcd_send_cmd (0x80|0x40);
	lcd_send_string(" Dr. Nguyen K. Hung");
	lcd_send_cmd (0x80|0x14);
	lcd_send_string("20020320    20020173");
	HAL_Delay(1000);
	lcd_clear();
	while(init_sensor() != 32)
	{
		lcd_send_cmd (0x80|0x00);//(0,0)
		lcd_send_string("Awaiting connection~");
	}
	lcd_send_cmd (0x80|0x00);
	lcd_send_string(" Sensor connected!");
	lcd_send_cmd (0x80|0x40);
	lcd_send_string("Hold still");
	lcd_send_cmd (0x80|0x14);
	lcd_send_string("     for calibration");

	calibrate();
	lcd_clear();
	
	float noiseFilter = 0.03; //////////TOLERANCE
	float triggerCount = 0.3;
	uint8_t risingFlag = 2; // 0 = not rising, 1 = rising, 2 = other
	uint8_t hitRockBottom = 0; // 0 = reach bottom recently, 1 = not...
		
	lcd_clear();
	
	uint8_t count = 0;
	float currentMag = 0;
	float previousMag = 0;
	float minDebug = 0;
	float maxDebug = 0;
	float recentPeak = 0;
	HAL_TIM_Base_Start_IT(&htim1);
  while (1)
  {
		if (resetflag == 1)
		{
			lcd_clear();
			resetflag = 0;
			lcd_send_cmd (0x80|0x40);// row2
			lcd_send_string("   Resetting...");
			HAL_Delay(200);
			count = 0;
			currentMag = 0;
			previousMag = 0;
			minDebug = 0;
			maxDebug = 0;
			recentPeak = 0;
			risingFlag = 2;
			juststartedflag = 1;
			hitRockBottom = 0;
			lcd_send_cmd (0x80|0x14);// row3
			lcd_send_string("  Recalibrating...");
			calibrate();
			lcd_clear();
			lcd_send_cmd (0x80|0x40);// row2
			lcd_send_string("   Reset successful!");	
			HAL_Delay(200);
			lcd_clear();			
		}
		if(pauseflag == 0)
		{
			getAccelerate();
			currentMag = getMagnitude(true_AccelX, true_AccelY, true_AccelZ) - referenceMag;
			if(fact(currentMag, previousMag) < noiseFilter)
			{
				if (juststartedflag != 1)
				{
					continue;
				}
				else 
				{
					juststartedflag = 0;
				}
			}
			if(currentMag > previousMag)
			{
				if(risingFlag == 0)
				{
					if(previousMag < -0.03)
					{
						hitRockBottom = 1;
					}			
				}
				risingFlag = 1;
			}
			else if(currentMag < previousMag)
			{
				if(risingFlag == 1)
				{
					if(previousMag > triggerCount)
					{
						if(hitRockBottom == 1)
						{
							count++;
							recentPeak = previousMag;
							hitRockBottom = 0;
						}
					}	
				}
				risingFlag = 0;
			}
			/*
			if(currentMag > maxDebug)
			{
				maxDebug = currentMag;
			}
			else if(currentMag < minDebug)
			{
				minDebug = currentMag;
			}
			*/
			lcd_send_cmd (0x80|0x00);//row1
			lcd_send_string("Pedometer by GrNo.23");
			lcd_send_string(bufstring);
			sprintf(bufstring,"%.2f  ", recentPeak);
			lcd_send_string(bufstring);
			
			lcd_send_cmd (0x80|0x40);// row2
			lcd_send_string(" CurrentMag = ");
			sprintf(bufstring,"%.2f",currentMag);
			lcd_send_string(bufstring);

				
			lcd_send_cmd (0x80|0x14);// row3
			lcd_send_string(" Steps = ");
			sprintf(bufstring,"%d   ",count);
			lcd_send_string(bufstring);
			sprintf(bufstring,"%.2f  ", recentPeak);
			lcd_send_string(bufstring);
			
			lcd_send_cmd (0x80|0x54);// row4
			lcd_send_string("SW1 pause||SW2 reset");
			
			previousMag = currentMag;
			HAL_Delay(100);		
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
			lcd_send_cmd (0x80|0x54);// row4
			lcd_send_string("   SW1 to unpause   ");
			while (1)
			{
				if(pauseflag == 0)
				{ 
					break;
				}
				lcd_send_cmd (0x80|0x40);// row2
				lcd_send_string("                    ");
				HAL_Delay(100);
				HAL_Delay(100);
				HAL_Delay(100);
				HAL_Delay(100);
				HAL_Delay(100);
				lcd_send_cmd (0x80|0x40);// row2
				lcd_send_string("      Paused!");
				HAL_Delay(100);
				HAL_Delay(100);
				HAL_Delay(100);
				HAL_Delay(100);
				HAL_Delay(100);
				
			}
			lcd_send_cmd (0x80|0x40);// row2
			lcd_send_string("                    ");
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		}
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	if (htim->Instance == TIM1 && pauseflag ==0)
	{
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);

	}
	else if (htim->Instance == TIM1 && pauseflag == 1)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET);
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

/***************************************************************************
********************************************************************************************************************************************************
										CUSTOM FUNCTION_ntuan
********************************************************************************************************************************************************
***************************************************************************/
	//referred to datasheet for reg addr
	//https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
	


int init_sensor(void)
{
	uint8_t sensor_who; 
	HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x75, 1, &sensor_who, 1, 1000); //read sensor's WHO_AM_I
	if(sensor_who == 0x68) //sensor successfully respond
	{
		uint8_t init_val = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x6B, 1, &init_val, 1, 1000); //write all 0 to power management reg (PWR_MGMT_1), power on
		init_val = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x19, 1, &init_val, 1, 1000); //set sample rate's divisor to SMPLRT_DIV_REG, default is 8khz, only 1khz is needed (quotient)
		init_val = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x1B, 1, &init_val, 1, 1000); //GYRO_CONFIG 
		init_val = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x1C, 1, &init_val, 1, 1000); // and ACCEL_CONFIG to full-scale
		//INSERT SENSOR'S READY! TO LCD
		return 32;
	}
}

void getAccelerate(void) //measure acceleration, give true values (1g = 9.8 m/s2)
{
	uint8_t rawbinary[6];
  HAL_I2C_Mem_Read(&hi2c1,0xD0,0x3B,1, rawbinary,6,100); //read ACCEL_XOUT_H to ACCEL_ZOUT_L
	raw_AccelX = (int16_t)(rawbinary[0] << 8 | rawbinary[1]); //ACCEL_XOUT_H and L
  raw_AccelY = (int16_t)(rawbinary[2] << 8 | rawbinary[3]); //Y
  raw_AccelZ = (int16_t)(rawbinary[4] << 8 | rawbinary[5]); //Z

  true_AccelX = raw_AccelX / 16384.00;
  true_AccelY = raw_AccelY / 16384.00;
  true_AccelZ = raw_AccelZ / 16384.00;
}
float fact(float a, float b)
{
	if(a >= b)
	{
		return a-b;
	}
	else
	{
		return b-a;
	}
}
float getMagnitude(float a, float b, float c)
{
	return sqrt(a*a + b*b + c*c);
}

void calibrate(void)
{
		HAL_Delay(2000);
		getAccelerate();
		referenceMag = getMagnitude(true_AccelX, true_AccelY, true_AccelZ);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_1)
	{
		resetflag = 1;
		pauseflag = 0;
	}	
	else if (GPIO_Pin == GPIO_PIN_2)
 {
   pauseflag = !pauseflag;	
 }
}

	//referred to datasheet for reg addr
	//https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
/***************************************************************************
********************************************************************************************************************************************************
										CUSTOM FUNCTION_ntuan
											CHECK GLOBAL VAR
********************************************************************************************************************************************************
***************************************************************************/



