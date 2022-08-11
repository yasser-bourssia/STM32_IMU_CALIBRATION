/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LSM303D_REG_CTRL0 0x1F
#define LSM303D_REG_CTRL1 0x20
#define LSM303D_REG_CTRL2 0x21
#define LSM303D_REG_CTRL3 0x22
#define LSM303D_REG_OUT_X_L_A 0x28
#define WHO_AM_I 0x0F

#define sensZ 1.11713409
#define sensY 1.03767407
#define sensX 1.02789497
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
 float alpha, beta, gama, sens, roll, pitch, M[9], MD[9];

 float resultAccel[3], x ,y ,z ,x1 ,yt ,z1, X,Y,Z;
 int8_t thres=0, test = 1;

 double totalX = 0,totalY = 0,totalZ = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t ReadReg_SPI1(uint8_t addr, uint8_t *byte);
void WriteReg_SPI1(uint8_t addr, uint8_t byte);
void Accel_config();
void Accel_read();
void offsetCorrection_Sensitivity();
void findAngles(float x, float y, float z, float *alpha, float *beta, float *gamma, float *roll, float *pitch);
void matrixRotation(float roll, float pitch, float *M, float *MD);
void rotateVector(float x1, float yt, float z1, float *x, float *y, float *z, float *M );

void average(float *x1, float *yt, float *z1);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t txData[2];
	uint8_t received;
	txData[0] = WHO_AM_I | 0x80;
	txData[1] = 0;



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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


  Accel_config();



 ReadReg_SPI1(WHO_AM_I, &received);

 ReadReg_SPI1(LSM303D_REG_CTRL1, &received);

 ReadReg_SPI1(LSM303D_REG_CTRL0, &received);

 ReadReg_SPI1(LSM303D_REG_CTRL2, &received);
 ReadReg_SPI1(LSM303D_REG_CTRL3, &received);

for (int i = 0; i< 1; i++){
	 // Accel_read();
	 // x1 = resultAccel[0];
	 // yt = resultAccel[1];
	 // z1 = resultAccel[2];
	  //offsetCorrection();
		average(&x1, &yt, &z1);
	  x1 = x1/sensX;
	  yt = yt/sensY;
	  z1 = z1/sensZ;

	  findAngles(x1, yt, z1, &alpha, &beta, &gama, &roll, &pitch);
	  matrixRotation(roll, pitch, M, MD);




}

 //offsetCorrection_Sensitivity();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  X = x;
	  Y = y;
	  Z = z;
	  Accel_read();
	  x1 = resultAccel[0];
	  yt = resultAccel[1];
	  z1 = resultAccel[2];
	  //offsetCorrection();
	  x1 = x1/sensX;
	  yt = yt/sensY;
	  z1 = z1/sensZ;


	  rotateVector(x1, yt, z1, &x, &y, &z, MD);

	  HAL_Delay(80);

	  //printf("X : %d \n", resultAccel[0]);
	  //printf("Y : %d \n", resultAccel[1]);
	  //printf("Z : %d \n", resultAccel[2]);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


uint8_t ReadReg_SPI1(uint8_t addr, uint8_t *byte){

	HAL_StatusTypeDef hal_status;
	uint8_t tx_data[2];
	uint8_t rx_data[2];
	tx_data[0] = addr | 0x80;
	tx_data[1] = 0;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

	hal_status = HAL_SPI_TransmitReceive (&hspi1, tx_data, rx_data, 2, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);

	HAL_Delay(1);

	if (hal_status == HAL_OK)
	{

	*byte = rx_data[1];
	return rx_data[1];


	}

	return rx_data[1];



}

void WriteReg_SPI1 (uint8_t addr, uint8_t byte){


	uint8_t tx_data[2];
	tx_data[0] = addr;
	tx_data[1] = byte;

	HAL_GPIO_WritePin (GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, tx_data, 2, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);

	HAL_Delay(1);



}

void Accel_config(){


	uint8_t txData[10], rxData[10];

	WriteReg_SPI1 (LSM303D_REG_CTRL1, 0x37);
	WriteReg_SPI1 (LSM303D_REG_CTRL2, 0x00);
	WriteReg_SPI1 (LSM303D_REG_CTRL0, 0x00);



}

void Accel_read(){


	uint8_t sendData[8];
	uint8_t receiveData[8];
	int16_t x,y,z;

	sendData[0] = LSM303D_REG_OUT_X_L_A | 0x80 | 0x40;

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

	  HAL_SPI_Transmit(&hspi1, sendData, 1,HAL_MAX_DELAY);
	  HAL_SPI_Receive(&hspi1, receiveData, 6, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);


	//x = (*((int16_t*) (receiveData+2))) * 0.000061;
	//y = (*((int16_t*) (receiveData+4))) * 0.000061;
	//z = (*((int16_t*) (receiveData+6))) * 0.000061;


	resultAccel[0] = (*((int16_t*) (receiveData))) * 0.000061 ;
	resultAccel[1] = (*((int16_t*) (receiveData+2))) * 0.000061 ;
	resultAccel[2] = (*((int16_t*) (receiveData+4))) * 0.000061 ;



}

void offsetCorrection_Sensitivity(){

	totalX = 0;
	totalY = 0;
	totalZ = 0;

	for (int i=0; i < 1000; i++){

		Accel_read();
		totalX+= resultAccel[0];
		totalY+= resultAccel[1];
		totalZ+= resultAccel[2];
	}

	totalX/= 1000;
	//if ((totalX & 0x8000) ^ (x1 & 0x8000)) totalX = -totalX;
	totalY/= 1000;
	//if ((totalY & 0x8000) ^ (yt & 0x8000)) totalY = -totalY;
	totalZ = totalZ/1000;

	sens = totalX * totalX + totalY * totalY + totalZ * totalZ;

	sens = sqrt(sens);

	//if ((totalZ & 0x8000) ^ (z1 & 0x8000)) totalZ = -totalZ;



}

void findAngles(float x, float y, float z, float *alpha, float *beta, float *gamma, float *roll, float *pitch){

	int32_t normSquared;
	float norm;
	normSquared = x*x+y*y+z*z;

	norm = sqrt(normSquared);

	*alpha = x/norm;

	*beta = y/norm;

	*gamma = z/norm;


	*alpha = acos(*alpha) * 180 / M_PI;
	*beta = acos(*beta) * 180 / M_PI;
	*gamma = acos(*gamma) * 180 / M_PI;

	*roll = atan2(y,z) * 180 / M_PI;
	*pitch = atan2(-x, sqrt((y * y) + (z * z ))) * 180 / M_PI;


}

void matrixRotation(float roll, float pitch, float *M, float *MD){

	M[0] = cos(pitch * M_PI / 180);
	M[1] = 0;
	M[2] = -sin(pitch * M_PI / 180);
	M[3] = sin(pitch * M_PI / 180) * sin(roll * M_PI / 180);
	M[4] = cos(roll * M_PI / 180);
	M[5] = sin(roll * M_PI / 180) * cos (pitch * M_PI / 180);
	M[6] = cos(roll * M_PI / 180) * sin(pitch * M_PI / 180);
	M[7] = - sin(roll * M_PI / 180);
	M[8] = cos(pitch * M_PI / 180) * cos(roll * M_PI / 180);

	MD[0] = M[0];
	MD[1] = M[3];
	MD[2] = M[6];
	MD[3] = M[1];
	MD[4] = M[4];
	MD[5] = M[7];
	MD[6] = M[2];
	MD[7] = M[5];
	MD[8] = M[8];




}

void rotateVector(float x1, float yt, float z1, float *x, float *y, float *z, float *M ){


	*x = M[0] * x1 + M[1] * yt + M[2] * z1;
	*y = M[3] * x1 + M[4] * yt + M[5] * z1;
	*z = M[6] * x1 + M[7] * yt + M[8] * z1;



}

void average(float* x1, float* yt, float* z1){

	double sumX=0, sumY=0, sumZ=0;



	for (int i = 0; i < 12; i++){


		Accel_read();
		sumX+= resultAccel[0];
		sumY+= resultAccel[1];
		sumZ+= resultAccel[2];
		HAL_Delay(50);


	}

	*x1 = sumX/12;
	*yt = sumY/12;
	*z1 = sumZ/12;


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
