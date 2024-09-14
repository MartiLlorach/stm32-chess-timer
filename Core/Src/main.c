/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "st7735.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BLACKS_TURN 0
#define WHITES_TURN 1

#define IN_GAME_MODE_SELECT 0
#define IN_GAME 1
#define IN_GAME_OVER 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

// FLAGS BEGIN
bool debouncing = false; // all buttons share the same debounce flag and timer, so only one button can be pressed at a time

bool whiteBtnPressed = false;
bool blackBtnPressed = false;
bool pauseBtnPressed = false;
bool resetBtnPressed = false;
bool gameModeUpBtnPressed = false;
bool gameModeDownBtnPressed = false;

bool isPaused = false;
// FLAGS END

// GAMEMODES BEGIN
struct gameMode {
	char *name;
	int secondsAvailable;
	int secondsGainedPerTurn;
	int movesUntilNextGameMode;
	int nextGameModeId;
	bool isVisible;
};

struct gameMode gameModes[] = {
		{"Bullet 1", 60, 0, -1, 0, true},
		{"Bullet 1|1", 60, 1, -1, 0, true},
		{"Bullet 2|1", 120, 1, -1, 0, true},
		{"Blitz 3", 180, 0, -1, 0, true},
		{"Blitz 3|2", 180, 2, -1, 0, true},
		{"Blitz 5", 300, 0, -1, 0, true},
		{"Rapid 10", 600, 0, -1, 0, true},
		{"Rapid 15|10", 900, 10, -1, 0, true},
		{"Rapid 30", 1800, 0, -1, 0, true},
		{"classic 45", 2700, 0, -1, 0, true},
		{"classic 60", 3600, 0, -1, 0, true},
		{"classic 90", 5400, 0, -1, 0, true},
};
int gameModesLength = sizeof(gameModes) / sizeof(gameModes[0]);

int currentGamemode = 0;
// GAMEMODES END

// GAME PARAMETERS BEGIN
int gameState = IN_GAME_MODE_SELECT;
int turn = BLACKS_TURN;
uint16_t blacksColor;
uint16_t whitesColor;

int whiteSeconds = 0;
int whiteTimer = 0;
int blackSeconds = 0;
int blackTimer = 0;
int moves = 0;

char gameSelectText[] = "Game select:";

// GAME PARAMETERS END

// SCREEN PARAMETERS BEGIN
struct coordinate {
	int x;
	int y;
};

struct coordinate gameSelectTextCoor = {22, 40};
struct coordinate gameModeTextCoor = {-1, 52};
struct coordinate whiteTimerCoor = {10, 10};
struct coordinate blackTimerCoor = {63, 100};
struct coordinate whiteIndicatorCoor = {107, 10};
struct coordinate blackIndicatorCoor = {10, 100};

int screenWidth = 128;
int screenHeight = 128;
// SCREEN PARAMETERS END

// TIMERS PARAMETERS BEGIN
int gameTimerPrescaler = 10000; //clock at 72Mhz -> timer triggers once per second
int gameTimerPeriod = 7200;
// TIMERS PARAMETERS END

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  __HAL_TIM_SET_PRESCALER(&htim1, 0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  ST7735_Init();

  ST7735_FillScreenFast(ST7735_BLACK);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch (gameState) {
	  	  case IN_GAME_MODE_SELECT:
	  		  GameModeSelectionLoop();
	  		  break;
	  	  case IN_GAME:
	  		  GameLoop();
	  		  break;
	  	  case IN_GAME_OVER:
	  		  GameOverLoop();
	  		  break;
	  }
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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 300;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 10000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7200;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  HAL_GPIO_WritePin(GPIOA, TFT_LED_Pin|TFT_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_RS_Pin|TFT_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TFT_LED_Pin TFT_CS_Pin */
  GPIO_InitStruct.Pin = TFT_LED_Pin|TFT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_RS_Pin TFT_RESET_Pin */
  GPIO_InitStruct.Pin = TFT_RS_Pin|TFT_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ModeDownBtn_Pin ModeUpBtn_Pin ResetBtn_Pin PauseBtn_Pin
                           BlackBtn_Pin WhiteBtn_Pin */
  GPIO_InitStruct.Pin = ModeDownBtn_Pin|ModeUpBtn_Pin|ResetBtn_Pin|PauseBtn_Pin
                          |BlackBtn_Pin|WhiteBtn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (debouncing){
		return;
	}

	switch (GPIO_Pin){
		case WhiteBtn_Pin:
			whiteBtnPressed = true;
			break;
		case BlackBtn_Pin:
			blackBtnPressed = true;
			break;
		case PauseBtn_Pin:
			pauseBtnPressed = true;
			break;
		case ResetBtn_Pin:
			resetBtnPressed = true;
			break;
		case ModeUpBtn_Pin:
			gameModeUpBtnPressed = true;
			break;
		case ModeDownBtn_Pin:
			gameModeDownBtnPressed = true;
			break;
		default:
			return;
	}

	__HAL_TIM_SET_PRESCALER(&htim1, 2000);

	HAL_TIM_Base_Start_IT(&htim2);
	debouncing = true;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3) {
		if (turn == BLACKS_TURN){
			blackSeconds--;
		}
		else if (turn == WHITES_TURN){
			whiteSeconds--;
		}
		return;
	}

	if (htim == &htim2) {
		__HAL_TIM_SET_PRESCALER(&htim1, 0);
		debouncing = false;
		HAL_TIM_Base_Stop_IT(&htim2);
	}
}

void GameModeSelectionLoop(){
	if (whiteBtnPressed){
		whiteBtnPressed = false;
		return;
	}
	if (blackBtnPressed){

		whiteSeconds = gameModes[currentGamemode].secondsAvailable;
		whiteTimer = 0;
		blackSeconds = gameModes[currentGamemode].secondsAvailable;
		blackTimer = 0;
		whitesColor = 0;
		blacksColor = 0;
		gameState = IN_GAME;
		turn = WHITES_TURN;
		isPaused = false;
		ST7735_FillScreenFast(ST7735_BLACK);
		HAL_TIM_Base_Start_IT(&htim3);

		blackBtnPressed = false;
		return;
	}
	if (resetBtnPressed){
		resetBtnPressed = false;
		return;
	}
	if (pauseBtnPressed){
		pauseBtnPressed = false;
		return;
	}
	if (gameModeUpBtnPressed){
		if (currentGamemode == gameModesLength){
			currentGamemode = -1;
		}
		currentGamemode++;
		while(!gameModes[currentGamemode].isVisible){
			currentGamemode++;
			if (currentGamemode > gameModesLength){
				currentGamemode = 0;
			}
		}

		ST7735_FillRectangleFast(0, gameModeTextCoor.y, screenWidth, 8, ST7735_BLACK);
		gameModeTextCoor.x = (screenWidth - strlen(gameModes[currentGamemode].name)*7)/2;
		ST7735_WriteString(gameModeTextCoor.x , gameModeTextCoor.y, gameModes[currentGamemode].name, Font_7x10, ST7735_WHITE, ST7735_BLACK);

		gameModeUpBtnPressed = false;
		return;
	}
	if (gameModeDownBtnPressed){
		if (currentGamemode == 0){
			currentGamemode = gameModesLength+1;
		}
		currentGamemode--;
		while(!gameModes[currentGamemode].isVisible){
			currentGamemode--;
			if (currentGamemode < 0){
				currentGamemode = gameModesLength;
			}
		}

		ST7735_FillRectangleFast(0, gameModeTextCoor.y, screenWidth, 8, ST7735_BLACK);
		gameModeTextCoor.x = (screenWidth - strlen(gameModes[currentGamemode].name)*7)/2;
		ST7735_WriteString(gameModeTextCoor.x , gameModeTextCoor.y, gameModes[currentGamemode].name, Font_7x10, ST7735_WHITE, ST7735_BLACK);
		gameModeDownBtnPressed = false;
		return;
	}

	  ST7735_WriteString(gameSelectTextCoor.x, gameSelectTextCoor.y, gameSelectText, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	  gameModeTextCoor.x = (screenWidth - strlen(gameModes[currentGamemode].name)*7)/2;
	  ST7735_WriteString(gameModeTextCoor.x , gameModeTextCoor.y, gameModes[currentGamemode].name, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	  ST7735_WriteString(blackIndicatorCoor.x , blackIndicatorCoor.y, ">", Font_11x18, ST7735_WHITE, ST7735_BLACK);

	return;
}

void GameLoop(){
	if (whiteBtnPressed){
		if (turn == WHITES_TURN){
			whiteSeconds += gameModes[currentGamemode].secondsGainedPerTurn;
			whiteTimer = __HAL_TIM_GET_COUNTER(&htim3);
			__HAL_TIM_SET_COUNTER(&htim3, blackTimer);
			turn = BLACKS_TURN;
			ST7735_FillRectangleFast(0, 0, screenWidth, screenHeight/2, ST7735_BLACK);
			blacksColor = 0;
		}
		else if (turn == BLACKS_TURN && isPaused){
			isPaused = false;
			__HAL_TIM_SET_COUNTER(&htim3, blackTimer);
			HAL_TIM_Base_Start_IT(&htim3);
		}
		whiteBtnPressed = false;
		return;
	}
	if (blackBtnPressed){
		if (turn == BLACKS_TURN){
			blackSeconds += gameModes[currentGamemode].secondsGainedPerTurn;
			blackTimer = __HAL_TIM_GET_COUNTER(&htim3);
			__HAL_TIM_SET_COUNTER(&htim3, whiteTimer);
			turn = WHITES_TURN;
			ST7735_FillRectangleFast(0, screenHeight/2, screenWidth, screenHeight/2, ST7735_BLACK);
			whitesColor = 0;
		}
		else if (turn == WHITES_TURN && isPaused){
			isPaused = false;
			__HAL_TIM_SET_COUNTER(&htim3, whiteTimer);
			HAL_TIM_Base_Start_IT(&htim3);
		}
		blackBtnPressed = false;
		return;
	}
	if (resetBtnPressed){
		gameState = IN_GAME_MODE_SELECT;
		ST7735_FillScreenFast(ST7735_BLACK);
		resetBtnPressed = false;
		return;
	}
	if (pauseBtnPressed){
		HAL_TIM_Base_Stop_IT(&htim3);
		isPaused = true;
		if (turn == BLACKS_TURN){
			ST7735_WriteString(blackIndicatorCoor.x , blackIndicatorCoor.y, "\"", Font_11x18, ST7735_WHITE, ST7735_BLACK);
		}
		else if (turn == WHITES_TURN){
			ST7735_WriteString(whiteIndicatorCoor.x , whiteIndicatorCoor.y, "\"", Font_11x18, ST7735_WHITE, ST7735_BLACK);
		}

		pauseBtnPressed = false;
		return;
	}
	if (gameModeUpBtnPressed){
		gameModeUpBtnPressed = false;
		return;
	}
	if (gameModeDownBtnPressed){
		gameModeDownBtnPressed = false;
		return;
	}

	int minutes = whiteSeconds / 60;
	int seconds = whiteSeconds % 60;
	char buf[5];
	sprintf(buf, "%02d:%02d", minutes, seconds);
	if (turn == WHITES_TURN) {
		float percentageRemaining = (float)whiteSeconds / (float)gameModes[currentGamemode].secondsAvailable;
		uint16_t color;
		if (percentageRemaining < 0.1){
			color = ST7735_RED;
		} else if (percentageRemaining < 0.2){
			color = ST7735_YELLOW;
		} else{
			color = ST7735_GREEN;
		}
		if (color != whitesColor) {
			ST7735_FillRectangleFast(0, 0, screenWidth, screenHeight/2, color);
			whitesColor = color;
		}
	}
	ST7735_WriteString(whiteTimerCoor.x , whiteTimerCoor.y, buf, Font_11x18, ST7735_WHITE, ST7735_BLACK);
	minutes = blackSeconds / 60;
	seconds = blackSeconds % 60;
	buf[5];
	sprintf(buf, "%02d:%02d", minutes, seconds);
	if (turn == BLACKS_TURN){
		float percentageRemaining = (float)blackSeconds / (float)gameModes[currentGamemode].secondsAvailable;
		uint16_t color;
		if (percentageRemaining < 0.1){
			color = ST7735_RED;
		} else if (percentageRemaining < 0.2){
			color = ST7735_YELLOW;
		} else{
			color = ST7735_GREEN;
		}
		if (color != blacksColor){
			ST7735_FillRectangleFast(0, screenHeight/2, screenWidth, screenHeight/2, color);
			blacksColor = color;
		}
	}
	ST7735_WriteString(blackTimerCoor.x , blackTimerCoor.y, buf, Font_11x18, ST7735_WHITE, ST7735_BLACK);

	if (turn == BLACKS_TURN && !isPaused) {
		ST7735_WriteString(blackIndicatorCoor.x , blackIndicatorCoor.y, ">", Font_11x18, ST7735_WHITE, ST7735_BLACK);
	}
	else if (turn == WHITES_TURN && !isPaused) {
		ST7735_WriteString(whiteIndicatorCoor.x , whiteIndicatorCoor.y, "<", Font_11x18, ST7735_WHITE, ST7735_BLACK);
	}

	if (whiteSeconds <= 0 || blackSeconds <= 0){
		UaUaUaUaUaaaa();
		gameState = IN_GAME_OVER;
		return;
	}
	return;
}

void GameOverLoop(){
	if (whiteBtnPressed){
		whiteBtnPressed = false;
		return;
	}
	if (blackBtnPressed){
		blackBtnPressed = false;
		return;
	}
	if (resetBtnPressed){
		gameState = IN_GAME_MODE_SELECT;
		ST7735_FillScreenFast(ST7735_BLACK);
		resetBtnPressed = false;
		return;
	}
	if (pauseBtnPressed){
		pauseBtnPressed = false;
		return;
	}
	if (gameModeUpBtnPressed){
		gameModeUpBtnPressed = false;
		return;
	}
	if (gameModeDownBtnPressed){
		gameModeDownBtnPressed = false;
		return;
	}

	return;
}

void UaUaUaUaUaaaa(){
	  //ua
	  __HAL_TIM_SET_PRESCALER(&htim1, 500);
	  HAL_Delay(100);
	  __HAL_TIM_SET_PRESCALER(&htim1, 250);
	  HAL_Delay(200);
	  __HAL_TIM_SET_PRESCALER(&htim1, 0);
	  HAL_Delay(10);
	  //ua
	  __HAL_TIM_SET_PRESCALER(&htim1, 500);
	  HAL_Delay(100);
	  __HAL_TIM_SET_PRESCALER(&htim1, 300);
	  HAL_Delay(200);
	  __HAL_TIM_SET_PRESCALER(&htim1, 0);
	  HAL_Delay(10);
	  //ua
	  __HAL_TIM_SET_PRESCALER(&htim1, 500);
	  HAL_Delay(100);
	  __HAL_TIM_SET_PRESCALER(&htim1, 350);
	  HAL_Delay(200);
	  __HAL_TIM_SET_PRESCALER(&htim1, 0);
	  HAL_Delay(10);
	  //uauauauauaaaa
	  int lo = 500;
	  int hi = 360;
	  while (hi < lo){
		  __HAL_TIM_SET_PRESCALER(&htim1, lo);
		  HAL_Delay(50);
		  __HAL_TIM_SET_PRESCALER(&htim1, hi);
		  HAL_Delay(50);
		  hi+=10;
	  }
	  HAL_Delay(500);
	  __HAL_TIM_SET_PRESCALER(&htim1, 0);
	  return;
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
