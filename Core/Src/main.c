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
#include "dmamux.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "SPM_Modbus.h"
#include "mcp23017.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//デバイス定義
int16_t model_no = 50;
int16_t version = 1;
int16_t sub_version = 0;

//シリアルデータ
uint8_t recv_data;
uint8_t rx_buf[256];
uint8_t tx_buf[256];
int rx_len=0;

//CANデータ
uint32_t can_id;
uint32_t can_dlc;
uint8_t can_data[8];

//アプリケーション変数
int16_t dev_id=1;
int16_t io_mode=0; //0通常動作　1:セーフティ動作
//通常動作時の入出力
int16_t io_in=0;
int16_t io_out=0;
//セーフティモード時の入出力
int16_t io_in_fillter = 0xff;	//入力のフィルター ビットが1のもののみセーフティが有効
int16_t io_norm_in=0x00;		//正常時の入力状態。これと異なる場合に異常と判定
int16_t io_norm_out=0x00;		//正常時の出力
int16_t io_anom_out=0xff;		//異常時の出力
int16_t io_state=0;				//0正常　1:異常
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_DMAMUX_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //IOexpander intialize
  MCP23017_HandleTypeDef hmcp;
  mcp23017_init(&hmcp, &hi2c1, MCP23017_ADDRESS_20);
  mcp23017_iodir(&hmcp, MCP23017_PORTA, MCP23017_IODIR_ALL_INPUT);
  mcp23017_iodir(&hmcp, MCP23017_PORTB, MCP23017_IODIR_ALL_OUTPUT);
  mcp23017_ggpu(&hmcp, MCP23017_PORTA, MCP23017_GPPU_ALL_ENABLED); // PORTA内蔵プルアップ(約100kΩ)有効

  //各レジスタ初期化
  SPM_ModbusInit();

  //レジスタ登録
  p_input_regs[0] = &model_no;   //モデルナンバー
  p_input_regs[1] = &version; //バージョン
  p_input_regs[2] = &sub_version; //サブバージョン
  p_input_regs[3] = &io_in; //サブバージョン
  p_input_regs[4] = &io_out; //サブバージョン
  p_input_regs[5] = &io_mode;
  p_input_regs[6] = &io_in_fillter;
  p_input_regs[7] = &io_norm_in;
  p_input_regs[8] = &io_norm_out;
  p_input_regs[9] = &io_anom_out;
  p_input_regs[10] = &io_state;

  p_holding_regs[0] = &dev_id;
  p_holding_regs[1] = &io_in;
  p_holding_regs[2] = &io_out;
  p_holding_regs[3] = &io_mode;
  p_holding_regs[4] = &io_in_fillter;
  p_holding_regs[5] = &io_norm_in;
  p_holding_regs[6] = &io_norm_out;
  p_holding_regs[7] = &io_anom_out;
  p_holding_regs[8] = &io_state;

  //Cyclic Function登録
  cycfunc0.rx_len = 2;
  cycfunc0.rx_adr[0] = &io_in;
  cycfunc0.rx_adr[1] = &io_out;
  cycfunc0.tx_len = 1;
  cycfunc0.tx_adr[0] = &io_out;

  ParamLoad();
  io_out=0;
  io_state=0;
  SPM_ModbusSetAddress(dev_id);

  HAL_UART_Receive_IT(&huart2, &recv_data, 1);

  HAL_TIM_Base_Start_IT(&htim14); // Start TIM14 for periodic interrupts
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // LED ON

  //MCP23017のテスト
  // uint8_t portB_val = 0;
  // while(1)
  // {
  //     // Read input from MCP23017 port A
  //     mcp23017_read_gpio(&hmcp, MCP23017_PORTA);
  //     uint8_t portA_val = hmcp.gpio[MCP23017_PORTA];
  //     //シリアル送信
  //     int16_t l = sprintf((char*)tx_buf, "Port A Value: 0x%02X\r\n", portA_val);
  //     HAL_UART_Transmit(&huart2, tx_buf, l, HAL_MAX_DELAY);
  //     //portBの出力をトグル
  //     portB_val = ~portB_val;
  //     hmcp.gpio[MCP23017_PORTB] = portB_val;	//書き込み
  //     mcp23017_write_gpio(&hmcp, MCP23017_PORTB);	//出力の書き込み
  //     HAL_Delay(500); // 500ms delay
  // }
  

  //FDCAN1 filter configuration
  // FDCAN_FilterTypeDef sFilterConfig;
  // sFilterConfig.IdType = FDCAN_STANDARD_ID;
  // sFilterConfig.FilterIndex = 0;
  // sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  // sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  // sFilterConfig.FilterID1 = 0x123; // Standard ID is configured as-is (no manual bit shift)
  // sFilterConfig.FilterID2 = 0x7FF; // Filter mask (accept all IDs)
  // if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  // {
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Filter Configuration Error\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  //   // Filter configuration Error
  //   Error_Handler();
  // }else{
    
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Filter Configuration Success\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  // }

  // // Start FDCAN module
  // if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  // {
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Start Error\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  //   // Start Error
  //   Error_Handler();
  // }else{
  //   TxDataLength = sprintf((char*)TxData, "FDCAN Start Success\r\n");
  //   HAL_UART_Transmit(&huart1, TxData, TxDataLength, HAL_MAX_DELAY);
  // }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//IO動作
	  	mcp23017_read_gpio(&hmcp, MCP23017_PORTA);	//入力を読み取り
  	    io_in = hmcp.gpio[MCP23017_PORTA];	//入力を取得
	    if(io_mode == 0){
	    	//通常動作時
	    	hmcp.gpio[MCP23017_PORTB] = (uint8_t)io_out;	//書き込み
	    }else{
	    	//セーフティモード時
	    	uint8_t io_in_fix = io_in & (uint8_t)io_in_fillter;	//入力のフィルタ
	    	if(io_in_fix != io_norm_in) io_state = 1;	//入力を判定
	    	if( io_state == 0){
	    		//正常時の出力
	    		io_out = io_norm_out;
	    	}else{
	    		//異常時の出力
	    		io_out = io_anom_out;
	    	}
	    	hmcp.gpio[MCP23017_PORTB] = (uint8_t)io_out;	//書き込み
	    }
	    mcp23017_write_gpio(&hmcp, MCP23017_PORTB);	//出力の書き込み

		  if(rx_len>0){
			  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
			  HAL_Delay(10);
			  uint8_t tx_buf[50];
			  int tx_len = SPM_ModbusTask(rx_buf, rx_len, tx_buf);
			  HAL_UART_Transmit(&huart2, tx_buf, tx_len, 100);
			  rx_len=0;
			  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		  }

		/*if(timer_1s==0){
			CAN_TxHeaderTypeDef TxHeader;
			uint32_t TxMailbox;
			uint8_t TxData[8];
			if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
				  TxHeader.StdId = 0x555;                 // CAN ID
				  TxHeader.RTR = CAN_RTR_DATA;            //
				  TxHeader.IDE = CAN_ID_STD;              //
				  TxHeader.DLC = 8;                       //
				  TxHeader.TransmitGlobalTime = DISABLE;  //
				  TxData[0] = 'H';
				  TxData[1] = 'E';
				  TxData[2] = 'L';
				  TxData[3] = 'L';
				  TxData[4] = 'O';
				  TxData[5] = '!';
				  TxData[6] = '!';
				  TxData[7] = '!';
				  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
			}

		    int8_t buf[50];
		    int16_t l = sprintf(buf,"PA=%d, PB=%d\r\n",
		    				hmcp.gpio[MCP23017_PORTA],
							hmcp.gpio[MCP23017_PORTB]);
		    HAL_UART_Transmit( &huart1, buf, l, 0xFFFF);
		}*/
		HAL_Delay(1);
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
}

volatile int timer_1s = 0; // タイマーカウンタ
volatile int timer_10ms = 0; // タイマーカウンタ
volatile int led_state[4] = {0,0,0,0}; // タイマーカウンタ
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM14)
    {
        timer_1s++;
        if(timer_1s>=1000)
        {
            timer_1s=0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle LED to indicate 1 second elapsed
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART2)
   {
	   rx_buf[rx_len] = recv_data;
	   rx_len++;
       HAL_UART_Receive_IT(&huart2, &recv_data, 1);
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
  while(1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4); // Toggle LED to indicate error
    HAL_Delay(100); // 0.1秒待機
  }
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
