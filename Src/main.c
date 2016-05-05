/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "usb_device.h"
#include <string.h>

/* USER CODE BEGIN Includes */
#include "mxconstants.h"
#include "usbd_cdc_if.h"
#define TEXT "Hi CDC FS"

#define  FREE_LEN(X) (APP_TX_DATA_SIZE-strlen((char*)X))
/* USER CODE END Includes */
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t uwPrescalerValue = 0;
uint32_t TimPeriod = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void CPU_CACHE_Enable(void);
static void printStr(char*,char*);
static void printU32(char*,uint32_t);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void prepareData(uint8_t* data,uint16_t len);

#define DLIM 64
TIM_HandleTypeDef htim5;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//void prepareData(uint8_t* data,uint16_t len)
//{
//	uint16_t i =0;
//	for (i =0 ; i <len ; i++)
//	{
//		data[i] += 1;
//		data[i] %= 10;
//		data[i] += 48;
//	}
//}
/* USER CODE END 0 */
// #define DATA_LEN 1024
#define DATA_LEN 3968
#define MAX_TIM_SEC 10
#define CLK_FACTOR 1 // 1 : 60MHz, 2 : 120 MHz, 4 : 216 MHz
#define TIM_CLK_FREQ 1000000

int main(void)
{

  /* USER CODE BEGIN 1 */
  USBD_CDC_HandleTypeDef *hcdc;
  uint16_t count=0;
  uint32_t wait =0,start,stop,diff;
  uint8_t dataLength = 0;
  uint8_t *dataPointer = NULL;
  /* USER CODE END 1 */
  CPU_CACHE_Enable();

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
//  MX_TIM5_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
//
//  uint8_t data[DATA_LEN]={"In consequence of an agreement between the sisters, Elizabeth wrote the next morning to their mother, to beg that the carriage might be sent for them in the course of the day.  But Mrs. Bennet, who had calculated on her daughters remaining at Netherfield till the following Tuesday, which would exactly finish Jane's week, could not bring herself to receive them with pleasure before.  Her answer, therefore, was not propitious, at least not to Elizabeth's wishes, for she was impatient to get home.  Mrs. Bennet sent them word that they could not possibly have the carriage before Tuesday; and in her postscript it was added, that if Mr. Bingley and his sister pressed them to stay longer, she could spare them very well.  Against staying longer, however, Elizabeth was positively resolved--nor did she much expect it would be asked; and fearful, on the contrary, as being considered as intruding themselves needlessly long, she urged Jane to borrow Mr. Bingley's carriage immediately, and at length it was settled that their original design of leaving Netherfield that morning should be mentioned, and the request made.The communication excited many professions of concern; and enough was said of wishing them to stay at least till the following day to work on Jane; and till the morrow their going was deferred.  Miss Bingley was then sorry that she had proposed the delay, for her jealousy and dislike of one sister much exceeded her affection for the other.The master of the house heard with real sorrow that they were to go so soon, and repeatedly tried to persuade Miss Bennet that it would not be safe for her--that she was not enough recovered; but Jane was firm where she felt herself to be right.To Mr. Darcy it was welcome intelligence--Elizabeth had been at Netherfield long enough.  She attracted him more than he liked--and Miss Bingley was uncivil to _her_, and more teasing than usual to himself.  He wisely resolved to be particularly careful that no sign of admiration should _now_ escape him, nothing that could elevate her with the hope of influencing his felicity; sensible that if such an idea had been suggested, his behaviour during the last day must have material weight in confirming or crushing it.  Steady to his purpose, he scarcely spoke ten words to her through the whole of Saturday, and though they were at one time left by themselves for half-an-hour, he adhered most conscientiously to his book, and would not even look at her.On Sunday, after morning service, the separation, so agreeable to almost all, took place.  Miss Bingley's civility to Elizabeth increased at last very rapidly, as well as her affection for Jane; and when they parted, after assuring the latter of the pleasure it would always give her to see her either at Longbourn or Netherfield, and embracing her most tenderly, she even shook hands with the former.  Elizabeth took leave of the whole party in the liveliest of spirits.They were not welcomed home very cordially by their mother. Mrs. Bennet wondered at their coming, and thought them very wrong to give so much trouble, and was sure Jane would have caught cold again.  But their father, though very laconic in his expressions of pleasure, was really glad to see them; he had felt their importance in the family circle.  The evening conversation, when they were all assembled, had lost much of its animation, and almost all its sense by the absence of Jane and Elizabeth.They found Mary, as usual, deep in the study of thorough-bass and human nature; and had some extracts to admire, and some new observations of threadbare morality to listen to.  Catherine and Lydia had information for them of a different sort.  Much had been done and much had been said in the regiment since the preceding Wednesday; several of the officers had dined lately with their uncle, a private had been flogged, and it had actually been hinted that Colonel Forster was going to be married. This is and example string to test "};
//  //uint8_t data[DATA_LEN] = {"consequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwaconsequence of an agreement between the sisters, Test the firmwa"};
//  uint8_t delimiter[DLIM]={"\n\r--------------------------------------------------------\n\r"};
//  /* USER CODE END 2 */
//
//
//  /* Infinite loop */
//
//  /* USER CODE BEGIN WHILE */
//  uwPrescalerValue = HAL_RCC_GetHCLKFreq();//HAL_RCC_GetSysClockFreq();//
//  htim5.Init.Prescaler =(uint32_t)((uwPrescalerValue/CLK_FACTOR) / TIM_CLK_FREQ) - 1;
//  //htim5.Init.Period = 90000000 - 1; // 9 second time.
//  htim5.Init.Period = (TIM_CLK_FREQ*MAX_TIM_SEC) - 1; // 9 second time.
//  HAL_TIM_Base_Init(&htim5);
//
//  HAL_Delay(5000);
//  //printU32(" Freq ",uwPrescalerValue);
//  htim5.Instance->CNT = 0;
//  HAL_TIM_Base_Start(&htim5);
//  HAL_Delay(40);
//  HAL_TIM_Base_Stop(&htim5);
//  //printStr(" Init Tim","");
//  //printU32(" ", htim5.Instance->CNT);
//  htim5.Instance->CNT = 0;
  while(1)
  {
	  HAL_Delay(300); // Check for every 300 ms.
	  if(GetDataArrived()!=0)
	  {
		  dataLength = GetDataArrived();
		  dataPointer = GetDataPtr();
		  CDC_Transmit_FS((uint8_t*)"\n_", 2);
		  CDC_Transmit_FS(dataPointer, dataLength);
		  CDC_Transmit_FS((uint8_t*)"_\n", 2);
		  SetDataArrived(0);
	  }
  }
//  while (1)
//  {
//  /* USER CODE BEGIN 3 */
//	  if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && (hcdc = hUsbDeviceFS.pClassData) != 0 && hcdc->TxState == 0)
//	  {
//		  //prepareData((uint8_t*)data,DATA_LEN);
//		  if(count == 0)
//		  {
//			  start = HAL_GetTick();
//			  htim5.Instance->CNT = 0;
//			  HAL_TIM_Base_Start(&htim5);
//			  //while ( CDC_Transmit_FS((uint8_t*)(delimiter),DLIM) != USBD_OK );
//		  }
//
//		  while ( CDC_Transmit_FS((uint8_t*)(data+count),64) != USBD_OK )
//		  {
//			  wait++;
//		  }
//		  count += 64;
//		  if(count >= DATA_LEN)
//		  {
//			  HAL_TIM_Base_Stop(&htim5);
//			  wait = 0;
//			  count = 0;
//			  stop = HAL_GetTick();
//			  diff = stop -start;
//			  printU32("\n\rTransfer Speed HAL ",diff);
//			  printU32("Transfer Speed TIM", htim5.Instance->CNT);
//			  htim5.Instance->CNT = 0;
//		  }
//	  }
//  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void) 
{
 //60 MHz
RCC_OscInitTypeDef RCC_OscInitStruct;
RCC_ClkInitTypeDef RCC_ClkInitStruct;
RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

__PWR_CLK_ENABLE();

__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLM = 20;
RCC_OscInitStruct.PLL.PLLN = 192;
RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
RCC_OscInitStruct.PLL.PLLQ = 5;
HAL_RCC_OscConfig(&RCC_OscInitStruct);

RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
						  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_CLK48;
PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

///*	 120 MHz */
//RCC_OscInitTypeDef RCC_OscInitStruct;
//RCC_ClkInitTypeDef RCC_ClkInitStruct;
//RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
//
//__PWR_CLK_ENABLE();
//
//__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
//
//RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
//RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//RCC_OscInitStruct.PLL.PLLM = 20;
//RCC_OscInitStruct.PLL.PLLN = 192;
//RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//RCC_OscInitStruct.PLL.PLLQ = 5;
//HAL_RCC_OscConfig(&RCC_OscInitStruct);
//
//RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//						  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
//
//PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
//PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
//HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
//
//HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
//
//HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

////	// MAX Speed
////
//RCC_OscInitTypeDef RCC_OscInitStruct;
//RCC_ClkInitTypeDef RCC_ClkInitStruct;
//RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
//
//__PWR_CLK_ENABLE();
//
//__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
//RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//RCC_OscInitStruct.PLL.PLLM = 25;
//RCC_OscInitStruct.PLL.PLLN = 432;
//RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//RCC_OscInitStruct.PLL.PLLQ = 9;
//HAL_RCC_OscConfig(&RCC_OscInitStruct);
//
//HAL_PWREx_ActivateOverDrive();
//
//RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//						  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
//RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
//HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
//
//PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
//PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
//HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
//
//HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
//
//HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

//	//	216 CLK Speed
//RCC_OscInitTypeDef RCC_OscInitStruct;
//RCC_ClkInitTypeDef RCC_ClkInitStruct;
//RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
//
//__PWR_CLK_ENABLE();
//
//__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
//RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//RCC_OscInitStruct.PLL.PLLM = 25;
//RCC_OscInitStruct.PLL.PLLN = 432;
//RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//RCC_OscInitStruct.PLL.PLLQ = 9;
//HAL_RCC_OscConfig(&RCC_OscInitStruct);
//
//HAL_PWREx_ActivateOverDrive();
//
//RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//						  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
//RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
//HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
//
//PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
//PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
//HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
//
//HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
//
//HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

//	/* 200 MHz*/
//	  RCC_OscInitTypeDef RCC_OscInitStruct;
//	  RCC_ClkInitTypeDef RCC_ClkInitStruct;
//	  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
//
//	  __PWR_CLK_ENABLE();
//
//	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//	  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
//	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//	  RCC_OscInitStruct.PLL.PLLM = 25;
//	  RCC_OscInitStruct.PLL.PLLN = 400;
//	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//	  RCC_OscInitStruct.PLL.PLLQ = 9;
//	  HAL_RCC_OscConfig(&RCC_OscInitStruct);
//
//	  HAL_PWREx_ActivateOverDrive();
//
//	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
//	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
//	  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
//
//	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
//	  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
//	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
//
//	  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
//
//	  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x007075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLE);

}
/* TIM5 init function */
void MX_TIM5_Init(void)
{

//  TIM_ClockConfigTypeDef sClockSourceConfig;
//  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim5);

//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);
//
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

}
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  /*__GPIOB_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();*/

	GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	  __GPIOE_CLK_ENABLE();
	  __GPIOG_CLK_ENABLE();
	  __GPIOB_CLK_ENABLE();
	  __GPIOD_CLK_ENABLE();
	  __GPIOC_CLK_ENABLE();
	  __GPIOA_CLK_ENABLE();
	  __GPIOJ_CLK_ENABLE();
	  __GPIOI_CLK_ENABLE();
	  __GPIOK_CLK_ENABLE();
	  __GPIOF_CLK_ENABLE();
	  __GPIOH_CLK_ENABLE();

	  /*Configure GPIO pin : LCD_B0_Pin */
	  GPIO_InitStruct.Pin = LCD_B0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
	  HAL_GPIO_Init(LCD_B0_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
	  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : QSPI_D2_Pin */
	  GPIO_InitStruct.Pin = QSPI_D2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
	  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
	  GPIO_InitStruct.Pin = RMII_TXD1_Pin|RMII_TXD0_Pin|RMII_TX_EN_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  /*Configure GPIO pins : PE1 PE0 FMC_D5_Pin FMC_D6_Pin
	                           FMC_D8_Pin FMC_D11_Pin FMC_D4_Pin FMC_D7_Pin
	                           FMC_D9_Pin FMC_D12_Pin FMC_D10_Pin */
	  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0|FMC_D5_Pin|FMC_D6_Pin
	                          |FMC_D8_Pin|FMC_D11_Pin|FMC_D4_Pin|FMC_D7_Pin
	                          |FMC_D9_Pin|FMC_D12_Pin|FMC_D10_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	  /*Configure GPIO pins : ARDUINO_SCL_D15_Pin ARDUINO_SDA_D14_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_SCL_D15_Pin|ARDUINO_SDA_D14_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin
	                           ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
	  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin
	                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin : ARDUINO_PWM_D3_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_PWM_D3_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	  HAL_GPIO_Init(ARDUINO_PWM_D3_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : SPDIF_RX0_Pin */
	  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
	  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : SDMMC_CK_Pin SDMMC_D3_Pin SDMMC_D2_Pin PC9
	                           PC8 */
	  GPIO_InitStruct.Pin = SDMMC_CK_Pin|SDMMC_D3_Pin|SDMMC_D2_Pin|GPIO_PIN_9
	                          |GPIO_PIN_8;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : ARDUINO_PWM_D9_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_PWM_D9_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	  HAL_GPIO_Init(ARDUINO_PWM_D9_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : PE5 PE6 */
	  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	  /*Configure GPIO pin : VCP_RX_Pin */
	  GPIO_InitStruct.Pin = VCP_RX_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	  HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : QSPI_NCS_Pin */
	  GPIO_InitStruct.Pin = QSPI_NCS_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
	  HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : PG15 PG8 PG1 PG0
	                           FMC_BA1_Pin FMC_BA0_Pin */
	  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_1|GPIO_PIN_0
	                          |FMC_BA1_Pin|FMC_BA0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  /*Configure GPIO pins : LCD_B1_Pin LCD_B2_Pin LCD_B3_Pin LCD_G4_Pin
	                           LCD_G1_Pin LCD_G3_Pin LCD_G0_Pin LCD_G2_Pin
	                           LCD_R7_Pin LCD_R5_Pin LCD_R6_Pin LCD_R4_Pin
	                           LCD_R3_Pin LCD_R1_Pin LCD_R2_Pin */
	  GPIO_InitStruct.Pin = LCD_B1_Pin|LCD_B2_Pin|LCD_B3_Pin|LCD_G4_Pin
	                          |LCD_G1_Pin|LCD_G3_Pin|LCD_G0_Pin|LCD_G2_Pin
	                          |LCD_R7_Pin|LCD_R5_Pin|LCD_R6_Pin|LCD_R4_Pin
	                          |LCD_R3_Pin|LCD_R1_Pin|LCD_R2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
	  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

	  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
	  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : Audio_INT_Pin */
	  GPIO_InitStruct.Pin = Audio_INT_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : FMC_D2_Pin FMC_D3_Pin FMC_D1_Pin FMC_D15_Pin
	                           FMC_D0_Pin FMC_D14_Pin FMC_D13_Pin */
	  GPIO_InitStruct.Pin = FMC_D2_Pin|FMC_D3_Pin|FMC_D1_Pin|FMC_D15_Pin
	                          |FMC_D0_Pin|FMC_D14_Pin|FMC_D13_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pins : SAI2_MCLKA_Pin SAI2_SCKA_Pin SAI2_FSA_Pin SAI2_SDA_Pin */
	  GPIO_InitStruct.Pin = SAI2_MCLKA_Pin|SAI2_SCKA_Pin|SAI2_FSA_Pin|SAI2_SDA_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
	  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	  /*Configure GPIO pins : LCD_DE_Pin LCD_B7_Pin LCD_B6_Pin LCD_B5_Pin
	                           LCD_G6_Pin LCD_G7_Pin LCD_G5_Pin */
	  GPIO_InitStruct.Pin = LCD_DE_Pin|LCD_B7_Pin|LCD_B6_Pin|LCD_B5_Pin
	                          |LCD_G6_Pin|LCD_G7_Pin|LCD_G5_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
	  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

	  /*Configure GPIO pin : LCD_B4_Pin */
	  GPIO_InitStruct.Pin = LCD_B4_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
	  HAL_GPIO_Init(LCD_B4_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : SAI2_SDB_Pin */
	  GPIO_InitStruct.Pin = SAI2_SDB_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
	  HAL_GPIO_Init(SAI2_SDB_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : PD3 */
	  GPIO_InitStruct.Pin = GPIO_PIN_3;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	  /*Configure GPIO pin : uSD_Detect_Pin */
	  GPIO_InitStruct.Pin = uSD_Detect_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : PF0 PF1 PF2 PF3
	                           PF4 PF5 PF12 PF15
	                           PF13 PF14 PF11 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
	                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12|GPIO_PIN_15
	                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_11;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	  /*Configure GPIO pins : LCD_HSYNC_Pin LCD_VSYNC_Pin LCD_R0_Pin LCD_CLK_Pin */
	  GPIO_InitStruct.Pin = LCD_HSYNC_Pin|LCD_VSYNC_Pin|LCD_R0_Pin|LCD_CLK_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
	  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
	  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : PG9 */
	  GPIO_InitStruct.Pin = GPIO_PIN_9;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : SDMMC_D0_Pin */
	  GPIO_InitStruct.Pin = SDMMC_D0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC;
	  HAL_GPIO_Init(SDMMC_D0_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : TP3_Pin NC2_Pin */
	  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	  /*Configure GPIO pin : ARDUINO_SCK_D13_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_SCK_D13_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	  HAL_GPIO_Init(ARDUINO_SCK_D13_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : DCMI_PWR_EN_Pin */
	  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  HAL_GPIO_Init(DCMI_PWR_EN_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : PH14 PH12 PH9 PH11
	                           PH10 */
	  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_12|GPIO_PIN_9|GPIO_PIN_11
	                          |GPIO_PIN_10;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	  /*Configure GPIO pin : ARDUINO_PWM_CS_D10_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_PWM_CS_D10_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	  HAL_GPIO_Init(ARDUINO_PWM_CS_D10_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : VCP_TX_Pin */
	  GPIO_InitStruct.Pin = VCP_TX_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	  HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : ARDUINO_PWM_D5_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_PWM_D5_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	  HAL_GPIO_Init(ARDUINO_PWM_D5_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : LCD_INT_Pin */
	  GPIO_InitStruct.Pin = LCD_INT_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : ARDUINO_RX_D0_Pin ARDUINO_TX_D1_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_RX_D0_Pin|ARDUINO_TX_D1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : ULPI_NXT_Pin */
	  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : FMC_SDNME_Pin PH3 */
	  GPIO_InitStruct.Pin = FMC_SDNME_Pin|GPIO_PIN_3;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  /*Configure GPIO pins : ARDUINO_A4_Pin ARDUINO_A5_Pin ARDUINO_A1_Pin ARDUINO_A2_Pin
	                           ARDUINO_A3_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_A4_Pin|ARDUINO_A5_Pin|ARDUINO_A1_Pin|ARDUINO_A2_Pin
	                          |ARDUINO_A3_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	  /*Configure GPIO pin : PC3 */
	  GPIO_InitStruct.Pin = GPIO_PIN_3;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
	  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
	  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB2 */
	  GPIO_InitStruct.Pin = GPIO_PIN_2;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
	  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pin : RMII_RXER_Pin */
	  GPIO_InitStruct.Pin = RMII_RXER_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
	  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : ARDUINO_A0_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_A0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(ARDUINO_A0_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : PA4 PA6 */
	  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : LCD_SCL_Pin LCD_SDA_Pin */
	  GPIO_InitStruct.Pin = LCD_SCL_Pin|LCD_SDA_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
	  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
	  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : ARDUINO_PWM_D6_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_PWM_D6_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
	  HAL_GPIO_Init(ARDUINO_PWM_D6_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
	  GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);


}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();
  /* Enable D-Cache */
  SCB_EnableDCache();
}
static void printStr(char* str ,char* data)
{
// Wait until USB endpoint available

	strncpy ((char*)UserTxBufferFS,str,FREE_LEN(UserTxBufferFS));
	strncat ((char*)UserTxBufferFS,data,FREE_LEN(UserTxBufferFS));
	strncat ((char*)UserTxBufferFS,"\r\n",FREE_LEN(UserTxBufferFS));
	while ( IsUSBTxReady() != 1);
		CDC_Transmit_FS((uint8_t*)UserTxBufferFS,strlen((char*)UserTxBufferFS));

}
static void printU32(char* str,uint32_t data)
{
// Wait until USB endpoint available

	strncpy ((char*)UserTxBufferFS,str,FREE_LEN(UserTxBufferFS));
	snprintf((char*)(UserTxBufferFS+strlen((char*)UserTxBufferFS)),FREE_LEN(UserTxBufferFS), "%u\r\n", data);
	while ( IsUSBTxReady() != 1);
			CDC_Transmit_FS((uint8_t*)UserTxBufferFS,strlen((char*)UserTxBufferFS));
}
/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
