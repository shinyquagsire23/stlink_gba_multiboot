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
#include "usb_device.h"
#include "usbd_cdc_if.h"

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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
uint32_t GBA_Send32_Tries(uint32_t data, uint32_t tries)
{
    uint32_t waited = 0;
    uint32_t out = 0;
    // Wait for SI LO
    while (tries != 0x1234ABCD && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) != GPIO_PIN_SET)
    {
      waited++;
      if (tries && waited > tries)
        return 0xFFFFFFFF;
      //HAL_Delay(1);
      DelayLil();
    }

    //HAL_Delay(1);

    DelayLil();
    DelayLil();
    DelayLil();
    DelayLil();


    for (int i = 0; i < 32; i++)
    {
      //HAL_Delay(1);
      DelayLil();

      // CLK HI -> LO
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

      // Shift out our data to SO
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (data & 0x80000000 ? GPIO_PIN_SET : GPIO_PIN_RESET));
      data = data << 1;

      DelayLil();
      //HAL_Delay(1);

      // Read in data from SI
      out = out << 1;
      out |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET ? 1 : 0);

      // GBA reads our data on this edge, LO -> HI
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    }

    // Set SO HI
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

    return out;
}

uint32_t GBA_Send32(uint32_t data)
{
  return GBA_Send32_Tries(data, 100);
}

uint32_t GBA_Send8_Tries(uint8_t data, uint32_t tries)
{
    uint32_t waited = 0;
    uint32_t out = 0;
    // Wait for SI LO
    while (tries != 0x1234ABCD && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) != GPIO_PIN_SET)
    {
      waited++;
      if (tries && waited > tries)
        return 0xFFFFFFFF;
      //HAL_Delay(1);
      DelayLil();
    }

    //HAL_Delay(1);

    DelayLil();
    DelayLil();
    DelayLil();
    DelayLil();


    for (int i = 0; i < 8; i++)
    {
      //HAL_Delay(1);
      DelayLil();

      // CLK HI -> LO
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

      // Shift out our data to SO
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (data & 0x80000000 ? GPIO_PIN_SET : GPIO_PIN_RESET));
      data = data << 1;

      DelayLil();
      //HAL_Delay(1);

      // Read in data from SI
      out = out << 1;
      out |= (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET ? 1 : 0);

      // GBA reads our data on this edge, LO -> HI
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    }

    // Set SO HI
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

    return out;
}

uint32_t GBA_Send8(uint8_t data)
{
  return GBA_Send8_Tries(data, 100);
}

void DelayLil()
{
	for (int i = 0; i < 1; i++)
	{
		__asm__("nop");
	}
}

void LED(int s)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, s ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

#if 0
extern const char GBbuffer[129488];
extern const int GBbuffer_length;

void SendMultiboot(){
    unsigned int read_from_gba = 0;
    uint32_t fp = 0;
    int i;
    uint32_t w;
    uint32_t r;
    int bit;
    uint32_t w2;
    uint32_t c;
    uint32_t f;
    uint32_t fcnt = 0;
    uint32_t h;
    uint32_t m;
    uint32_t mbfsize = GBbuffer_length - 0xC0;



    while(read_from_gba!=0x72026202)
        {

                    read_from_gba = GBA_Send32(0x6202);
                    DelayLil();


        }
    LED(1);


            read_from_gba = GBA_Send32(0x00006202);
            read_from_gba = GBA_Send32(0x00006102);

                fp=0;
                for(i=0; i<32; i++)
                {
                    w = GBbuffer[fp];
                    w = GBbuffer[fp+1] << 8 | w;
                	//w = 0x12345678; // TODO
                    fcnt += 2;
                    fp += 2;
                    r = GBA_Send32(w);
                }


            //LED(0);
                //fp=0;
                for(i=0; i<32; i++)
                {
                    w = GBbuffer[fp];
                    w = GBbuffer[fp+1] << 8 | w;
                	//w = 0x12345678;
                    fcnt += 2;
                    fp += 2;
                    r = GBA_Send32(w);
                }


                mbfsize = (mbfsize+0x0f)&0xfffffff0;    //align length to xfer to 16
                //fp=0;
                for(i=0; i<32; i++)
                {
                    w = GBbuffer[fp];
                    w = GBbuffer[fp+1] << 8 | w;
                	//w = 0x12345678;
                    fcnt += 2;
                    fp += 2;
                    r = GBA_Send32(w);
                }



    r = GBA_Send32(0x00006200);
    r = GBA_Send32(0x00006202);
    r = GBA_Send32(0x000063d1);
    r = GBA_Send32(0x000063d1);
 m = ((r & 0x00ff0000) >>  8) + 0xffff00d1;
     h = ((r & 0x00ff0000) >> 16) + 0xf;
    r = GBA_Send32((((r >> 16) + 0xf) & 0xff) | 0x00006400);
    r = GBA_Send32((mbfsize - 0x190) / 4);
     f = (((r & 0x00ff0000) >> 8) + h) | 0xffff0000;
     c = 0x0000c387;


     int led_state = 0;

        fp=0xC0;
    while(fcnt < mbfsize)
    {
    	LED(led_state);
    	if (fp % 0x1000 == 0) led_state = !led_state;
        //if (fp==64)return;
        w = GBbuffer[fp];
        w = GBbuffer[fp+1] <<  8 | w;
        w = GBbuffer[fp+2] << 16 | w;
        w = GBbuffer[fp+3] << 24 | w;
        //w = 0x12345678;
        fp += 4;
        w2 = w;
        for(bit=0; bit<32; bit++)
        {
            if((c ^ w) & 0x01)
            {
                c = (c >> 1) ^ 0x0000c37b;
            }
            else
            {
                c = c >> 1;
            }
            w = w >> 1;
        }
        m = (0x6f646573 * m) + 1;
        GBA_Send32(w2 ^ ((~(0x02000000 + fcnt)) + 1) ^m ^0x43202f2f);
        fcnt = fcnt + 4;
    }

    LED(1);


    for(bit=0; bit<32; bit++)
    {
        if((c ^ f) & 0x01)
        {
            c =( c >> 1) ^ 0x0000c37b;
        }
        else
        {
            c = c >> 1;
        }

        f = f >> 1;
    }

        while(read_from_gba!=0x00750065)
        {
                    read_from_gba = GBA_Send32(0x00000065);
                DelayLil();
        }
    r = GBA_Send32(0x00000066);
                DelayLil();
    r = GBA_Send32(c);
                DelayLil();
                //ResetRequest=1;

}
#endif

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
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Clock high = not ready
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  //SendMultiboot();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	// A5 = B13 = SO
	// B6 = SD
	// B14 = SC
	// B8 = SI
	// LED ON
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	  //LED(1);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8));
	//HAL_Delay(100);

	//GBA_Send32(0x12345678);
	//const char* test_send = "asdf asdf\r\n";
	//CDC_Transmit_FS(test_send, strlen(test_send));

	// LED OFF
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	//LED(0);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8));
	//HAL_Delay(100);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9
                          |GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 PA9
                           PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB13 PB14
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);*/

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

