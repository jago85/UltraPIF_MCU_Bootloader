
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef *hspi_flash;
unsigned int CRCTable[ 256 ];
uint32_t FlashBuffer[1024];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define APP_START_ADDR 0x08004000
#define APP_FLASH_SIZE (112 * 1024)

#define CRC32_POLYNOMIAL     0x04C11DB7

// CRC32_MAGIC is the result of the CRC calculation of a block with the valid
// CRC at the end of the data
#define CRC32_MAGIC          0x2144DF1C

static uint32_t Reflect(uint32_t ref, char ch )
{
   char i;
   uint32_t value = 0;
   /* Swap bit 0 for bit 7.
    * bit 1 for bit 6, etc.
    */
   for (i = 1; i < (ch + 1); i++)
   {
      if(ref & 1)
         value |= 1 << (ch - i);
      ref >>= 1;
   }
   return value;
}

void CRC_BuildTable(void)
{
   int i, j;
   uint32_t crc;

   for (i = 0; i < 256; i++)
   {
      crc = Reflect( i, 8 ) << 24;
      for (j = 0; j < 8; j++)
         crc = (crc << 1) ^ (crc & (1 << 31) ? CRC32_POLYNOMIAL : 0);
      CRCTable[i] = Reflect( crc, 32 );
   }
}

uint32_t CRC_Calculate(void *buffer, uint32_t count)
{
   uint32_t crc = 0xffffffff;
   uint8_t *p = (uint8_t*) buffer;
   while (count--)
      crc = (crc >> 8) ^ CRCTable[(crc & 0xFF) ^ *p++];
   return ~crc;
}

void CRC_Initialize(uint32_t *crc)
{
    *crc = 0xffffffff;
}

void CRC_Finalize(uint32_t *crc)
{
    *crc = ~(*crc);
}

void CRC_CalculateCont(uint32_t *crc, void *buffer, uint32_t count)
{
   uint8_t *p = (uint8_t*) buffer;
   uint32_t crcTemp = *crc;
   while (count--)
       crcTemp = (crcTemp >> 8) ^ CRCTable[(crcTemp & 0xFF) ^ *p++];
   *crc = crcTemp;
}

typedef void (*VoidFunc)(void);

void StartApp(void)
{
    uint32_t stackAddress = *(__IO uint32_t*) APP_START_ADDR;
    VoidFunc jump_app = *(__IO VoidFunc*)(APP_START_ADDR + 4);

    __disable_irq();

    // Disable all peripheral interrupts
    HAL_NVIC_DisableIRQ(SysTick_IRQn);
    HAL_NVIC_DisableIRQ(USART2_IRQn);

    HAL_NVIC_DisableIRQ(WWDG_IRQn);
    HAL_NVIC_DisableIRQ(RTC_IRQn);
    HAL_NVIC_DisableIRQ(FLASH_IRQn);
    HAL_NVIC_DisableIRQ(RCC_IRQn);
    HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
    HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
    HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel4_5_IRQn);
    HAL_NVIC_DisableIRQ(ADC1_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
    HAL_NVIC_DisableIRQ(TIM6_IRQn);
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
    HAL_NVIC_DisableIRQ(TIM14_IRQn);
    HAL_NVIC_DisableIRQ(TIM15_IRQn);
    HAL_NVIC_DisableIRQ(TIM16_IRQn);
    HAL_NVIC_DisableIRQ(TIM17_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_IRQn);
    HAL_NVIC_DisableIRQ(I2C2_IRQn);
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    HAL_NVIC_DisableIRQ(USART3_6_IRQn);

    __set_MSP(stackAddress);
    jump_app();
}

void ReadJedecId(void)
{
    uint8_t cmd[1];
    uint8_t res[3];

    cmd[0] = 0x9f;

    memset(res, 0, sizeof(res));

    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, cmd, 1, 1000);
    HAL_SPI_Receive(&hspi1, res, 3, 1000);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

#define CS_ENABLED GPIO_PIN_RESET
#define CS_DISABLED GPIO_PIN_SET

void FLASH_SetCs(GPIO_PinState pinState)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, pinState);
}

void FLASH_Read(uint32_t addr, uint16_t size, uint8_t *dst)
{
    uint8_t cmd[4];

    cmd[0] = 0x03;
    cmd[1] = (addr >> 16);
    cmd[2] = (addr >> 8);
    cmd[3] = (addr >> 0);

    FLASH_SetCs(CS_ENABLED);

    HAL_SPI_Transmit_DMA(hspi_flash, cmd, sizeof(cmd));
    while (hspi_flash->State != HAL_SPI_STATE_READY) ;

    HAL_SPI_Receive_DMA(hspi_flash, dst, size);
    while (hspi_flash->State != HAL_SPI_STATE_READY) ;

    FLASH_SetCs(CS_DISABLED);
}

bool CheckFirmwareUpdate(void)
{
    uint32_t crc;

    CRC_Initialize(&crc);
    for (uint32_t spiAddr = 0; spiAddr < APP_FLASH_SIZE; spiAddr += sizeof(FlashBuffer))
    {
        FLASH_Read(spiAddr, sizeof(FlashBuffer), (uint8_t *)FlashBuffer);
        CRC_CalculateCont(&crc, FlashBuffer, sizeof(FlashBuffer));
    }
    CRC_Finalize(&crc);

    return (crc == CRC32_MAGIC);
}

void UpdateFirmware(void)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInit;
    uint32_t PageError;

    EraseInit.PageAddress = APP_START_ADDR;
    EraseInit.NbPages = APP_FLASH_SIZE / 2048;
    EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    HAL_FLASHEx_Erase(&EraseInit, &PageError);

    for (uint32_t spiAddr = 0; spiAddr < APP_FLASH_SIZE; spiAddr += sizeof(FlashBuffer))
    {
        FLASH_Read(spiAddr, sizeof(FlashBuffer), (uint8_t *)FlashBuffer);
        for (int i = 0; i < sizeof(FlashBuffer)/sizeof(uint32_t); i++)
        {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APP_START_ADDR + spiAddr + (i << 2), FlashBuffer[i]);
        }
    }
    HAL_FLASH_Lock();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
    CRC_BuildTable();
    uint32_t crc = CRC_Calculate((void *)APP_START_ADDR, APP_FLASH_SIZE);

    //HAL_Delay(250);

    // Start the application if the CRC is valid
    if (crc == CRC32_MAGIC)
    {
        StartApp();
    }

    hspi_flash = &hspi1;

    // check if there is an valid image in the firmware update buffer
    if (CheckFirmwareUpdate())
    {
        UpdateFirmware();
        StartApp();
    }
    else
    {
        // no valid firmware update found
        // maybe this is flashed by the debugger
        if ((0xFFFFFFFF != *(__IO uint32_t*)APP_START_ADDR)
                && (0x00000000 != *(__IO uint32_t*)APP_START_ADDR))
        {
            // TODO: Should we update the CRC here?
            StartApp();
        }
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, UFM_SN_Pin|FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : UFM_SN_Pin FLASH_CS_Pin */
  GPIO_InitStruct.Pin = UFM_SN_Pin|FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FPGA_CS_Pin */
  GPIO_InitStruct.Pin = FPGA_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FPGA_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
