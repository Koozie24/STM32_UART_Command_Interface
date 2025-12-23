/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// define HSI speed of 16MHz
#define HSI_SPEED   (16000000)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/*function to wait some number of 32 bit integer cpu cycles*/
static void delay(volatile uint32_t n){
  for(int i=0; i < n; i++){}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*enable the clock registers of USART2 and GPIOA, if not enabled already*/
void enable_clocks(){
  //enable clock for usart2
  if(!(RCC->APB1ENR & RCC_APB1ENR_USART2EN)) RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  //enable clock for GPIOA (USART TX -> PA2 |||| RX->PA3)
  if(!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN)) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
}

/*Clears bits at MODER2 and then set to ob10 alternate config mode */
void configure_usart2_tx_pin(){
  // PA2 for TX || PA3 for RX
  // register &= ~(mask << (pin * bits per pin)) clear field
  // register |= (value << (pin * bits per pin)) set field

  //need to clear bit 4/5 3U == 0b11 so we shift 0b11 four bits left and get 0b110000 - where our pin bits are 4/5
  //clear bits
  GPIOA->MODER &= ~(3U << 4U);
  //set to alternate function
  GPIOA->MODER |= (2U << 4U);

  //usart2 alternate function is AF7 and uses AFRL 0-7 AFRH 8-15 4 bits per pin 0b1111 = 15
  GPIOA->AFR[0] &= ~(15U << 8U);
  //set AF7 field
  GPIOA->AFR[0] |= (7U << 8U);

  //configure output type otyper to push pull for tx pin 2
  GPIOA->OTYPER &= ~(1U << 2U);

  //pupdr - what voltage should the pin sit when not used - set no pull for tx
  GPIOA->PUPDR &= ~(3U << 4U);

  //ospeedr - output speed how fast are signals - medium speed to match baud
  GPIOA->OSPEEDR &= ~(3U << 4U);
  // set to medium speed 0b01 == 1
  GPIOA->OSPEEDR |= (1U << 4U);
}

void configure_gpioa_led_pin(){
  //setup pa5 as output
  //clear bit 11:10
  GPIOA->MODER &= ~(3U << (5U * 2U));
  //set to 01 as in refrence sheet
  GPIOA->MODER |= (1U << (5U * 2U));
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  enable_clocks();
  configure_usart2_tx_pin();
  configure_gpioa_led_pin();
  /* USER CODE END 1 */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  //
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    GPIOA->ODR ^= (1U << 5);
    delay(1000000);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
