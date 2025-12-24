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

// struct to store a pins number and port char
typedef struct {
  uint8_t pin_num;
  char port_char;
}Pin_Port_Combo; 

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// define HSI speed of 16MHz
#define CPU_CLOCK_HZ   (16000000U)
#define BAUD_RATE (115200U)
#define SYSTICK_HZ (1000U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  Pin_Port_Combo leds[5] = {
    {10U, 'A'}, //  yellow
    {8U, 'A'}, //   red
    {9U, 'A'}, //   green
    {5U, 'B'}, //   white
    {4U, 'B'} //    blue
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void init_systick(void){
  // calculate num cycles per tick
  SysTick->LOAD = (CPU_CLOCK_HZ / SYSTICK_HZ) - 1;
  // reset the coutner value
  SysTick->VAL = 0;
  // set source to HSI 
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

// declare tick counter to 0 32 bit integer can store something like 49 days worth of ticks before reset to 0
volatile uint32_t number_ticks = 0;

/* handler function to increment number of ticks*/
//void SysTick_Handler(void){
  //number_ticks++;
//}

/* function to wait some number of 32 bit integer cpu cycles*/
static void delay(volatile uint32_t n){
  for(volatile int i=0; i < n; i++){}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* enable the clock registers of USART2 and GPIOA, if not enabled already*/
void enable_clocks(void){
  // enable clock for usart2
  if(!(RCC->APB1ENR & RCC_APB1ENR_USART2EN)) RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  // enable clock for GPIOA (USART TX -> PA2 |||| RX->PA3)
  if(!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN)) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  // enable clock for GPIOB 
  if(!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOBEN)) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
}

/* Clears bits at MODER2 and then set to ob10 alternate config mode */
void configure_usart2_tx_pin(void){
  // PA2 for TX || PA3 for RX
  // register &= ~(mask << (pin * bits per pin)) clear field
  // register |= (value << (pin * bits per pin)) set field

  // need to clear bit 4/5 3U == 0b11 so we shift 0b11 four bits left and get 0b110000 - where our pin bits are 4/5
  // clear bits
  GPIOA->MODER &= ~(3U << 4U);
  // set to alternate function
  GPIOA->MODER |= (2U << 4U);

  // usart2 alternate function is AF7 and uses AFRL 0-7 AFRH 8-15 4 bits per pin 0b1111 = 15
  GPIOA->AFR[0] &= ~(15U << 8U);
  // set AF7 field
  GPIOA->AFR[0] |= (7U << 8U);

  // configure output type otyper to push pull for tx pin 2
  GPIOA->OTYPER &= ~(1U << 2U);

  // pupdr - what voltage should the pin sit when not used - set no pull for tx
  GPIOA->PUPDR &= ~(3U << 4U);

  // ospeedr - output speed how fast are signals - medium speed to match baud
  GPIOA->OSPEEDR &= ~(3U << 4U);
  // set to medium speed 0b01 == 1
  GPIOA->OSPEEDR |= (1U << 4U);
}

void configure_gpioa_led_pins(void){

  // setup pa5 as output (onboard led)
  // clear bit 11:10
  // GPIOA->MODER &= ~(3U << (5U * 2U));
  // set to 01 as in refrence sheet
  // GPIOA->MODER |= (1U << (5U * 2U));

  // setup PA10 as output
  // clear bits 21:20
  GPIOA->MODER &= ~(3U << (10U * 2U));
  // set to 01
  GPIOA->MODER |= (1U << (10U * 2U));

  // setup PA8 as output
  // clear bits 17:16
  GPIOA->MODER &= ~(3U << (8U * 2U));
  // set to 01 mode
  GPIOA->MODER |= (1U << (8U * 2U));

  // set PA9 as output
  //clear bits 19:18
  GPIOA->MODER &= ~(3U << (9U * 2U));
  //set to 01 mode
  GPIOA->MODER |= (1U << (9U * 2U));
}

void configure_gpiob_led_pins(void){
  // clear bits 9:8
  GPIOB->MODER &= ~(3U << (4U * 2U));
  // set mode 01
  GPIOB->MODER |= (1U << (4U * 2U));

  // clear bits 11:10
  GPIOB->MODER &= ~(3U << (5U * 2U));
  // set mode
  GPIOB->MODER |= (1U << (5U * 2U));
}

void drive_pin(Pin_Port_Combo x){
  if(x.port_char == 'A') GPIOA->ODR ^= (1U << x.pin_num);
  if(x.port_char == 'B') GPIOB->ODR ^= (1U << x.pin_num);
}

/* function goes through leds' one by one turning them on then off, using the HSI for sysclock*/
void snake_leds(void){
  // declare persistant variables 
  static uint32_t step = 0;
  static int i = 0;

  //if not one second passed
  if((number_ticks - step) < 1000) return;
  step = number_ticks;

  //led logic
  if(i == 0) drive_pin(leds[0]);
  else{
    drive_pin(leds[i-1]);
    drive_pin(leds[i]);
  }
  i++;


  if(i > 5) { // reset i
    i = 0;
  }
}


void reverse_snake_leds(void){
  for(int i=4; i>=0; i--){
      if(i == 4) drive_pin(leds[i]); // toggle on first led
      else{
        drive_pin(leds[i+1]); // toggle off last led
        drive_pin(leds[i]); // toggle on current led
      }
      delay(1000000);
    }

    drive_pin(leds[0]); // turn off last led
}


void alternate_leds(void){
  int front = 0;
  int back = 4;
  int alternate = 0; // flips each time if even left, if odd, right


  while(front <= back){
    if(!alternate){ //if 0 do front of array
      drive_pin(leds[front]);
      front++;
    }
    else{//if not zero go to back of array
      drive_pin(leds[back]);
      back--;
    }

    alternate = !alternate; //flip alternate
    delay(1000000);
  }

  //turn off leds
  for(int i = 0; i < 5; i++){
    drive_pin(leds[i]);
  }
}

void one_by_one(void){

  for(int i=0; i < 5; i++){
    drive_pin(leds[i]);
    delay(1000000);
  }
  for(int i=0; i < 5; i++){
    drive_pin(leds[i]);
  }
  delay(1000000);
}

void reverse_one_by_one(void){
  for(int i=4; i >= 0; i--){
    drive_pin(leds[i]);
    delay(1000000);
  }
  for(int i=4; i >= 0; i--){
    drive_pin(leds[i]);
  }
  delay(1000000);
} 

void police(void){
  int red = 1;
  int blue = 4;

  drive_pin(leds[red]);
  delay(500000);
  drive_pin(leds[blue]);
  delay(500000);
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
  init_systick();
  configure_usart2_tx_pin();
  configure_gpioa_led_pins();
  configure_gpiob_led_pins();

  //uint32_t stepper = number_ticks;

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

   snake_leds();
    
    //snake_leds();
    //delay(1000000);
    /*
    reverse_snake_leds();
    delay(1000000);
    alternate_leds();
    delay(1000000);
    one_by_one();
    reverse_one_by_one();
    */
    //police();
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

