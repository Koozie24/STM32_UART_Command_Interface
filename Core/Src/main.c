/*To run on ACM1 use picocom in terminal: 

  picocom -b 115200 /dev/ttyACM1

*/
#include "main.h"

// struct to store a pins number and port char
typedef struct {
  uint8_t pin_num;
  char port_char;
}Pin_Port_Combo; 

// enum for naming convention to int mapping of led patterns
typedef enum {
  NONE,
  SNAKE,
  REVERSE_SNAKE,
  ALTERNATE,
  ONE_BY_ONE,
  REVERSE_ONE_BY_ONE,
  POLICE
} Pattern;

typedef struct {
  Pattern current_p;
  uint8_t active_flag;
}LED_STATE;

// define HSI speed of 16MHz
#define CPU_CLOCK_HZ   (16000000U)
#define PCLK1_HZ       (16000000U)       
#define BAUD_RATE      (115200U)
#define SYSTICK_HZ     (1000U)

  Pin_Port_Combo leds[5] = {
    {10U, 'A'}, //  yellow
    {8U, 'A'}, //   red
    {9U, 'A'}, //   green
    {5U, 'B'}, //   white
    {4U, 'B'} //    blue
};

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

/* enable the clock registers of USART2 and GPIOA, if not enabled already*/
void enable_clocks(void){
  // enable clock for usart2
  if(!(RCC->APB1ENR & RCC_APB1ENR_USART2EN)) RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  // enable clock for GPIOA (USART TX -> PA2 |||| RX->PA3)
  if(!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN)) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  // enable clock for GPIOB 
  if(!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOBEN)) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
}

/* Configure GPIO pins for USART2 TX
Clears bits at MODER2 and then set to ob10 alternate config mode */
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

/*configure the gpio pins for USART2 RX*/
void configure_usart2_rx_pin(void){ //PA3
  GPIOA->MODER &= ~(3U << 6U); //clear bits
  GPIOA->MODER |= (2U << 6U); //set to alternate function

  GPIOA->AFR[0] &= ~(15U << 12U);
  GPIOA->AFR[0] |= (7U << 12U); //set af7 field

  GPIOA->OTYPER &= ~(1U << 3U); //set push pull for pin 3

  GPIOA->PUPDR &= ~(3U << 6U); // set no pull for rx

  GPIOA->OSPEEDR &= ~(3U << 6U);
  GPIOA->OSPEEDR |= (1U << 6U); // set medium speed
}

void enable_usart2(void){
  /*
  PA2/PA3 for USART2 TX/RX respective
  https://vivonomicon.com/2020/06/28/bare-metal-stm32-programming-part-10-uart-communication/

  pclk1 = 16,000,000
  baud = 115200

  one bit takes 1/115200 seconds
  meaning we have 16000000/115200 = 138.888.. clock cycles per bit or each USART bit takes 138.88 plck1 clock cycles

  usart divier = mantissa (whole num) + (fraction/16) 
  */
  
  USART2->CR1 &= ~(1U << 13U); //clear UE bits

  uint16_t uartdiv = ((PCLK1_HZ) + (BAUD_RATE / 2U)) / BAUD_RATE;   // set divider - number of clock tickets usart get per one bit time

  USART2->BRR = (((uartdiv / 16U) << 4U) | ((uartdiv % 16U) << 0U));    // configure baud rate reg

  USART2->CR1 &= ~(1U << 12U);  // reset word bits
  USART2->CR1 &= ~(1U << 15U);  // reset oversampling bits
  USART2->CR2 &= ~(3U << 12U);  // reset stop bits to one stop bit
  USART2->CR1 &= ~(1U << 10U);  // disable parity 
  USART2->CR1 &= ~(1U << 2U);   // reset rx
  USART2->CR1 &= ~(1U << 3U);   // reset tx


  USART2->CR1 |= (1U << 2U);    // reciever enabled (1) at bit 2
  USART2->CR1 |= (1U << 3U);    // Transmit enabled (1) at bit 

  USART2->CR1 |= (1U << 13U);   // UE 1 is enable and it is bit 13
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

/*function loops through leds struct and turns resets bits on all gpio led pins*/
void reset_all_leds(void){
  int num_leds = sizeof(leds) / sizeof(leds[0]);

  for(int i=0; i < num_leds; i++){
    if(leds[i].port_char == 'A') GPIOA->ODR &= ~(1U << leds[i].pin_num);
    if(leds[i].port_char == 'B') GPIOB->ODR &= ~(1U << leds[i].pin_num);
  }
}

/* function goes through leds' one by one turning them on then off, using the HSI for sysclock*/
void snake_leds(uint8_t active){
  // declare persistant variables 
  static uint32_t step = 0;
  static int i = 0;

  if(!active){
    step = number_ticks;
    i = 0;
    return;
  }

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

void reverse_snake_leds(uint8_t active){
  static uint32_t step = 0;
  static int i = 4;

  if(!active){
    step = number_ticks;
    i = 4;
    return;
  }

  if((number_ticks - step) < 1000) return;
  step = number_ticks;

  if(i == 4) {
    drive_pin(leds[i]);
    i--;
  }
  else if(i < 4 && i > -1){
    drive_pin(leds[i+1]);
    drive_pin(leds[i]);
    i--;
  }
  else if(i < 0){
   drive_pin(leds[0]);
    i = 4;
  }
}

void alternate_leds(uint8_t active){
  static uint32_t step = 0;
  static int front = 0;
  static int back = 4;
  static int alternate = 0;

  if(!active){
    step = number_ticks;
    front = 0;
    back = 4;
    alternate = 0;
    return;
  }

  if((number_ticks - step) < 1000) return;
  step = number_ticks;


  if(front <= back){
    if(!alternate){
      drive_pin(leds[front]);
      front++;
    }
    else{
      drive_pin(leds[back]);
      back--;
    }

    alternate = !alternate;
  }
  else{
    for(int i=0; i < 5; i++){
      drive_pin(leds[i]);
    }
    front = 0;
    back = 4;
    alternate = 0;
  }
}

void one_by_one_leds(uint8_t active){
  static uint32_t step = 0;
  static int i = 0;

  if(!active){
    step = number_ticks;
    i = 0;
    return;
  }

  if((number_ticks - step) < 1000) return;
  number_ticks = step;

  if(i < 5){
    drive_pin(leds[i++]);
  }
  else{
    for(int j=0; j < 5; j++){
      drive_pin(leds[j]);
      i = 0;
    }
  }
}

void reverse_one_by_one_leds(uint8_t active){
  static uint32_t step = 0;
  static int i = 4;

   if(!active){
    step = number_ticks;
    i = 4;
    return;
  }

  if((number_ticks - step) < 1000) return;
  number_ticks = step;

  if(i >= 0){
    drive_pin(leds[i--]);
  }
  else{
    for(int j=4; j >= 0; j--){
      drive_pin(leds[j]);
      i = 4;
    }
  }
} 

void police_leds(uint8_t active){
  static uint32_t step = 0;
  int red = 1;
  int blue = 4;
  static int alternate = 0;

  if(!active){
    step = number_ticks;
    alternate = 0;
    return;
  }
  if((number_ticks - step) < 200) return;
  number_ticks = step;
 
  if(alternate == 0) {
    drive_pin(leds[red]);
    alternate++;
  }
  else {
    drive_pin(leds[blue]);
    alternate--;
  }
}

/*function sends a charcter over tx by polling transmit data register, when ready writes 8 bits data to data register*/
void usart2_putc(char c){
  while(!(USART2->SR & (1U << 7U))) {};
  USART2->DR = c; 
}

/*function recieves a character over rx by polling read if data register data is ready to be read (1) and then reads the bits from data register*/
int usart2_getch(char *rx_c){
  if(USART2->SR & (1U << 5U)){
    *rx_c = USART2->DR;
    return 1;
  }
  else return 0;
}

void send_char_array_to_putc(const char text[]){
  int iter = 0;
  char c = text[iter];
  while(c != '\0'){
    c = text[iter];

    usart2_putc(c);
    iter++;
  }
}
/*function prints some instructions to terminal screen using putc*/
void instructions(void){
  const char start_instructs[] =
  "Hello! Input a command to control the LED's:\r\n"
  "1: Snake\r\n"
  "2: Reverse Snake\r\n"
  "3: Alternate LEDs\r\n"
  "4: 1-by-1\r\n"
  "5: Reverse 1-by-1\r\n"
  "6: Police\r\n"
  "7: Stop Current\r\n\0";
  
  send_char_array_to_putc(start_instructs);
}

/*function reads in terminal clear screen character sequence on usart2_putc*/
void clear_screen(void){
  const char clr_seq[] = "\x1B[2J\x1B[H";
  send_char_array_to_putc(clr_seq);
}

void display_last_cmd(char cmd){
  const char text[] = "You entered command: ";

  send_char_array_to_putc(text);


  usart2_putc(cmd);
  usart2_putc('\r');
  usart2_putc('\n');
}

void cmd_stop_without_pattern(void){
  clear_screen();
  const char text[] = "You entered the command: 7 to stop an LED pattern. No pattern is currently running. \r\nYou must start a new LED pattern (1-6).\r\n\r\n\r\n";

  send_char_array_to_putc(text);

  instructions();
}

void cmd_new_pattern_without_stopping_old(char cmd){
  clear_screen();
  display_last_cmd(cmd);
  const char text[] = "\r\nYou tried to start a new pattern without stopping the one already running. \r\nYou must stop the current LED pattern (7).\r\n\r\n\r\n";

  send_char_array_to_putc(text);

  instructions();
}

void some_other_invalid_cmd_input(char cmd){
  clear_screen();
  display_last_cmd(cmd);
  const char text[] = "\r\nYour input was not valid. If there is an LED pattern currently running, you may stop it (7) and start a new pattern (1-6). \r\n"
  " If there is not an LED pattern currently running, input a number that corresponds to the pattenr that you want to see (1-6).\r\n\r\n\r\n";

  send_char_array_to_putc(text);

  instructions();
}

void instructions_after_stop_leds(void){
  const char text[] = "You have stopped the LED Pattern. Please pick a new pattern to run.\r\n\r\n\r\n";

  send_char_array_to_putc(text);

  instructions();
}

void display_led_pattern_name(uint8_t pattern){
  const char *text = "";
  switch (pattern){
    case 1:
      text = "You started LED Pattern: Snake\r\n";
      break;
    case 2:
      text = "You started LED Pattern: Reverse Snake\r\n";
      break;
    case 3:
      text = "You started LED Pattern: Alternate\r\n";
      break;
    case 4:
      text= "You started LED Pattern: 1-by-1\r\n";
      break;
    case 5:
      text = "You started LED Pattern: Reverse 1-by-1\r\n";
      break;
    case 6:
      text = "You started LED Pattern: Police\r\n";
      break;
    default: 
      break;
  }
  send_char_array_to_putc(text);
}

void input_command_handler(char cmd, LED_STATE *led_pattern){
  int cmd_as_int = (int)(cmd - '0');

  if(!led_pattern->active_flag && cmd_as_int < 7 && cmd_as_int > 0){ //command recieved for an led pattern and no patterns active currently
    switch(cmd_as_int){
      case 1:
        led_pattern->active_flag = 1;
        led_pattern->current_p = SNAKE;
        break;
      case 2:
        led_pattern->active_flag = 1;
        led_pattern->current_p = REVERSE_SNAKE;
        break;
      case 3:
        led_pattern->active_flag = 1;
        led_pattern->current_p = ALTERNATE;
        break;
      case 4:
        led_pattern->active_flag = 1;
        led_pattern->current_p = ONE_BY_ONE;
        break;
      case 5:
        led_pattern->active_flag = 1;
        led_pattern->current_p = REVERSE_ONE_BY_ONE;
        break;
      case 6:
        led_pattern->active_flag = 1;
        led_pattern->current_p = POLICE;
        break;
      default:
        break;
    }
    uint8_t current_pattern = led_pattern->current_p; //get snapshot to of pattern to pass into function safer maybe?
    display_last_cmd(cmd);
    display_led_pattern_name(current_pattern);
  }
  else if(led_pattern->active_flag && cmd_as_int == 7){
    led_pattern->active_flag = 0;
    led_pattern->current_p = NONE;
    reset_all_leds();
    instructions_after_stop_leds();
  }
  else{
    if(cmd_as_int == 7 && led_pattern->current_p == NONE) cmd_stop_without_pattern(); //display entered stop while nothing is running and inform them options
    else if(cmd_as_int > 0 && cmd_as_int < 7 && led_pattern->active_flag == 1) cmd_new_pattern_without_stopping_old(cmd); // display entered new pattern without stopping pattern already running
    else some_other_invalid_cmd_input(cmd);// some unknown issue and echo command
  }
  
}

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
  configure_usart2_rx_pin();
  configure_gpioa_led_pins();
  configure_gpiob_led_pins();
  enable_usart2();
  //Pattern LED_PATTERNS;

  instructions();
  LED_STATE state_tracker = {.current_p=NONE, .active_flag=0};
  while (1)
  {

    if(state_tracker.active_flag){ // if active flag true, call led function again
      switch(state_tracker.current_p){
        case(SNAKE):
          snake_leds(state_tracker.active_flag);
          break;
        case(REVERSE_SNAKE):
          reverse_snake_leds(state_tracker.active_flag);
          break;
        case(ALTERNATE):
          alternate_leds(state_tracker.active_flag);
          break;
        case(ONE_BY_ONE):
          one_by_one_leds(state_tracker.active_flag);
          break;
        case(REVERSE_ONE_BY_ONE):
          reverse_one_by_one_leds(state_tracker.active_flag);
          break;
        case(POLICE):
          police_leds(state_tracker.active_flag);
          break;
        case(NONE):
          break;
      }
    }

    char recieved;
    if(usart2_getch(&recieved)){ //poll for a command
      clear_screen();
      input_command_handler(recieved, &state_tracker);
    }

  }

}

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