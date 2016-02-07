#include "stm32l1xx_hal.h"
#include "stm32l152c_discovery_glass_lcd.h"
#include "tsl.h"
#include "tsl_user.h"
#include "init.h"


void EXTI0_IRQHandler(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void itoa(char* a, int i, int n);
void display_time(void);

volatile int sec = 0;

enum STATE {RESET_STATE, RIGHT, RIGHT_OVERLAP, LEFT, LEFT_OVERLAP, BOTH_OFF, BOTH_ON};
volatile enum STATE state = RESET_STATE;
volatile enum STATE prev_state = RESET_STATE;


int main(void)
{
    InitAll();
    InitRelayPins();

    BSP_LED_On(LED_GREEN);
    BSP_LED_On(LED_BLUE);

    BSP_LCD_GLASS_DisplayString((uint8_t*)"READY");

    // RIGHT:       00 - 27 min: blue led on, relay 1 on
    // RIGHT_OVERLAP: 28 - 29 min: same + green flashing
    // LEFT:        30 - 57 min:  green led on, relay 2 on
    // LEFT_OVERLAP:  58 - 59 min:  same + blue flashing
    // BOTH_OFF:    both relays off, both LEDs off, counters not active
    // BOTH_ON:     both relays on, both LEDs on, counters not active
    
    // USER press: BOTH_OFF
    // left button: jump to state LEFT
    // right button: jump to state RIGHT
    // left + right button: jump to state BOTH_ON
 
    const int TIME_OVERLAP = 2 * 60;
    const int TIME_RIGHT = 30 * 60 - TIME_OVERLAP;
    const int TIME_LEFT = 15 * 60 - TIME_OVERLAP;
        
    while (1)
    {
      switch(state) {
        case RESET_STATE:
          green_off();
          blue_off();
          relay_1_off();
          relay_2_off();
          relay_3_off();
          relay_4_off();
          set_sec(0);
          
          state = RIGHT;
          break;
        
        case RIGHT:
          if (prev_state != RIGHT) {
            prev_state = RIGHT;
            blue_on();
            green_off();
            relay_1_off();
            relay_2_on();
            set_sec(TIME_RIGHT);
          }
          if (!sec) {
            state = RIGHT_OVERLAP;
          }
          break;

        case RIGHT_OVERLAP:
          if (prev_state != RIGHT_OVERLAP) {
            prev_state = RIGHT_OVERLAP; 
            blue_on();
            relay_1_on();
            relay_2_on();
            set_sec(TIME_OVERLAP);
          }
          if (sec & 1) {
            green_on();
          }
          else {
            green_off();            
          }
          if (!sec) {
            state = LEFT;
          }
          break;

        case LEFT:
          if (prev_state != LEFT) {
            prev_state = LEFT;
            green_on();
            blue_off();
            relay_1_on();
            relay_2_off();
            set_sec(TIME_LEFT);
          }
          if (!sec) {
            state = LEFT_OVERLAP;
          }
          break;

        case LEFT_OVERLAP:
          if (prev_state != LEFT_OVERLAP) {
            prev_state = LEFT_OVERLAP;
            green_on();
            relay_1_on();
            relay_2_on();
            set_sec(TIME_OVERLAP);
          }
          if (sec & 1) {
            blue_on();
          }
          else {
            blue_off();            
          }
          if (!sec) {
            state = RIGHT;
          }
          break;

        case BOTH_OFF:
          if (prev_state != BOTH_OFF) {
            green_off();
            blue_off();
            relay_1_off();
            relay_2_off();            
            set_sec(0);
          }
          break;

        case BOTH_ON:
          if (prev_state != BOTH_ON) {
            green_on();
            blue_on();
            relay_1_on();
            relay_2_on();
            set_sec(0);
          }
          break;
      }

      // Read touch buttons
      int buttonId = getTouchButtons();

      // LEFT button
      if (buttonId == 0) {
        prev_state = RESET_STATE;
        state = LEFT;
      }

      // RIGHT button
      if (buttonId == 3) {
        prev_state = RESET_STATE;
        state = RIGHT;
      }
    }

  // Will never get here.
  return 0;
}

// User button handler.
void EXTI0_IRQHandler(void)
{
    BSP_LED_Toggle(LED_GREEN);
    HAL_GPIO_EXTI_IRQHandler(B1_Pin);
}

// One second timebase handler.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    BSP_LED_Toggle(LED_BLUE);
}

// Convert {i} to an {n} digit ASCII string stored in {a}.
void itoa(char* a, int i, int n) {
    int j;
    for (j = n - 1; j >= 0; --j) {
        if (i) {
            a[j] = (i % 10) + '0';
            i /= 10;
        }
        else {
            a[j] = '0';
        }
    }
}


static inline void InitRelayPins(void)
{
  // Relay 1, PA5
  *(volatile uint32_t*)GPIOA_MODER |= (1 << (5 * 2));
  // Relay 2, PA11
  *(volatile uint32_t*)GPIOA_MODER |= (1 << (11 * 2));
  // Relay 3, PA12
  *(volatile uint32_t*)GPIOA_MODER |= (1 << (12 * 2));
  // Relay 3, PC12
  *(volatile uint32_t*)GPIOC_MODER |= (1 << (12 * 2));
}



// User button
void EXTI0_IRQHandler(void)
{
  prev_state = RESET_STATE;
  if (state == BOTH_ON) {
    state = BOTH_OFF;
  }
  else {
    state = BOTH_ON;
  }
  EXTI_ClearITPendingBit(EXTI_Line0);
}


void display_time(void) {
    int s = sec;
    if (s < 0) {
        s = -s;
    }

    int h = s / (60 * 60);
    int m = s / 60 % 60;
    s = s % 60;
    
    char v[7];
    itoa(v, h, 2);
    itoa(v + 2, m, 2);
    itoa(v + 4, s, 2);
    v[6] = '\0';

    LCD_GLASS_DisplayString((uint8_t*)v);
}

void set_sec(int s) {
  sec = s;
  display_time();
}


  if (sec) {
    --sec;
  }

  display_time();


void relay_1_on(void) {
    GPIO_LOW(GPIOA, GPIO_Pin_5);
}

void relay_1_off(void) {
    GPIO_HIGH(GPIOA, GPIO_Pin_5);
}

void relay_2_on(void) {
    GPIO_LOW(GPIOC, GPIO_Pin_12);
}

void relay_2_off(void) {
    GPIO_HIGH(GPIOC, GPIO_Pin_12);
}

void relay_3_on(void) {
    GPIO_LOW(GPIOA, GPIO_Pin_12);
}

void relay_3_off(void) {
    GPIO_HIGH(GPIOA, GPIO_Pin_12);
}

void relay_4_on(void) {
    GPIO_LOW(GPIOA, GPIO_Pin_11);
}

void relay_4_off(void) {
    GPIO_HIGH(GPIOA, GPIO_Pin_11);
}

// Check if HAL_Delay() works. If it doesn't, try setting it up.
//#define delay()						\
//do {							\
//  register unsigned int i;				\
//  for (i = 0; i < 300000; ++i)				\
//    __asm__ __volatile__ ("nop\n\t":::"memory");	\
//} while (0)

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}

#endif
