/**-------------------------------------------------------------------
 \date  26.10.2025
 *
 *   STM32G030C8T6   
 *   ------------ 
 *  |            |
 *  |            |
 *  |            |
 *  |       PD.2 | ---->  LED
 *  |            | 
 *  |            |
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 *
 *
 *\ authors        PivnevNikolay 
 *\ сode debugging ScuratovaAnna + PivnevNikolay 
 */
//**************************  Пример первый **************************
#include "stm32g0xx.h"
#include "stdbool.h"

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

#define GPIO_Pin_0  ((uint16_t)0x0001)
#define GPIO_Pin_1  ((uint16_t)0x0002)
#define GPIO_Pin_2  ((uint16_t)0x0004)//Led PD2
#define GPIO_Pin_3  ((uint16_t)0x0008)
#define GPIO_Pin_4  ((uint16_t)0x0010)
#define GPIO_Pin_5  ((uint16_t)0x0020)
#define GPIO_Pin_6  ((uint16_t)0x0040)
#define GPIO_Pin_7  ((uint16_t)0x0080)
#define GPIO_Pin_8  ((uint16_t)0x0100)
#define GPIO_Pin_9  ((uint16_t)0x0200)
#define GPIO_Pin_10 ((uint16_t)0x0400)
#define GPIO_Pin_11 ((uint16_t)0x0800)
#define GPIO_Pin_12 ((uint16_t)0x1000)
#define GPIO_Pin_13 ((uint16_t)0x2000)
#define GPIO_Pin_14 ((uint16_t)0x4000)
#define GPIO_Pin_15 ((uint16_t)0x8000)

#define Input_mode                  (0x0UL)
#define General_purpose_output_mode (0x1UL)
#define Alternate_function_mode     (0x2UL)
#define Analog_mode                 (0x3UL)

#define Output_push_pull            (0x0UL)
#define Output_open_drain           (0x1UL)

#define Low_speed                   (0x0UL)
#define Medium_speed                (0x1UL)
#define High_speed                  (0x2UL)
#define Very_high_speed             (0x3UL)

#define No_pull_up_No_pull_down     (0x0UL)
#define Pull_up                     (0x1UL)
#define Pull_down                   (0x2UL)

// Структура для представления светодиода
typedef struct {
  GPIO_TypeDef *port;   // Порт GPIO
  u16 pin;              // Номер пина
  bool active_state;    // Логика управления (0 - активный низкий, 1 - активный высокий)
} LED_HandleTypeDef;

// Конфигурация вывода для светодиода на отладочной плате STM32G030C8T6 (PD2)
LED_HandleTypeDef STM32G030_Led = {
    .port = GPIOD,      // Port
    .pin = GPIO_Pin_2,  // Pin
    .active_state = 1   // Active high level
};

void LED_On(LED_HandleTypeDef *led);
void LED_Off(LED_HandleTypeDef *led);
void LED_Toggle(LED_HandleTypeDef *led);
void delay_ms(u32 ms);

void PORTC_13_INIT(void) {
  SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIODEN);                                                        // Запуск тактирования порта C !!!
  MODIFY_REG(GPIOD->MODER, GPIO_MODER_MODE2, General_purpose_output_mode << GPIO_MODER_MODE2_Pos); // Настройка GPIOD пин 2 на выход (output mode)
  MODIFY_REG(GPIOD->OTYPER, GPIO_OTYPER_OT2, Output_push_pull << GPIO_OTYPER_OT2_Pos);             // Настройка GPIOD пин 2 в режим Push-Pull
  MODIFY_REG(GPIOD->OSPEEDR, GPIO_OSPEEDR_OSPEED2, High_speed << GPIO_OSPEEDR_OSPEED2_Pos);        // Настройка GPIOD пин 2 в режим High_speed
  MODIFY_REG(GPIOD->PUPDR, GPIO_PUPDR_PUPD2, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD13_Pos);    // Настройка GPIOD пин 2 в режим High_speed
}

int main(void) {
  PORTC_13_INIT();
  LED_On(&STM32G030_Led);
  delay_ms(500);
  LED_Off(&STM32G030_Led);
  delay_ms(500);
  LED_On(&STM32G030_Led);
  delay_ms(250);
  LED_Off(&STM32G030_Led);
  delay_ms(250);
  LED_On(&STM32G030_Led);
  delay_ms(100);
  LED_Off(&STM32G030_Led);
  delay_ms(100);
  LED_On(&STM32G030_Led);
  delay_ms(50);
  LED_Off(&STM32G030_Led);
  delay_ms(50);
  while (1) {
    LED_Toggle(&STM32G030_Led);// Переключение состояния светодиода
    delay_ms(25);// Задержка
  }
}

// Включение светодиода
void LED_On(LED_HandleTypeDef *led) {
  if (led->active_state) {
    led->port->BSRR = led->pin; // Установить пин (SET)
  } else {
    led->port->BSRR = (u32)led->pin << 16; // Сбросить пин (RESET)
  }
}

// Выключение светодиода
void LED_Off(LED_HandleTypeDef *led) {
  if (led->active_state) {
    led->port->BSRR = (u32)led->pin << 16; // Сбросить пин (RESET)
  } else {
    led->port->BSRR = led->pin; // Установить пин (SET)
  }
}

// Переключение состояния светодиода
void LED_Toggle(LED_HandleTypeDef *led) {
  led->port->ODR ^= led->pin; // Инвертировать состояние через регистр ODR
}

// Простая функция задержки
void delay_ms(u32 ms) {
  for (__IO u32 q = 0; q < ms * 2000; q++) {
    __NOP();
  }
}
/*************************** End of file ****************************/