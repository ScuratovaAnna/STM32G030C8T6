/**-------------------------------------------------------------------
 \date  26.10.2025
 *
 *   STM32G030C8T6
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |        PD.2| ---->  LED
 *  |            |
 *  |        PB.6| <----  Button
 *  |            |
 *  |       +3.3V|
 *  |         GND|
 *
 * В качестве кнопки использую  touch button ttp223
 *
 *
 *\ authors        ScuratovaAnna
 *\ сode debugging ScuratovaAnna
 */
//********************************************************************
#include "stm32g0xx.h"
#include "stdbool.h"

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

volatile bool condition = false;

#define GPIO_Pin_0  ((uint16_t)0x0001)
#define GPIO_Pin_1  ((uint16_t)0x0002)
#define GPIO_Pin_2  ((uint16_t)0x0004)//Led PD2
#define GPIO_Pin_3  ((uint16_t)0x0008)
#define GPIO_Pin_4  ((uint16_t)0x0010)
#define GPIO_Pin_5  ((uint16_t)0x0020)
#define GPIO_Pin_6  ((uint16_t)0x0040)//Button PB6
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



void PORTD_2_INIT(void) {
  SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIODEN);                                                        // Запуск тактирования порта D!!!
  MODIFY_REG(GPIOD->MODER, GPIO_MODER_MODE2, General_purpose_output_mode << GPIO_MODER_MODE2_Pos); // Настройка GPIOD пин 2 на выход (output mode)
  MODIFY_REG(GPIOD->OTYPER, GPIO_OTYPER_OT2, Output_push_pull << GPIO_OTYPER_OT2_Pos);             // Настройка GPIOD пин 2 в режим Push-Pull
  MODIFY_REG(GPIOD->OSPEEDR, GPIO_OSPEEDR_OSPEED2, High_speed << GPIO_OSPEEDR_OSPEED2_Pos);        // Настройка GPIOD пин 2 в режим High_speed
  MODIFY_REG(GPIOD->PUPDR, GPIO_PUPDR_PUPD2, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD2_Pos);    // Настройка GPIOD пин 2 в режим High_speed
}

void PORTB_6_INIT_Button(void) {
  SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);                                                      // Запуск тактирования порта B
  MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE6, Input_mode << GPIO_MODER_MODE6_Pos);                // Настройка GPIOB пин 6 на выход (Input_mode)
  MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD6, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD6_Pos);   // Настройка GPIOB пин 6 в режим No_pull_up_No_pull_down
}

int main(void) {
  PORTD_2_INIT();
  PORTB_6_INIT_Button();
  while (1) {
    condition = READ_BIT(GPIOB->IDR, GPIO_IDR_ID6);
    (condition == false) ? (SET_BIT(GPIOD->BSRR, GPIO_BSRR_BS2)) : (SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR2));
    //(condition == true) ? (SET_BIT(GPIOD->BSRR, GPIO_BSRR_BS2)) : (SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR2));
    //--------------------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------
    //((GPIOB->IDR & 0x1Ul)==0x1UL)? (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS6)) : (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR6));//PB.0
    //--------------------------------------------------------------------
    //Если в проекте был настроен на вход вывод PB.1, то
    //((GPIOB->IDR & 0x2Ul)==0x1UL)? (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS6)) : (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR6));
    //--------------------------------------------------------------------
    //Если в проекте был настроен на вход вывод PB.2, то
    //((GPIOB->IDR & 0x4Ul)==0x1UL)? (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS6)) : (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR6));
    //--------------------------------------------------------------------
    //Если в проекте был настроен на вход вывод PB.3, то
    //((GPIOB->IDR & 0x8Ul)==0x1UL)? (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS6)) : (SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR6));
    //--------------------------------------------------------------------
    //#define GPIO_Pin_0  ((uint16_t)0x0001)//Px.0(PA.0; PB.0; PC.0 ... Px.0)
    //#define GPIO_Pin_1  ((uint16_t)0x0002)//Px.1
    //#define GPIO_Pin_2  ((uint16_t)0x0004)//Px.2
    //#define GPIO_Pin_3  ((uint16_t)0x0008)//Px.3 и т.д.
    //#define GPIO_Pin_4  ((uint16_t)0x0010)
    //#define GPIO_Pin_5  ((uint16_t)0x0020)
    //#define GPIO_Pin_6  ((uint16_t)0x0040)
    //#define GPIO_Pin_7  ((uint16_t)0x0080)
    //#define GPIO_Pin_8  ((uint16_t)0x0100)
    //#define GPIO_Pin_9  ((uint16_t)0x0200)
    //#define GPIO_Pin_10 ((uint16_t)0x0400)
    //#define GPIO_Pin_11 ((uint16_t)0x0800)
    //#define GPIO_Pin_12 ((uint16_t)0x1000)
    //#define GPIO_Pin_13 ((uint16_t)0x2000)
    //#define GPIO_Pin_14 ((uint16_t)0x4000)
    //#define GPIO_Pin_15 ((uint16_t)0x8000)//Px.15
    //согласно выше приведённому можно использовать макросы, для того чтобы код было удобно читать и сам код был более понятным:
    //((GPIOB->IDR & 0x40UL)==0x0)? (SET_BIT(GPIOD->BSRR, GPIO_BSRR_BS2)) : (SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR2));
    //((GPIOB->IDR & GPIO_Pin_6)==false)? (SET_BIT(GPIOD->BSRR, GPIO_BSRR_BS2)) : (SET_BIT(GPIOD->BSRR, GPIO_BSRR_BR2));
  }
}
/*************************** End of file ****************************/