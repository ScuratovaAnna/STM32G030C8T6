/**-----------------------------------------------------------------------
 \date  10.11.2025
 *
 *
 *  STM32G030C8T6
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |        PD.2| ---->  LED
 *  |            |
 *  |            |
 *  |        PA.8| ----> MCO _|¯|_  MCO  64/32 = 2 MHz (SYSCLK/32)
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 *     PLL 64MHz
 *
 *\ authors        ScuratovaAnna 
 *\ сode debugging ScuratovaAnna
 *
/******************************************************************************/
#include "stm32g0xx.h"

#define PA8_MCO_AF0  (0x0000000U)

#define GPIO_Pin_0  ((uint16_t)0x0001)
#define GPIO_Pin_1  ((uint16_t)0x0002)
#define GPIO_Pin_2  ((uint16_t)0x0004) // Led PD2
#define GPIO_Pin_3  ((uint16_t)0x0008)
#define GPIO_Pin_4  ((uint16_t)0x0010)
#define GPIO_Pin_5  ((uint16_t)0x0020)
#define GPIO_Pin_6  ((uint16_t)0x0040)
#define GPIO_Pin_7  ((uint16_t)0x0080)
#define GPIO_Pin_8  ((uint16_t)0x0100) // MCO PA8
#define GPIO_Pin_9  ((uint16_t)0x0200)
#define GPIO_Pin_10 ((uint16_t)0x0400)
#define GPIO_Pin_11 ((uint16_t)0x0800)
#define GPIO_Pin_12 ((uint16_t)0x1000)
#define GPIO_Pin_13 ((uint16_t)0x2000)
#define GPIO_Pin_14 ((uint16_t)0x4000)
#define GPIO_Pin_15 ((uint16_t)0x8000)

#define Input_mode (0x0UL)
#define General_purpose_output_mode (0x1UL)
#define Alternate_function_mode (0x2UL)
#define Analog_mode (0x3UL)

#define Output_push_pull (0x0UL)
#define Output_open_drain (0x1UL)

#define Low_speed (0x0UL)
#define Medium_speed (0x1UL)
#define High_speed (0x2UL)
#define Very_high_speed (0x3UL)

#define No_pull_up_No_pull_down (0x0UL)
#define Pull_up (0x1UL)
#define Pull_down (0x2UL)

// RCC_SYSCLK_DIV  AHB prescaler
#define RCC_SYSCLK_DIV_1     0x00000000U                                                               // SYSCLK not divided
#define RCC_SYSCLK_DIV_2     RCC_CFGR_HPRE_3                                                           // SYSCLK divided by 2
#define RCC_SYSCLK_DIV_4    (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_0)                                        // SYSCLK divided by 4
#define RCC_SYSCLK_DIV_8    (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_1)                                        // SYSCLK divided by 8
#define RCC_SYSCLK_DIV_16   (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_0)                      // SYSCLK divided by 16
#define RCC_SYSCLK_DIV_64   (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2)                                        // SYSCLK divided by 64
#define RCC_SYSCLK_DIV_128  (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_0)                      // SYSCLK divided by 128
#define RCC_SYSCLK_DIV_256  (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_1)                      // SYSCLK divided by 256
#define RCC_SYSCLK_DIV_512  (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_0)    // SYSCLK divided by 512

// RCC_SYS_CLKSOURCE  System clock switch
#define RCC_SYS_CLKSOURCE_HSI   0x00000000U                     // HSI selection as system clock
#define RCC_SYS_CLKSOURCE_HSE   RCC_CFGR_SW_0                   // HSE selection as system clock
#define RCC_SYS_CLKSOURCE_PLL   RCC_CFGR_SW_1                   // PLL selection as system clock
#define RCC_SYS_CLKSOURCE_LSI  (RCC_CFGR_SW_1 | RCC_CFGR_SW_0)  // LSI selection used as system clock
#define RCC_SYS_CLKSOURCE_LSE   RCC_CFGR_SW_2                   // LSE selection used as system clock

// RCC_SYS_CLKSOURCE_STATUS  System clock switch status
#define RCC_SYS_CLKSOURCE_STATUS_HSI   0x00000000U                       // HSI used as system clock
#define RCC_SYS_CLKSOURCE_STATUS_HSE   RCC_CFGR_SWS_0                    // HSE used as system clock
#define RCC_SYS_CLKSOURCE_STATUS_PLL   RCC_CFGR_SWS_1                    // PLL used as system clock
#define RCC_SYS_CLKSOURCE_STATUS_LSI  (RCC_CFGR_SWS_1 | RCC_CFGR_SWS_0)  // LSI used as system clock
#define RCC_SYS_CLKSOURCE_STATUS_LSE   RCC_CFGR_SWS_2                    // LSE used as system clock

// RCC_APB1_DIV  APB low-speed prescaler (APB1)
#define RCC_APB1_DIV_1    0x00000000U                                            // HCLK not divided
#define RCC_APB1_DIV_2    RCC_CFGR_PPRE_2                                        // HCLK divided by 2
#define RCC_APB1_DIV_4   (RCC_CFGR_PPRE_2 | RCC_CFGR_PPRE_0)                     // HCLK divided by 4
#define RCC_APB1_DIV_8   (RCC_CFGR_PPRE_2 | RCC_CFGR_PPRE_1)                     // HCLK divided by 8
#define RCC_APB1_DIV_16  (RCC_CFGR_PPRE_2 | RCC_CFGR_PPRE_1 | RCC_CFGR_PPRE_0)   // HCLK divided by 16

// RCC_MCO1SOURCE  MCO1 SOURCE selection
#define RCC_MCO1SOURCE_NOCLOCK  0x00000000U                                                // MCO output disabled, no clock on MCO
#define RCC_MCO1SOURCE_SYSCLK   RCC_CFGR_MCOSEL_0                                          // SYSCLK selection as MCO1 source
#define RCC_MCO1SOURCE_HSI     (RCC_CFGR_MCOSEL_0 | RCC_CFGR_MCOSEL_1)                     // HSI16 selection as MCO1 source
#define RCC_MCO1SOURCE_HSE      RCC_CFGR_MCOSEL_2                                          // HSE selection as MCO1 source
#define RCC_MCO1SOURCE_PLLCLK  (RCC_CFGR_MCOSEL_0 | RCC_CFGR_MCOSEL_2)                     // Main PLL selection as MCO1 source
#define RCC_MCO1SOURCE_LSI     (RCC_CFGR_MCOSEL_1 | RCC_CFGR_MCOSEL_2)                     // LSI selection as MCO1 source
#define RCC_MCO1SOURCE_LSE     (RCC_CFGR_MCOSEL_0 | RCC_CFGR_MCOSEL_1 | RCC_CFGR_MCOSEL_2) // LSE selection as MCO1 source

// RCC_MCO1_DIV  MCO1 prescaler
#define RCC_MCO1_DIV_1     0x00000000U                                                  // MCO1 not divided
#define RCC_MCO1_DIV_2     RCC_CFGR_MCOPRE_0                                            // MCO1 divided by 2
#define RCC_MCO1_DIV_4     RCC_CFGR_MCOPRE_1                                            // MCO1 divided by 4
#define RCC_MCO1_DIV_8    (RCC_CFGR_MCOPRE_1 | RCC_CFGR_MCOPRE_0)                       // MCO1 divided by 8
#define RCC_MCO1_DIV_16    RCC_CFGR_MCOPRE_2                                            // MCO1 divided by 16
#define RCC_MCO1_DIV_32   (RCC_CFGR_MCOPRE_2 | RCC_CFGR_MCOPRE_0)                       // MCO1 divided by 32
#define RCC_MCO1_DIV_64   (RCC_CFGR_MCOPRE_2 | RCC_CFGR_MCOPRE_1)                       // MCO1 divided by 64
#define RCC_MCO1_DIV_128  (RCC_CFGR_MCOPRE_2 | RCC_CFGR_MCOPRE_1 | RCC_CFGR_MCOPRE_0)   // MCO1 divided by 128

// SYSTEM_LATENCY FLASH LATENCY
#define FLASH_LATENCY_0   0x00000000U                                 // FLASH Zero Latency cycle
#define FLASH_LATENCY_1   FLASH_ACR_LATENCY_0                         // FLASH One Latency cycle
#define FLASH_LATENCY_2   FLASH_ACR_LATENCY_1                         // FLASH Two wait states
#define FLASH_LATENCY_3  (FLASH_ACR_LATENCY_1 | FLASH_ACR_LATENCY_0)  // FLASH Three wait states

// RCC_PLLSOURCE  PLL entry clock source
#define RCC_PLLSOURCE_NONE  0x00000000U            // No clock
#define RCC_PLLSOURCE_HSI   RCC_PLLCFGR_PLLSRC_HSI // HSI16 clock selected as PLL entry clock source
#define RCC_PLLSOURCE_HSE   RCC_PLLCFGR_PLLSRC_HSE // HSE clock selected as PLL entry clock source

// RCC_PLLM_DIV  PLL division factor (PLLM)
#define RCC_PLLM_DIV_1   0x00000000U                                // PLL division factor by 1
#define RCC_PLLM_DIV_2  (RCC_PLLCFGR_PLLM_0)                        // PLL division factor by 2
#define RCC_PLLM_DIV_3  (RCC_PLLCFGR_PLLM_1)                        // PLL division factor by 3
#define RCC_PLLM_DIV_4  ((RCC_PLLCFGR_PLLM_1 | RCC_PLLCFGR_PLLM_0)) // PLL division factor by 4
#define RCC_PLLM_DIV_5  (RCC_PLLCFGR_PLLM_2)                        // PLL division factor by 5
#define RCC_PLLM_DIV_6  ((RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_0)) // PLL division factor by 6
#define RCC_PLLM_DIV_7  ((RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_1)) // PLL division factor by 7
#define RCC_PLLM_DIV_8  (RCC_PLLCFGR_PLLM)                          // PLL division factor by 8

// RCC_PLLR_DIV  PLL division factor (PLLR)
#define RCC_PLLR_DIV_2  (RCC_PLLCFGR_PLLR_0)                      // Main PLL division factor for PLLCLK (system clock) by 2
#define RCC_PLLR_DIV_3  (RCC_PLLCFGR_PLLR_1)                      // Main PLL division factor for PLLCLK (system clock) by 3
#define RCC_PLLR_DIV_4  (RCC_PLLCFGR_PLLR_1 | RCC_PLLCFGR_PLLR_0) // Main PLL division factor for PLLCLK (system clock) by 4
#define RCC_PLLR_DIV_5  (RCC_PLLCFGR_PLLR_2)                      // Main PLL division factor for PLLCLK (system clock) by 5
#define RCC_PLLR_DIV_6  (RCC_PLLCFGR_PLLR_2 | RCC_PLLCFGR_PLLR_0) // Main PLL division factor for PLLCLK (system clock) by 6
#define RCC_PLLR_DIV_7  (RCC_PLLCFGR_PLLR_2 | RCC_PLLCFGR_PLLR_1) // Main PLL division factor for PLLCLK (system clock) by 7
#define RCC_PLLR_DIV_8  (RCC_PLLCFGR_PLLR)                        // Main PLL division factor for PLLCLK (system clock) by 8

void System_ClockConfig(void);
void Gpio_Config(void);
void MCO_PA8_GPIO_Config(void);
void delay_ms(uint32_t ms);

int main(void) {
  System_ClockConfig();
  Gpio_Config();
  MCO_PA8_GPIO_Config();

  MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOSEL, RCC_MCO1SOURCE_SYSCLK);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOPRE, RCC_MCO1_DIV_32);// MCO  64/32 = 2 MHz (SYSCLK/32)

  while (1) {
    delay_ms(300);
    WRITE_REG(GPIOD->ODR, READ_REG(GPIOD->ODR) ^ GPIO_BSRR_BS2);
  }
}

void System_ClockConfig(void) {
  // Set FLASH Latency
  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_LATENCY_2);
  // Get FLASH Latency
  while (READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY != FLASH_LATENCY_2)) {
  }

  // HSI configuration and activation
  // Enable HSI oscillator
  SET_BIT(RCC->CR, RCC_CR_HSION);
  // Check if HSI clock is ready
  while (READ_BIT(RCC->CR, RCC_CR_HSIRDY == 0UL)) {
  }

  // Main PLL configuration and activation
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC, RCC_PLLSOURCE_HSI);
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM, RCC_PLLM_DIV_1);
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN, (8 << RCC_PLLCFGR_PLLN_Pos));
  MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLR, RCC_PLLR_DIV_2);
  // Enable PLL
  SET_BIT(RCC->CR, RCC_CR_PLLON);
  // Enable PLL output mapped on SYSCLK domain
  SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLREN);

  // Check if PLL Ready
  while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0UL) {
  }
  // Set AHB prescaler
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_SYSCLK_DIV_1);

  // Sysclk activation on the main PLL
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SYS_CLKSOURCE_PLL);
  // Get the system clock source
  while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS != RCC_SYS_CLKSOURCE_STATUS_PLL)) {
  }
  // Set APB1 prescaler
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, RCC_APB1_DIV_1);
  // HCLK clock frequency
  SystemCoreClock = 64000000;
}

void Gpio_Config(void) {
  SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIODEN);

  MODIFY_REG(GPIOD->MODER, GPIO_MODER_MODE2, General_purpose_output_mode << GPIO_MODER_MODE2_Pos); // Настройка GPIOD пин 2 на выход (output mode)
  MODIFY_REG(GPIOD->OTYPER, GPIO_OTYPER_OT2, Output_push_pull << GPIO_OTYPER_OT2_Pos);             // Настройка GPIOD пин 2 в режим Push-Pull
  MODIFY_REG(GPIOD->OSPEEDR, GPIO_OSPEEDR_OSPEED2, High_speed << GPIO_OSPEEDR_OSPEED2_Pos);        // Настройка GPIOD пин 2 в режим High_speed
  MODIFY_REG(GPIOD->PUPDR, GPIO_PUPDR_PUPD2, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD2_Pos);     // Настройка GPIOD пин 2 в режим No_pull_up_No_pull_down
}

void MCO_PA8_GPIO_Config(void) {
  SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);

  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE8, Alternate_function_mode << GPIO_MODER_MODE8_Pos);
  MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL8, PA8_MCO_AF0 << GPIO_AFRH_AFSEL8_Pos);
  // AFR[0] (AFRL - Low): Управляет пинами PA0 - PA7.
  // AFR[1] (AFRH - High): Управляет пинами PA8 - PA15
  MODIFY_REG(GPIOA->OTYPER, GPIO_OTYPER_OT8, Output_push_pull << GPIO_OTYPER_OT8_Pos);
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED8, Very_high_speed << GPIO_OSPEEDR_OSPEED8_Pos);
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD8, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD8_Pos);
}

void delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms * 1000; i++) {
    __NOP(); // No operation instruction
  }
}
/******************************** End of file *********************************/