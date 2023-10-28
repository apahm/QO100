
#ifndef LCD_CONFIG_H
#define LCD_CONFIG_H 

#define _LCD_USE_FREERTOS 0
#define _LCD_USE_MENU_LIB 0

#define _LCD_COLS         40
#define _LCD_ROWS         4

#define _LCD_RS_PORT      GPIOA
#define _LCD_RS_PIN       GPIO_PIN_5

#define _LCD_E_PORT       GPIOA
#define _LCD_E_PIN        GPIO_PIN_2

#define _LCD_E_2_PORT       GPIOA
#define _LCD_E_2_PIN        GPIO_PIN_4

#define _LCD_RW_PORT      GPIOA
#define _LCD_RW_PIN       GPIO_PIN_3

#define _LCD_D4_PORT      GPIOA
#define _LCD_D4_PIN		  GPIO_PIN_0

#define _LCD_D5_PORT      GPIOC
#define _LCD_D5_PIN       GPIO_PIN_15

#define _LCD_D6_PORT      GPIOC
#define _LCD_D6_PIN       GPIO_PIN_14

#define _LCD_D7_PORT      GPIOA
#define _LCD_D7_PIN       GPIO_PIN_1

#endif

