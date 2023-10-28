
#ifndef LCD_H
#define LCD_H 
/*
+++   Nima Askari
+++   www.github.com/NimaLTD
+++   www.instagram.com/github.NimaLTD 
+++   Version: 1.1.0
*/


#include "gpio.h"


void LCD_Init(void);


void LCD_Init_High(void);
void LCD_DisplayOn_High(void);
void LCD_DisplayOff_High(void);
void LCD_Clear_High(void);
void LCD_Puts_High(uint8_t x, uint8_t y, char* str);
void LCD_BlinkOn_High(void);
void LCD_BlinkOff_High(void);
void LCD_CursorOn_High(void);
void LCD_CursorOff_High(void);
void LCD_ScrollLeft_High(void);
void LCD_ScrollRight_High(void);
void LCD_CreateChar_High(uint8_t location, uint8_t* data);
void LCD_PutCustom_High(uint8_t x, uint8_t y, uint8_t location);
void LCD_Put_High(uint8_t Data);

void LCD_Init_Low(void);
void LCD_DisplayOn_Low(void);
void LCD_DisplayOff_Low(void);
void LCD_Clear_Low(void);
void LCD_Puts_Low(uint8_t x, uint8_t y, char* str);
void LCD_BlinkOn_Low(void);
void LCD_BlinkOff_Low(void);
void LCD_CursorOn_Low(void);
void LCD_CursorOff_Low(void);
void LCD_ScrollLeft_Low(void);
void LCD_ScrollRight_Low(void);
void LCD_CreateChar_Low(uint8_t location, uint8_t* data);
void LCD_PutCustom_Low(uint8_t x, uint8_t y, uint8_t location);
void LCD_Put_Low(uint8_t Data);

void LCD_Init_Port(void);

#endif

