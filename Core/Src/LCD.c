
#include "LCD.h"
#include "LCD_Config.h"
#if _LCD_USE_FREERTOS==1
#include "cmsis_os.h"
#endif

typedef struct {
	uint8_t DisplayControl;
	uint8_t DisplayFunction;
	uint8_t DisplayMode;
	uint8_t currentX;
	uint8_t currentY;
  
} LCD_Options_t;

/* Private functions */
static void LCD_Cmd_High(uint8_t cmd);
static void LCD_Cmd4bit_High(uint8_t cmd);
static void LCD_Data_High(uint8_t data);
static void LCD_CursorSet_High(uint8_t col, uint8_t row);

static void LCD_Cmd_Low(uint8_t cmd);
static void LCD_Cmd4bit_Low(uint8_t cmd);
static void LCD_Data_Low(uint8_t data);
static void LCD_CursorSet_Low(uint8_t col, uint8_t row);

//############################################################################################
/* Private variable */
static LCD_Options_t LCD_Opts_High;
static LCD_Options_t LCD_Opts_Low;
//############################################################################################
/* Pin definitions */
#define LCD_RS_LOW              HAL_GPIO_WritePin(_LCD_RS_PORT, _LCD_RS_PIN,GPIO_PIN_RESET)
#define LCD_RS_HIGH             HAL_GPIO_WritePin(_LCD_RS_PORT, _LCD_RS_PIN,GPIO_PIN_SET)
#define LCD_E_LOW               HAL_GPIO_WritePin(_LCD_E_PORT,  _LCD_E_PIN,GPIO_PIN_RESET)
#define LCD_E_HIGH              HAL_GPIO_WritePin(_LCD_E_PORT,  _LCD_E_PIN,GPIO_PIN_SET)
#define LCD_E_BLINK             LCD_E_HIGH; LCD_Delay_us(50); LCD_E_LOW; LCD_Delay_us(50)

#define LCD_E_2_LOW               HAL_GPIO_WritePin(_LCD_E_2_PORT,  _LCD_E_2_PIN,GPIO_PIN_RESET)
#define LCD_E_2_HIGH              HAL_GPIO_WritePin(_LCD_E_2_PORT,  _LCD_E_2_PIN,GPIO_PIN_SET)
#define LCD_E_2_BLINK             LCD_E_2_HIGH; LCD_Delay_us(50); LCD_E_2_LOW; LCD_Delay_us(50)
//############################################################################################
/* Commands*/
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80
/* Flags for display entry mode */
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00
/* Flags for display on/off control */
#define LCD_DISPLAYON           0x04
#define LCD_CURSORON            0x02
#define LCD_BLINKON             0x01
/* Flags for display/cursor shift */
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00
/* Flags for function set */
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00
//############################################################################################
void  LCD_Delay_us(uint16_t  us)
{
  uint32_t  Div = (SysTick->LOAD+1)/1000;
  uint32_t  StartMicros = HAL_GetTick()*1000 + (1000- SysTick->VAL/Div);
  while((HAL_GetTick()*1000 + (1000-SysTick->VAL/Div)-StartMicros < us));  
}
//############################################################################################
void  LCD_Delay_ms(uint8_t  ms)
{
  #if _LCD_USE_FREERTOS==1
  osDelay(ms);
  #else
  HAL_Delay(ms);
  #endif  
}
//############################################################################################
void LCD_Init_High(void)
{	
	while(HAL_GetTick()<200)
    LCD_Delay_ms(1);
	/* Set cursor pointer to beginning for LCD */
	LCD_Opts_High.currentX = 0;
	LCD_Opts_High.currentY = 0;
	LCD_Opts_High.DisplayFunction = LCD_4BITMODE | LCD_5x8DOTS | LCD_1LINE;
	if (_LCD_ROWS > 1)
		LCD_Opts_High.DisplayFunction |= LCD_2LINE;
	/* Try to set 4bit mode */
	LCD_Cmd4bit_High(0x03);
	LCD_Delay_ms(5);
	/* Second try */
	LCD_Cmd4bit_High(0x03);
	LCD_Delay_ms(5);
	/* Third goo! */
	LCD_Cmd4bit_High(0x03);
	LCD_Delay_ms(5);
	/* Set 4-bit interface */
	LCD_Cmd4bit_High(0x02);
	LCD_Delay_ms(5);
	/* Set # lines, font size, etc. */
	LCD_Cmd_High(LCD_FUNCTIONSET | LCD_Opts_High.DisplayFunction);
	/* Turn the display on with no cursor or blinking default */
	LCD_Opts_High.DisplayControl = LCD_DISPLAYON;
	LCD_DisplayOn_High();
	LCD_Clear_High();
	/* Default font directions */
	LCD_Opts_High.DisplayMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	LCD_Cmd_High(LCD_ENTRYMODESET | LCD_Opts_High.DisplayMode);
	LCD_Delay_ms(5);
}
//############################################################################################
void LCD_Clear_High(void)
{
	LCD_Cmd_High(LCD_CLEARDISPLAY);
	LCD_Delay_ms(5);
}
//############################################################################################
void LCD_Puts_High(uint8_t x, uint8_t y, char* str)
{
	LCD_CursorSet_High(x, y);
	while (*str)
  {
		if (LCD_Opts_High.currentX >= _LCD_COLS)
    {
			LCD_Opts_High.currentX = 0;
			LCD_Opts_High.currentY++;
			LCD_CursorSet_High(LCD_Opts_High.currentX, LCD_Opts_High.currentY);
		}
		if (*str == '\n')
    {
			LCD_Opts_High.currentY++;
			LCD_CursorSet_High(LCD_Opts_High.currentX, LCD_Opts_High.currentY);
		}
    else if (*str == '\r')
    {
			LCD_CursorSet_High(0, LCD_Opts_High.currentY);
		}
    else
    {
			LCD_Data_High(*str);
			LCD_Opts_High.currentX++;
		}
		str++;
	}
}
//############################################################################################
void LCD_DisplayOn_High(void)
{
	LCD_Opts_High.DisplayControl |= LCD_DISPLAYON;
	LCD_Cmd_High(LCD_DISPLAYCONTROL | LCD_Opts_High.DisplayControl);
}
//############################################################################################
void LCD_DisplayOff_High(void)
{
	LCD_Opts_High.DisplayControl &= ~LCD_DISPLAYON;
	LCD_Cmd_High(LCD_DISPLAYCONTROL | LCD_Opts_High.DisplayControl);
}
//############################################################################################
void LCD_BlinkOn_High(void)
{
	LCD_Opts_High.DisplayControl |= LCD_BLINKON;
	LCD_Cmd_High(LCD_DISPLAYCONTROL | LCD_Opts_High.DisplayControl);
}
//############################################################################################
void LCD_BlinkOff_High(void)
{
	LCD_Opts_High.DisplayControl &= ~LCD_BLINKON;
	LCD_Cmd_High(LCD_DISPLAYCONTROL | LCD_Opts_High.DisplayControl);
}
//############################################################################################
void LCD_CursorOn_High(void)
{
	LCD_Opts_High.DisplayControl |= LCD_CURSORON;
	LCD_Cmd_High(LCD_DISPLAYCONTROL | LCD_Opts_High.DisplayControl);
}
//############################################################################################
void LCD_CursorOff_High(void)
{
	LCD_Opts_High.DisplayControl &= ~LCD_CURSORON;
	LCD_Cmd_High(LCD_DISPLAYCONTROL | LCD_Opts_High.DisplayControl);
}
//############################################################################################
void LCD_ScrollLeft_High(void)
{
	LCD_Cmd_High(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
//############################################################################################
void LCD_ScrollRight_High(void)
{
	LCD_Cmd_High(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}
//############################################################################################
void LCD_CreateChar_High(uint8_t location, uint8_t *data)
{
	uint8_t i;
	/* We have 8 locations available for custom characters */
	location &= 0x07;
	LCD_Cmd_High(LCD_SETCGRAMADDR | (location << 3));
	
	for (i = 0; i < 8; i++) {
		LCD_Data_High(data[i]);
	}
}

void LCD_Put_High(uint8_t Data)
{
	LCD_Data_High(Data);
}

//############################################################################################
void LCD_PutCustom_High(uint8_t x, uint8_t y, uint8_t location)
{
	LCD_CursorSet_High(x, y);
	LCD_Data_High(location);
}
//############################################################################################
static void LCD_Cmd_High(uint8_t cmd)
{
	LCD_RS_LOW;
	LCD_Cmd4bit_High(cmd >> 4);
	LCD_Cmd4bit_High(cmd & 0x0F);
}
//############################################################################################
static void LCD_Data_High(uint8_t data)
{
	LCD_RS_HIGH;
	LCD_Cmd4bit_High(data >> 4);
	LCD_Cmd4bit_High(data & 0x0F);
}
//############################################################################################
static void LCD_Cmd4bit_High(uint8_t cmd)
{
	HAL_GPIO_WritePin(_LCD_D7_PORT, _LCD_D7_PIN, (GPIO_PinState)(cmd & 0x08));
	HAL_GPIO_WritePin(_LCD_D6_PORT, _LCD_D6_PIN, (GPIO_PinState)(cmd & 0x04));
	HAL_GPIO_WritePin(_LCD_D5_PORT, _LCD_D5_PIN, (GPIO_PinState)(cmd & 0x02));
	HAL_GPIO_WritePin(_LCD_D4_PORT, _LCD_D4_PIN, (GPIO_PinState)(cmd & 0x01));
	LCD_E_BLINK;
}
//############################################################################################
static void LCD_CursorSet_High(uint8_t col, uint8_t row)
{
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	if (row >= _LCD_ROWS)
		row = 0;
	LCD_Opts_High.currentX = col;
	LCD_Opts_High.currentY = row;
	LCD_Cmd_High(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void LCD_Init_Port(void)
{
	  GPIO_InitTypeDef  gpio;
	  gpio.Mode = GPIO_MODE_OUTPUT_PP;
	  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	  gpio.Pull = GPIO_NOPULL;

	  gpio.Pin = _LCD_RS_PIN;
	  HAL_GPIO_Init(_LCD_RS_PORT,&gpio);
	  gpio.Pin = _LCD_E_PIN;
	  HAL_GPIO_Init(_LCD_E_PORT,&gpio);
	  gpio.Pin = _LCD_RW_PIN;
	  HAL_GPIO_Init(_LCD_RW_PORT,&gpio);
	  gpio.Pin = _LCD_D4_PIN;
	  HAL_GPIO_Init(_LCD_D4_PORT,&gpio);
	  gpio.Pin = _LCD_D5_PIN;
	  HAL_GPIO_Init(_LCD_D5_PORT,&gpio);
	  gpio.Pin = _LCD_D6_PIN;
	  HAL_GPIO_Init(_LCD_D6_PORT,&gpio);
	  gpio.Pin = _LCD_D7_PIN;
	  HAL_GPIO_Init(_LCD_D7_PORT,&gpio);
	  gpio.Pin = _LCD_E_2_PIN;
	  HAL_GPIO_Init(_LCD_E_2_PORT,&gpio);
}

//############################################################################################
void LCD_Init_Low(void)
{
	while(HAL_GetTick()<200)
    LCD_Delay_ms(1);
	/* Set cursor pointer to beginning for LCD */
	LCD_Opts_Low.currentX = 0;
	LCD_Opts_Low.currentY = 0;
	LCD_Opts_Low.DisplayFunction = LCD_4BITMODE | LCD_5x8DOTS | LCD_1LINE;
	if (_LCD_ROWS > 1)
		LCD_Opts_Low.DisplayFunction |= LCD_2LINE;
	/* Try to set 4bit mode */
	LCD_Cmd4bit_Low(0x03);
	LCD_Delay_ms(5);
	/* Second try */
	LCD_Cmd4bit_Low(0x03);
	LCD_Delay_ms(5);
	/* Third goo! */
	LCD_Cmd4bit_Low(0x03);
	LCD_Delay_ms(5);
	/* Set 4-bit interface */
	LCD_Cmd4bit_Low(0x02);
	LCD_Delay_ms(5);
	/* Set # lines, font size, etc. */
	LCD_Cmd_Low(LCD_FUNCTIONSET | LCD_Opts_Low.DisplayFunction);
	/* Turn the display on with no cursor or blinking default */
	LCD_Opts_Low.DisplayControl = LCD_DISPLAYON;
	LCD_DisplayOn_Low();
	LCD_Clear_Low();
	/* Default font directions */
	LCD_Opts_Low.DisplayMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	LCD_Cmd_Low(LCD_ENTRYMODESET | LCD_Opts_Low.DisplayMode);
	LCD_Delay_ms(5);
}
//############################################################################################
void LCD_Clear_Low(void)
{
	LCD_Cmd_Low(LCD_CLEARDISPLAY);
	LCD_Delay_ms(5);
}
//############################################################################################
void LCD_Puts_Low(uint8_t x, uint8_t y, char* str)
{
	LCD_CursorSet_Low(x, y);
	while (*str)
  {
		if (LCD_Opts_Low.currentX >= _LCD_COLS)
    {
			LCD_Opts_Low.currentX = 0;
			LCD_Opts_Low.currentY++;
			LCD_CursorSet_Low(LCD_Opts_Low.currentX, LCD_Opts_Low.currentY);
		}
		if (*str == '\n')
    {
			LCD_Opts_Low.currentY++;
			LCD_CursorSet_Low(LCD_Opts_Low.currentX, LCD_Opts_Low.currentY);
		}
    else if (*str == '\r')
    {
			LCD_CursorSet_Low(0, LCD_Opts_Low.currentY);
		}
    else
    {
			LCD_Data_Low(*str);
			LCD_Opts_Low.currentX++;
		}
		str++;
	}
}
//############################################################################################
void LCD_DisplayOn_Low(void)
{
	LCD_Opts_Low.DisplayControl |= LCD_DISPLAYON;
	LCD_Cmd_Low(LCD_DISPLAYCONTROL | LCD_Opts_Low.DisplayControl);
}
//############################################################################################
void LCD_DisplayOff_Low(void)
{
	LCD_Opts_Low.DisplayControl &= ~LCD_DISPLAYON;
	LCD_Cmd_Low(LCD_DISPLAYCONTROL | LCD_Opts_Low.DisplayControl);
}
//############################################################################################
void LCD_BlinkOn_Low(void)
{
	LCD_Opts_Low.DisplayControl |= LCD_BLINKON;
	LCD_Cmd_Low(LCD_DISPLAYCONTROL | LCD_Opts_Low.DisplayControl);
}
//############################################################################################
void LCD_BlinkOff_Low(void)
{
	LCD_Opts_Low.DisplayControl &= ~LCD_BLINKON;
	LCD_Cmd_Low(LCD_DISPLAYCONTROL | LCD_Opts_Low.DisplayControl);
}
//############################################################################################
void LCD_CursorOn_Low(void)
{
	LCD_Opts_Low.DisplayControl |= LCD_CURSORON;
	LCD_Cmd_Low(LCD_DISPLAYCONTROL | LCD_Opts_Low.DisplayControl);
}
//############################################################################################
void LCD_CursorOff_Low(void)
{
	LCD_Opts_Low.DisplayControl &= ~LCD_CURSORON;
	LCD_Cmd_Low(LCD_DISPLAYCONTROL | LCD_Opts_Low.DisplayControl);
}
//############################################################################################
void LCD_ScrollLeft_Low(void)
{
	LCD_Cmd_Low(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
//############################################################################################
void LCD_ScrollRight_Low(void)
{
	LCD_Cmd_Low(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}
//############################################################################################
void LCD_CreateChar_Low(uint8_t location, uint8_t *data)
{
	uint8_t i;
	/* We have 8 locations available for custom characters */
	location &= 0x07;
	LCD_Cmd_Low(LCD_SETCGRAMADDR | (location << 3));

	for (i = 0; i < 8; i++) {
		LCD_Data_Low(data[i]);
	}
}

void LCD_Put_Low(uint8_t Data)
{
	LCD_Data_Low(Data);
}

//############################################################################################
void LCD_PutCustom_Low(uint8_t x, uint8_t y, uint8_t location)
{
	LCD_CursorSet_Low(x, y);
	LCD_Data_Low(location);
}
//############################################################################################
static void LCD_Cmd_Low(uint8_t cmd)
{
	LCD_RS_LOW;
	LCD_Cmd4bit_Low(cmd >> 4);
	LCD_Cmd4bit_Low(cmd & 0x0F);
}
//############################################################################################
static void LCD_Data_Low(uint8_t data)
{
	LCD_RS_HIGH;
	LCD_Cmd4bit_Low(data >> 4);
	LCD_Cmd4bit_Low(data & 0x0F);
}
//############################################################################################
static void LCD_Cmd4bit_Low(uint8_t cmd)
{
	HAL_GPIO_WritePin(_LCD_D7_PORT, _LCD_D7_PIN, (GPIO_PinState)(cmd & 0x08));
	HAL_GPIO_WritePin(_LCD_D6_PORT, _LCD_D6_PIN, (GPIO_PinState)(cmd & 0x04));
	HAL_GPIO_WritePin(_LCD_D5_PORT, _LCD_D5_PIN, (GPIO_PinState)(cmd & 0x02));
	HAL_GPIO_WritePin(_LCD_D4_PORT, _LCD_D4_PIN, (GPIO_PinState)(cmd & 0x01));
	LCD_E_2_BLINK;
}
//############################################################################################
static void LCD_CursorSet_Low(uint8_t col, uint8_t row)
{
	uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	if (row >= _LCD_ROWS)
		row = 0;
	LCD_Opts_Low.currentX = col;
	LCD_Opts_Low.currentY = row;
	LCD_Cmd_Low(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void LCD_Init(void)
{
	  LCD_Init_Port();
	  LCD_Init_High();
	  LCD_Init_Low();
}

