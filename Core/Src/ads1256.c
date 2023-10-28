/******************************************************************************
 * ������:      ads1256.c
 * �����:       Celeron (c) 2018 
 * ����������:  ������� �������� 24-������� ��� "ADS1256" � SPI �����������.
 ******************************************************************************/
 
#include "stm32f1xx_hal.h"          // ���������� HAL API
#include <stdio.h>                  // printf...

#include "ads1256_defs.h"           // ���������� ����������� ��������� � ������ ���������� ADS1256
#include "ads1256.h"                // ��������� ��������� ������� � �����



//============================================================================
// ���������� ��� (���������� �����)
//============================================================================


// ���������� SPI ���������, �� ������� ����� ������� ���
#define SPI_ADC_HANDLE   hspi2

// ��������� ������ ��� ���������� SPI/��� (External variables)
extern SPI_HandleTypeDef SPI_ADC_HANDLE;


// ��������� SPI: 
//  CPOL = Low; CPHA=2 Edge.
//
// ���������:  https://ru.wikipedia.org/wiki/Serial_Peripheral_Interface
//  CPOL = 0 � ������ ������������� ���������� � ������� ������;
//  CPHA = 1 � ������� ������ ������������ �� ������� ������ ������� �������������.
//
// ADS1256 Datasheet:  http://www.ti.com/lit/ds/symlink/ads1256.pdf
//  ADS1256 Datasheet / Figure 1. Serial Interface Timing (page 6) ���������� ��������� ����� ����������.
//  ��. T4 T5 - ��� �������� �������� DIN �������� ������������ ������� ��������.
//  ������, ����� DOUT ��������� � ��� �� ���� (�� ������� ������) - ��� �������� SPI, ���� �����! 
//  � �������� T7 T8 ������������� "�������" (����������� ��������� �������). 
//  ���������: �������� T7 T8 ����������� ������������ "������� ��������", ������ ��� �� ������ �������� ���������� ������������ �������. �� ������ ����� ������������ ���� ��������� ������ (���������� �������� �������� "������ ������� ������" - � �� ����� �������� � MISO=DOUT).
//
// STM32CubeMX ���������� ������������
//  ������� "Configuration" / � ������ "Connectivity" / ���� ������ ���������������� "SPIx" ����������
//  Clock Polarity (CPOL) = Low
//  Clock Phase    (CPHA) = 2 Edge



//-------------------------------------
// ���� DRDY (Data Ready - ���������� ����������)
//-------------------------------------

// �������� ����� DRDY (���������� ������ � �������) (� �����, ����������� ��� ��������� � ����������� �� ������� - �� ��������� � "����������" � "�����������", � ��. ���������� �������)
#define  ADS1256_DRDY_PIN         GPIO_PIN_13
#define  ADS1256_DRDY_PORT        GPIOD

#if defined(ADS1256_DRDY_PORT) && defined (ADS1256_DRDY_PIN)
  // �������� ����� �� ����������� ���� (����� ������ � ������������������, �������������� ��� ������ �����������; 
  //  ������ ������� ��������� ��������������� ����� � ���������� ���� DRDY ��� ����������� ���)
  #define  ADS1256_DRDY_BUSY()    (HAL_GPIO_ReadPin(ADS1256_DRDY_PORT, ADS1256_DRDY_PIN) == GPIO_PIN_SET )
  #define  ADS1256_DRDY_READY()   (HAL_GPIO_ReadPin(ADS1256_DRDY_PORT, ADS1256_DRDY_PIN) == GPIO_PIN_RESET )

#else
  // �������� ����� �� ������������ ������� �������� ���� DRDY, �� ����������� �������� ����������, ����� SPI 
  //  (����� ��������, ������ ������������� ����������� - ������� ������ ������������ ����� "Read data continously"!)
  #define  ADS1256_DRDY_BUSY()    (ADS1256_API_GetDataReadyBit() == ADS1256_STATUS_DRDY_BUSY)
  #define  ADS1256_DRDY_READY()   (ADS1256_API_GetDataReadyBit() == ADS1256_STATUS_DRDY_READY)

  // ��� �������: ������ ��������� �������� ���������� (������������ �� �������������!) - ��������, ���� ������ ������ ��������� ������ � � �������� ����������...
  //#define  ADS1256_DRDY_BUSY()    0 
  //#define  ADS1256_DRDY_READY()   1
                                    // ����������: 0 ��� 1 - ����� ��� �� ��������� ����� ��� ����, ����� ��� ���������� ��������!
                                    // ��������� ����� ��������-��������, ����� ������ ��� "������ ���������".
#endif



//-------------------------------------
// ���� CS (Chip Select - ��������� ��������)
//-------------------------------------

// �����: ��������� ������� CS ����� ���������� SPI ��������� ������ �������� ���������� (������ ADS1256), ��� ��������������� ��� ����������� ��������� ��������� �����.  
//  (�������: ���� ������� /CS ������ �����-reset, �� �� ���� ���������� ADS1256, � ������ ���������� ���������� SPI ������ ����������, ������� �������� "������� �����������" - �.�. ���������� "��������� ������������ ���" �����������, �� ���������� "�������� �������� ���������", ���������� ����� �������, � ������������ "���� ���������" ��������� �������� ���������� SPI...)
//  Datasheet (page 26): "CS must remain low for the duration of the serial communication. When CS is taken high, the serial interface is reset and DOUT enters a high impedance state. CS may be permanently tied low..."
//
// ������ CS - ���������� ������ ������ ������! 
//  �������������: ������ ������������ ���� ������ ����������� � ������ SPI ����������. � ������ ��������������� �������� ����� "������-�����" ��������� ��������� �� ������ ��������� CS.


// �������� ����� CS (��������� ���������� SPI � ��������� ADS1256 ������ - ����������� ��� ������� ������� � �������� ������)
#define  ADS1256_CS_PORT          GPIOB, GPIO_PIN_12

#ifdef   ADS1256_CS_PORT
  // ���������� ����� CS �������������� �������, ����� ���� �����-������...
  #define ADS1256_CS_ON()      HAL_GPIO_WritePin (ADS1256_CS_PORT, GPIO_PIN_RESET)
  #define ADS1256_CS_OFF()     HAL_GPIO_WritePin (ADS1256_CS_PORT, GPIO_PIN_SET)
  //#define ADS1256_CS_TOGGLE()  HAL_GPIO_TogglePin(ADS1256_CS_PORT)

#else
  // ���� ����������� �������� CS-����� �� ������� �� ������������:
  //  �.�. ���� �������� ��� ������ � /CS �� ������� �������� ������ � �����; 
  //  ���� �� ������� ������������ "���������� SPI.NSS" ������...
  #define ADS1256_CS_ON()
  #define ADS1256_CS_OFF()
  //#define ADS1256_CS_TOGGLE()
  
#endif



//-------------------------------------
// ��������� ������� � ��������� 
//-------------------------------------


// ���������� ������� ������, ������������ ���������� ���, ��
//  (����������� ����� F_CLKIN = 7.68MHz)
#define  ADS1256_F_CLKIN  7680000


// ������� ������������� ��� (���������� ������� � �������), SPS
// ����������� ���������� ��� ������ ���������� ������������ (����������� ������)
//  ������: ��������� API �����, ����������, ������������� � ������� �������� ������������� �� ������������ ���� (��� �������� ���������� �������).
//
//  �������� ���������� �������� ��������� F_DATA = [2.5 .. 30000] SPS  (��� F_CLKIN=7.68MHz)
//  �����: �������� ������ ����� �������� ������ �� ������������ ���� = 30000, 15000, 7500, 3750, 2000, 1000, 500, 100, 60, 50, 30, 25, 15, 10, 5, 2.5
#define  ADS1256_F_DATA   15000  /*���������� �������� ��� ������� ���������� ������������, �� � �� ���� ��� ������������ - ������ � �������, � 2 ���� ���������*/
//#define  ADS1256_F_DATA   7500  /*����� ������������ ���*/


//-------------------------------------
// ��������������, ������ ���� �������� ��������� ������� ���� SPI �� STM32Cube:
#define  STM32_MCU_SYSCLK             72000000                /*������� ����*/
#define  STM32_MCU_APBCLK             ((STM32_MCU_SYSCLK)/2)  /*������� ������������ ���� (����������)*/
#define  STM32_SPI_BAUDRATEPRESCALER  64                      /*������������ APB ��������� ������� SPI*/

// �����, ������� ���������� SPI, ��
#define  ADS1256_F_SCLK               ((STM32_MCU_APBCLK)/(STM32_SPI_BAUDRATEPRESCALER))


//-------------------------------------
// ��������: �������� ���������� �������� ������� ���������� SPI �� datasheet:
//  ������ SPI ������ ���� � ������� T_SCLK = [ 4*(1/F_CLKIN) .. 10*(1/F_DATA) ], �.�. �� 4� ������ ���������� ������� �� 10� �������.
//  �������� ���������� �������� ������� F_SCLK = F_DATA/10 .. F_CLKIN/4
#define  ADS1256_F_SCLK_MAX   ((ADS1256_F_CLKIN)/4)   /*������������ ������� �������� ���������� SPI, ��� ������� ���������� ��� ��� �������� ������������ ����������� ������� ������, ����� 4� ������� �� ����������� ���������� �������*/
//#define  ADS1256_F_SCLK_MIN   ((ADS1256_F_DATA)/10)   /*����� ��������� ���������� �������� SPI, ����� ��������� ��� ������������ � �� �������������� timeout*/

// ������: ������ ������� ��������� �������� SPI ������������ �������� - ����� �������� ��������� ����� ����������� ��� � ����������� ������ RDATAC, ��� ������ ������� �������������...
//#define  ADS1256_F_SCLK_MIN   ((ADS1256_F_DATA)*24*2)   /*���� � ���������: �����, ���� �������� 24-������ ����� + ��������� �����, �.�. ��������� ��������, ��� ��� �������� �������� ����� ������� ���������������� �������*/

// � ������, �������� "�� ���" (�� ����� ����������� ����������):
//    OneSampleCycle = 50*T_CLKIN(deadtime) + 24bit*T_SCLK(fetch new sample) + 50*T_CLKIN(deadtime) + 16*T_CLKIN(DRDY update period) = 1/F_DATA(samples per sec)
//    116/F_CLKIN + 24/F_SCLK = 1/F_DATA
//    F_SCLK = 24*F_CLKIN*F_DATA/(F_CLKIN-116*F_DATA)
#define  ADS1256_F_SCLK_MIN   (24*(ADS1256_F_CLKIN)*(ADS1256_F_DATA)/((ADS1256_F_CLKIN) - 116*(ADS1256_F_DATA)))

#if ((ADS1256_F_SCLK) <= (ADS1256_F_SCLK_MIN)) || ((ADS1256_F_SCLK) >= (ADS1256_F_SCLK_MAX))
#error "������: �������� ADS1256_F_SCLK ��� ������ �����������!"
#endif


//-------------------------------------
// �������� ����� ��������� � ��������� ���:

// ��������������� ������: ����������� ������� ��������� ������� �� "������ ����������� �������" � "���"
#define _TCLKIN_TO_US(T_CLKIN)   ((T_CLKIN) * 1000000 / (ADS1256_F_CLKIN) +1)

// Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT (RDATA, RDATAC, RREG Commands)
#define ADS1256_DELAY_T6_CLKIN   (50*2)                                   /* �����, ������� = 50 T_CLKIN, �� ������� ��������� �������� */
#define ADS1256_DELAY_T6_US      _TCLKIN_TO_US(ADS1256_DELAY_T6_CLKIN)    /* � ��� */

// Final SCLK falling edge of command to first SCLK rising edge of next command (�������� ����� ���������)
//  ����� ������� ���������� �������, ��� ����� ����� �� ���������, ����� ������������� � �������� ��������� �������
#define ADS1256_DELAY_T11_CLKIN  (24*2)                                   /* ��������� �������� ���������� 4 T_CLKIN, �� ����������� ����� 24 T_CLKIN... �����, ������ ��� �������� */
#define ADS1256_DELAY_T11_US     _TCLKIN_TO_US(ADS1256_DELAY_T11_CLKIN)   /* � ��� */

// ������� ��� ���������� �������� �����-������ (���������� � HAL-�������), ��
#define HAL_IO_TIMEOUT   50



//============================================================================
// ������� ���
//============================================================================


// ���������� ����� � �������
#ifdef USE_FULL_ASSERT

  // Note: Recipe from  http://qaru.site/questions/21384/is-there-a-printf-converter-to-print-in-binary-format
  #define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
  #define BYTE_TO_BINARY(byte)  \
    (byte & 0x80 ? '1' : '0'), \
    (byte & 0x40 ? '1' : '0'), \
    (byte & 0x20 ? '1' : '0'), \
    (byte & 0x10 ? '1' : '0'), \
    (byte & 0x08 ? '1' : '0'), \
    (byte & 0x04 ? '1' : '0'), \
    (byte & 0x02 ? '1' : '0'), \
    (byte & 0x01 ? '1' : '0') 


  // ���� ������
  static volatile uint8_t  ADS1256_LogOn   = 0;   // ���� "���� ��������?"
  static    char *volatile ADS1256_LogFile = 0;   // ��������� "����� ��������� ����"
  static volatile uint32_t ADS1256_LogLine = 0;


  // ������
  void ADS1256_Log_Print(uint8_t* caption, uint32_t value)
  {
    if(ADS1256_LogOn)
    {
      printf( "ADS1256 driver (LOG ON = file \"%s\" on line %d):\t\"%s\" = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", 
              ADS1256_LogFile, 
              ADS1256_LogLine,
              caption, 
              value, 
              BYTE_TO_BINARY(value));
    }
    return;
  }
  
  
  // Forward declarations
  uint8_t ADS1256_Command_ReadFromRegister(uint8_t regaddr);


  //-------------------------------------
  // ����������� �� ���������� � ������������� � ���������� ������� ������� �������� ���� ���������
  void ADS1256_Log_DumpRegisters(uint8_t* caption)
  {
    uint8_t value;
    printf("\r\n\r\n%s\r\n", caption);
    printf("--- ADS1256 driver / DUMP REGISTERS (start) ---\r\n");
    
    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);
    printf("ADS1256_REGISTER_STATUS (0x00) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_MUX);
    printf("ADS1256_REGISTER_MUX    (0x01) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);
    printf("ADS1256_REGISTER_ADCON  (0x02) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_DRATE);
    printf("ADS1256_REGISTER_DRATE  (0x03) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_IO);
    printf("ADS1256_REGISTER_IO     (0X04) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_OFC0);
    printf("ADS1256_REGISTER_OFC0   (0x05) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_OFC1);
    printf("ADS1256_REGISTER_OFC1   (0x06) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_OFC2);
    printf("ADS1256_REGISTER_OFC2   (0x07) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_FSC0);
    printf("ADS1256_REGISTER_FSC0   (0x08) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));
    
    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_FSC1);
    printf("ADS1256_REGISTER_FSC1   (0x09) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));
    
    value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_FSC2);
    printf("ADS1256_REGISTER_FSC2   (0x0A) = 0x%02X = 0b%c%c%c%c %c%c%c%c\r\n", value, BYTE_TO_BINARY(value));

    printf("--- ADS1256 driver / DUMP REGISTERS (end) ---\r\n");
  }  


  //-------------------------------------
  // ���������
  #define ADS1256_LOG_ON()  {ADS1256_LogOn   = 1;        \
                             ADS1256_LogFile = __FILE__; \
                             ADS1256_LogLine = __LINE__; }

  #define ADS1256_LOG_OFF() {ADS1256_LogOn   = 0;}

  #define ADS1256_LOG_PRINT(CAPTION, VALUE)  { ADS1256_Log_Print((CAPTION),(VALUE)); }
  
  #define ADS1256_LOG_DUMPREGISTERS(CAPTION) { ADS1256_Log_DumpRegisters((#CAPTION)); }

#else

  // ���������
  #define ADS1256_LOG_ON()
  #define ADS1256_LOG_OFF()
  #define ADS1256_LOG_PRINT(CAPTION, VALUE)
  #define ADS1256_LOG_DUMPREGISTERS(CAPTION)

#endif



 

//============================================================================
// �������������� ������� ���  (���������� ��������)
//============================================================================

// ��������: ������ ������������ "�������������� �������" � API ������ � � "���������� ����" - ��������� ��� �� ������������ ������ CS!
//  ��������� � ������������ ��������� ������� �� ������ ������ �� ���� ����� � ����, � ������ ���� ����� ������ ��� �������...
//
// ����������:
//  ��������, �� ������ ������������ ����� ADS1256_Command_EnsureDRDY(), ���� ������ DRDY ������ ������� �� ��������� ���� GPIO � �� ���������� �� SPI.
//  �����, �� ���������� ������ ������������ "�������������� ������", ���� �� ������ �� ����������� ������ CS � ���������� ������������, � ��������������� ����� (/CS) �� "������� �����������" (Slave) ������ ��������� � ���� ����� (/CS=GND -> CS=='1').
//  Datasheet (page 26): "CS must remain low for the duration of the serial communication. When CS is taken high, the serial interface is reset and DOUT enters a high impedance state. CS may be permanently tied low..."



//-------------------------------------
// ������������ ���������  
//-------------------------------------

//-------------------------------------
// RESET: Reset Registers to Default Values
//
// Description: Returns all registers except the CLK0 and CLK1 bits in the ADCON register to their default values.
//  This command will also stop the Read Continuous mode: in this case, issue the RESET command after DRDY goes low.
//
void ADS1256_Command_Reset(void)
{
  // ���������� �������� �� ���������� ���
  while(ADS1256_DRDY_BUSY());

  // �����
  uint8_t TxBuffer = ADS1256_COMMAND_RESET;
  // �������
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);
  
  // Datasheet: "After a reset, the ADS1255/6 performs self-calibration... "
  // ���������� �������� �� ���������� ���  (���� ���������� ��������������)
  while(ADS1256_DRDY_BUSY());
}



//-------------------------------------
// ������ ������ �� ���������� ����������������� �������� ���
// RREG: Read from Registers
uint8_t ADS1256_Command_ReadFromRegister(uint8_t regaddr)
{
  // ���������� �������� �� ���������� ���
  //while(ADS1256_DRDY_BUSY());     //�����, �� ��������� ������������ ������, �.�. "������ ���������" �� �������� ����� ���...
  
  // ������
  uint8_t TxBuffer;                                                                         // Buffer dlya otpravlyaemih dannih
  uint8_t RxBuffer = 0;                                                                     // Buffer dlya prinimaemih dannih

  // ������
  TxBuffer = ADS1256_COMMAND_RREG | (regaddr & 0xF);                                        // Ukazivaem adres registra s kotorogo nachinaetsa chtenie
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));  // Peredaem adress nachalnogo registra

  TxBuffer = 0;                                                                             // Ukazivaem kolichestvo registrov iz kotorih budut schitivatsa dannie (zapisivaetsa 1 + TO KOLICHESTVO KOTOROE UKAZIVAEM, V DANOM SLUCHAE POLUCHAETSA 1+0=1)
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));  // Peredaem kolichestvo registrov

  // ������������ ��������, ���� ���������� ��� ���������� �����
  delay_us(ADS1256_DELAY_T6_US);
  
  // �����
  assert_param(HAL_OK == HAL_SPI_Receive(&SPI_ADC_HANDLE, &RxBuffer, 1, HAL_IO_TIMEOUT));   // Schitivaem dannie iz registra

  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);
  
  return RxBuffer;                                                                          // Vozvrashaem peremennoy znachenie vibrannogo registra
}



//-------------------------------------
// ������ ������ � ��������� ���������������� ������� ���
// WREG: Write to Register
void ADS1256_Command_WriteToRegister(uint8_t regaddr, uint8_t value)
{
  // ���������� �������� �� ���������� ���
  while(ADS1256_DRDY_BUSY());
  
  // ������
  uint8_t TxBuffer;                                                                         // Buffer dlya otpravlyaemih dannih

  // �������
  TxBuffer = ADS1256_COMMAND_WREG | (regaddr & 0xF);                                        // Ukazivaem adres registra s kotorogo nachinaetsa chtenie
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));  // Peredaem adress nachalnogo registra

  TxBuffer = 0;                                                                             // Ukazivaem kolichestvo registrov iz kotorih budut schitivatsa dannie (zapisivaetsa 1 + TO KOLICHESTVO KOTOROE UKAZIVAEM, V DANOM SLUCHAE POLUCHAETSA 1+0=1)
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));  // Peredaem kolichestvo registrov

  // ������
  TxBuffer = value;                                                                         // Ukazivaem kolichestvo registrov iz kotorih budut schitivatsa dannie (zapisivaetsa 1 + TO KOLICHESTVO KOTOROE UKAZIVAEM, V DANOM SLUCHAE POLUCHAETSA 1+0=1)
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));  // Peredaem kolichestvo registrov

  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);
}




//-------------------------------------
// ����������
//-------------------------------------

//-------------------------------------
// SELFCAL: Self Offset and Gain Calibration
//
// Description: Performs a self offset and self gain calibration. 
//  The Offset Calibration Register (OFC) and Full-Scale Calibration Register (FSC) are updated after this operation. 
//  DRDY goes high at the beginning of the calibration. It goes low after the calibration completes and settled data is ready. 
//  Do not send additional commands after issuing this command until DRDY goes low indicating that the calibration is complete.
//
// Note:  For the best performance, it is strongly recommended to perform an additional self-calibration 
//  by issuing the SELFCAL command after the power supplies and voltage reference have had time to settle to their final values.
//
void ADS1256_Command_SelfCalibration(void)
{
  // ���������� �������� �� ���������� ���
  while(ADS1256_DRDY_BUSY());
  
  // �����
  uint8_t TxBuffer = ADS1256_COMMAND_SELFCAL;
  // �������
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);

  // ���������� �������� �� ���������� ���  (���� ���������� ��������������)
  while(ADS1256_DRDY_BUSY());
}



//-------------------------------------
// SELFOCAL: Self Offset Calibration
void ADS1256_Command_SelfOffsetCalibration(void)
{
  // ���������� �������� �� ���������� ���
  while(ADS1256_DRDY_BUSY());
  
  // �����
  uint8_t TxBuffer = ADS1256_COMMAND_SELFOCAL;
  // �������
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);

  // ���������� �������� �� ���������� ���  (���� ���������� ��������������)
  while(ADS1256_DRDY_BUSY());
}



//-------------------------------------
// SELFGCAL: Self Gain Calibration
void ADS1256_Command_SelfGainCalibration(void)
{
  // ���������� �������� �� ���������� ���
  while(ADS1256_DRDY_BUSY());
  
  // �����
  uint8_t TxBuffer = ADS1256_COMMAND_SELFGCAL;
  // �������
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);

  // ���������� �������� �� ���������� ���  (���� ���������� ��������������)
  while(ADS1256_DRDY_BUSY());
}



//-------------------------------------
// SYSOCAL: System Offset Calibration
void ADS1256_Command_SystemOffsetCalibration(void)
{
  // ���������� �������� �� ���������� ���
  while(ADS1256_DRDY_BUSY());
  
  // �����
  uint8_t TxBuffer = ADS1256_COMMAND_SYSOCAL;
  // �������
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);

  // ���������� �������� �� ���������� ���  (���� ���������� ��������������)
  while(ADS1256_DRDY_BUSY());
}



//-------------------------------------
// SYSGCAL: System Gain Calibration
void ADS1256_Command_SystemGainCalibration(void)
{
  // ���������� �������� �� ���������� ���
  while(ADS1256_DRDY_BUSY());
  
  // �����
  uint8_t TxBuffer = ADS1256_COMMAND_SYSGCAL;
  // �������
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);

  // ���������� �������� �� ���������� ���  (���� ���������� ��������������)
  while(ADS1256_DRDY_BUSY());
}




//-------------------------------------
// ������������� (��� ��������� ������� � ����� �������� ������ �������)
//-------------------------------------

//-------------------------------------
// SYNC: Synchronize the A/D Conversion
//
// Description: This command synchronizes the A/D conversion. 
//  To use, first shift in the command SYNC...
//  ...Then shift in the command WAKEUP. 
//  Synchronization occurs on the first CLKIN rising edge of WAKEUP command.
//
void ADS1256_Command_Synchronize(void)
{
  // ���������� �������� �� ���������� ���
  while(ADS1256_DRDY_BUSY());
  
  // �����
  uint8_t TxBuffer = ADS1256_COMMAND_SYNC;
  // �������
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);
}



//-------------------------------------
// STANDBY: Standby Mode / One-Shot Mode
//
// Description: This command puts the ADS1255/6 into a low-power Standby mode. 
//  After issuing the STANDBY command, make sure there is no more activity on SCLK while CS is low, as this will interrupt Standby mode. If CS is high, SCLK activity is allowed during Standby mode. 
//  To exit Standby mode, issue the WAKEUP command. This command can also be used to perform single conversions (see One-Shot Mode section).
//  
void ADS1256_Command_Standby(void)
{
  // ���������� �������� �� ���������� ���
  while(ADS1256_DRDY_BUSY());
  
  // �����
  uint8_t TxBuffer = ADS1256_COMMAND_STANDBY;
  // �������
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);
}



//-------------------------------------
// WAKEUP: Complete Synchronization or Exit Standby Mode
// 
// Description: Used in conjunction with the SYNC and STANDBY commands. 
//  Two values (all zeros or all ones) are available for this command.
//
void ADS1256_Command_WakeUp(void)
{
  // ���������� �������� �� ���������� ���
  //while(ADS1256_DRDY_BUSY());     //�����, ������ ������� ����������, �� ��������� deadlock - ���������� � ����� ��������� ��� ������...
  
  // �����
  uint8_t TxBuffer = ADS1256_COMMAND_WAKEUP0;
  // �������
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));

  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);
}



//-------------------------------------

// ����������� ����������� ����������� (����� ��������������� ������ � �������� ����������)
#define ADS1256_COMMAND_CONVERT() \
  ADS1256_Command_Synchronize();  \
  ADS1256_Command_WakeUp();




//============================================================================
// ��������� ������� ���������� ���  (��������������� API ������, ���������� ��������)
//============================================================================


//-------------------------------------
// ���������� ������� ����������
//-------------------------------------


//-------------------------------------
// ����������� �����: Reset Registers to Default Values

void ADS1256_API_Reset(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //��������� ��������
  ADS1256_Command_Reset();
  ADS1256_CS_OFF();                                                           //��������� ��������
}



//-------------------------------------
// ������� ������� SYNC: Synchronize the A/D Conversion
//  (������������� �����������... �� ���� ������� ����� �� ����������, �� ������� WAKEUP)

void ADS1256_API_Synchronize(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //��������� ��������
  ADS1256_Command_Synchronize();
  ADS1256_CS_OFF();                                                           //��������� ��������
}



//-------------------------------------
// ������� ������� STANDBY: Standby Mode / One-Shot Mode
//  (������� ���, ��������� � ����� ����������� �����������������... ��� ����, ����������� ������� ������������������, �� ��� �� ������������� ��������� ������ �������, ��� ����� ������� SYNC)

void ADS1256_API_Standby(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //��������� ��������
  ADS1256_Command_Standby();
  ADS1256_CS_OFF();                                                           //��������� ��������
}



//-------------------------------------
// ������� ������� WAKEUP: Complete Synchronization or Exit Standby Mode
//  ([��������� ���, ���� ��� � ������ ������]... � ����� ����� �����, ���������� �����������)

void ADS1256_API_WakeUp(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //��������� ��������
  ADS1256_Command_WakeUp();
  ADS1256_CS_OFF();                                                           //��������� ��������
}




//-------------------------------------
// ��������� � �������� STATUS
//-------------------------------------


//-------------------------------------
// ��������� ������������� ����������
// ID3, ID2, ID1, ID0 = Factory Programmed Identification Bits (Read Only) 

uint8_t ADS1256_API_GetDeviceID(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //�������� �������� �� ��������
  value = (value >> ADS1256_STATUS_ID) & ADS1256_STATUS_IDMASK;               //������� ������ ����
  
  ADS1256_CS_OFF();                                                           //��������� ��������
  
  return value;                                                               //��������� �������� �����
}  



//-------------------------------------
// ������� ����� � ������ "������ �������������"
// ORDER = Data Output Bit Order

uint8_t ADS1256_API_GetBitOrderMode(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //�������� �������� �� ��������
  value = (value >> ADS1256_STATUS_ORDER) & 1;                                //������� ������ ����
  
  ADS1256_CS_OFF();                                                           //��������� ��������

  return value;                                                               //��������� �������� ����
}  


void ADS1256_API_SetBitOrderMode(uint8_t bit)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //�������� ������ �������� �� ��������
  
  if(bit)
    value |= (1 << ADS1256_STATUS_ORDER);                                     //���������� ���
  else
    value &= ~(1 << ADS1256_STATUS_ORDER);                                    //����� ���
  
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_STATUS, value);            //��������� ������� � ������� ����� �������� (� ����������� �����)
  
  ADS1256_CS_OFF();                                                           //��������� ��������
}  



//-------------------------------------
// ����� "������������� ��������� ��������� ��������������, ����� ������ ������������� ������ ���"
// ACAL = Auto Calibration
//
// Note: When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes the 
//  PGA   (bits 0-2 of ADCON register), 
//  DR    (bits 7-0 in the DRATE register) or 
//  BUFEN (bit 1 in the STATUS register) values.
//

uint8_t ADS1256_API_GetAutoCalibrationMode(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //�������� �������� �� ��������
  value = (value >> ADS1256_STATUS_ACAL) & 1;                                 //������� ������ ����
  
  ADS1256_CS_OFF();                                                           //��������� ��������

  return value;                                                               //��������� �������� ����
}  


void ADS1256_API_SetAutoCalibrationMode(uint8_t bit)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //�������� ������ �������� �� ��������
  
  if(bit)
    value |= (1 << ADS1256_STATUS_ACAL);                                      //���������� ���
  else
    value &= ~(1 << ADS1256_STATUS_ACAL);                                     //����� ���
  
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_STATUS, value);            //��������� ������� � ������� ����� �������� (� ����������� �����)

  ADS1256_CS_OFF();                                                           //��������� ��������
}  


//���������� ���������� ���������: TRUE ���� �������������� ��������; FALSE ���������.
extern __inline uint8_t ADS1256_API_IfAutoCalibrationOn(void)
{
  return (ADS1256_API_GetAutoCalibrationMode() == ADS1256_STATUS_ACAL_ON);
}



//-------------------------------------
// ���/���� ������� ����������� (����������� �������� �� 10..80���, �� ��������� ������������ �������� �� 0..3V)
// BUFEN = Analog Input Buffer Enable

uint8_t ADS1256_API_GetInputBufferMode(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //�������� �������� �� ��������
  value = (value >> ADS1256_STATUS_BUFEN) & 1;                                //������� ������ ����

  ADS1256_CS_OFF();                                                           //��������� ��������

  return value;                                                               //��������� �������� ����
}  


void ADS1256_API_SetInputBufferMode(uint8_t bit)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //�������� ������ �������� �� ��������
  
  if(bit)
    value |= (1 << ADS1256_STATUS_BUFEN);                                     //���������� ���
  else
    value &= ~(1 << ADS1256_STATUS_BUFEN);                                    //����� ���

  // 1) ������������� �����
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_STATUS, value);            //��������� ������� � ������� ����� �������� (� ����������� �����)

  // 2) ��������� �������������� (��� �����, ��������� �� ������� ����� ���������� ����������� �� ��)
  ADS1256_Command_SelfCalibration();
  
  // 3) ����������� ����������� ����������� (����� ��������������� ������ � �������� ����������)
  ADS1256_COMMAND_CONVERT();
  
  ADS1256_CS_OFF();                                                           //��������� ��������
}  


//���������� ���������� ���������: TRUE ���� ����� �������; FALSE ��������.
extern __inline uint8_t ADS1256_API_IfInputBufferOn(void)
{
  return (ADS1256_API_GetInputBufferMode() == ADS1256_STATUS_BUFEN_ON);
}



//-------------------------------------
// ��������� ������ ���������� ��� � ������ ������ ��� �������� ������� (��� DRDY)
// DRDY = Data Ready (Read Only) = Duplicates the state of the DRDY pin

uint8_t ADS1256_API_GetDataReadyBit(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_STATUS);  //�������� �������� �� ��������
  value = (value >> ADS1256_STATUS_DRDY) & 1;                                 //������� ������ ����

  ADS1256_CS_OFF();                                                           //��������� ��������

  return value;                                                               //��������� �������� ����
}


//���������� ���������� ���������: TRUE ���� ������ ������; FALSE ������ �� ������, ��� �����������.
extern __inline uint8_t ADS1256_API_IfDataReady(void)
{
  //������������� ����������� ������� ����������: 
  //  ������ ��������� ���������� ����������� ���� DRDY;
  //  � ������ ���� ��������� ���� �� ������������, �� ������ ��� DRDY.
  return (ADS1256_DRDY_READY());                                              // (**) ������: ��� �������� DATAC-������, � ��� ���������� ��������� ���� DRDY - ���� ����� ������ ����� ���������� FALSE.
}



//-------------------------------------
// ��������������� �������: �������� ���������� ��� � ������ ������ ��� �������� ������� (������ DRDY)
void ADS1256_API_WaitDRDY(void)
{
  // ������, ��������� � ���� �������: ���� ������ ���������� ���� �� �������� �����, �� SPI ��������� �������� �� �����!
  #if !(defined(ADS1256_DRDY_PORT) && defined (ADS1256_DRDY_PIN))
    ADS1256_CS_ON();                                                          //��������� ��������
  #endif
  
  // ���������� �������� �� ���������� ���
  while(ADS1256_DRDY_BUSY());                                                 // (**) ������: ��� �������� DATAC-������, � ��� ���������� ��������� ���� DRDY - ����� ����� ���� (����������� �������� �� �����), ������������ � ��������� ��������� �� ���������� (��� �����), �� � "���������� ���" �� ���������. 
                                                                              // �������: ��� ���������� ���� DRDY, ��������� ������������ DATAC-����� (�� �������� �� �����)!
  #if !(defined(ADS1256_DRDY_PORT) && defined (ADS1256_DRDY_PIN))
    ADS1256_CS_OFF();                                                         //��������� ��������
  #endif
}  




//-------------------------------------
// ��������� �������������� ������� ������� (������� MUX)
//-------------------------------------

// ���������: 
//  ��� ADS1255, ����� ������ = [0..1]
//  ��� ADS1256, ����� ������ = [0..7]


//-------------------------------------
// ���������� ������� ��� "�������������� ���������", ��� ��������� �������������� ����������
extern __inline void ADS1256_API_SetMultiplexerSingleP(uint8_t PositiveChannelNumber) 
{
  ADS1256_API_SetMultiplexerDifferential(PositiveChannelNumber, 0xFF);
}


//-------------------------------------
// ���������� ������� ��� "�������������� ���������", ��� ��������� �������������� ����������
extern __inline void ADS1256_API_SetMultiplexerSingleN(uint8_t NegativeChannelNumber) 
{
  ADS1256_API_SetMultiplexerDifferential(0xFF, NegativeChannelNumber);
}


//-------------------------------------
// ���������� ������� ��� "����������������� ���������", ��� ��������� ������� ����������
void ADS1256_API_SetMultiplexerDifferential(uint8_t PositiveChannelNumber, uint8_t NegativeChannelNumber) 
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  uint8_t value;
  uint8_t muxp;
  uint8_t muxn;

  // ��������� ��������, ����������� � �������
  switch (PositiveChannelNumber) 
  {
    case 0:
      muxp = ADS1256_MUXP_AIN0;
      break;
    case 1:
      muxp = ADS1256_MUXP_AIN1;
      break;
    case 2:
      muxp = ADS1256_MUXP_AIN2;
      break;
    case 3:
      muxp = ADS1256_MUXP_AIN3;
      break;
    case 4:
      muxp = ADS1256_MUXP_AIN4;
      break;
    case 5:
      muxp = ADS1256_MUXP_AIN5;
      break;
    case 6:
      muxp = ADS1256_MUXP_AIN6;
      break;
    case 7:
      muxp = ADS1256_MUXP_AIN7;
      break;
    default:
      muxp = ADS1256_MUXP_AINCOM;
  }

  switch (NegativeChannelNumber) 
  {
    case 0:
      muxn = ADS1256_MUXN_AIN0;
      break;
    case 1:
      muxn = ADS1256_MUXN_AIN1;
      break;
    case 2:
      muxn = ADS1256_MUXN_AIN2;
      break;
    case 3:
      muxn = ADS1256_MUXN_AIN3;
      break;
    case 4:
      muxn = ADS1256_MUXN_AIN4;
      break;
    case 5:
      muxn = ADS1256_MUXN_AIN5;
      break;
    case 6:
      muxn = ADS1256_MUXN_AIN6;
      break;
    case 7:
      muxn = ADS1256_MUXN_AIN7;
      break;
    default:
      muxn = ADS1256_MUXN_AINCOM;
  }

  value = muxp | muxn;
  
  
  ADS1256_CS_ON();                                                            //��������� ��������
  
  // 1) ������������� ������������� �� ����� ������
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_MUX, value);
  
  // 2) ����������� ����������� ����������� (����� ��������������� ������ � �������� ����������)
  ADS1256_COMMAND_CONVERT();
  
  ADS1256_CS_OFF();                                                           //��������� ��������
}




//-------------------------------------
// ��������� � �������� ADCON
//-------------------------------------

//-------------------------------------
// ��������� "����������� �������� ������� �� �����" (��� ������������ ������� ���������)
// CLK1, CLK0 = D0/CLKOUT Clock Out Rate Setting
//
// ����������: �� ������ ����� ������� GPIO �� Output, ����� ������������� ���� ����� ������...

uint8_t ADS1256_API_GetClockOutMode(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);   //�������� �������� �� ��������
  value = (value >> ADS1256_ADCON_CLK) & ADS1256_ADCON_CLKMASK;               //������� ������ ����

  ADS1256_CS_OFF();                                                           //��������� ��������

  return value;                                                               //��������� �������� �����
}


void ADS1256_API_SetClockOutMode(uint8_t bits)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);   //�������� ������ �������� �� ��������
  
  // ������� ������� ������ ������� �����, �� �����
  value &= ~(ADS1256_ADCON_CLKMASK << ADS1256_ADCON_CLK);
  
  // ����� ���������� ����� �������� �����
  value |= ((bits & ADS1256_ADCON_CLKMASK) << ADS1256_ADCON_CLK);
  
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_ADCON, value);             //��������� ������� � ������� ����� �������� (� ������������ ������)

  ADS1256_CS_OFF();                                                           //��������� ��������
}


//���������� ���������� ���������: TRUE ���� ���������� �������� ������� ������� (DIO0=CLKOUT); FALSE �������� (DIO0 ������� GPIO).
extern __inline uint8_t ADS1256_API_IfClockOutOn(void)
{
  return (ADS1256_API_GetClockOutMode() != ADS1256_ADCON_CLK_OFF);
}




//-------------------------------------
// ��������� "���������� ����, ������������ ��� ����������� ������� �������, ������������� �� ����� ���"
// SDCS = Sensor Detection Current Sources
//
// ������� �������� ����� �����: ���� ���� �������� ���������� - ������ ������������ ������ ������������. 
//  � ����� ���������� ���� ������������� �������, �� ���������� ����������, �� ���������� �������� ������������ ���� (������� ������� ���� �������������).
//  �.�. ��������� ������� ���� - ����� ��������� ���������� ��� - ������� �����... � �����, ���������� "�������� ����" � ��� �������� "������� ������" � �������.
//

uint8_t ADS1256_API_GetSensorDetectionCurrentSources(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);   //�������� �������� �� ��������
  value = (value >> ADS1256_ADCON_SDCS) & ADS1256_ADCON_SDCSMASK;             //������� ������ ����

  ADS1256_CS_OFF();                                                           //��������� ��������

  return value;                                                               //��������� �������� �����
}


void ADS1256_API_SetSensorDetectionCurrentSources(uint8_t bits)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);   //�������� ������ �������� �� ��������
  
  // ������� ������� ������ ������� �����, �� �����
  value &= ~(ADS1256_ADCON_SDCSMASK << ADS1256_ADCON_SDCS);
  
  // ����� ���������� ����� �������� �����
  value |= ((bits & ADS1256_ADCON_SDCSMASK) << ADS1256_ADCON_SDCS);
  
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_ADCON, value);             //��������� ������� � ������� ����� �������� (� ������������ ������)

  ADS1256_CS_OFF();                                                           //��������� ��������
}


//���������� ���������� ���������: TRUE ���� ������� ���� ��������; FALSE �������� �������� (���������� �����).
extern __inline uint8_t ADS1256_API_IfSensorDetectionCurrentSourcesOn(void)
{
  return (ADS1256_API_GetSensorDetectionCurrentSources() != ADS1256_ADCON_SDCS_OFF);
}




//-------------------------------------
// ��������� "��������� � ��������������� �������������" (PGA) ��� �������� �������
// PGA = Programmable Gain Amplifier Setting

uint8_t ADS1256_API_GetProgrammableGainAmplifierMode(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);   //�������� �������� �� ��������
  value = (value >> ADS1256_ADCON_PGA) & ADS1256_ADCON_PGAMASK;               //������� ������ ����

  ADS1256_CS_OFF();                                                           //��������� ��������

  return value;                                                               //��������� �������� �����
}  


void ADS1256_API_SetProgrammableGainAmplifierMode(uint8_t bits)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_ADCON);   //�������� ������ �������� �� ��������
  
  // ������� ������� ������ ������� �����, �� �����
  value &= ~(ADS1256_ADCON_PGAMASK << ADS1256_ADCON_PGA);
  
  // ����� ���������� ����� �������� �����
  value |= ((bits & ADS1256_ADCON_PGAMASK) << ADS1256_ADCON_PGA);
  
  // 1) ������������� �����
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_ADCON, value);             //��������� ������� � ������� ����� �������� (� ������������ ������)

  // 2) ��������� �������������� (��� �����, ��������� ������������� ���������� ������������� �������� �������)
  ADS1256_Command_SelfCalibration();
  
  // 3) ����������� ����������� ����������� (����� ��������������� ������ � �������� ����������)
  ADS1256_COMMAND_CONVERT();
  
  ADS1256_CS_OFF();                                                           //��������� ��������
}  




//-------------------------------------
// ��������� ������ �����-������ (������� I/O: GPIO)
//-------------------------------------

// TODO: � ���� �������� ������ ������� �� �������������, ������� ���� ������� ���������� ���� "��������� ������ ������ GPIO"...
// ������: �� ���������, ��� ����� GPIO ��������� �� Input, ������� ������� �� ���������.




//-------------------------------------
// ��������� ������� ������������� ��� (���������� ������� � �������) (������� DRATE)
//-------------------------------------

//-------------------------------------
// ��������������� �������: ���������� �������� ������� �������������� ����� �� ��������� ������� ������������� (������������ SPS -> DRATE)
uint8_t ADS1256_Convert_SPS2DRATE(uint16_t SPS)
{
  if      (SPS <= 3)
    // ������: �����, �� ����� ����, ������� ������������ ������� ����� = 2.5 SPS ... �� � �� ��������� ����� ������� (������� ���������� � ��������� ������), �� � �� ����� ��� ��� - ������� ��������� �� �����!
    return ADS1256_DRATE_2_5SPS;
  else if (SPS <= 5)
    return ADS1256_DRATE_5SPS;
  else if (SPS <= 10)
    return ADS1256_DRATE_10SPS;
  else if (SPS <= 15)
    return ADS1256_DRATE_15SPS;
  else if (SPS <= 25)
    return ADS1256_DRATE_25SPS;
  else if (SPS <= 30)
    return ADS1256_DRATE_30SPS;
  else if (SPS <= 50)
    return ADS1256_DRATE_50SPS;
  else if (SPS <= 60)
    return ADS1256_DRATE_60SPS;
  else if (SPS <= 100)
    return ADS1256_DRATE_100SPS;
  else if (SPS <= 500)
    return ADS1256_DRATE_500SPS;
  else if (SPS <= 1000)
    return ADS1256_DRATE_1000SPS;
  else if (SPS <= 2000)
    return ADS1256_DRATE_2000SPS;
  else if (SPS <= 3750)
    return ADS1256_DRATE_3750SPS;
  else if (SPS <= 7500)
    return ADS1256_DRATE_7500SPS;
  else if (SPS <= 15000)
    return ADS1256_DRATE_15000SPS;
  else
    return ADS1256_DRATE_30000SPS;
}


//-------------------------------------
// ��������������� �������: �������������� ������� ������������� �� ���������� ��������� �������� �������� (������������ DRATE -> SPS)
uint16_t ADS1256_Convert_DRATE2SPS(uint8_t DRATE)
{
  switch(DRATE)
  {
    case ADS1256_DRATE_30000SPS:
      return 30000;
    case ADS1256_DRATE_15000SPS:
      return 15000;
    case ADS1256_DRATE_7500SPS:
      return 7500;
    case ADS1256_DRATE_3750SPS:
      return 3750;
    case ADS1256_DRATE_2000SPS:
      return 2000;
    case ADS1256_DRATE_1000SPS:
      return 1000;
    case ADS1256_DRATE_500SPS:
      return 500;
    case ADS1256_DRATE_100SPS:
      return 100;
    case ADS1256_DRATE_60SPS:
      return 60;
    case ADS1256_DRATE_50SPS:
      return 50;
    case ADS1256_DRATE_30SPS:
      return 30;
    case ADS1256_DRATE_25SPS:
      return 25;
    case ADS1256_DRATE_15SPS:
      return 15;
    case ADS1256_DRATE_10SPS:
      return 10;
    case ADS1256_DRATE_5SPS:
      return 5;
    case ADS1256_DRATE_2_5SPS:
      // ������: �����, �� ����� ����, ������� ������������ ������� ����� = 2.5 SPS ... �� � �� ��������� ����� ������� (������� ���������� � ��������� ������), �� � �� ����� ��� ��� - ������� ��������� �� �����!
      return 3;
    default:
      // �� ������ ������� - �������� ����������� "����������� ���������� ��� ������ ���������� ������������" ��������
      return ADS1256_F_DATA;
  }
}



//-------------------------------------
// ������� �������������� ������������������:
// 1) ������������� ������� DRATE �� ����� ������� �������������
// 2) ��������� �������������� (��� �����, ��������� ������������� ���������� �������� ������ �����).
// 3) ����������� ����������� ����������� (����� ��������������� ������ � �������� ����������)
void ADS1256_API_SetDataRateMode(uint8_t DRATE)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  // ��������: ������ ������������� ������� ���� ����������� ���������� ��� ������ ���������� ������������ (����������� ������)
  if(ADS1256_Convert_DRATE2SPS(DRATE) > ADS1256_F_DATA) 
    return;

  ADS1256_CS_ON();                                                            //��������� ��������

  // 1) ������������� ������� DRATE �� ����� ������� �������������
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_DRATE, DRATE);

  // 2) ��������� �������������� (��� �����, ��������� ������������� ���������� �������� ������ �����)
  ADS1256_Command_SelfCalibration();
  
  // 3) ����������� ����������� ����������� (����� ��������������� ������ � �������� ����������)
  ADS1256_COMMAND_CONVERT();

  ADS1256_CS_OFF();                                                           //��������� ��������
}


//-------------------------------------
// ������������ ��������������� �������: 

// ���������� ����� ������� ������������� � SPS = [2.5 .. 30000]
//  ����������: ��� ������������ �� ����� ��������, � ����� ����� ���������� ��������, ���������� ���������� ��������� �������... � ��� ������������ ���������� ���������, 
//  ������������� ��������� �������� �� ������������ ����: SPS = 30000, 15000, 7500, 3750, 2000, 1000, 500, 100, 60, 50, 30, 25, 15, 10, 5, 3 (2.5)
extern __inline void ADS1256_API_SetDataRateModeSPS(uint16_t SPS)
{
  ADS1256_API_SetDataRateMode( ADS1256_Convert_SPS2DRATE(SPS) );
}


// ���������� ������� ������� ������������� (� �������� SPS)
extern __inline uint16_t ADS1256_API_GetDataRateModeSPS(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                            //��������� ��������

  uint8_t value = ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_DRATE);   //�������� �������� �� ��������

  ADS1256_CS_OFF();                                                           //��������� ��������

  return ADS1256_Convert_DRATE2SPS(value);                                    //����������� ��� ������ � ��������� ���������
}




//-------------------------------------
// ��������� ��������� ������������� ������������� OFCx � FSCx
//-------------------------------------

// CALIBRATION
//  Offset and gain errors can be minimized using the ADS1255/6 onboard calibration circuitry. 
//  Offset errors are corrected with the Offset Calibration (OFC) register and, likewise,
//  full-scale errors are corrected with the Full-Scale Calibration (FSC) register.


//-------------------------------------
// ��������� �������� ��������� OFC (Offset Calibration Coefficient)
uint32_t ADS1256_API_GetOffsetCalibrationCoefficient(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                                    //��������� ��������

  uint32_t result = 0;
  result  |= ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_OFC2);                 //�������� �������� �� �������� OFC2 (�������)
  result <<= 8;                                                                       //����� ������� ����������� �����, ��������� ��������� 8-��� � ������� �������
  result  |= ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_OFC1);                 //�������� �������� �� �������� OFC1
  result <<= 8;                                                                       //����� ������� ����������� �����, ��������� ��������� 8-��� � ������� �������
  result  |= ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_OFC0);                 //�������� �������� �� �������� OFC0 (�������)

  ADS1256_CS_OFF();                                                                   //��������� ��������

  return result;
}


//-------------------------------------
// ���������� �������� � �������� OFC (Offset Calibration Coefficient)
void ADS1256_API_SetOffsetCalibrationCoefficient(uint32_t value)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                                    //��������� ��������

  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_OFC0, (uint8_t) (value & 0xFF));   //�������� �������� � ������� OFC0 (�������)
  value >>= 8;                                                                        //����� ������ ��������, ��������� ��������� ���� �� ������� �������� �� ������� ������ 8-���
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_OFC1, (uint8_t) (value & 0xFF));   //�������� �������� � ������� OFC1
  value >>= 8;                                                                        //����� ������ ��������, ��������� ��������� ���� �� ������� �������� �� ������� ������ 8-���
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_OFC2, (uint8_t) (value & 0xFF));   //�������� �������� � ������� OFC2 (�������)

  ADS1256_CS_OFF();                                                                   //��������� ��������
}



//-------------------------------------
// ��������� �������� ��������� FSC (Fullscale Callibration Coefficient)
uint32_t ADS1256_API_GetFullscaleCallibrationCoefficient(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                                    //��������� ��������

  uint32_t result = 0;
  result  |= ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_FSC2);                 //�������� �������� �� �������� FSC2 (�������)
  result <<= 8;                                                                       //����� ������� ����������� �����, ��������� ��������� 8-��� � ������� �������
  result  |= ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_FSC1);                 //�������� �������� �� �������� FSC1
  result <<= 8;                                                                       //����� ������� ����������� �����, ��������� ��������� 8-��� � ������� �������
  result  |= ADS1256_Command_ReadFromRegister(ADS1256_REGISTER_FSC0);                 //�������� �������� �� �������� FSC0 (�������)

  ADS1256_CS_OFF();                                                                   //��������� ��������

  return result;
}


//-------------------------------------
// ���������� �������� � �������� FSC (Fullscale Callibration Coefficient)
void ADS1256_API_SetFullscaleCallibrationCoefficient(uint32_t value)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                                    //��������� ��������

  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_FSC0, (uint8_t) (value & 0xFF));   //�������� �������� � ������� FSC0 (�������)
  value >>= 8;                                                                        //����� ������ ��������, ��������� ��������� ���� �� ������� �������� �� ������� ������ 8-���
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_FSC1, (uint8_t) (value & 0xFF));   //�������� �������� � ������� FSC1
  value >>= 8;                                                                        //����� ������ ��������, ��������� ��������� ���� �� ������� �������� �� ������� ������ 8-���
  ADS1256_Command_WriteToRegister(ADS1256_REGISTER_FSC2, (uint8_t) (value & 0xFF));   //�������� �������� � ������� FSC2 (�������)

  ADS1256_CS_OFF();                                                                   //��������� ��������
}


//-------------------------------------
// ��������� �������������� 
// (*) Note: ����� ��������� ������� BUFEN, PGA, DRATE - ������������ ������������� ��������� "Self Calibration"...
void ADS1256_API_DoSelfCalibration(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return;

  ADS1256_CS_ON();                                                            //��������� ��������

  // 1) ��������� �������������� (��� �����, ��������� ������������� ���������� �������� ������ �����)
  ADS1256_Command_SelfCalibration();
  
  // 2) ����������� ����������� ����������� (����� ��������������� ������ � �������� ����������)
  ADS1256_COMMAND_CONVERT();

  ADS1256_CS_OFF();                                                           //��������� ��������
}




//============================================================================
// ��������� ������
//============================================================================

//-------------------------------------
// ������������� ���
void ADS1256_Init(void)
{
  //HAL_Delay(100);
  //ADS1256_LOG_DUMPREGISTERS("ADS1256_Init: ����� �������������� ���");
  //HAL_Delay(1000);
  

  // ������� RESET: Reset Registers to Default Values
  ADS1256_API_Reset();

  //ADS1256_LOG_DUMPREGISTERS("ADS1256_Init: �������� ��� ����� RESET");
  //HAL_Delay(1000);
  
  
  // ���������� ������� ����� � ������ "������ �������������" (��� ORDER � �������� STATUS) = MSB (��-���������)
  ADS1256_API_SetBitOrderMode( ADS1256_STATUS_ORDER_MSB );

  // ��������� "����������� �������� ������� �� �����" (��� ������������ ������� ���������) (���� CLKx �������� ADCON)
  //  ������������� ���������, ���� �� �� ������������...
  ADS1256_API_SetClockOutMode( ADS1256_ADCON_CLK_OFF );
  
  // ��������� �������������� ������� ������� (������� MUX)
  ADS1256_API_SetMultiplexerSingleP(0);   //����� ������ AINPx = [0..7]

  
  // (*) �������� ������� ����������� (����������� �������� �� 10..80���, �� ��������� ������������ �������� �� 0..3V) (��� BUFEN � �������� STATUS)
  ADS1256_API_SetInputBufferMode( ADS1256_STATUS_BUFEN_ON );              //DEBUG: ����� �������� � �������� �������
  
  // (*) ��������� "��������� � ��������������� �������������" ��� �������� ������� (���� PGAx �������� ADCON)
  ADS1256_API_SetProgrammableGainAmplifierMode( ADS1256_ADCON_PGA_2 );    //DEBUG: ����� �������� � �������� �������

  // (*) ��������� ������� ������������� ��� (���������� ������� � �������) (������� DRATE)
  //  ���������� �������� �������� DRATE, ��� ��������������� �������� ������������� SPS = 30000, 15000, 7500, 3750, 2000, 1000, 500, 100, 60, 50, 30, 25, 15, 10, 5, 2.5
  ADS1256_API_SetDataRateModeSPS( ADS1256_F_DATA );
  //ADS1256_API_SetDataRateModeSPS( 2000 );  //DEBUG
  //ADS1256_API_SetDataRateModeSPS( 15 );   //DEBUG: ��� ���������� ���� = 16, ����� ~1 SPS (���� ���������, ��� ����� �� 1��)

  // (*) ����������: ��� ������ ������������� ������ (BUFEN,PGA,SPS) - � ���� ��������������� API �������, �����, ������ ����������� "��������������"...
  ADS1256_API_DoSelfCalibration();    //(����, ��� ��� � �� ���������)
  
  // �������� ����� "��������������� ������� ���������� ����� ������ ������������� ������ ���" (��� ACAL � �������� STATUS)
  //  ������: ��� ������� ������ �� ������, ���� ���������������� ��������� ��� ����� �������������� ��� �����-�� �������� ������ API ������� ����� ��������.
  //  ������ ��� ��������������� API-������� (*) ����� � ���� ������� ������ "��������� ��������������"...
  ADS1256_API_SetAutoCalibrationMode( ADS1256_STATUS_ACAL_ON );


  //HAL_Delay(100);
  //ADS1256_LOG_DUMPREGISTERS("ADS1256_Init: ����� ������������� ���");
  //HAL_Delay(1000);
}



//-------------------------------------
// DEBUG: ������������ ���
void ADS1256_Test(void)
{
  HAL_Delay(100);
  ADS1256_LOG_DUMPREGISTERS("ADS1256_Test");
  HAL_Delay(1000);
}




//============================================================================
// ��������� ���������: ���������� ������ ������ ����������� ���
//============================================================================


//-------------------------------------
// ������ �� ��������� ���������� ���������� �������������� (�������������� �����)
int32_t ADS1256_Command_ReadData(void)
{
  // ���������� �������� �� ���������� ���
  while(ADS1256_DRDY_BUSY());
  // �����: �����, ���� ����������, ��� ������ ���������� ������ ������.
  // ����������: � datasheet ���� ������ ����������� ������� �������� ������������� ���������� ������� - 
  //  ��� ������� ����� ������ ����������� ������, � ���� ����������/��������� ��������� �� ������ ������ (���� ~DRDY=1), 
  //  �� ���������� ����� ������ ������ (�� ����� <T18, ��. Table 13, page 20) �������� ��������� ����������� ������ (���� �� �� ��� ������� ����� �������)... 
  //  �� �� �� ����� ������ � ��� �����!
  
  
  // ������
  uint8_t TxBuffer;                                                                         //Buffer dlya otpravlyaemih dannih
  uint8_t RxBuffer;                                                                         //Buffer dlya prinimaemih dannih

  // ������
  TxBuffer = ADS1256_COMMAND_RDATA;                                                         //������� "������ ������ ���������"
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));  

  // ������������ ��������, ���� ���������� ��� ���������� �����
  delay_us(ADS1256_DELAY_T6_US);
  
  
  // �������� ���������� - �������� �����! ����������: �������� ���������� ������ ��������� �����������/����������� �������� � ���� ������, �� ������.
  int32_t value;

  // �����
  value = 0;
  for(uint8_t i = 0; i < 3; i++)                                                            //���� �� ���� ������ ����������:
  {
    value <<= 8;                                                                            //��������� ���������� ����� � ������� �������.
    RxBuffer = 0;
    assert_param(HAL_OK == HAL_SPI_Receive(&SPI_ADC_HANDLE, &RxBuffer, 1, HAL_IO_TIMEOUT)); //�������� ��������� ����...
    value |= RxBuffer;                                                                      //���������� ��� � ������� �������� �����.
  }
  
  // ���������: ��� ������������� �����, ������� �������� � "�������������� ����" - ������� (���������) ���� ������ ���� �������� "���������". 
  // ��������� �������������� ����� ��������: ������� � ������� ����, �������� (��������) ����� - � ������ ������, 23� ���.
  if(value & (1 << 23))
    value |= 0xFF000000;
  
  
  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);
  
  return value;
}



//-------------------------------------
// ���������� ��������� �������������� ���������� (��������� � �����: �� 2/SPS ���)
int32_t ADS1256_API_ConvertDataOnce(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                                          //��������� ��������
  
  // ��������� ����������� �����
  ADS1256_COMMAND_CONVERT();
  
  // �������� ���������
  int32_t result = ADS1256_Command_ReadData();
  
  ADS1256_CS_OFF();                                                                         //��������� ��������

  return result;
}



//-------------------------------------
// ������� ��������� ��������� �����������, �� ��������� ������ ������ (������������ ������)
int32_t ADS1256_API_ReadLastData(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  if(ADS1256_API_IfDataContinuousMode())
    return ADS1256_STATUS_DATAC_ERROR;

  ADS1256_CS_ON();                                                                          //��������� ��������
  
  // �������� ���������
  int32_t result = ADS1256_Command_ReadData();
  
  ADS1256_CS_OFF();                                                                         //��������� ��������

  return result;
}




//============================================================================
// ��������� �����������: ����������� � ������������� ������ ������ ����������� ���, � ������� ������������ ����������
//============================================================================


// ���� "����� ��������� ����������� ������ �������"
volatile static uint8_t ADS1256_DATAC_Active = 0;

// ���� "������ �� ��������� ������ ��������� �����������" 
//  (��������������� ���������� API-������� ADS1256_API_StopDATAC
//  � ����� �������������� �������� ��������� � ������������ ����������)
volatile static uint8_t ADS1256_DATAC_RequestToStop = 0;


// ��������������� ���������� ��� ��������� ������ �� ���, ����� ����������� ����������  (�������� � ���������� ������������ ����, ����� ���� �������� �� ������ �������)
#define         ADS1256_DATA_FRAME_SIZE         3                           /* ����� ���� � ������� ������ ��� (3 ����� = 24 ����) */
volatile static uint8_t ADS1256_DATAC_TxBuffer[ ADS1256_DATA_FRAME_SIZE ];  // ����� ��������
volatile static uint8_t ADS1256_DATAC_RxBuffer[ ADS1256_DATA_FRAME_SIZE ];  // ����� ������


// ��������� �� callback-������� ��� �����������/����� ���������� ���������� ����������� ��� (������������ � ����������� ������)
//  �����: ������ ������� ������ ���� ������������� � ����� �����, ��������� ����������� �� ����������� �����������!
volatile static TDataRegistrator ADS1256_DataRegistrator = 0;


// ��� ������ "��������� �����������" (DATAC): 
//  ���� SPI ��������? ��� ��-�� ������ � ���������� (������� ������� ������� �������������, �� ��������� SPI)
//  ����� ����� ����� ����� ������ (���� ������� DRDY ������� ������), ��� ����� ������� ���������� ��������� �����������?
//  �� SPI-������ ����� �������, ����� ��������� ���� �� �������!
//
// ������������������ SPI, � ������ DATAC: ���������� ������ ������� DRDY, ������� ����������, ������ ��� ������������� �������� SPI:
//  0 = ������� ���������
//  1 = ���������� ����� �� ��� ������ ������ ��������� DRDY, ������ �� ������ ��������� SPI (�� �������������)
//  2 = ���������� �� ������ ���, ���� SPI �� ������������� ��� �� ��� �������... (�������������)
#define  ADS1256_SPI_DATAC_TIMEOUT_COUNT   2



//-------------------------------------
// �������� ��������� �� ������� ����������� (������� ��� ���������� ����� ��� ����������� ���������� - ��������� Wrapper...)
extern __inline TDataRegistrator ADS1256_API_GetDataRegistrator(void)
{
  return ADS1256_DataRegistrator;
}



//-------------------------------------
// ���������� callback-������� ��� �����������/����� ���������� ���������� ����������� ���  (����������� ���������� �� "����������� ���������� SPI" �� ���������� ������� ���������� ���������� �����������)
extern __inline void ADS1256_API_SetDataRegistrator(TDataRegistrator pFunc)
{
  ADS1256_DataRegistrator = pFunc;
}



//-------------------------------------
// ������� ������� ��� "RDATAC: Read Data Continuous" (�������������� �����)
void ADS1256_Command_RunDataContinuousMode(void)
{
  // ���������� �������� �� ���������� ���
  while(ADS1256_DRDY_BUSY());
  
  // ������
  uint8_t TxBuffer;                                                                         //Buffer dlya otpravlyaemih dannih
  // ������
  TxBuffer = ADS1256_COMMAND_RDATAC;                                                        //������� "������ ������ ���������"
  assert_param(HAL_OK == HAL_SPI_Transmit(&SPI_ADC_HANDLE, &TxBuffer, 1, HAL_IO_TIMEOUT));
  
  // ������������ "�������� ����� ���������", ���� ��� ������������� ����� ������ ��� �������� �������
  delay_us(ADS1256_DELAY_T11_US);
}



//-------------------------------------
// ������������ ����� "RDATAC: Read Data Continuous" ���������
//  (��������: ����� ������� ����� ������ ������� ������ API-������� ������� �������� ��� �������� ������! ������, ����� ������� ����� ADS1256_API_StopDataContinuousMode[Synchronous], ��� ��������� ��������� �����������...)
void ADS1256_API_RunDataContinuousMode(void)
{
  // ��������: ���� ��� ��������� � ������ "��������� �����������", �� �� ���������� ��������� �������!
  //  (�����, ��������� ���� "��������� ������ �������"...)
  if(ADS1256_API_IfDataContinuousMode())
    return;
  
  // ��������: ������ ������������ ����� "��������� �����������" � �������������� ������ SPI, �.�. � ������ "1-WIRE SPI" (BIDIMODE=1)!
  //  Datasheet (page 29): "Avoid using the continuous read mode (RDATAC) when DIN and DOUT are connected together."
  if(SPI_ADC_HANDLE.Init.Direction != SPI_DIRECTION_2LINES)
    return;


  // ������� ������� ��� ��� ������������ ��������� ������
  ADS1256_CS_ON();                                                                          //��������� ��������
  ADS1256_Command_RunDataContinuousMode();
  ADS1256_CS_OFF();                                                                         //��������� ��������

  // ���������� ����
  ADS1256_DATAC_RequestToStop = 0;  //�����: ���� ���� ������� ����� �� ��������� ����� ADS1256_DATAC_Active - ��� ������� �������� ����� � ADS1256_API_StopDataContinuousMode().
  ADS1256_DATAC_Active = 1;         //�����: ���� ���� ����������� ������������� ������ ��� ����� ������ ������� RDATAC, ��������� ���� �������� ����������� ����������, ������� ��������� ���������� �������� �������...
}



//-------------------------------------
// ������� ������ ��������� ������ "SDATAC: Stop Read Data Continuous" ����������
//  (��������: ���������� �������� ����� �� - ��� �� ����������� ��������� ������ SDATAC!)
//  (��� ��������������� � �����������)
void ADS1256_API_StopDataContinuousMode(void)
{
  // ��������: ������������ ������ ���� ��� ��� ��������� � ������ "��������� �����������"
  //  (�����: ������ �������� ������ �������������� �������� - �����, ������ �������� � �� ��������� �� ���������� ������� ��������, � ����� ��� �� ��������� ������� - ����!)
  if(ADS1256_API_IfDataContinuousMode())
  {
    ADS1256_DATAC_RequestToStop = 1;
    
    // ��������� ����� �������� (����� ������ ������):
    //  ���� ADS1256_DATAC_RequestToStop ��� ���� ���������� � �����...
    //  � � ������ ����� ������������ ������� If �� �� ��������� ����� ������� - ���� ��������� ���������� SPI � ���������� �������?

    // �������������: ���� ������� ��� �����������, �� ������ ������.
    if(!ADS1256_API_IfDataContinuousMode())
      ADS1256_DATAC_RequestToStop = 0;
  }
}



//-------------------------------------
// ���������� ����� "SDATAC: Stop Read Data Continuous" ���������
//  (����������: ����� ���������� ���� �������, ��� ����� ������ ��������� � ����������!)
void ADS1256_API_StopDataContinuousModeSynchronous(void)
{
  // ������� ������ ��������� ���
  ADS1256_API_StopDataContinuousMode();

  // ���������� �������� �� ����������� ��������� ���
  while(ADS1256_API_IfDataContinuousMode());
}



//-------------------------------------
// ���������� ���������� ���������: TRUE ���� ����� "��������� �����������" ������ �������; FALSE ����� �������� (��� �������� ��� ������������)
extern __inline uint8_t ADS1256_API_IfDataContinuousMode(void)
{
  return (ADS1256_DATAC_Active != 0);
}



//-------------------------------------
// ���������� ���������� EXTI ����� GPIO, �������������� ���������� ���� DRDY, � ������������ �� ������������ "�� �����" �������!
//  (������� ��������� SPI ��� ������� ���������� ���������� �����������)
extern __inline void ADS1256_INTERRUPT_OnDRDY(uint16_t GPIO_Pin)
{
  // ��������: ��� ����������� ������������ ������ � ������ "��������� �����������" ���
  //  �����: ������ ��������� ���������� EXTI ������������� ������ �����, ����� ��������� ����������� �������� �������/������ �� SPI ��� ��������� � ���� ������������!
  if( ADS1256_API_IfDataContinuousMode() && GPIO_Pin == ADS1256_DRDY_PIN )
  {  

    #if defined(ADS1256_SPI_DATAC_TIMEOUT_COUNT) && (ADS1256_SPI_DATAC_TIMEOUT_COUNT != 0)
    // ��������: ���� ���������� �������� ������ SPI ��� ��� �� �����������? ������, ���-�� ����� �� ���...
    if( !(SPI_ADC_HANDLE.State == HAL_SPI_STATE_READY  &&
          SPI_ADC_HANDLE.Lock  == HAL_UNLOCKED         ))
    {
      // ����������: SPI �� ����� ��� �������� ���������� �����! 
      //  ������, ���� ����������� ��������������� ��� "�������� �������������" > "�������� SPI"
      //  ���� ��������� ���� � ����� ������� SPI � �� ����� (� ���� ����� ����, ����� � ������������ ��������� ���������� ���������� ������������ - ����� ������� "����������� ����������"...)
      
      volatile static uint8_t AttemptsCounter = 0;

      AttemptsCounter++;
      if(AttemptsCounter < ADS1256_SPI_DATAC_TIMEOUT_COUNT)
        return;

      AttemptsCounter = 0;

      
      #ifdef  USE_FULL_ASSERT
      printf("DEBUG: ���������� � ADS1256_INTERRUPT_OnDRDY() ���� ������ ������, � SPI �� ����� - ���������...\r\n");
      #endif 

      // ����� ��������� ���� �� �������� - ������� �������, ������� �� �������� ����� ������������� SPI
      assert_param(HAL_OK == HAL_SPI_Abort_IT(&SPI_ADC_HANDLE));    //Abort ongoing transfer (Interrupt mode)
      
      // �����: ��������� �������� �� SPI  (����� ������������� ��������� ����� - �����, ��� ������ �������?)
      ADS1256_CS_OFF();

      // � �� ���� ���� ��� �� �������� �������� � ��� - ����� ������� �� ���� CS...
      return;
    }
    #endif    
    
    
    // (���������: ��� ��! ������ ������, SPI �����... ���������� ������� ���������� ������ ������.)
    
    // ����� �������� ���������������� � ����������� �� ������� ������� �� ���������:
    if(ADS1256_DATAC_RequestToStop != 0)
    {
      // ��������� ��������� �������� - � ����� �������� ������������ ������� ��������...
      for(int i=0; i<ADS1256_DATA_FRAME_SIZE; i++)
        ADS1256_DATAC_TxBuffer[i] = ADS1256_COMMAND_SDATAC;    
    }
    else
    {
      // �����, � ����� �������� ������������ ����������� ������� 
      for(int i=0; i<ADS1256_DATA_FRAME_SIZE; i++)
        ADS1256_DATAC_TxBuffer[i] = 0;      //����������: 0 == ADS1256_COMMAND_WAKEUP0
    }
  
    
    // �����: ��������� �������� �� ���� SPI  (����� ������� ������������ � ���)
    ADS1256_CS_ON();

    // ��������/�������� ����� ������ �� SPI � ����������� ������ (�� �����������)
    assert_param(HAL_OK == HAL_SPI_TransmitReceive_IT(&SPI_ADC_HANDLE, 
                                           (uint8_t*) ADS1256_DATAC_TxBuffer, 
                                           (uint8_t*) ADS1256_DATAC_RxBuffer, 
                                                      ADS1256_DATA_FRAME_SIZE));
  }
}



//-------------------------------------
// ���������� ���������� SPI, ������� ����������� ����� �������� ����� ������ ������ (3 ����).
//  ����������: ������ ������� ������������� ���������� ��� CallBack � HAL-�������� SPI (������� ������ �� ��������� "����������").
extern __inline void ADS1256_INTERRUPT_OnSPI(SPI_HandleTypeDef *hspi)
{
  // ���������: ���������� SPI "������ ��������� ����" ������� - ��������� ��� ���������, ����� ����� ����� ��� ������������� ��������, � ������ ���������� �� �������� � ����� � ���.
  //  � �� ����� ���, ���������� SPI ��� "�������� ������" ������� - ��� ���������, ����� ��������� ���� ��� ������ ��� ��������� �� ������ � ���������� �������, �� �������� ��� ��� ���� (��� �������, �� �����, ��������: "����� ����, ����� �������� ��������� ������")...
  //  ������, ��������������� ������� "�������� ������ ��������� ���������", ����� ���������� ���������� HAL_SPI_TxRxCpltCallback() - ������������ ��� ��������� HAL, ������ ��� �� ������� �� ������������ SPI ������...
  
  
  // ��������: ��� ����������� ������������ ������ � ������ "��������� �����������" ���,
  //  � ������ ��� ����������� ������ SPI, �������������� ��� (���� �� ��������� � ����������������).
  if(ADS1256_API_IfDataContinuousMode() &&
     hspi == &SPI_ADC_HANDLE)
  {
    // (���������: ����� ������ ��������� � "������ ������")
    
    // ������ ��������� �����
    int32_t value = ADS1256_DATAC_RxBuffer[0]<<16 |   //������� ���� (MSB) �������� ������
                    ADS1256_DATAC_RxBuffer[1]<< 8 |   //������� ����
                    ADS1256_DATAC_RxBuffer[2];        //������� ���� (LSB) ���������
    
    // ���������: ��� ������������� �����, ������� �������� � "�������������� ����" - ������� (���������) ���� ������ ���� �������� "���������". 
    // ��������� �������������� ����� ��������: ������� � ������� ����, �������� (��������) ����� - � ������ ������, 23� ���.
    if(value & (1 << 23))
      value |= 0xFF000000;
    
    // ���� ��������� "����������� ������", �� ��������� ��� ���������� ������  (�����, ������ ����������)
    if(ADS1256_DataRegistrator)
      (*ADS1256_DataRegistrator)(value);    
    
    
    // ��������: ���� ��������� ��������� ��������?
    //  ������: ����� �� �� ����� ������������ �������� ����� ADS1256_DATAC_RequestToStop!=0, �� ��������� ����� ��������. 
    //  � �� �� �����, ���� ����� �������� ��� ������������������ �������� "SDATAC", �� ��� ��� �������������� �� ������ � ����������� - ��� ����� �������� ������� ��������...
    if(ADS1256_DATAC_TxBuffer[0] == ADS1256_COMMAND_SDATAC)
    {
      // ���������� �������
      ADS1256_DATAC_Active = 0;
      // � ����� ������ "������ �� ���������"
      ADS1256_DATAC_RequestToStop = 0;
    }
    
    
    // �����: ��������� �������� �� SPI  (�� ���������� ����������� ������������ � ���)
    ADS1256_CS_OFF();
  }
}


