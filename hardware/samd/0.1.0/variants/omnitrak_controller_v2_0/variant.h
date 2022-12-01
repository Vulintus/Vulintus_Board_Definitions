/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_OMNITRAK_CONTROLLER_V2_0_
#define _VARIANT_OMNITRAK_CONTROLLER_V2_0_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610


/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK        (F_CPU)

#define VARIANT_GCLK0_FREQ (F_CPU)
#define VARIANT_GCLK1_FREQ (48000000UL)
#define VARIANT_GCLK2_FREQ (100000000UL)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (97)
#define NUM_DIGITAL_PINS     (94)
#define NUM_ANALOG_INPUTS    (16)
#define NUM_ANALOG_OUTPUTS   (2)
#define analogInputToDigitalPin(p) ((p < 5) ? 43 + (p) : (p < 10) ? 67 + (p) - 5 : (p < 14) ? 82 + (p) - 10 : (p == 14) ? 91 : (p == 15) ? 96 :-1)

/* 
  if (p < 5) {                          (p < 5) ?
    43 + (p);                           43 + (p)
  }             
  else if (p < 10) {                    : (p < 10) ?
    67 + (p) - 5;                       67 + (p) - 5
  }
  else if (p < 14) {                    : (p < 14) ?
    82 + (p) - 10;                      82 + (p) - 10
  }
  else if (p == 18) {                   : (p == 14) ?
    91;                                 91
  }
  else if (p == 18) {                   : (p == 15) ?
    91;                                 96
  }
  else {
    -1;
  }
*/

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LEDs
#define PIN_LED_R           (30)        //Red channel of the RGB LED.
#define PIN_LED_G           (31)        //Green channel of the RGB LED.
#define PIN_LED_B           (32)        //Blue channel of the RGB LED.
#define LED_R               PIN_LED_R
#define LED_G               PIN_LED_G
#define LED_B               PIN_LED_B


/*
 * Analog pins
 */
#define PIN_A0              (43)
#define PIN_A1              (PIN_A0 + 1)
#define PIN_A2              (PIN_A0 + 2)
#define PIN_A3              (PIN_A0 + 3)
#define PIN_A4              (PIN_A0 + 4)
#define PIN_A5              (67)
#define PIN_A6              (PIN_A5 + 1)
#define PIN_A7              (PIN_A5 + 2)
#define PIN_A8              (PIN_A5 + 3)
#define PIN_A9              (PIN_A5 + 4)
#define PIN_A10             (82)
#define PIN_A11             (PIN_A10 + 1)
#define PIN_A12             (PIN_A10 + 2)
#define PIN_A13             (PIN_A10 + 3)
#define PIN_A14             (91)

#define PIN_DAC0            PIN_A10
#define PIN_DAC1            PIN_A11

static const uint8_t A0   = PIN_A0;
static const uint8_t A1   = PIN_A1;
static const uint8_t A2   = PIN_A2;
static const uint8_t A3   = PIN_A3;
static const uint8_t A4   = PIN_A4;
static const uint8_t A5   = PIN_A5;
static const uint8_t A6   = PIN_A6;
static const uint8_t A7   = PIN_A7;
static const uint8_t A8   = PIN_A8;
static const uint8_t A9   = PIN_A9;
static const uint8_t A10  = PIN_A10;
static const uint8_t A11  = PIN_A11;
static const uint8_t A12  = PIN_A12;
static const uint8_t A13  = PIN_A13;
static const uint8_t A14  = PIN_A14;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		  12

#define PIN_V_IN_ADC        PIN_A0      //24V power supply voltage monitor.
#define PIN_I_IN_ADC        PIN_A1      //24V power supply current monitor.
#define PIN_9_UVLO          PIN_A2      //Under-voltage lock-out level on the 9V power supply.
#define PIN_9V_ADC          PIN_A3      //9V power supply voltage monitor.
#define PIN_5V_ADC          PIN_A4      //9V power supply voltage monitor.

#define PIN_P1_IOUT         PIN_A5      //OTMP Port 1 current monitor.
#define PIN_P2_IOUT         PIN_A6      //OTMP Port 2 current monitor.
#define PIN_P3_IOUT         PIN_A7      //OTMP Port 3 current monitor.
#define PIN_P4_IOUT         PIN_A8      //OTMP Port 4 current monitor.
#define PIN_P5_IOUT         PIN_A9      //OTMP Port 5 current monitor.

#define PIN_BNC_OUT_1       PIN_DAC0    //BNC digital-to-analog output #1.
#define PIN_BNC_OUT_2       PIN_DAC1    //BNC digital-to-analog output #2.
#define PIN_BNC_IN_1        PIN_A12     //BNC analog-to-digital input #1.
#define PIN_BNC_IN_2        PIN_A13     //BNC analog-to-digital input #2.

#define PIN_BOARD_ID        PIN_A14     //Board ID analog reference.


// Other pins
#define PIN_ATN             (40)
static const uint8_t ATN = PIN_ATN;

// NINA W102 Module
#define PIN_NINA_RST        (17)
#define PIN_NINA_GPIO_5     (18)
#define PIN_NINA_GPIO_8     (19)
#define PIN_NINA_CS         (20)
#define PIN_NINA_DEBUG      (21)
#define PIN_NINA_ACK        (22)
#define PIN_NINA_BOOT       (23)
#define NINA_GPIO0          PIN_NINA_BOOT
#define NINA_RESETN         PIN_NINA_RST
#define NINA_ACK            PIN_NINA_ACK

// TFT Display
#define PIN_TFT_RST         (26)
#define PIN_TFT_LED         (27)
#define PIN_TFT_DC          (28)
#define PIN_TFT_CS          (29)

// RV-3208-C7 Real-Time Clock
#define PIN_RTC_INT         (33)
#define PIN_RTC_EVI         (34)
#define PIN_RTC_CLKO        (35)

// User Input
#define PIN_ENC_SW          (36)
#define PIN_ENC_A           (37)
#define PIN_ENC_B           (38)
#define PIN_QT_S_BTN        (39)
#define PIN_QT_WHEEL1       (40)
#define PIN_QT_WHEEL2       (41)
#define PIN_QT_WHEEL3       (42)

// Power Control
#define PIN_PWR_SHDN        (48)
#define PIN_I_IN_RESET      (49)
#define PIN_PWR_FAULT       (50)
#define PIN_9V_ON_FLAG      (51)

// OTMP Power Control
#define PIN_P1_24V_EN       (62)
#define PIN_P2_24V_EN       (63)
#define PIN_P3_24V_EN       (64)
#define PIN_P4_24V_EN       (65)
#define PIN_P5_24V_EN       (66)
#define PIN_P1_ALERT        (72)
#define PIN_P2_ALERT        (73)
#define PIN_P3_ALERT        (74)
#define PIN_P4_ALERT        (75)
#define PIN_P5_ALERT        (76)
#define PIN_P1_RST          (77)
#define PIN_P2_RST          (78)
#define PIN_P3_RST          (79)
#define PIN_P4_RST          (80)
#define PIN_P5_RST          (81)

// BNC I/O
#define PIN_BNC_SW_1        (86)       
#define PIN_BNC_SW_2        (87)   
#define PIN_BNC_EN_1        (88)
#define PIN_BNC_EN_2        (89)

// Fan Control
#define PIN_FAN             (90)

/*
 * Serial interfaces
 */

// Serial1, OTMP Port 1 (SERCOM0)
#define PIN_SERIAL1_TX      (52)
#define PIN_SERIAL1_RX      (53)
#define PAD_SERIAL1_TX      (UART_TX_PAD_0)
#define PAD_SERIAL1_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL1		  sercom0

// Serial2, OTMP Port 2 (SERCOM3)
#define PIN_SERIAL2_TX      (54)
#define PIN_SERIAL2_RX      (55)
#define PAD_SERIAL2_TX      (UART_TX_PAD_0)
#define PAD_SERIAL2_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL2		  sercom3

// Serial3, OTMP Port 3 (SERCOM5)
#define PIN_SERIAL3_TX      (56)
#define PIN_SERIAL3_RX      (57)
#define PAD_SERIAL3_TX      (UART_TX_PAD_0)
#define PAD_SERIAL3_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL3		  sercom5

// Serial4, OTMP Port 4 (SERCOM2)
#define PIN_SERIAL4_TX      (58)
#define PIN_SERIAL4_RX      (59)
#define PAD_SERIAL4_TX      (UART_TX_PAD_0)
#define PAD_SERIAL4_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL4		  sercom2

// Serial5, OTMP Port 5 (SERCOM7)
#define PIN_SERIAL5_TX      (60)
#define PIN_SERIAL5_RX      (61)
#define PAD_SERIAL5_TX      (UART_TX_PAD_0)
#define PAD_SERIAL5_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL5		  sercom7

// Serial6, NINA Programming Serial (SERCOM6)
#define PIN_SERIAL6_TX      (25)
#define PIN_SERIAL6_RX      (24)
#define PAD_SERIAL6_TX      (UART_TX_PAD_0)
#define PAD_SERIAL6_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL6		  sercom6

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO        (1)
#define PIN_SPI_MOSI        (0)
#define PIN_SPI_SCK         (2)
#define PERIPH_SPI          sercom1
#define PAD_SPI_TX          SPI_PAD_0_SCK_1
#define PAD_SPI_RX          SERCOM_RX_PAD_3

static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

static const uint8_t COPI = PIN_SPI_MOSI;
static const uint8_t CIPO = PIN_SPI_MISO;


// Needed for SD library
#define PIN_SPI1_MISO       (9)         //SD_DAT0
#define PIN_SPI1_MOSI       (7)         //SD_CMD
#define PIN_SPI1_SCK        (8)         //SD_CLK
#define PIN_SPI1_SS         (12)        //SD_DAT3

#define SDCARD_SPI          SPI1
#define SDCARD_MISO_PIN     PIN_SPI1_MISO
#define SDCARD_MOSI_PIN     PIN_SPI1_MOSI
#define SDCARD_SCK_PIN      PIN_SPI1_SCK
#define SDCARD_SS_PIN       PIN_SPI1_SS

#define PIN_SD_DETECT       (5)         //SD card detection
#define PIN_SD_WP           (6)         //SD card write protection
#define PIN_SD_CMD          (7)         //Command line for SDHC operation.
#define PIN_SD_CLK          (8)         //Clock line for SDHC operation.
#define PIN_SD_DAT0         (9)         //Data 0 line for SDHC operation.
#define PIN_SD_DAT1         (10)        //Data 1 line for SDHC operation.
#define PIN_SD_DAT2         (11)        //Data 2 line for SDHC operation.
#define PIN_SD_DAT3         (12)        //Data 3 line for SDHC operation.

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA        (3)
#define PIN_WIRE_SCL        (4)
#define PERIPH_WIRE         sercom4
#define WIRE_IT_HANDLER     SERCOM4_Handler
#define WIRE_IT_HANDLER_0   SERCOM4_0_Handler
#define WIRE_IT_HANDLER_1   SERCOM4_1_Handler
#define WIRE_IT_HANDLER_2   SERCOM4_2_Handler
#define WIRE_IT_HANDLER_3   SERCOM4_3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_DM          (94)
#define PIN_USB_DP          (95)
#define PIN_USB_HOST_ENABLE (92)
#define PIN_USB_DETECT      (93)
#define USB_DETECT          PIN_USB_DETECT


#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;
extern SERCOM sercom6;
extern SERCOM sercom7;

extern Uart Serial1;
extern Uart Serial2;
extern Uart Serial3;
extern Uart Serial4;
extern Uart Serial5;
extern Uart Serial6;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif /* _VARIANT_OMNITRAK_CONTROLLER_V2_0_ */
