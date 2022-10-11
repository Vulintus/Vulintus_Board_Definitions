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

#ifndef _VARIANT_HABITRAK_V1_0_
#define _VARIANT_HABITRAK_V1_0_

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
#define PINS_COUNT           (71)
#define NUM_DIGITAL_PINS     (49)
#define NUM_ANALOG_INPUTS    (2)
#define NUM_ANALOG_OUTPUTS   (0)
#define analogInputToDigitalPin(p) (69 + (p))

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

/*
 * LEDs
 */
#define PIN_LED_STATUS      (30)
#define PIN_LED_CHG         (31)
#define PIN_LED_U_BTN       (37)
#define PIN_LED_D_BTN       (38)
#define PIN_LED_TFT         (27)
#define LED_STATUS          PIN_LED_STATUS
#define LED_CHG             PIN_LED_CHG
#define LED_U_BTN           PIN_LED_U_BTN
#define LED_D_BTN           PIN_LED_D_BTN
#define LED_BUILTIN         PIN_LED_STATUS

/*
 * Analog pins
 */
#define PIN_A0              (69)
#define PIN_A1              (70)
#define PIN_BOARD_ID        (PIN_A0)
#define PIN_VREF            (PIN_A1)

#define PIN_DAC0            PIN_A0
#define PIN_DAC1            PIN_A1

static const uint8_t A0   = PIN_A0;
static const uint8_t A1   = PIN_A1;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		  12

/*
 * Serial interfaces
 */
// Serial1 (QTouch)
#define PIN_SERIAL1_RX      (6)
#define PIN_SERIAL1_TX      (5)
#define PAD_SERIAL1_TX      (UART_TX_PAD_0)
#define PAD_SERIAL1_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL1		  sercom3

// Serial2 (Bluetooth)
#define PIN_SERIAL2_RX      (11)
#define PIN_SERIAL2_TX      (12)
#define PAD_SERIAL2_TX      (UART_TX_PAD_0)
#define PAD_SERIAL2_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL2		  sercom0


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO        (1)
#define PIN_SPI_MOSI        (0)
#define PIN_SPI_SCK         (2)
#define PERIPH_SPI          sercom2
#define PAD_SPI_TX          SPI_PAD_0_SCK_1
#define PAD_SPI_RX          SERCOM_RX_PAD_3

static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// Needed for SD library
#define PIN_SPI1_MISO       (18)
#define PIN_SPI1_MOSI       (16)
#define PIN_SPI1_SCK        (17)
#define PIN_SPI1_SS         (21)

#define SDCARD_SPI          SPI1
#define SDCARD_MISO_PIN     PIN_SPI1_MISO
#define SDCARD_MOSI_PIN     PIN_SPI1_MOSI
#define SDCARD_SCK_PIN      PIN_SPI1_SCK
#define SDCARD_SS_PIN       PIN_SPI1_SS

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA        (3)
#define PIN_WIRE_SCL        (4)
#define PERIPH_WIRE         sercom7
#define WIRE_IT_HANDLER     SERCOM7_Handler
#define WIRE_IT_HANDLER_0   SERCOM7_0_Handler
#define WIRE_IT_HANDLER_1   SERCOM7_1_Handler
#define WIRE_IT_HANDLER_2   SERCOM7_2_Handler
#define WIRE_IT_HANDLER_3   SERCOM7_3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define PIN_WIRE1_SDA       (13)
#define PIN_WIRE1_SCL       (14)
#define PERIPH_WIRE1        sercom1
#define WIRE1_IT_HANDLER    SERCOM1_Handler
#define WIRE1_IT_HANDLER_0  SERCOM1_0_Handler
#define WIRE1_IT_HANDLER_1  SERCOM1_1_Handler
#define WIRE1_IT_HANDLER_2  SERCOM1_2_Handler
#define WIRE1_IT_HANDLER_3  SERCOM1_3_Handler

static const uint8_t SDA1 = PIN_WIRE1_SDA;
static const uint8_t SCL1 = PIN_WIRE1_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (71)
#define PIN_USB_DM          (23)
#define PIN_USB_DP          (22)

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0

//SDHC Pins
#define PIN_SDHC_CLK	      (17)
#define PIN_SDHC_CMD          (16)
#define PIN_SDHC_DAT0	      (18)
#define PIN_SDHC_DAT1	      (19)
#define PIN_SDHC_DAT2	      (20)
#define PIN_SDHC_DAT3	      (21)

//PCC Pins
#define PIN_PCC_DEN1        (53)
#define PIN_PCC_DEN2        (54)
#define PIN_PCC_CLK         (55)
#define PIN_PCC_XCLK	    (56)
#define PIN_PCC_D0          (59)
#define PIN_PCC_D1          (60)
#define PIN_PCC_D2          (61)
#define PIN_PCC_D3          (62)
#define PIN_PCC_D4          (63)
#define PIN_PCC_D5          (64)
#define PIN_PCC_D6          (65)
#define PIN_PCC_D7          (66)
#define PIN_PCC_D8          (67)
#define PIN_PCC_D9          (68)

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

#endif /* _VARIANT_HABITRAK_V1_0_ */
