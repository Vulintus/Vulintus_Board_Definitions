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

#ifndef _VARIANT_LINEAR_AUTOPOSITIONER_V2_0_
#define _VARIANT_LINEAR_AUTOPOSITIONER_V2_0_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK	(F_CPU)

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
#define PINS_COUNT           (33u)
#define NUM_DIGITAL_PINS     (33u)
#define NUM_ANALOG_INPUTS    (6u)
#define NUM_ANALOG_OUTPUTS   (0u)
#define analogInputToDigitalPin(p)  ((p < 4u) ? 17u + (p) : (p == 4u) ? 24u : (p == 5u) ? 31u :-1)

/* 
  if (p < 4) {                          (p < 4u) ?
    17 + (p);                           17u + (p)
  }             
  else if (p == 4) {                    : (p == 4u) ?
    24;                                 24u
  }
  else if (p == 5) {                    : (p == 5u) ?
    31;                                 31u
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
#define NEOPIXEL_BUILTIN     (12u)
#define PIN_NEOPIXEL         NEOPIXEL_BUILTIN
#define PIN_NEOPIX           NEOPIXEL_BUILTIN


/*
 * Analog pins
 */
#define PIN_A0               (17u)
#define PIN_A1               (PIN_A0 + 1)
#define PIN_A2               (PIN_A0 + 2)
#define PIN_A3               (PIN_A0 + 3)
#define PIN_A4               (24u)
#define PIN_A4               (31u)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;

#define ADC_RESOLUTION		12

#define PIN_24V_ADC         PIN_A0      //24V power supply voltage monitor.
#define PIN_15V_ADC         PIN_A1      //15V power supply current monitor.
#define PIN_15V_UVLO        PIN_A2      //Under-voltage lock-out level on the 15V power supply.
#define PIN_5V_ADC          PIN_A3      //5V power supply voltage monitor.

#define PIN_SLOT            PIN_A4      //Slot detector.

#define PIN_BOARD_ID        PIN_A5      //Board ID analog reference.

// Other pins
#define PIN_ATN             (28ul)
static const uint8_t ATN = PIN_ATN;

// Display / Status
#define PIN_OLED_RST        (11u)
#define PIN_SPKR            (13u)

// User Input
#define PIN_QT_BTN_U        (14u)
#define PIN_QT_BTN_S        (15u)
#define PIN_QT_BTN_D        (16u)

// Power Control
#define PIN_15V_ON_FLAG     (21u)

// 24VPB
#define PIN_24VPB_CLK_IN    (7u)       
#define PIN_24VPB_CLK_OUT   (8u)   
#define PIN_24VPB_TRG       (9u)
#define PIN_24VPB_BLK       (10u)

// Calibration
#define PIN_VLX_GPIO        (22u)
#define VLX_XSHUT           (23u)

// DRV8434S Stepper Driver
#define PIN_DRV_DIR         (25u)
#define PIN_DRV_STEP        (26u)
#define PIN_DRV_SLP         (27u)
#define PIN_DRV_EN          (28u)
#define PIN_DRV_FLT         (29u)
#define PIN_DRV_CS          (30u)


/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIAL1_RX       (6ul)
#define PIN_SERIAL1_TX       (5ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_2)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_3)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (1u)
#define PIN_SPI_MOSI         (0u)
#define PIN_SPI_SCK          (2u)
#define PERIPH_SPI           sercom1
#define PAD_SPI_TX           SPI_PAD_2_SCK_3
#define PAD_SPI_RX           SERCOM_RX_PAD_0

static const uint8_t SS	  = (26u) ;	// SERCOM4 last PAD is present on A2 but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;
static const uint8_t COPI = PIN_SPI_MOSI ;
static const uint8_t CIPO = PIN_SPI_MISO ;


/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (3u)
#define PIN_WIRE_SCL         (4u)
#define PERIPH_WIRE          sercom3
#define WIRE_IT_HANDLER      SERCOM3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (32ul)
#define PIN_USB_DM          (33ul)
#define PIN_USB_DP          (34ul)
#define PIN_USB_DETECT      (32ul)

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0

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

extern Uart Serial5;
extern Uart Serial1;

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

#endif /* _VARIANT_LINEAR_AUTOPOSITIONER_V2_0_ */