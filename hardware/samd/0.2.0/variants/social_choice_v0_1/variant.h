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

#ifndef _VARIANT_SOCIAL_CHOICE_V0_1_
#define _VARIANT_SOCIAL_CHOICE_V0_1_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

// Non-pin definitions specific to this circuit board iteration.
#define CIRCUIT_BOARD_VER   1               // Divide by 10 to match the Eagle design version.
#define STEPPER_DRIVER_MODEL_DRV8434S       // Stepper driver model.
#define STEPPER_DRIVER_CURRENT_MAX		2500  // Maximum possible stepper coil current, in mA


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
#define PINS_COUNT           (23u)
#define NUM_DIGITAL_PINS     (23u)
#define NUM_ANALOG_INPUTS    (3u)
#define NUM_ANALOG_OUTPUTS   (0u)
#define analogInputToDigitalPin(p)  ((p < 2u) ? 11u + (p) : (p == 2u) ? 21u : -1)

/* 
  if (p < 2) {                          (p < 2u) ?
    11 + (p));                          11u + (p)
  }             
  else if (p == 2) {                    : (p == 2u) ?
    21;                                 21u
  }
  else {
    -1;
  }
*/

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
// #define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https:// github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LEDs
#define NEOPIXEL_BUILTIN     (5u)       // PA09
#define PIN_NEOPIXEL         NEOPIXEL_BUILTIN
#define PIN_NEOPIX           NEOPIXEL_BUILTIN


/*
 * Analog pins
 */
#define PIN_A0              (11u)        // PA10
#define PIN_A1              (PIN_A0 + 1) // PA04
#define PIN_A2              (21u)        // PA02
#define PIN_DAC0            (21u)        // PA02, required, even if not used.

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t DAC0 = PIN_DAC0;

#define ADC_RESOLUTION		12

#define PIN_VIN_ADC         PIN_A0      // Input power supply voltage monitor.
#define PIN_5V_ADC          PIN_A1      // 5V supply voltage monitor.

#define PIN_BOARD_ID        PIN_A1      // Board ID analog reference.

// Speaker  
#define PIN_SPKR            (6u)        // PA31
#define PIN_PAM_EN_1        (7u)        // PA01
#define PIN_PAM_EN_2        (8u)        // PA00

// User Input
#define PIN_BTN_CW          (9u)        // PA30
#define PIN_BTN_CCW         (10u)       // PA11

// DRV8434S Stepper Driver
#define PIN_DRV_DIR         (13u)       // PA16
#define PIN_DRV_STEP        (14u)       // PA17
#define PIN_DRV_EN          (15u)       // PB18
#define PIN_DRV_SLP         (16u)       // PA23
#define PIN_DRV_CS          (17u)       // PB19
#define PIN_DRV_FLT         (18u)       // PA28
#define PIN_DRV_VREF        (19u)       // PA22

// EEPROM
#define PIN_EEPROM_CS       (20u)       // PA08


/*
 * Serial interfaces
 */

// Serial1, OTMP (SERCOM3)
#define PIN_SERIAL1_TX      (0ul)       // PA22
#define PIN_SERIAL1_RX      (1ul)       // PA23
#define PAD_SERIAL1_TX      (UART_TX_PAD_0)
#define PAD_SERIAL1_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL1		  sercom2


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MOSI         (2u)       // PA06
#define PIN_SPI_MISO         (3u)       // PA05
#define PIN_SPI_SCK          (4u)       // PA07
#define PIN_SPI_SS			     (17u)      // PA19
#define PERIPH_SPI           sercom0
#define PAD_SPI_TX           SPI_PAD_2_SCK_3    // PICO/MOSI is on PAD[2], SCK is on PAD[3]
#define PAD_SPI_RX           SERCOM_RX_PAD_1    // POCI/MISO is on PAD[1]

static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;
static const uint8_t SS	  = PIN_SPI_SS ;

#define PIN_SPI_PICO        PIN_SPI_MOSI
#define PIN_SPI_POCI        PIN_SPI_MISO
static const uint8_t PICO = PIN_SPI_MOSI ;
static const uint8_t POCI = PIN_SPI_MISO ;


/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 0


/*
 * USB
 */
#define PIN_USB_HOST_ENABLE   (22ul)    // PA27
#define PIN_USB_DM            (23ul)    // PA24
#define PIN_USB_DP            (24ul)    // PA25
#define PIN_USB_DETECT        (22ul)    // PA27


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
//                           pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif /* _VARIANT_SOCIAL_CHOICE_V0_1_ */