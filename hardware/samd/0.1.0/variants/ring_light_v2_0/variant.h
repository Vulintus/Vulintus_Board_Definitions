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

#ifndef _VARIANT_RING_LIGHT_V2_0_
#define _VARIANT_RING_LIGHT_V2_0_

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
 *        Clock Configuration
 *----------------------------------------------------------------------------*/

/** Master clock frequency (also Fcpu frequency) */
#define VARIANT_MCK		(48000000ul)

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
  extern "C" unsigned int PINCOUNT_fn();
#endif // __cplusplus


/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT                  (PINCOUNT_fn())
#define NUM_DIGITAL_PINS            (9u)
#define NUM_ANALOG_INPUTS           (2u)
#define NUM_ANALOG_OUTPUTS          (0u)
#define analogInputToDigitalPin(p)  (6 + (p))

#define digitalPinToPort(P)         ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)      ( 1 << g_APinDescription[P].ulPin )
// #define analogInPinToBit(P)         ( )
#define portOutputRegister(port)    ( &(port->OUT.reg) )
#define portInputRegister(port)     ( &(port->IN.reg) )
#define portModeRegister(port)      ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)         ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )


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
#define PIN_NEOPIX        (4u)          //NEOPIX_3V3/SWCLK
#define PIN_IR_LEDS       (5u)          //IR_LEDS
#define LED_BUILTIN       PIN_IR_LEDS


/*
 * Analog pins
 */
#define PIN_A0            (6ul)         //VIN_ADC   (6)
#define PIN_A1            (PIN_A0 + 1)  //BOARD_ID  (7)

static const uint8_t A0   = PIN_A0;
static const uint8_t A1   = PIN_A1;

#define ADC_RESOLUTION		12

#define PIN_V_IN_ADC        PIN_A0      //24V power supply voltage monitor.
#define PIN_BOARD_ID        PIN_A1      //Board ID analog reference.


/*
 * Other pins
 */

// The ATN pin may be used in the future as the first SPI chip select.
// On boards that do not have the Arduino physical form factor, it can to set to any free pin.
#define PIN_ATN             (8u)
static const uint8_t ATN = PIN_ATN;

// Vulintus Peripheral Bus (VPB)
#define PIN_VPB_CLK_IN    (0u)
#define PIN_VPB_CLK_OUT   (1u)
#define PIN_VPB_TRG       (2u)
#define PIN_VPB_BLOCK     (3u)


/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIAL1_TX    (1ul)         //PA14, SERCOM 0.0
#define PIN_SERIAL1_RX    (2ul)         //PA09, SERCOM 0.3
#define PAD_SERIAL1_TX    (UART_TX_PAD_0)
#define PAD_SERIAL1_RX    (SERCOM_RX_PAD_3)
#define SERCOM_SERIAL1    sercom0


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 0


/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 0


/*
 * USB
 */
#define PIN_USB_DP          (10ul)
#define PIN_USB_DM          (9ul)
// #define PIN_USB_HOST_ENABLE (92ul)
#define PIN_USB_DETECT      (8ul)
#define USB_DETECT          PIN_USB_DETECT


/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

  #include "SERCOM.h"
  #include "Uart.h"

  /*	=========================
  *	===== SERCOM DEFINITION
  *	=========================
  */
  extern SERCOM sercom0;
  extern SERCOM sercom1;

  //Hardware serial ports.
  extern Uart Serial1;

#endif // __cplusplus

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

// Alias Serial to SerialUSB
#define SerialUSB                   Serial

#endif /* _VARIANT_RING_LIGHT_V2_0_ */
