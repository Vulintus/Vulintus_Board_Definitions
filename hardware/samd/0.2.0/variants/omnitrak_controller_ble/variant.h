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

#ifndef _VARIANT_OMNITRAK_CONTROLLER_BLE_
#define _VARIANT_OMNITRAK_CONTROLLER_BLE_

// The definitions here need a SAMD core >=1.6.10
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
extern "C" unsigned int PINCOUNT_fn();
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT                  (PINCOUNT_fn())
#define NUM_DIGITAL_PINS            (19u)
#define NUM_ANALOG_INPUTS           (2u)
#define NUM_ANALOG_OUTPUTS          (2u)
#define analogInputToDigitalPin(p) ((p < 2) ? 13 : -1)

/* 
  if (p < 2) {                          (p < 4) ?
    13 + (p);                           13 + (p)
  }             
  else {
    -1;                                 : -1
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


/*
 * Status LEDs
 */
#define PIN_LED_R           (10u)         // PA18 - Red channel of the RGB LED.
#define PIN_LED_G           (11u)         // PA17 - Green channel of the RGB LED.
#define PIN_LED_B           (12u)         // PA16 - Blue channel of the RGB LED.
#define LED_R               PIN_LED_R
#define LED_G               PIN_LED_G
#define LED_B               PIN_LED_B
#define LED_BUILTIN         PIN_LED_G


/*
 * Analog pins
 */
#define PIN_A0             (84ul)        // PA02 - BNC_OUT_1  (84)
#define PIN_A1             (PIN_A0 + 1) // PA05 - BNC_OUT_2  (85)
#define PIN_A2             (PIN_A0 + 2) // PB09 - BNC_IN_1   (86)
#define PIN_A3             (PIN_A0 + 3) // PB08 - BNC_IN_2   (87)

#define PIN_DAC0            PIN_A0
#define PIN_DAC1            PIN_A1

static const uint8_t A0   = PIN_A0;
static const uint8_t A1   = PIN_A1;
static const uint8_t A2   = PIN_A2;
static const uint8_t A3   = PIN_A3;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		  12


/*
 * Digital pins
 */

// NINA W102 Module
#define PIN_NINA_RST        (3u)         // PC10 - nNINA_RST    (3)
#define PIN_NINA_CS         (4u)         // PA06 - nNINA_CS     (4)
#define PIN_NINA_DEBUG      (5u)         // PA01 - nNINA_DEBUG  (5)
#define PIN_NINA_ACK        (6u)         // PA07 - nNINA_ACK    (6)
#define PIN_NINA_BOOT       (7u)         // PC07 - nNINA_BOOT   (7)

#define NINA_GPIO0          PIN_NINA_BOOT
#define NINA_RESETN         PIN_NINA_RST
#define NINA_ACK            PIN_NINA_ACK


/*
 * Serial interfaces
 */

// Serial1, NINA Programming Serial (SERCOM6)
#define PIN_SERIAL1_TX      (8ul)        // PC04
#define PIN_SERIAL1_RX      (9ul)        // PC05
#define PAD_SERIAL1_TX      (UART_TX_PAD_0)
#define PAD_SERIAL1_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL1		  sercom6

#define SerialNina          Serial1

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MOSI        (0u)         // PC27 (S1.0)
#define PIN_SPI_MISO        (1u)         // PB22 (S1.2)
#define PIN_SPI_SCK         (2u)         // PC28 (S1.1)
#define PERIPH_SPI          sercom1
#define PAD_SPI_TX          SPI_PAD_0_SCK_1       // PICO/MOSI on PAD 0, SCK on PAD1
#define PAD_SPI_RX          SERCOM_RX_PAD_2       // POCI/MISO on PAD 2

static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_SPI_PICO        PIN_SPI_MOSI
#define PIN_SPI_POCI        PIN_SPI_MISO
static const uint8_t PICO = PIN_SPI_PICO;
static const uint8_t POCI = PIN_SPI_POCI;

#define SPIWIFI SPI

#define SPIWIFI_SS          PIN_NINA_CS
#define SPIWIFI_ACK         PIN_NINA_ACK
#define SPIWIFI_RESET       PIN_NINA_RST


/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 0


/*
 * USB
 */
#define PIN_USB_DM          (19ul)        // PA24
#define PIN_USB_DP          (20ul)        // PA25
#define PIN_USB_HOST_ENABLE (17ul)        // PA15 - SAMBA    (94)
#define PIN_USB_DETECT      (18ul)        // PC06
#define USB_DETECT          PIN_USB_DETECT


/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0


/*
 * Other pins?? 
 */
#define PIN_ATN             (21u)       // PA03 - I don't think we need this definition....
static const uint8_t ATN = PIN_ATN;


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
extern SERCOM sercom1;    // SPI, SerialHCI
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;
extern SERCOM sercom6;    // SerialNina
extern SERCOM sercom7;

// OTMP serial ports.
extern Uart Serial1;

// NINA programming serial port.
extern Uart Serial6;

// SerialHCI - Required for NINA operation, no idea what it does.
extern Uart SerialHCI;                            // Create the option of a serial on the main SPI bus (SERCOM1)
#define PIN_SERIALHCI_RX    PIN_SPI_POCI          // SPI POCI
#define PIN_SERIALHCI_TX    PIN_SPI_PICO          // SPI PICO
#define PAD_SERIALHCI_TX    (UART_TX_PAD_0)       // SPI PICO is on SERCOM1.0
#define PAD_SERIALHCI_RX    (SERCOM_RX_PAD_2)     // SPI POCI is on SERCOM1.2
#define PIN_SERIALHCI_RTS   PIN_NINA_CS           // NINA chip-select
#define PIN_SERIALHCI_CTS   PIN_SPI_SCK           // SPI SCK

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
//                           pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial

// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

// Alias Serial to SerialUSB
#define SerialUSB                   Serial

#endif /* _VARIANT_OMNITRAK_CONTROLLER_BLE_ */