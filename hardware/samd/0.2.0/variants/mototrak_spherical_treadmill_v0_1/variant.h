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

#ifndef _VARIANT_SPHERICAL_TREADMILL_V0_1_
#define _VARIANT_SPHERICAL_TREADMILL_V0_1_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

// Non-pin definitions specific to this circuit board iteration.
#define CIRCUIT_BOARD_VER           1           // Divide by 10 to match the Eagle design version.
#define OMNITRAK_NUM_CUE_LED		    1           // We'll treat the status LED as a cue LED.
#define OMNITRAK_NUM_OPTICAL_FLOW	  2           // Number of optical flow sensors.
#define R_TOP_VIN                   10000.0f    // Input voltage monitor top resistor value, in ohms.
#define R_BOTTOM_VIN                1000.0f     // Input voltage monitor bottom resistor value, in ohms.

// I2C Addresses.
#define I2C_ADDR_OLED				0x3C		    // OLED display.

// Nonvolatile memory (Flash) address assignments (60 bytes total). //
#define NVM_ADDR_VULINTUS_ALIAS		0     	// Starting address for the Vulintus-set alias (30 bytes).
#define NVM_ADDR_USERSET_ALIAS  	30     	// Starting address for the user-set device alias (30 bytes).



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
#define NUM_ANALOG_INPUTS    (2u)
#define NUM_ANALOG_OUTPUTS   (0u)
#define analogInputToDigitalPin(p)  ((p == 0u) ? 15u : (p == 1u) ? 21u : -1)

/* 
  if (p == 0) {                         (p == 0u) ?
    15;                                 15u
  }             
  else if (p == 1) {                    : (p == 1u) ?
    21;                                 21u
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
#define PIN_LED_R       (8u)
#define PIN_LED_G       (9u)
#define PIN_LED_B       (10u)
#define LED_BUILTIN     PIN_LED_B


/*
 * Analog pins
 */
#define PIN_A0              (15u)       //PA02
#define PIN_A1              (21u)       //PA05
#define PIN_DAC0            (26u)       //PA02, required, even if not used.

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t DAC0 = PIN_DAC0;

#define ADC_RESOLUTION		12

#define PIN_VIN_ADC         PIN_A0      //Input power supply voltage monitor.

#define PIN_BOARD_ID        PIN_A1      //Board ID analog reference.


// Display / Status
#define PIN_OLED_RST        (7u)        //PA00

// Speaker  
#define PIN_SPKR            (11u)       //PA08
#define PIN_PAM_EN_1        (12u)       //PA15
#define PIN_PAM_EN_2        (13u)       //PA14

// User Input
#define PIN_BTN             (14u)       //PA07

// PMW3389 Optical Mouse Sensors
#define PIN_PMW_RST         (16u)       //PA06
#define PIN_PMW_CS_1        (17u)       //PA27
#define PIN_PMW_CS_2        (18u)       //PA18
#define PIN_PMW_MOT_1       (19u)       //PA19
#define PIN_PMW_MOT_2       (20u)       //PA04


/*
 * Serial interfaces
 */

// Serial1, OTMP (SERCOM3)
#define PIN_SERIAL1_TX      (0ul)       //PA22
#define PIN_SERIAL1_RX      (1ul)       //PA23
#define PAD_SERIAL1_TX      (UART_TX_PAD_0)
#define PAD_SERIAL1_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL1		  sercom3


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MOSI         (2u)       //PA10
#define PIN_SPI_MISO         (3u)       //PA09
#define PIN_SPI_SCK          (4u)       //PA11
#define PIN_SPI_SS			     (12u)      //PA15
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
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (5u)       //PA16
#define PIN_WIRE_SCL         (6u)       //PA17
#define PERIPH_WIRE          sercom1
#define WIRE_IT_HANDLER      SERCOM1_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;


/*
 * USB
 */
// #define PIN_USB_HOST_ENABLE   (22ul)    //PA28 << Fixed samd21_host.c so this doesn't have to be defined.
#define PIN_USB_DM            (23ul)    //PA24
#define PIN_USB_DP            (24ul)    //PA25
#define PIN_USB_DETECT        (22ul)    //PA28


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
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial

// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

// Alias Serial to SerialUSB
#define SerialUSB                   Serial

#endif /* _VARIANT_SPHERICAL_TREADMILL_V0_1_ */