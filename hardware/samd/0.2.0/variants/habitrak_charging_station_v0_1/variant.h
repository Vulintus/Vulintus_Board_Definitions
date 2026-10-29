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

#ifndef _VARIANT_HABITRAK_CHARGING_STATION_V0_1_
#define _VARIANT_HABITRAK_CHARGING_STATION_V0_1_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

// Non-pin definitions specific to this circuit board iteration.
#define CIRCUIT_BOARD_VER   01          // Divide by 10 to match the Eagle design version.

// I2C Addresses.
#define I2C_ADDR_CYPD3177		0x08		    // CYPD3177 USB-C power deliver controller.
#define I2C_ADDR_BQ27441		0x55		    // BQ27441 fuel gauges.
#define I2C_ADDR_MCP40D17T	0x2E		    // MCP40D17T digital potentiometers (charging current adjustment).
#define I2C_ADDR_OLED				0x3C		    // OLED displays


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
#ifdef __cplusplus
  extern "C" unsigned int PINCOUNT_fn();
#endif
#define PINS_COUNT           (PINCOUNT_fn())
#define NUM_DIGITAL_PINS     (32u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (0u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? 24u + (p) : (p < 8u) ? 25u + (p): -1)

/* 
  if (p < 6) {                          (p < 6u) ?
    24 + (p);                           24u + (p)
  }             
  else if (p < 8) {                     :(p < 8u) ?
    25 + (p);                           25u + (p)
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
// None controlled by the SAMD21.


/*
 * Analog pins
 */
#define PIN_A0               (24u)
#define PIN_A1               (PIN_A0 + 1)
#define PIN_A2               (PIN_A0 + 2)
#define PIN_A3               (PIN_A0 + 3)
#define PIN_A4               (PIN_A0 + 4)
#define PIN_A5               (PIN_A0 + 5)
#define PIN_A6               (31u)
#define PIN_A7               (PIN_A6 + 1)
#define PIN_DAC0             (38ul)     //Required, even if not used.

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6;
static const uint8_t A7  = PIN_A7;
static const uint8_t DAC0 = PIN_DAC0;

#define ADC_RESOLUTION		12

// NTC Thermistors
#define PIN_NTC0            PIN_A0      // Charging slot 0 thermistor.
#define PIN_NTC1            PIN_A1      // Charging slot 1 thermistor.
#define PIN_NTC2            PIN_A2      // Charging slot 2 thermistor.
#define PIN_NTC3            PIN_A3      // Charging slot 3 thermistor.
#define PIN_NTC4            PIN_A4      // Charging slot 4 thermistor.
#define PIN_NTC5            PIN_A5      // Charging slot 5 thermistor.

// Power Supply Monitoring
#define PIN_VIN_ADC         PIN_A6      // Power input voltage monitor.

#define PIN_BOARD_ID        PIN_A7      //Board ID analog reference.

// Other pins
#define PIN_ATN             (37ul)
static const uint8_t ATN = PIN_ATN;

// USB-C Power Delivery
#define PIN_USB_INT         (33u)
#define PIN_USB_FAULT       (34u)
#define PIN_PGOOD           (30u)


// Charger Enable.
#define PIN_CHG_EN0         (12u)
#define PIN_CHG_EN1         (13u)
#define PIN_CHG_EN2         (14u)
#define PIN_CHG_EN3         (15u)
#define PIN_CHG_EN4         (16u)
#define PIN_CHG_EN5         (17u)

// MCP Charger Status.
#define PIN_CHG_STAT0        (18u)
#define PIN_CHG_STAT1        (19u)
#define PIN_CHG_STAT2        (20u)
#define PIN_CHG_STAT3        (21u)
#define PIN_CHG_STAT4        (22u)
#define PIN_CHG_STAT5        (23u)


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 0


/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 6

#define PIN_WIRE_SDA         (0u)
#define PIN_WIRE_SCL         (1u)
#define PERIPH_WIRE          sercom2
#define WIRE_IT_HANDLER      SERCOM2_Handler
#define PIN_WIRE_SDA0        PIN_WIRE_SDA
#define PIN_WIRE_SCL0        PIN_WIRE_SCL

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;
static const uint8_t SDA0 = PIN_WIRE_SDA;
static const uint8_t SCL0 = PIN_WIRE_SCL;

#define PIN_WIRE1_SDA        (2u)
#define PIN_WIRE1_SCL        (3u)
#define PERIPH_WIRE1         sercom3
#define WIRE1_IT_HANDLER     SERCOM3_Handler

static const uint8_t SDA1 = PIN_WIRE1_SDA;
static const uint8_t SCL1 = PIN_WIRE1_SCL;

#define PIN_WIRE2_SDA        (4u)
#define PIN_WIRE2_SCL        (5u)
#define PERIPH_WIRE2         sercom5
#define WIRE2_IT_HANDLER     SERCOM5_Handler

static const uint8_t SDA2 = PIN_WIRE2_SDA;
static const uint8_t SCL2 = PIN_WIRE2_SCL;

#define PIN_WIRE3_SDA        (6u)
#define PIN_WIRE3_SCL        (7u)
#define PERIPH_WIRE3         sercom0
#define WIRE3_IT_HANDLER     SERCOM0_Handler

static const uint8_t SDA3 = PIN_WIRE3_SDA;
static const uint8_t SCL3 = PIN_WIRE3_SCL;

#define PIN_WIRE4_SDA        (8u)
#define PIN_WIRE4_SCL        (9u)
#define PERIPH_WIRE4         sercom4
#define WIRE4_IT_HANDLER     SERCOM4_Handler

static const uint8_t SDA4 = PIN_WIRE4_SDA;
static const uint8_t SCL4 = PIN_WIRE4_SCL;

#define PIN_WIRE5_SDA        (10u)
#define PIN_WIRE5_SCL        (11u)
#define PERIPH_WIRE5         sercom1
#define WIRE5_IT_HANDLER     SERCOM1_Handler

static const uint8_t SDA5 = PIN_WIRE5_SDA;
static const uint8_t SCL5 = PIN_WIRE5_SCL;


/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (37ul)
#define PIN_USB_DM          (35ul)
#define PIN_USB_DP          (36ul)


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
// #define SERIAL_PORT_HARDWARE        Serial1
// #define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif /* _VARIANT_HABITRAK_CHARGING_STATION_V0_1_ */