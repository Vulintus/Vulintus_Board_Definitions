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

#ifndef _VARIANT_LIQUID_DISPENSER_V2_0_
#define _VARIANT_LIQUID_DISPENSER_V2_0_

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
#define PINS_COUNT           (32u)
#define NUM_DIGITAL_PINS     (32u)
#define NUM_ANALOG_INPUTS    (9u)
#define NUM_ANALOG_OUTPUTS   (0u)
#define analogInputToDigitalPin(p)  ((p == 0u) ?  14 : (p == 1u) ?  21 : (p < 8u) ? 21u + (p) : (p == 8u) ? 30u :-1)

/* 
  if (p == 0) {                         (p == 0u) ?
    14;                                 14
  }          
  else if (p == 1) {                    : (p == 1u) ?
    21;                                 21u
  }   
  else if (p < 8) {                     : (p < 7u) ?
    21 + (p);                           21u + (p)
  }
  
  else if (p == 8) {                    : (p == 8u) ?
    30;                                 30u
  }
  else {
    -1;                                 :-1
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
#define NEOPIXEL_BUILTIN     (9u)
#define PIN_NEOPIXEL         NEOPIXEL_BUILTIN
#define PIN_NEOPIX           NEOPIXEL_BUILTIN

/*
 * Analog pins
 */
#define PIN_A0               (14ul)
#define PIN_A1               (21u)
#define PIN_A2               (23u)
#define PIN_A3               (PIN_A2 + 1)
#define PIN_A4               (PIN_A2 + 2)
#define PIN_A5               (PIN_A2 + 3)
#define PIN_A6               (PIN_A2 + 4)
#define PIN_A7               (PIN_A2 + 5)
#define PIN_A8               (30u)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6 ;
static const uint8_t A7  = PIN_A7 ;

#define ADC_RESOLUTION		12

#define PIN_VIN               PIN_A0     //Input Voltage Monitor.

#define PIN_DRV_VREF          PIN_A1     //Stepper Driver Current-Control Voltage Reference.

#define PIN_LOADCELL_1        PIN_A2     //Water Scale Loadcell #1 Signal.
#define PIN_LOADCELL_VREF_1   PIN_A3     //Water Scale Loadcell #1 Baseline.
#define PIN_LOADCELL_2        PIN_A4     //Water Scale Loadcell #2 Signal.
#define PIN_LOADCELL_VREF_2   PIN_A5     //Water Scale Loadcell #2 Baseline.
#define PIN_DET_L             PIN_A6     //Water Detector Signal - Left.
#define PIN_DET_R             PIN_A7     //Water Detector Signal - Right.

#define PIN_BOARD_ID          PIN_A8     //Board ID analog reference.


// Other pins
#define PIN_ATN              (4ul)
static const uint8_t ATN = PIN_ATN;

// 24VPB
#define PIN_24VPB_CLK_IN    (2u)       
#define PIN_24VPB_CLK_OUT   (3u)   
#define PIN_24VPB_TRG       (4u)
#define PIN_24VPB_BLK       (5u)

// Display / Status
#define PIN_OLED_RST        (8u)
#define PIN_SPKR            (10u)

// User Input
#define PIN_BTN_L           (11u)
#define PIN_BTN_C           (12u)
#define PIN_BTN_R           (13u)

// DRV8434S Stepper Driver
#define PIN_DRV_DIR         (15u)
#define PIN_DRV_STEP        (16u)
#define PIN_DRV_EN          (17u)
#define PIN_DRV_MS0         (18u)
#define PIN_DRV_MS1         (19u)
#define PIN_DRV_FLT         (20u)

// Pump Selection (Left/Right).
#define PIN_PUMP_SEL        (22u)

// Water Detectors.
#define PIN_DET_INT         (29u)


/*
 * Serial interfaces
 */

// Serial1
#define PIN_SERIAL1_RX       (0ul)
#define PIN_SERIAL1_TX       (1ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_2)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_3)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 0

#define PIN_SPI_MISO         (28u)
#define PIN_SPI_MOSI         (29u)
#define PIN_SPI_SCK          (30u)
#define PERIPH_SPI           sercom4
#define PAD_SPI_TX           SPI_PAD_2_SCK_3
#define PAD_SPI_RX           SERCOM_RX_PAD_0

static const uint8_t SS	  = PIN_A2 ;	// SERCOM4 last PAD is present on A2 but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;


/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (0u)
#define PIN_WIRE_SCL         (1u)
#define PERIPH_WIRE          sercom3
#define WIRE_IT_HANDLER      SERCOM3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (31ul)
#define PIN_USB_DM          (32ul)
#define PIN_USB_DP          (33ul)

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

#endif /* _VARIANT_LIQUID_DISPENSER_V2_0_ */

