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

#ifndef _VARIANT_PELLET_DISPENSER_V6_0_
#define _VARIANT_PELLET_DISPENSER_V6_0_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

// Non-pin definitions specific to this circuit board iteration.
#define CIRCUIT_BOARD_VER             60    // Divide by 10 to match the Eagle design version.
#define STEPPER_DRIVER_MODEL_DRV8434S       // Stepper driver model.
#define STEPPER_DRIVER_CURRENT_MAX		2500  // Maximum possible stepper coil current, in mA
#define NUM_NEOPIX                    4     // Number of NeoPixels.
#define EEPROM_SIZE                   256   // Available EEPROM bytes.

// I2C Addresses.
#define I2C_ADDR_MCP40D18T	0x2E		    // MCP40D18T digital potentiometer (pellet detector reference).
#define I2C_ADDR_OLED				0x3C		    // OLED display.
#define I2C_ADDR_EEPROM 		0x50		    // AT24C02D I2C EEPROM.

// EEPROM address assignments (256 bytes total). //
#define NVM_ADDR_VULINTUS_ALIAS		0     	// Starting address for the Vulintus-set alias (30 bytes).
#define NVM_ADDR_USERSET_ALIAS  	30     	// Starting address for the user-set device alias (30 bytes).
#define NVM_ADDR_DISPENSER_NUM  	60     	// Address for the user-set dispenser number (1 byte).


/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK			  (48000000ul)


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

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
#define NUM_ANALOG_INPUTS    (7u)
#define NUM_ANALOG_OUTPUTS   (0u)
#define analogInputToDigitalPin(p)  ((p < 4u) ?  17u + (p) : (p == 4u) ?  27u : (p < 7u) ? 30u + (p) : -1)

/* 
  if (p < 4) {                          (p < 4u) ?
    17 + (p);                           17u + (p)
  }          
  else if (p == 4) {                    : (p == 4u) ?
    27;                                 27u
  }   
  else if (p < 7) {                     : (p < 7u) ?
    30 + (p);                           30u + (p)
  }
  else {
    -1;                                 :-1
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
#define NEOPIXEL_BUILTIN     (12u)      // PA15
#define PIN_NEOPIXEL         NEOPIXEL_BUILTIN
#define PIN_NEOPIX           NEOPIXEL_BUILTIN

/*
 * Analog pins
 */
#define PIN_A0               (17u)        // PA02
#define PIN_A1               (PIN_A0 + 1) // PA04
#define PIN_A2               (PIN_A0 + 2) // PA06
#define PIN_A3               (PIN_A0 + 3) // PB09
#define PIN_A4               (27u)        // PB02
#define PIN_A5               (30u)        // PA09
#define PIN_A6               (PIN_A5 + 1) // PA07
#define PIN_DAC0             (17u)        // PA02, required, even if not used.

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6 ;

#define ADC_RESOLUTION		12

#define PIN_5V_ADC            PIN_A0     // PA02, 5V Supply Voltage Monitor.
#define PIN_15V_ADC           PIN_A1     // PA04, 15V Supply Voltage Monitor.
#define PIN_24V_UVLO          PIN_A2     // PA06, 24V Under-Voltage Lock-Out Monitor.
#define PIN_15V_ON_FLAG       PIN_A3     // PB09, 15V Supply On Flag.

#define PIN_SLOT_A            PIN_A4     // Slot detector, analog signal.

#define PIN_BOARD_ID          PIN_A6     // Board ID analog reference.

// Other pins
#define PIN_ATN              (32ul)      // PA28
static const uint8_t ATN = PIN_ATN;

// Vulintus Peripheral Bux (VPB)
#define PIN_VPB_CLK_IN      (5u)         // PA14
#define PIN_VPB_CLK_OUT     (6u)         // PA12
#define PIN_VPB_TRG         (7u)         // PA05
#define PIN_VPB_BLOCK       (8u)         // PA17

// Display / Status
#define PIN_OLED_RST        (11u)        // PB08
#define PIN_SPKR            (13u)        // PA27
#define PIN_PAM_EN_1        (14u)        // PB23
#define PIN_PAM_EN_2        (15u)        // PB22

// User Input
#define PIN_BTN             (16u)        // PB03

// DRV8434S Stepper Driver
#define PIN_DRV_DIR         (21u)        // PA19
#define PIN_DRV_STEP        (22u)        // PA18
#define PIN_DRV_EN          (23u)        // PB11
#define PIN_DRV_SLP         (24u)        // PA21
#define PIN_DRV_CS          (25u)        // PB10
#define PIN_DRV_FLT         (26u)        // PA13

// Pellet Detector.
#define PIN_SLOT_D          (28u)        // PA20

// Dehumidifier.
#define PIN_RDS_ON          (29u)        // PA16

// USB
#define PIN_USB_DETECT      (32ul)       // PA28


/*
 * Serial interfaces
 */

// Serial (same as I2C, SERCOM3)
#define PIN_SERIAL_RX       (1ul)        // PA23
#define PIN_SERIAL_TX       (0ul)        // PA22
#define PAD_SERIAL_TX       (UART_TX_PAD_0)
#define PAD_SERIAL_RX       (SERCOM_RX_PAD_1)

// Serial1 (same as SPI, SERCOM0)
#define PIN_SERIAL1_RX       (4ul)       // PA11
#define PIN_SERIAL1_TX       (2ul)       // PA10
#define PAD_SERIAL1_TX       (UART_TX_PAD_2)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_3)


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

// SPI0 (SERCOM0)
#define PIN_SPI_MISO         (3u)        // PA08
#define PIN_SPI_MOSI         (2u)        // PA10
#define PIN_SPI_SCK          (4u)        // PA11
#define PERIPH_SPI           sercom0
#define PAD_SPI_TX           SPI_PAD_2_SCK_3    // PICO/MOSI is on PAD[2], SCK is on PAD[3]
#define PAD_SPI_RX           SERCOM_RX_PAD_0    // POCI/MISO is on PAD[0]

static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_SPI_PICO        PIN_SPI_MOSI
#define PIN_SPI_POCI        PIN_SPI_MISO
static const uint8_t PICO = PIN_SPI_PICO;
static const uint8_t POCI = PIN_SPI_POCI;


/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (0u)        // PA22
#define PIN_WIRE_SCL         (1u)        // PA23
#define PERIPH_WIRE          sercom3
#define WIRE_IT_HANDLER      SERCOM3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;


/*
 * USB
 */
#define PIN_USB_HOST_ENABLE   (9ul)
#define PIN_USB_DM            (33ul)
#define PIN_USB_DP            (34ul)


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

  #include "SERCOM.h"
  #include "Uart.h"
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

  // extern Uart Serial;
  // extern Uart Serial1;

#endif

#ifdef __cplusplus
  extern "C" {
#endif    // __cplusplus
    unsigned int PINCOUNT_fn();
#ifdef __cplusplus
  }
#endif    // __cplusplus

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

#endif    // _VARIANT_PELLET_DISPENSER_V6_0_