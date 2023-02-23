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

#ifndef _VARIANT_HABITRAK_V1_1_
#define _VARIANT_HABITRAK_V1_1_

#define VULINTUS_HABITRAK               // Used for conditional compilation in Vulintus libraries.

// The definitions here need a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

// Non-pin definitions specific to this circuit board iteration.
#define CIRCUIT_BOARD_VER   11          // Divide by 10 to match the Eagle design version.


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
#define PINS_COUNT           (PINCOUNT_fn())
#define NUM_DIGITAL_PINS     (76u)
#define NUM_ANALOG_INPUTS    (2u)
#define NUM_ANALOG_OUTPUTS   (0u)
#define analogInputToDigitalPin(p) ((p == 0u) ? 50u : (p == 1u) ? 73u : -1)

/* 
  if (p == 0) {                         (p == 0u) ?
    50;                                 50u
  }             
  else if (p == 1) {                    : (p == 1u) ?
    91;                                 73u
  }
  else {
    -1;                                 : -1
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


/*
 * Status LEDs
 */
#define PIN_LED_R           (28u)       //PB17, Only enabled when nQI_CHG is low.
#define PIN_LED_G           (29u)       //PD10
#define PIN_LED_B           (30u)       //PB13
#define PIN_LED_IR          (42u)       //PB02
#define LED_STATUS          PIN_LED_B
#define LED_CHG             PIN_LED_R
#define LED_BUILTIN         PIN_LED_G


/*
 * Analog pins
 */
#define PIN_A0              (50u)       //PB06
#define PIN_A1              (73u)       //PC30
#define PIN_A2              (78u)       //PA03


#define PIN_DAC0            (9u)        //PA02, unused, but define in case required by libraries
#define PIN_DAC1            (13u)       //PA05, unused, but define in case required by libraries

static const uint8_t A0   = PIN_A0;
static const uint8_t A1   = PIN_A1;
static const uint8_t A2   = PIN_A2;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		  12

#define PIN_BAT_NTC         PIN_A0
#define PIN_BOARD_ID        PIN_A1
#define PIN_VREF            PIN_A2


/*
 * Digital pins
 */

// NINA W102 WiFi-Bluetooth module.
#define PIN_NINA_RST        (7u)        //PB05
#define PIN_NINA_BOOT       (8u)        //PB07
#define PIN_NINA_ACK        (9u)        //PA02
#define PIN_NINA_DEBUG      (10u)       //PA07
#define PIN_NINA_CS         (11u)       //PD01
#define NINA_RST            PIN_NINA_RST
#define NINA_RESETN         PIN_NINA_RST      //Required for NINA library.
#define NINA_BOOT           PIN_NINA_BOOT
#define NINA_GPIO0          PIN_NINA_BOOT     //Required for NINA library.
#define NINA_ACK            PIN_NINA_ACK      //Required for NINA library.
#define NINA_DEBUG          PIN_NINA_DEBUG
#define NINA_CS             PIN_NINA_CS

// TFT display.
#define PIN_TFT_RST         (25u)       //PC22
#define PIN_TFT_LED         (24u)       //PC22
#define PIN_TFT_DC          (26u)       //PC18
#define PIN_TFT_CS          (27u)       //PC16
#define TFT_RST             PIN_TFT_RST
#define TFT_LED             PIN_TFT_LED
#define TFT_DC              PIN_TFT_DC
#define TFT_CS              PIN_TFT_CS

// RV-3208-C7 Real-Time Clock
#define PIN_RTC_INT         (31u)       //PD21
#define PIN_RTC_EVI         (32u)       //PB16
#define PIN_RTC_CLK         (33u)       //PA00

// User Input
#define PIN_BTN_U           (34u)       //PB00
#define PIN_BTN_S           (35u)       //PA15
#define PIN_BTN_D           (36u)       //PB01
#define PIN_QT_MODE         (37u)       //PB04
#define PIN_LED_BTN_U       (40u)       //PA01
#define PIN_LED_BTN_D       (41u)       //PC23
#define PIN_BTN_U_TEST      (38u)       //PC26
#define PIN_BTN_D_TEST      (39u)       //PC24
#define LED_BTN_U           PIN_LED_BTN_U
#define LED_BTN_D           PIN_LED_BTN_D

// VL53L0X Distance Sensor
#define PIN_VLX_GPIO        (43u)       //PB09
#define PIN_VLX_XSHUT       (44u)       //PC06

// Pushbuton On-Off controller
#define PIN_MAX_INT         (45u)       //PC00
#define PIN_MAX_CLR         (46u)       //PC02

// BH1749 Light/Color Sensor
#define PIN_RGB_INT         (47u)       //PC04

// BME6888 Temperature-Pressure-Humidity-Gas Sensor
#define PIN_BME_CS          (48u)       //PB18
#define BME_CS              PIN_BME_CS

// BQ27441 Battery Fuel Gauge.
#define PIN_BQ_GPO          (49u)       //PD20

// LSM303D Accelerometer-Magnetometer
#define PIN_LSM_INT1        (52u)       //PB22
#define PIN_LSM_INT2        (53u)       //PB23
#define PIN_LSM_CS          (54u)       //PC27
#define LSM_CS              PIN_LSM_CS



/*
 * Serial interfaces
 */

// Serial1, NINA W102 Programming (SERCOM0)
#define PIN_SERIAL1_TX      (12u)       //PA04 (SA0.0)
#define PIN_SERIAL1_RX      (13u)       //PA05 (SA0.1)
#define PAD_SERIAL1_TX      (UART_TX_PAD_0)
#define PAD_SERIAL1_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL1		  sercom0

// Serial2, QTouch Debugging (SERCOM3)
#define PIN_SERIAL2_TX      (5u)        //PB20 (S3.0, SA7.1)
#define PIN_SERIAL2_RX      (6u)        //PB21 (S3.1, SA7.0)
#define PAD_SERIAL2_TX      (UART_TX_PAD_0)
#define PAD_SERIAL2_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL2		  sercom3


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MOSI        (0u)        //PB26 (S2.0, SA4.1)
#define PIN_SPI_MISO        (1u)        //PB29 (S2.3, SA4.3)
#define PIN_SPI_SCK         (2u)        //PB27 (S2.1, SA4.0)
#define PERIPH_SPI          sercom2
#define PAD_SPI_TX          SPI_PAD_0_SCK_1
#define PAD_SPI_RX          SERCOM_RX_PAD_3

static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_SPI_PICO        PIN_SPI_MOSI
#define PIN_SPI_POCI        PIN_SPI_MISO

static const uint8_t PICO = PIN_SPI_PICO;
static const uint8_t POCI = PIN_SPI_POCI;

#define SPIWIFI_SS          PIN_NINA_CS
#define SPIWIFI_ACK         PIN_NINA_ACK
#define SPIWIFI_RESET       PIN_NINA_RST


// Needed for SD library
#define PIN_SPI1_MISO       (17u)       //PA09 (S0.1, SA2.0)
#define PIN_SPI1_MOSI       (15u)       //PA08 (S0.0, SA2.1)
#define PIN_SPI1_SCK        (16u)       //PB11 (SA4.3)
#define PIN_SPI1_SS         (20u)       //PB10 (SA4.2)
// #define PERIPH_SPI1   sercom2             
#define PAD_SPI1_TX         SPI_PAD_0_SCK_3   
#define PAD_SPI1_RX         SERCOM_RX_PAD_1     
static const uint8_t SS1   = PIN_SPI1_SS;   
static const uint8_t MOSI1 = PIN_SPI1_MOSI; 
static const uint8_t MISO1 = PIN_SPI1_MISO;   
static const uint8_t SCK1  = PIN_SPI1_SCK;    

#define SDCARD_SPI          SPI1
#define SDCARD_MISO_PIN     PIN_SPI1_MISO
#define SDCARD_MOSI_PIN     PIN_SPI1_MOSI
#define SDCARD_SCK_PIN      PIN_SPI1_SCK
#define SDCARD_SS_PIN       PIN_SPI1_SS

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

//I2C bus.
#define PIN_WIRE_SDA        (3u)        //PD08 (S7.0, SA6.1)
#define PIN_WIRE_SCL        (4u)        //PD09 (S7.1, SA6.0)
#define PERIPH_WIRE         sercom7
#define WIRE_IT_HANDLER     SERCOM7_Handler
// #define WIRE_IT_HANDLER_0   SERCOM7_0_Handler
// #define WIRE_IT_HANDLER_1   SERCOM7_1_Handler
// #define WIRE_IT_HANDLER_2   SERCOM7_2_Handler
// #define WIRE_IT_HANDLER_3   SERCOM7_3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

//SCCB bus.
#define PIN_WIRE1_SDA       (55u)       //PC13 (S7.1, SA6.0)
#define PIN_WIRE1_SCL       (56u)       //PC12 (S7.0, SA6.1)
#define PERIPH_WIRE1        sercom6
#define WIRE1_IT_HANDLER    SERCOM6_Handler
// #define WIRE1_IT_HANDLER_0  SERCOM6_0_Handler
// #define WIRE1_IT_HANDLER_1  SERCOM6_1_Handler
// #define WIRE1_IT_HANDLER_2  SERCOM6_2_Handler
// #define WIRE1_IT_HANDLER_3  SERCOM6_3_Handler

static const uint8_t SDA1 = PIN_WIRE1_SDA;
static const uint8_t SCL1 = PIN_WIRE1_SCL;


/*
 * USB
 */
#define PIN_USB_DM          (77u)       //PA24
#define PIN_USB_DP          (76u)       //PA25
#define PIN_USB_HOST_ENABLE (75u)       //PA27, unconnected
#define PIN_USB_DETECT      (74u)       //PB19
#define USB_DETECT          PIN_USB_DETECT


/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0


/*
 * SDHC Interfaces (SDHC0)
*/
#define PIN_SD_DETECT       (14u)       //PB12
#define PIN_SD_CMD          (15u)       //PA08
#define PIN_SD_CLK	        (16u)       //PB11
#define PIN_SD_DAT0	        (17u)       //PA09
#define PIN_SD_DAT1	        (18u)       //PA10
#define PIN_SD_DAT2	        (19u)       //PA11
#define PIN_SD_DAT3	        (20u)       //PB10
#define SD_CMD              PIN_SD_CMD
#define SD_CLK              PIN_SD_CLK
#define SD_DAT0             PIN_SD_DAT0
#define SD_DAT1             PIN_SD_DAT1
#define SD_DAT2             PIN_SD_DAT2
#define SD_DAT3             PIN_SD_DAT3



/*
 * Parallel Capture Controller (PCC)
*/
#define PIN_PCC_DEN1        (71u)       //PA12 (PCC / DEN1)
#define PIN_PCC_DEN2        (72u)       //PA13 (PCC / DEN2)
#define PIN_PCC_CLK         (60u)       //PA14 (PCC / CLK)
#define PIN_PCC_XCLK	      (57u)       //PC11 (TCCO/WO[0], TCC1/WO[4])
#define PIN_PCC_D0          (61u)       //PA16 (PCC/DATA[0])
#define PIN_PCC_D1          (62u)       //PA17 (PCC/DATA[1])
#define PIN_PCC_D2          (63u)       //PA18 (PCC/DATA[2])
#define PIN_PCC_D3          (64u)       //PA19 (PCC/DATA[3])
#define PIN_PCC_D4          (65u)       //PA20 (PCC/DATA[4])
#define PIN_PCC_D5          (66u)       //PA21 (PCC/DATA[5])
#define PIN_PCC_D6          (67u)       //PA22 (PCC/DATA[6])
#define PIN_PCC_D7          (68u)       //PA23 (PCC/DATA[7])
#define PIN_PCC_D8          (69u)       //PB14 (PCC/DATA[8])
#define PIN_PCC_D9          (70u)       //PB15 (PCC/DATA[9])

#define PIN_SCCB_SDA        PIN_WIRE1_SDA
#define PIN_SCCB_SCL        PIN_WIRE1_SCL
#define PIN_CAM_VSYNC       PIN_PCC_DEN1
#define PIN_CAM_HSYNC       PIN_PCC_DEN2
#define PIN_CAM_PCLK        PIN_PCC_CLK
#define PIN_CAM_XCLK        PIN_PCC_XCLK
#define PIN_CAM_RST         (58u)       //PC15
#define PIN_CAM_PWDN        (59u)       //PC14


/*
 * Other pins?? 
 */
#define PIN_ATN             (40u)       //I don't think we need this definition....
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
extern SERCOM sercom0;    //NINA W102 Programming Serial
extern SERCOM sercom1;    //[UNUSED]
extern SERCOM sercom2;    //SPI, SerialHCI
extern SERCOM sercom3;    //QTouch Debugging Serial
extern SERCOM sercom4;    //[UNUSED]
extern SERCOM sercom5;    //[UNUSED]
extern SERCOM sercom6;    //SCCB
extern SERCOM sercom7;    //I2C

extern Uart Serial1;      //Serial1, NINA W102 Programming (SERCOM0)
extern Uart Serial2;      //Serial2, QTouch Debugging (SERCOM2)

// SerialHCI - Required for NINA operation, no idea what it does.
extern Uart SerialHCI;
#define PIN_SERIALHCI_RX  (1ul)           //NINA_MISO, PB29 (S2.3, SA4.3)
#define PIN_SERIALHCI_TX  (0ul)           //NINA_MOSI, PB26 (S2.0, SA4.1)
#define PAD_SERIALHCI_TX  (UART_TX_PAD_0)    
#define PAD_SERIALHCI_RX  (SERCOM_RX_PAD_3)
#define PIN_SERIALHCI_RTS (11u)          //NINA_CS,  PD01
#define PIN_SERIALHCI_CTS (2u)           //NINA_SCK, PB27 (S2.1, SA4.0)

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
#define SERIAL_PORT_HARDWARE        Serial2
#define SERIAL_PORT_HARDWARE_OPEN   Serial2

// Alias Serial1 to SerialNina (useful in libraries)
#define SerialNina                  Serial1
#define SPIWIFI                     SPI

// Alias Serial to SerialUSB
#define SerialUSB                   Serial

//This define is needed for the ArduinoBLE and WiFiNINA libraries to fully function properly.
#define ARDUINO_SAMD_NANO_33_IOT

#endif /* _VARIANT_HABITRAK_V1_1_ */
