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

#ifndef _VARIANT_OMNITRAK_CONTROLLER_V2_1_
#define _VARIANT_OMNITRAK_CONTROLLER_V2_1_

// The definitions here need a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

// Non-pin definitions specific to this circuit board iteration.
#define CIRCUIT_BOARD_VER   21          // Divide by 10 to match the Eagle design version.
#define NUM_OTMP_PORTS      5           // Number of OTMP ports.


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
#define NUM_DIGITAL_PINS     (97u)
#define NUM_ANALOG_INPUTS    (15u)
#define NUM_ANALOG_OUTPUTS   (2u)
#define analogInputToDigitalPin(p) ((p < 5) ? 46 + (p) : (p < 10) ? 70 + (p) : (p < 14) ? 85 + (p) : (p == 14) ? 94 : (p == 15) ? 99 : -1)

/* 
  if (p < 5) {                          (p < 5) ?
    46 + (p);                           46 + (p)
  }             
  else if (p < 10) {                    : (p < 10) ?
    70 + (p) - 5;                       70 + (p)
  }
  else if (p < 14) {                    : (p < 14) ?
    85 + (p) - 10;                      85 + (p)
  }
  else if (p == 14) {                   : (p == 14) ?
    94;                                 94
  }
  else if (p == 15) {                   : (p == 15) ?
    99;                                 99
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
#define PIN_LED_R           (30u)         //PA18 - Red channel of the RGB LED.
#define PIN_LED_G           (31u)         //PA17 - Green channel of the RGB LED.
#define PIN_LED_B           (32u)         //PA16 - Blue channel of the RGB LED.
#define PIN_LED_DEMUX_A     (41u)         //PB15 - Optional demultiplexer input for encoder LED indication.
#define PIN_LED_DEMUX_B     (42u)         //PB18 - Optional demultiplexer input for encoder LED indication.
#define PIN_LED_DEMUX_C     (43u)         //PA31 - Optional demultiplexer input for encoder LED indication.
#define PIN_LED_DEMUX_D     (44u)         //PA30 - Optional demultiplexer input for encoder LED indication.
#define LED_R               PIN_LED_R
#define LED_G               PIN_LED_G
#define LED_B               PIN_LED_B
#define LED_BUILTIN         PIN_LED_G


/*
 * Analog pins
 */
#define PIN_A0              (46ul)        //PC30 - V_IN_ADC   (46)
#define PIN_A1              (PIN_A0 + 1)  //PD00 - I_IN_ADC   (47)
#define PIN_A2              (PIN_A0 + 2)  //PB05 - 9V_UVLO    (48)
#define PIN_A3              (PIN_A0 + 3)  //PB04 - 9V_ADC     (49)
#define PIN_A4              (PIN_A0 + 4)  //PB06 - 5V_ADC     (50)
#define PIN_A5              (70ul)        //PD01 - P1_IOUT    (70)
#define PIN_A6              (PIN_A5 + 1)  //PC31 - P2_IOUT    (71)
#define PIN_A7              (PIN_A5 + 2)  //PC00 - P3_IOUT    (72)
#define PIN_A8              (PIN_A5 + 3)  //PC01 - P4_IOUT    (73)
#define PIN_A9              (PIN_A5 + 4)  //PC02 - P5_IOUT    (74)
#define PIN_A10             (85ul)        //PA02 - BNC_OUT_1  (85)
#define PIN_A11             (PIN_A10 + 1) //PA05 - BNC_OUT_2  (86)
#define PIN_A12             (PIN_A10 + 2) //PB09 - BNC_IN_1   (87)
#define PIN_A13             (PIN_A10 + 3) //PB08 - BNC_IN_2   (88)
#define PIN_A14             (91ul)        //PB07 - BOARD_ID   (94)

#define PIN_DAC0            PIN_A10
#define PIN_DAC1            PIN_A11

static const uint8_t A0   = PIN_A0;
static const uint8_t A1   = PIN_A1;
static const uint8_t A2   = PIN_A2;
static const uint8_t A3   = PIN_A3;
static const uint8_t A4   = PIN_A4;
static const uint8_t A5   = PIN_A5;
static const uint8_t A6   = PIN_A6;
static const uint8_t A7   = PIN_A7;
static const uint8_t A8   = PIN_A8;
static const uint8_t A9   = PIN_A9;
static const uint8_t A10  = PIN_A10;
static const uint8_t A11  = PIN_A11;
static const uint8_t A12  = PIN_A12;
static const uint8_t A13  = PIN_A13;
static const uint8_t A14  = PIN_A14;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION		  12

#define PIN_V_IN_ADC        PIN_A0        //24V power supply voltage monitor.
#define PIN_I_IN_ADC        PIN_A1        //24V power supply current monitor.
#define PIN_9V_UVLO         PIN_A2        //Under-voltage lock-out level on the 9V power supply.
#define PIN_9V_ADC          PIN_A3        //9V power supply voltage monitor.
#define PIN_5V_ADC          PIN_A4        //9V power supply voltage monitor.

#define PIN_P1_IOUT         PIN_A5        //OTMP Port 1 current monitor.
#define PIN_P2_IOUT         PIN_A6        //OTMP Port 2 current monitor.
#define PIN_P3_IOUT         PIN_A7        //OTMP Port 3 current monitor.
#define PIN_P4_IOUT         PIN_A8        //OTMP Port 4 current monitor.
#define PIN_P5_IOUT         PIN_A9        //OTMP Port 5 current monitor.

#define PIN_BNC_OUT_1       PIN_DAC0      //BNC digital-to-analog output #1.
#define PIN_BNC_OUT_2       PIN_DAC1      //BNC digital-to-analog output #2.
#define PIN_BNC_IN_1        PIN_A12       //BNC analog-to-digital input #1.
#define PIN_BNC_IN_2        PIN_A13       //BNC analog-to-digital input #2.

#define PIN_BOARD_ID        PIN_A14       //Board ID analog reference.


/*
 * Digital pins
 */

// Vulintus Peripheral Bus (VPB)
#define PIN_VPB_CLK_IN      (13u)         //PB02
#define PIN_VPB_CLK_OUT     (14u)         //PB01
#define PIN_VPB_TRG         (15u)         //PB00
#define PIN_VPB_BLOCK       (16u)         //PB03

// NINA W102 Module
#define PIN_NINA_RST        (17u)         //PC10
#define PIN_NINA_CS         (18u)         //PA06 
#define PIN_NINA_DEBUG      (19u)         //PA01
#define PIN_NINA_ACK        (20u)         //PA07
#define PIN_NINA_BOOT       (21u)         //PC07
#define NINA_GPIO0          PIN_NINA_BOOT
#define NINA_RESETN         PIN_NINA_RST
#define NINA_ACK            PIN_NINA_ACK

// TFT Display
#define PIN_TFT_RST         (26u)         //PC13
#define PIN_TFT_LED         (27u)         //PC11
#define PIN_TFT_DC          (28u)         //PB13
#define PIN_TFT_CS          (29u)         //PC14
#define TFT_RST             PIN_TFT_RST
#define TFT_LED             PIN_TFT_LED
#define TFT_DC              PIN_TFT_DC
#define TFT_CS              PIN_TFT_CS
#define TFT_WIDTH           160
#define TFT_HEIGHT          128
#define TFT_ROTATION        1

// RV-3208-C7 Real-Time Clock
#define PIN_RTC_INT         (33u)         //PC15
#define PIN_RTC_EVI         (34u)         //PA14
#define PIN_RTC_CLKO        (35u)         //PA00

// User Input
#define PIN_ENC_SW          (36u)         //PD10
#define PIN_ENC_A           (37u)         //PD09
#define PIN_ENC_B           (38u)         //PD08
#define PIN_QT_RST          (39u)         //PB12
#define PIN_QT_INT          (40u)         //PA04

// Piezo Speaker
#define PIN_SPKR            (45u)         //PB14

// Power Control
#define PIN_PWR_SHDN        (51u)         //PB17
#define PIN_I_IN_RESET      (52u)         //PB16
#define PIN_PWR_FAULT       (53u)         //PD21
#define PIN_9V_ON_FLAG      (54u)         //PD20

// OTMP Power Control
#define PIN_P1_24V_EN       (65u)         //PB19
#define PIN_P2_24V_EN       (66u)         //PB21
#define PIN_P3_24V_EN       (67u)         //PA21
#define PIN_P4_24V_EN       (68u)         //PB29
#define PIN_P5_24V_EN       (69u)         //PA27
#define PIN_P1_ALERT        (75u)         //PB26
#define PIN_P2_ALERT        (76u)         //PB27
#define PIN_P3_ALERT        (77u)         //PB28
#define PIN_P4_ALERT        (78u)         //PC24
#define PIN_P5_ALERT        (79u)         //PC25
#define PIN_P1_RST          (80u)         //PB20
#define PIN_P2_RST          (81u)         //PA20
#define PIN_P3_RST          (82u)         //PB23
#define PIN_P4_RST          (83u)         //PC26
#define PIN_P5_RST          (84u)         //PC03

// BNC I/O
#define PIN_BNC_SW_1        (89u)         //PC18
#define PIN_BNC_SW_2        (90u)         //PC19
#define PIN_BNC_EN_1        (91u)         //PC20
#define PIN_BNC_EN_2        (92u)         //PC21

// Fan Control
#define PIN_FAN             (93u)         //PA19


/*
 * Serial interfaces
 */

// Serial1, OTMP Port 1 (SERCOM0)
#define PIN_SERIAL1_TX      (55ul)        //PC17
#define PIN_SERIAL1_RX      (56ul)        //PC16
#define PAD_SERIAL1_TX      (UART_TX_PAD_0)
#define PAD_SERIAL1_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL1		  sercom0

// Serial2, OTMP Port 2 (SERCOM3)
#define PIN_SERIAL2_TX      (57ul)        //PC23
#define PIN_SERIAL2_RX      (58ul)        //PC22
#define PAD_SERIAL2_TX      (UART_TX_PAD_0)
#define PAD_SERIAL2_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL2		  sercom3

// Serial3, OTMP Port 3 (SERCOM5)
#define PIN_SERIAL3_TX      (59ul)        //PA23
#define PIN_SERIAL3_RX      (60ul)        //PA22
#define PAD_SERIAL3_TX      (UART_TX_PAD_0)
#define PAD_SERIAL3_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL3		  sercom5

// Serial4, OTMP Port 4 (SERCOM2)
#define PIN_SERIAL4_TX      (61ul)        //PB25
#define PIN_SERIAL4_RX      (62ul)        //PB24
#define PAD_SERIAL4_TX      (UART_TX_PAD_0)
#define PAD_SERIAL4_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL4		  sercom2

// Serial5, OTMP Port 5 (SERCOM7)
#define PIN_SERIAL5_TX      (63ul)        //PC12
#define PIN_SERIAL5_RX      (64ul)        //PB31
#define PAD_SERIAL5_TX      (UART_TX_PAD_0)
#define PAD_SERIAL5_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL5		  sercom7

// Serial6, NINA Programming Serial (SERCOM6)
#define PIN_SERIAL6_TX      (23ul)        //PC04
#define PIN_SERIAL6_RX      (22ul)        //PC05
#define PAD_SERIAL6_TX      (UART_TX_PAD_0)
#define PAD_SERIAL6_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL6		  sercom6


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MOSI        (0u)         //PC27 (S1.0)
#define PIN_SPI_MISO        (1u)         //PB22 (SA1.2) ????????????????????????????
#define PIN_SPI_SCK         (2u)         //PC28 (S1.1)
#define PERIPH_SPI          sercom1
#define PAD_SPI_TX          SPI_PAD_0_SCK_1       //?!?!
#define PAD_SPI_RX          SERCOM_RX_PAD_2       //?!?!

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
#define PIN_SPI1_MISO       (9u)          //PA09 - SD_DAT0, S0.1/SA2.0
#define PIN_SPI1_MOSI       (7u)          //PA08 - SD_CMD,  S0.0/SA2.1
#define PIN_SPI1_SCK        (8u)          //PB11 - SD_CLK,  SA4.3
#define PIN_SPI1_SS         (12u)         //PB10 - SD_DAT3, SA4.2
// #define PERIPH_SPI1   sercom2             
#define PAD_SPI1_TX   SPI_PAD_0_SCK_3   
#define PAD_SPI1_RX   SERCOM_RX_PAD_1     
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
#define WIRE_INTERFACES_COUNT 1

//I2C bus.
#define PIN_WIRE_SDA        (3u)          //PA13
#define PIN_WIRE_SCL        (4u)          //PA12
#define PERIPH_WIRE         sercom4
#define WIRE_IT_HANDLER     SERCOM4_Handler
// #define WIRE_IT_HANDLER_0   SERCOM4_0_Handler
// #define WIRE_IT_HANDLER_1   SERCOM4_1_Handler
// #define WIRE_IT_HANDLER_2   SERCOM4_2_Handler
// #define WIRE_IT_HANDLER_3   SERCOM4_3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// I2C Addresses.
#define I2C_ADDR_AT42QT2120   0x1C        // AT42QT2120 QTouch controller.
#define I2C_ADDR_MCP40D17     0x2E        // MCP40D17 digital potentiometer (for volume control).
#define I2C_ADDR_RV3028C7     0x52        // RV-3028-C7 real-time clock.
#define I2C_ADDR_ATECC608     0x60        // ATECC608x cryptographic co-processor.
#define I2C_ADDR_BME680       0x76        // BME680 temperature/pressure/humidity/gas sensor.


/*
 * USB
 */
#define PIN_USB_DM          (97ul)        //PA24
#define PIN_USB_DP          (98ul)        //PA25
#define PIN_USB_HOST_ENABLE (95ul)        //PA15
#define PIN_USB_DETECT      (96ul)        //PC06
#define USB_DETECT          PIN_USB_DETECT


/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0


/*
 * SDHC Interfaces (SDHC0)
*/
#define PIN_SD_DETECT       (5u)          //PD12 - SD card detection
#define PIN_SD_WP           (6u)          //PD11 - SD card write protection
#define PIN_SD_CMD          (7u)          //PA08 - Command line for SDHC operation.
#define PIN_SD_CLK          (8u)          //PB11 - Clock line for SDHC operation.
#define PIN_SD_DAT0         (9u)          //PA09 - Data 0 line for SDHC operation.
#define PIN_SD_DAT1         (10u)         //PA10 - Data 1 line for SDHC operation.
#define PIN_SD_DAT2         (11u)         //PA11 - Data 2 line for SDHC operation.
#define PIN_SD_DAT3         (12u)         //PB10 - Data 3 line for SDHC operation.
#define SD_CMD              PIN_SD_CMD
#define SD_CLK              PIN_SD_CLK
#define SD_DAT0             PIN_SD_DAT0
#define SD_DAT1             PIN_SD_DAT1
#define SD_DAT2             PIN_SD_DAT2
#define SD_DAT3             PIN_SD_DAT3


/*
 * Other pins?? 
 */
#define PIN_ATN             (40u)       //PA04 - I don't think we need this definition....
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
extern SERCOM sercom0;    //OTMP Port 1
extern SERCOM sercom1;    //SPI, SerialHCI
extern SERCOM sercom2;    //OTMP Port 4
extern SERCOM sercom3;    //OTMP Port 2
extern SERCOM sercom4;    //I2C
extern SERCOM sercom5;    //OTMP Port 3
extern SERCOM sercom6;    //SerialNina
extern SERCOM sercom7;    //OTMP Port 5

//OTMP serial ports.
extern Uart Serial1;
extern Uart Serial2;
extern Uart Serial3;
extern Uart Serial4;
extern Uart Serial5;

//NINA programming serial port.
extern Uart Serial6;

// SerialHCI - Required for NINA operation, no idea what it does.
extern Uart SerialHCI;
#define PIN_SERIALHCI_TX (0ul)           //PC27, NINA_MOSI, S1.0
#define PIN_SERIALHCI_RX (1ul)           //PB22, NINA_MISO, S7.2/SA1.2
#define PAD_SERIALHCI_TX (UART_TX_PAD_0)    
#define PAD_SERIALHCI_RX (SERCOM_RX_PAD_2)
#define PIN_SERIALHCI_RTS (18u)          //PA06 - NINA_CS,  SA0.2
#define PIN_SERIALHCI_CTS (2u)           //PC28 - NINA_SCK, S1.1

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
#define SERIAL_PORT_HARDWARE        Serial6
#define SERIAL_PORT_HARDWARE_OPEN   Serial6

// Alias Serial1 to SerialNina (useful in libraries)
#define SerialNina                  Serial6
#define SPIWIFI                     SPI

// Alias Serial to SerialUSB
#define SerialUSB                   Serial

#endif /* _VARIANT_OMNITRAK_CONTROLLER_V2_1_ */