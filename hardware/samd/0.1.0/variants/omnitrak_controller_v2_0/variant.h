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

#ifndef _VARIANT_OMNITRAK_CONTROLLER_V2_0_
#define _VARIANT_OMNITRAK_CONTROLLER_V2_0_

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
#define PINS_COUNT           (PINCOUNT_fn())
#define NUM_DIGITAL_PINS     (94u)
#define NUM_ANALOG_INPUTS    (15u)
#define NUM_ANALOG_OUTPUTS   (2u)
#define analogInputToDigitalPin(p) ((p < 5) ? 43 + (p) : (p < 10) ? 67 + (p) - 5 : (p < 14) ? 82 + (p) - 10 : (p == 14) ? 91 : -1)

/* 
  if (p < 5) {                          (p < 5) ?
    43 + (p);                           43 + (p)
  }             
  else if (p < 10) {                    : (p < 10) ?
    67 + (p) - 5;                       67 + (p) - 5
  }
  else if (p < 14) {                    : (p < 14) ?
    82 + (p) - 10;                      82 + (p) - 10
  }
  else if (p == 18) {                   : (p == 14) ?
    91;                                 91
  }
  else if (p == 18) {                   : (p == 15) ?
    91;                                 96
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
#define PIN_LED_R           (30u)        //Red channel of the RGB LED.
#define PIN_LED_G           (31u)        //Green channel of the RGB LED.
#define PIN_LED_B           (32u)        //Blue channel of the RGB LED.
#define LED_R               PIN_LED_R
#define LED_G               PIN_LED_G
#define LED_B               PIN_LED_B


/*
 * Analog pins
 */
#define PIN_A0              (43ul)        //V_IN_ADC   (43)
#define PIN_A1              (PIN_A0 + 1)  //I_IN_ADC   (44)
#define PIN_A2              (PIN_A0 + 2)  //9V_UVLO    (45)
#define PIN_A3              (PIN_A0 + 3)  //9V_ADC     (46)
#define PIN_A4              (PIN_A0 + 4)  //5V_ADC     (47)
#define PIN_A5              (67ul)        //P1_IOUT    (67)
#define PIN_A6              (PIN_A5 + 1)  //P2_IOUT    (68)
#define PIN_A7              (PIN_A5 + 2)  //P3_IOUT    (69)
#define PIN_A8              (PIN_A5 + 3)  //P4_IOUT    (70)
#define PIN_A9              (PIN_A5 + 4)  //P5_IOUT    (71)
#define PIN_A10             (82ul)        //BNC_OUT_1  (82)
#define PIN_A11             (PIN_A10 + 1) //BNC_OUT_2  (83)
#define PIN_A12             (PIN_A10 + 2) //BNC_IN_1   (84)
#define PIN_A13             (PIN_A10 + 3) //BNC_IN_2   (85)
#define PIN_A14             (91ul)        //BOARD_ID   (91)

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

#define PIN_V_IN_ADC        PIN_A0      //24V power supply voltage monitor.
#define PIN_I_IN_ADC        PIN_A1      //24V power supply current monitor.
#define PIN_9V_UVLO         PIN_A2      //Under-voltage lock-out level on the 9V power supply.
#define PIN_9V_ADC          PIN_A3      //9V power supply voltage monitor.
#define PIN_5V_ADC          PIN_A4      //9V power supply voltage monitor.

#define PIN_P1_IOUT         PIN_A5      //OTMP Port 1 current monitor.
#define PIN_P2_IOUT         PIN_A6      //OTMP Port 2 current monitor.
#define PIN_P3_IOUT         PIN_A7      //OTMP Port 3 current monitor.
#define PIN_P4_IOUT         PIN_A8      //OTMP Port 4 current monitor.
#define PIN_P5_IOUT         PIN_A9      //OTMP Port 5 current monitor.

#define PIN_BNC_OUT_1       PIN_DAC0    //BNC digital-to-analog output #1.
#define PIN_BNC_OUT_2       PIN_DAC1    //BNC digital-to-analog output #2.
#define PIN_BNC_IN_1        PIN_A12     //BNC analog-to-digital input #1.
#define PIN_BNC_IN_2        PIN_A13     //BNC analog-to-digital input #2.

#define PIN_BOARD_ID        PIN_A14     //Board ID analog reference.


/*
 * Digital pins
 */

// Vulintus Peripheral Bus (VPB)
#define PIN_VPB_CLK_IN      (13u)
#define PIN_VPB_CLK_OUT     (14u)
#define PIN_VPB_TRG         (15u)
#define PIN_VPB_BLOCK       (16u)

// NINA W102 Module
#define PIN_NINA_RST        (17u)
#define PIN_NINA_GPIO_5     (18u)
#define PIN_NINA_GPIO_8     (19u)
#define PIN_NINA_CS         (20u)
#define PIN_NINA_DEBUG      (21u)
#define PIN_NINA_ACK        (22u)
#define PIN_NINA_BOOT       (23u)
#define NINA_GPIO0          PIN_NINA_BOOT
#define NINA_RESETN         PIN_NINA_RST
#define NINA_ACK            PIN_NINA_ACK

// TFT Display
#define PIN_TFT_RST         (26u)
#define PIN_TFT_LED         (27u)
#define PIN_TFT_DC          (28u)
#define PIN_TFT_CS          (29u)

// RV-3208-C7 Real-Time Clock
#define PIN_RTC_INT         (33u)
#define PIN_RTC_EVI         (34u)
#define PIN_RTC_CLKO        (35u)

// User Input
#define PIN_ENC_SW          (36u)
#define PIN_ENC_A           (37u)
#define PIN_ENC_B           (38u)
#define PIN_QT_S_BTN        (39u)
#define PIN_QT_WHEEL1       (40u)
#define PIN_QT_WHEEL2       (41u)
#define PIN_QT_WHEEL3       (42u)

// Power Control
#define PIN_PWR_SHDN        (48u)
#define PIN_I_IN_RESET      (49u)
#define PIN_PWR_FAULT       (50u)
#define PIN_9V_ON_FLAG      (51u)

// OTMP Power Control
#define PIN_P1_24V_EN       (62u)
#define PIN_P2_24V_EN       (63u)
#define PIN_P3_24V_EN       (64u)
#define PIN_P4_24V_EN       (65u)
#define PIN_P5_24V_EN       (66u)
#define PIN_P1_ALERT        (72u)
#define PIN_P2_ALERT        (73u)
#define PIN_P3_ALERT        (74u)
#define PIN_P4_ALERT        (75u)
#define PIN_P5_ALERT        (76u)
#define PIN_P1_RST          (77u)
#define PIN_P2_RST          (78u)
#define PIN_P3_RST          (79u)
#define PIN_P4_RST          (80u)
#define PIN_P5_RST          (81u)

// BNC I/O
#define PIN_BNC_SW_1        (86u)       
#define PIN_BNC_SW_2        (87u)   
#define PIN_BNC_EN_1        (88u)
#define PIN_BNC_EN_2        (89u)

// Fan Control
#define PIN_FAN             (90u)


/*
 * Serial interfaces
 */

// Serial1, OTMP Port 1 (SERCOM0)
#define PIN_SERIAL1_TX      (52ul)        //PC17
#define PIN_SERIAL1_RX      (53ul)        //PC16
#define PAD_SERIAL1_TX      (UART_TX_PAD_0)
#define PAD_SERIAL1_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL1		  sercom0

// Serial2, OTMP Port 2 (SERCOM3)
#define PIN_SERIAL2_TX      (54ul)        //PC23
#define PIN_SERIAL2_RX      (55ul)        //PC22
#define PAD_SERIAL2_TX      (UART_TX_PAD_0)
#define PAD_SERIAL2_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL2		  sercom3

// Serial3, OTMP Port 3 (SERCOM5)
#define PIN_SERIAL3_TX      (56ul)        //PA23
#define PIN_SERIAL3_RX      (57ul)        //PA22
#define PAD_SERIAL3_TX      (UART_TX_PAD_0)
#define PAD_SERIAL3_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL3		  sercom5

// Serial4, OTMP Port 4 (SERCOM2)
#define PIN_SERIAL4_TX      (58ul)        //PB25
#define PIN_SERIAL4_RX      (59ul)        //PB24
#define PAD_SERIAL4_TX      (UART_TX_PAD_0)
#define PAD_SERIAL4_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL4		  sercom2

// Serial5, OTMP Port 5 (SERCOM7)
#define PIN_SERIAL5_TX      (60ul)        //PB30
#define PIN_SERIAL5_RX      (61ul)        //PB31
#define PAD_SERIAL5_TX      (UART_TX_PAD_0)
#define PAD_SERIAL5_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL5		  sercom7

// Serial6, NINA Programming Serial (SERCOM6)
#define PIN_SERIAL6_TX      (25ul)        //PC04
#define PIN_SERIAL6_RX      (24ul)        //PC05
#define PAD_SERIAL6_TX      (UART_TX_PAD_0)
#define PAD_SERIAL6_RX      (SERCOM_RX_PAD_1)
#define SERCOM_SERIAL6		  sercom6


/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MOSI        (0u)         //PC27 (S1.0)
#define PIN_SPI_MISO        (1u)         //PA30 (SA1.2)
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
#define PIN_SPI1_MISO       (9u)        //SD_DAT0,  PA09, S0.1/SA2.0
#define PIN_SPI1_MOSI       (7u)        //SD_CMD,   PA08, S0.0/SA2.1
#define PIN_SPI1_SCK        (8u)        //SD_CLK,   PB11, SA4.3
#define PIN_SPI1_SS         (12u)       //SD_DAT3,  PB10, SA4.2
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
#define PIN_WIRE_SDA        (3u)
#define PIN_WIRE_SCL        (4u)
#define PERIPH_WIRE         sercom4
#define WIRE_IT_HANDLER     SERCOM4_Handler
// #define WIRE_IT_HANDLER_0   SERCOM4_0_Handler
// #define WIRE_IT_HANDLER_1   SERCOM4_1_Handler
// #define WIRE_IT_HANDLER_2   SERCOM4_2_Handler
// #define WIRE_IT_HANDLER_3   SERCOM4_3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;


/*
 * USB
 */
#define PIN_USB_DM          (94ul)
#define PIN_USB_DP          (95ul)
#define PIN_USB_HOST_ENABLE (92ul)
#define PIN_USB_DETECT      (93ul)
#define USB_DETECT          PIN_USB_DETECT


/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0


/*
 * SDHC Interfaces (SDHC0)
*/
#define PIN_SD_DETECT       (5u)         //SD card detection
#define PIN_SD_WP           (6u)         //SD card write protection
#define PIN_SD_CMD          (7u)         //Command line for SDHC operation.
#define PIN_SD_CLK          (8u)         //Clock line for SDHC operation.
#define PIN_SD_DAT0         (9u)         //Data 0 line for SDHC operation.
#define PIN_SD_DAT1         (10u)        //Data 1 line for SDHC operation.
#define PIN_SD_DAT2         (11u)        //Data 2 line for SDHC operation.
#define PIN_SD_DAT3         (12u)        //Data 3 line for SDHC operation.
#define SD_CMD              PIN_SD_CMD
#define SD_CLK              PIN_SD_CLK
#define SD_DAT0             PIN_SD_DAT0
#define SD_DAT1             PIN_SD_DAT1
#define SD_DAT2             PIN_SD_DAT2
#define SD_DAT3             PIN_SD_DAT3


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
#define PIN_SERIALHCI_TX (0ul)           //NINA_MOSI, PC27, S1.0
#define PIN_SERIALHCI_RX (1ul)           //NINA_MISO, PA30, S7.2/SA1.2
#define PAD_SERIALHCI_TX (UART_TX_PAD_0)    
#define PAD_SERIALHCI_RX (SERCOM_RX_PAD_2)
#define PIN_SERIALHCI_RTS (20u)          //NINA_CS,  PA06, SA0.2
#define PIN_SERIALHCI_CTS (2u)           //NINA_SCK, PC28, S1.1

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

// Alias Serial1 to SerialNina (useful in libraries)
#define SerialNina                  Serial6
#define SPIWIFI                     SPI

// Alias Serial to SerialUSB
#define SerialUSB                   Serial

#endif /* _VARIANT_OMNITRAK_CONTROLLER_V2_0_ */