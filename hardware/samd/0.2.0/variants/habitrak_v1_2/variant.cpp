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

#include "variant.h"
#include "Arduino.h"

/*
 * Pins descriptions
 * TCC0 IOSET4
 * TCC1 IOSET2
 * TCC2 IOSET1
 * TCC3 IOSET1
 * TCC4 IOSET1
 */


const PinDescription g_APinDescription[]=
{
//{ _EPortType, [Port Number], _EPioType, [Pin Attributes], _EAnalogChannel, _ETCChannel, _ETCChannel, EExt_Interrupts }

  // SPI Bus (SERCOM2)
  { PORTB, 26, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SPI_PICO        (0)  SERCOM / SPI
  { PORTB, 29, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SPI_POCI        (1)  SERCOM / SPI
  { PORTB, 27, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SPI_SCK         (2)  SERCOM / SPI

  // I2C Bus (SERCOM7)
  { PORTD, 8,  PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //I2C_SDA         (3)  SERCOM / I2C
  { PORTD, 9,  PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //I2C_SCL         (4)  SERCOM / I2C
  
  // NINA W102 WiFi/Bluetooth Module
  { PORTA, 2,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //nNINA_RST       (5)  DIGITAL OUT
  { PORTD, 1,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //nNINA_BOOT      (6)  DIGITAL OUT
  { PORTC, 2,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, //NINA_ACK        (7)  EXTINT/2
  { PORTB, 4,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //nNINA_DEBUG     (8)  DIGITAL OUT
  { PORTB, 5,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //nNINA_CS        (9)  DIGITAL OUT

  // NINA W102 Programming Serial (SERCOM0)
  { PORTA, 4,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SAMD_TX/NINA_RX (10) SERCOM / SERIAL
  { PORTA, 5,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SAMD_RX/NINA_TX (11) SERCOM / SERIAL

  // SD Card (SDHC0)
  { PORTB, 12, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12   }, //nSD_DETECT      (12) EXTINT/12
  { PORTA, 8,  PIO_SDHC,       PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI  }, //SD_CMD          (13) SDHC SDCMD
  { PORTB, 11, PIO_SDHC,       PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SD_CLK          (14) SDHC SDCK
  { PORTA, 9,  PIO_SDHC,       PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SD_DAT0         (15) SDHC SDDAT/0
  { PORTA, 10, PIO_SDHC,       PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SD_DAT1         (16) SDHC SDDAT/1
  { PORTA, 11, PIO_SDHC,       PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SD_DAT2         (17) SDHC SDDAT/2
  { PORTB, 10, PIO_SDHC,       PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SD_DAT3         (18) SDHC SDDAT/3

  // SAMD51 Programming
  { PORTA, 31, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SAMD_SWDIO      (19) PROGRAMMING
  { PORTA, 30, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SAMD_SWCLK      (20) PROGRAMMING
  { PORTB, 30, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SAMD_SWO        (21) PROGRAMMING

  // TFT Display Digital I/O
  { PORTA, 15, PIO_DIGITAL,    PIN_ATTR_PWM_E,      No_ADC_Channel, TC3_CH1,    NOT_ON_TIMER, EXTERNAL_INT_NONE }, //TFT_LED         (22) PWM TC3 WO/1
  { PORTC, 20, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //TFT_RST         (23) DIGITAL OUT
  { PORTC, 18, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //TFT_DC          (24) DIGITAL OUT
  { PORTC, 16, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //nTFT_CS         (25) DIGITAL OUT

  // Status and Flash LEDs
  { PORTB, 17, PIO_DIGITAL,    PIN_ATTR_PWM_E,      No_ADC_Channel, TC6_CH1,    NOT_ON_TIMER, EXTERNAL_INT_NONE }, //LED_R           (26) PWM TC6 WO/1
  { PORTD, 10, PIO_DIGITAL,    PIN_ATTR_PWM_F,      No_ADC_Channel, TCC0_CH3,   NOT_ON_TIMER, EXTERNAL_INT_NONE }, //LED_G           (27) PWM TCC0 WO/3
  { PORTD, 12, PIO_DIGITAL,    PIN_ATTR_PWM_F,      No_ADC_Channel, TCC0_CH5,   NOT_ON_TIMER, EXTERNAL_INT_NONE }, //LED_B           (28) PWM TCC0 WO/5

  // Real-Time Clock Digital I/O
  { PORTD, 21, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11   }, //nRTC_INT        (29) EXTINT/11
  { PORTB, 16, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //RTC_EVI         (30) DIGITAL OUT
  { PORTA, 0,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //RTC_CLKO        (31) DIGITAL IN

  // User Input
  { PORTB, 0,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0    }, //BTN_U           (32) EXTINT/0
  { PORTC, 30, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14   }, //BTN_S           (33) EXTINT/14
  { PORTA, 1,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1    }, //BTN_D           (34) EXTINT/1
  { PORTB, 1,  PIO_ANALOG,     PIN_ATTR_ANALOG,     ADC_Channel13,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1    }, //BTN_U (ADC)     (35) ADC0 AIN/13  
  { PORTC, 31, PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel13,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15   }, //BTN_S (ADC)     (36) ADC1 AIN/13
  { PORTB, 3,  PIO_ANALOG,     PIN_ATTR_ANALOG,     ADC_Channel15,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    }, //BTN_D (ADC)     (37) ADC0 AIN/15
  { PORTC, 0,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //QT_MODE         (38) DIGITAL OUT

  // Infrared Flash
  { PORTB, 2,  PIO_DIGITAL,    PIN_ATTR_PWM_E,      No_ADC_Channel, TC6_CH0,    NOT_ON_TIMER, EXTERNAL_INT_NONE }, //LED_IR          (39) PWM TC6 WO/0

  // VLX53L0X Time-of-Flight Distance Sensor
  { PORTB, 9,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9    }, //VLX_GPIO        (40) EXTINT/9
  { PORTC, 6,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //VLX_XSHUT       (41) DIGITAL OUT

  // MAX16150 Pushbutton On/Off Controller
  { PORTC, 26, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10   }, //nMAX_INT        (42) EXTINT/10
  { PORTC, 24, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //MAX_CLR         (43) DIGITAL OUT

  // BH1749 Color Sensor
  { PORTC, 4,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4    }, //nRGB_INT        (44) EXTINT/4

  // BME688 Temperature/Pressure/Humidity/Gas Sensor
  { PORTB, 18, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //BME_CS          (45) DIGITAL OUT

  // BQ27441 Battery Fuel Gauge
  { PORTD, 20, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //BQ_GPO          (46) DIGITAL IN

  // Power Monitoring 
  { PORTB, 6,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel8,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //BAT_NTC         (47) ADC1 AIN/8
  { PORTB, 21, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5    }, //nQI_CHG_3V3     (48) EXTINT/5

  // LSM303D eCompass
  { PORTB, 22, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6    }, //LSM_INT1        (49) EXTINT/6
  { PORTB, 23, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    }, //LSM_INT2        (50) EXTINT/7
  { PORTC, 27, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //nLSM_CS         (51) DIGITAL OUT

  // SCCB Bus (SERCOM6)
  { PORTC, 13, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SCCB_SDA        (52) SERCOM / I2C 
  { PORTC, 12, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //SCCB_SCL        (53) SERCOM / I2C

  // OV7725 Camera
  { PORTC, 11, PIO_DIGITAL,    PIN_ATTR_TIMER_ALT,  No_ADC_Channel, NOT_ON_PWM, TCC1_CH5,     EXTERNAL_INT_NONE }, //CAM_XCLK        (54) TIMER TCC1 WO/5
  { PORTC, 15, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //nCAM_RST        (55) DIGITAL OUT
  { PORTC, 14, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_PWDN        (56) DIGITAL OUT
  { PORTA, 14, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_PCLK        (57) PCC CLK
  { PORTA, 16, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_D0          (58) PCC DATA/0
  { PORTA, 17, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_D1          (59) PCC DATA/1
  { PORTA, 18, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_D2          (60) PCC DATA/2
  { PORTA, 19, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_D3          (61) PCC DATA/3
  { PORTA, 20, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_D4          (62) PCC DATA/4
  { PORTA, 21, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_D5          (63) PCC DATA/5
  { PORTA, 22, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_D6          (64) PCC DATA/6
  { PORTA, 23, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_D7          (65) PCC DATA/7
  { PORTB, 14, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_D8          (66) PCC DATA/8
  { PORTB, 15, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_D9          (67) PCC DATA/9
  { PORTA, 12, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_VSYNC       (68) PCC DEN1
  { PORTA, 13, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //CAM_HSYNC       (69) PCC DEN2

  // Board ID
  { PORTA, 7,  PIO_ANALOG,     PIN_ATTR_ANALOG,     ADC_Channel7,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //BOARD_ID        (70) ADC0 AIN/7
  
  // USB
  { PORTB, 19, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    }, //USB_DETECT      (71) EXTINT/3
  { PORTA, 27, PIO_COM,        PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //USB_HOSTEN      (72) DIGITAL OUT
  { PORTA, 25, PIO_COM,        PIN_ATTR_NONE,       No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //USB_D_P         (73) USB DATA+
  { PORTA, 24, PIO_COM,        PIN_ATTR_NONE,       No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //USB_D_N         (74) USB DATA-


  // Analog Reference
  { PORTA, 3,  PIO_ANALOG,     PIN_ATTR_ANALOG,     ADC_Channel1,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //VREF            (75) ANALOG

} ;

extern "C" {
    unsigned int PINCOUNT_fn() {
        return (sizeof(g_APinDescription) / sizeof(g_APinDescription[0]));
    }
}

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TCC3, TCC4, TC0, TC1, TC2, TC3, TC4, TC5, TC6, TC7 } ;
const uint32_t GCLK_CLKCTRL_IDs[TCC_INST_NUM+TC_INST_NUM] = { TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID, TCC3_GCLK_ID, TCC4_GCLK_ID, TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID, TC5_GCLK_ID, TC6_GCLK_ID, TC7_GCLK_ID } ;

void initVariant() {
  // NINA - SPI boot
  pinMode(NINA_GPIO0, OUTPUT);
  digitalWrite(NINA_GPIO0, HIGH);

  // disable NINA
  pinMode(NINA_RESETN, OUTPUT);
  digitalWrite(NINA_RESETN, LOW);
}

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;
SERCOM sercom6( SERCOM6 ) ;
SERCOM sercom7( SERCOM7 ) ;

//NOTE: the SAMD51 has 4 total interrupt handles for each SERCOM, and you have 
//      to define them all, like so:
          // void SERCOM0_0_Handler()
          // {
          //   Serial1.IrqHandler();
          // }
          // void SERCOM0_1_Handler()
          // {
          //   Serial1.IrqHandler();
          // }
          // void SERCOM0_2_Handler()
          // {
          //   Serial1.IrqHandler();
          // }
          // void SERCOM0_3_Handler()
          // {
          //   Serial1.IrqHandler();
          // }
//      For the SAMD21, you only need to define one interrupt handler:
          // void SERCOM0_Handler()
          // {
          //   Serial1.IrqHandler();
          // }

//Serial 1 (NINA Programming Serial, SERCOM0) handler.
Uart Serial1( &sercom0, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;
void SERCOM0_0_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM0_1_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM0_2_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM0_3_Handler()
{
  Serial1.IrqHandler();
}

//Serial HCI (Something to do with the NINA, SERCOM2) handler.
Uart SerialHCI( &sercom2, PIN_SERIALHCI_RX, PIN_SERIALHCI_TX, PAD_SERIALHCI_RX, PAD_SERIALHCI_TX, PIN_SERIALHCI_RTS, PIN_SERIALHCI_CTS);
void SERCOM2_0_Handler()
{
  SerialHCI.IrqHandler();
}
void SERCOM2_1_Handler()
{
  SerialHCI.IrqHandler();
}
void SERCOM2_2_Handler()
{
  SerialHCI.IrqHandler();
}
void SERCOM2_3_Handler()
{
  SerialHCI.IrqHandler();
}