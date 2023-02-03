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
  { PORTB, 26, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12  }, //SPI_PICO        (0)  SERCOM / SPI
  { PORTB, 29, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //SPI_POCI        (1)  SERCOM / SPI
  { PORTB, 27, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13  }, //SPI_SCK         (2)  SERCOM / SPI

  // I2C Bus (SERCOM7)
  { PORTD, 8,  PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //I2C_SDA         (3)  SERCOM / I2C
  { PORTD, 9,  PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //I2C_SCL         (4)  SERCOM / I2C

  // QTouch Serial (SERCOM3)
  { PORTB, 20, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //QT_TX           (5)  SERCOM / SERIAL
  { PORTB, 21, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //QT_RX           (6)  SERCOM / SERIAL
  
  // NINA W102 WiFi/Bluetooth Module
  { PORTB, 5,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //nNINA_RST       (7)  DIGITAL OUT
  { PORTB, 7,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //nNINA_BOOT      (8)  DIGITAL OUT
  { PORTA, 2,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //NINA_ACK        (9)  XINT
  { PORTA, 7,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //nNINA_DEBUG     (10) DIGITAL OUT
  { PORTD, 1,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1   }, //nNINA_CS        (11) DIGITAL OUT

  // NINA W102 Programming Serial (SERCOM0)
  { PORTA, 4,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //SAMD_TX/NINA_RX (12) SERCOM / SERIAL
  { PORTA, 5,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //SAMD_RX/NINA_TX (13) SERCOM / SERIAL

  // SD Card (SDHC0)
  { PORTB, 12, PIO_DIGITAL,    PIN_ATTR_DIGITAL,     No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12  }, //nSD_DETECT     (14) XINT
  { PORTA, 8,  PIO_SDHC,       PIN_ATTR_DIGITAL,     No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI }, //SD_CMD         (15) SDHC SDCMD
  { PORTB, 11, PIO_SDHC,       PIN_ATTR_DIGITAL,     No_ADC_Channel, TC5_CH1,    TC5_CH1,      EXTERNAL_INT_11  }, //SD_CLK         (16) SDHC SDCK
  { PORTA, 9,  PIO_SDHC,       PIN_ATTR_DIGITAL,     No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //SD_DAT0        (17) SDHC SDDAT/0
  { PORTA, 10, PIO_SDHC,       PIN_ATTR_DIGITAL,     No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //SD_DAT1        (18) SDHC SDDAT/1
  { PORTA, 11, PIO_SDHC,       PIN_ATTR_DIGITAL,     No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //SD_DAT2        (19) SDHC SDDAT/2
  { PORTB, 10, PIO_SDHC,       PIN_ATTR_DIGITAL,     No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //SD_DAT3        (20) SDHC SDDAT/3

  // SAMD51 Programming
  { PORTA, 31, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //SAMD_SWDIO      (21) PROGRAMMING
  { PORTA, 30, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //SAMD_SWCLK      (22) PROGRAMMING
  { PORTB, 30, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //SAMD_SWO        (23) PROGRAMMING

  // TFT Display Digital I/O
  { PORTC, 22, PIO_DIGITAL,    PIN_ATTR_PWM_F,      No_ADC_Channel, TCC0_CH6,   NOT_ON_TIMER, EXTERNAL_INT_6   }, //TFT_LED         (24) PWM TCC0 WO/6
  { PORTC, 20, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //TFT_RST         (25) DIGITAL OUT
  { PORTC, 18, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //TFT_DC          (26) DIGITAL OUT
  { PORTC, 16, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0   }, //nTFT_CS         (27) DIGITAL OUT

  // Status and Flash LEDs
  { PORTB, 17, PIO_DIGITAL,    PIN_ATTR_PWM_G,      No_ADC_Channel, TCC0_CH5,   NOT_ON_TIMER, EXTERNAL_INT_1   }, //LED_R           (28) PWM TCC0 WO/5
  { PORTD, 10, PIO_DIGITAL,    PIN_ATTR_PWM_F,      No_ADC_Channel, TCC0_CH3,   NOT_ON_TIMER, EXTERNAL_INT_5   }, //LED_G           (29) PWM TCC0 WO/3
  { PORTB, 13, PIO_DIGITAL,    PIN_ATTR_PWM_G,      No_ADC_Channel, TCC0_CH1,   NOT_ON_TIMER, EXTERNAL_INT_13  }, //LED_B           (30) PWM TCC0 WO/1

  // Real-Time Clock Digital I/O
  { PORTD, 21, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //nRTC_INT        (31) XINT
  { PORTB, 16, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0   }, //RTC_EVI         (32) DIGITAL OUT
  { PORTA, 0,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0   }, //RTC_CLKO        (33) DIGITAL IN

  // User Input (Peripheral Touch Controller and Digital I/O)
  { PORTB, 0,  PIO_DIGITAL,    PIN_ATTR_PWM_E,      No_ADC_Channel, TC7_CH0,    NOT_ON_TIMER, EXTERNAL_INT_0   }, //U_BTN           (34) PTC
  { PORTA, 15, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //BTN_S           (35) XINT
  { PORTB, 1,  PIO_DIGITAL,    PIN_ATTR_PWM_E,      No_ADC_Channel, TC7_CH1,    NOT_ON_TIMER, EXTERNAL_INT_1   }, //D_BTN           (36) PTC
  { PORTB, 4,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //QT_MODE         (37) DIGITAL OUT
  { PORTC, 26, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //BTN_U_TEST      (38) XINT
  { PORTC, 24, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8   }, //BTN_D_TEST      (39) XINT
  { PORTA, 1,  PIO_DIGITAL,    PIN_ATTR_PWM_E,      No_ADC_Channel, TC6_CH0,    NOT_ON_TIMER, EXTERNAL_INT_2   }, //U_BTN_LED       (40) PWM TC6 WO/0
  { PORTC, 23, PIO_DIGITAL,    PIN_ATTR_PWM_F,      No_ADC_Channel, TCC0_CH7,   NOT_ON_TIMER, EXTERNAL_INT_3   }, //D_BTN_LED       (41) PWM TCC0 WO/7
  { PORTB, 2,  PIO_DIGITAL,    PIN_ATTR_PWM_E,      No_ADC_Channel, TC2_CH1,    NOT_ON_TIMER, EXTERNAL_INT_1   }, //LED_IR          (42) PWM TC2 WO/1

  // VLX53L0X Time-of-Flight Distance Sensor
  { PORTB, 9, PIO_DIGITAL,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //VLX_GPIO        (43) XINT
  { PORTC, 6, PIO_DIGITAL,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //VLX_XSHUT       (44) DIGITAL OUT

  // MAX16150 Pushbutton On/Off Controller
  { PORTC, 0, PIO_DIGITAL,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0   }, //nMAX_INT        (45) XINT
  { PORTC, 2, PIO_DIGITAL,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //MAX_CLR         (46) DIGITAL

  // BH1749 Color Sensor
  { PORTC, 4,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //nRGB_INT        (47) XINT

  // BME688 Temperature/Pressure/Humidity/Gas Sensor
  { PORTB, 18, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //BME_CS          (48) PIO_DIGITAL_28

  // BQ27441 Battery Fuel Gauge
  { PORTD, 20, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //BQ_GPO          (49) XINT (SHARES INTERRUPT CHANNEL WITH BTN_U_TEST)

  // Power Monitoring 
  { PORTB, 6,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel8,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //BAT_NTC         (50) ADC1 AIN/8
  { PORTB, 28, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //nQI_CHG_3V3     (51) XINT

  // LSM303D eCompass
  { PORTB, 22, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //LSM_INT1        (52) XINT
  { PORTB, 23, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //LSM_INT2        (53) XINT
  { PORTC, 27, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //nLSM_CS         (54) DIGITAL OUT

  // SCCB Bus (SERCOM6)
  { PORTC, 13, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13  }, //SCCB_SDA        (55) SERCOM / I2C 
  { PORTC, 12, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12  }, //SCCB_SCL        (56) SERCOM / I2C

  // OV7725 Camera
  { PORTC, 11, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //CAM_XCLK        (57) DIGITAL OUT
  { PORTC, 15, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //nCAM_RST        (58) DIGITAL OUT
  { PORTC, 14, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //CAM_PWDN        (59) DIGITAL OUT
  { PORTA, 14, PIO_DIGITAL,    PIN_ATTR_PWM_F,      No_ADC_Channel, TCC2_CH0,   NOT_ON_TIMER, EXTERNAL_INT_14  }, //CAM_PCLK        (60) PCC CLK
  { PORTA, 16, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0   }, //CAM_D0          (61) PCC DATA/0
  { PORTA, 17, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1   }, //CAM_D1          (62) PCC DATA/1
  { PORTA, 18, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //CAM_D2          (63) PCC DATA/2
  { PORTA, 19, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //CAM_D3          (64) PCC DATA/3
  { PORTA, 20, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //CAM_D4          (65) PCC DATA/4
  { PORTA, 21, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //CAM_D5          (66) PCC DATA/5
  { PORTA, 22, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //CAM_D6          (67) PCC DATA/6
  { PORTA, 23, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //CAM_D7          (68) PCC DATA/7
  { PORTB, 14, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //CAM_D8          (69) PCC DATA/8
  { PORTB, 15, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //CAM_D9          (70) PCC DATA/9
  { PORTA, 12, PIO_DIGITAL,    PIN_ATTR_PWM_E,      No_ADC_Channel, TC2_CH0,    TC2_CH0,      EXTERNAL_INT_12  }, //CAM_VSYNC       (71) PCC DEN1
  { PORTA, 13, PIO_DIGITAL,    PIN_ATTR_PWM_E,      No_ADC_Channel, TC2_CH1,    TC2_CH1,      EXTERNAL_INT_13  }, //CAM_HSYNC       (72) PCC DEN1

  // Board ID
  { PORTC, 30, PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel12,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //BOARD_ID        (73) ADC1 AIN/12
  
  // USB
  { PORTB, 19, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //USB_DETECT      (74) XINT
  { PORTA, 27, PIO_COM,        PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //USB_HOSTEN      (75) DIGITAL OUT
  { PORTA, 25, PIO_COM,        PIN_ATTR_NONE,       No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //USB_D_P         (76) USB DATA+
  { PORTA, 24, PIO_COM,        PIN_ATTR_NONE,       No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8   }, //USB_D_N         (77) USB DATA-


  // Analog Reference
  { PORTA, 3,  PIO_ANALOG,     PIN_ATTR_ANALOG,     ADC_Channel1,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //VREF            (78) ANALOG

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

//Serial 2 (QTouch Debugging Serial, SERCOM3) handler.
Uart Serial2( &sercom3, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX ) ;
void SERCOM3_0_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_2_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_3_Handler()
{
  Serial2.IrqHandler();
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