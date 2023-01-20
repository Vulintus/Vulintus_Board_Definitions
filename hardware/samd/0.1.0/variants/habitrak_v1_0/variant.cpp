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

  // SPI Bus (SERCOM2)
  { PORTB, 26, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 },   //SPI_MOSI    (0)   PIO_SERCOM_01
  { PORTB, 29, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },   //SPI_MISO    (1)   PIO_SERCOM_02
  { PORTB, 27, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 },   //SPI_SCK     (2)   PIO_SERCOM_03

  // I2C Bus (SERCOM7)
  { PORTD, 8, PIO_SERCOM, PIN_ATTR_PWM_F, No_ADC_Channel, TCC0_CH1, NOT_ON_TIMER, EXTERNAL_INT_3 },         //I2C_SDA     (3)   PIO_SERCOM_04
  { PORTD, 9, PIO_SERCOM, PIN_ATTR_PWM_F, No_ADC_Channel, TCC0_CH2, NOT_ON_TIMER, EXTERNAL_INT_4 },         //I2C_SCL     (4)   PIO_SERCOM_05

  // QTouch Serial (Serial1, SERCOM3)
  { PORTB, 20, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },    //QT_TX       (5)   PIO_SERCOM_06
  { PORTB, 21, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },    //QT_RX       (6)   PIO_SERCOM_07
  
  // ATWINC3400 Digital I/O
  { PORTB, 5, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },    //WINC_RST    (7)   PIO_DIGITAL_01
  { PORTB, 7, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },    //WINC_EN     (8)   PIO_DIGITAL_02
  { PORTA, 2, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },    //WINC_IRQ    (9)   PIO_DIGITAL_03
  { PORTD, 1, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },    //WINC_CS     (10)  PIO_DIGITAL_04

  // ATWINC3400 Bluetooth Serial (Serial2, SERCOM0)
  { PORTA, 5, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },     //WINC_BT_TX (Winc TX, SAMD RX)  (11)  PIO_SERCOM_08
  { PORTA, 4, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },     //WINC_BT_RX (Winc RX, SAMD TX)  (12)  PIO_SERCOM_09

  // ATWINC3400 Debug I2C (SERCOM1)
  { PORTC, 27, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },  //WINC_SDA_M  (13)  PIO_DIGITAL_05
  { PORTC, 28, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 },  //WINC_SCL_M  (14)  PIO_DIGITAL_06

  // SD Card Digital I/O
  { PORTB, 12, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 },         //SD_DETECT   (15)  PIO_COM_01
  { PORTA, 8,  PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI },        //SD_CMD      (16)  PIO_COM_02
  { PORTB, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },         //SD_CLK      (17)  PIO_COM_03
  { PORTA, 9,  PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },          //SD_DAT0     (18)  PIO_COM_04
  { PORTA, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },         //SD_DAT1     (19)  PIO_COM_05
  { PORTA, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },         //SD_DAT2     (20)  PIO_COM_06
  { PORTB, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },         //SD_DAT3     (21)  PIO_COM_07

  // USB (SERCOM3 or SERCOM5)
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },          //USB_D+      (22)  PIO_COM_08
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },          //USB_D-      (23)  PIO_COM_09
  { PORTB, 19, PIO_COM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },       //USB_DETECT  (24)  PIO_COM_10

  // Programming
  // { PORTA, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },  //SWDIO
  // { PORTA, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },  //SWCLK
  { PORTB, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },  //SWO         (25)  PIO_DIGITAL_07

  // TFT Display Digital I/O
  { PORTC, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },   //TFT_RST     (26)  PIO_DIGITAL_08
  { PORTC, 22, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC0_CH6, NOT_ON_TIMER, EXTERNAL_INT_6 },       //TFT_LED     (27)  PIO_DIGITAL_09
  { PORTC, 18, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },   //TFT_DC      (28)  PIO_DIGITAL_10
  { PORTC, 16, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },   //TFT_CS      (29)  PIO_DIGITAL_11

  // LEDs Digital I/O
  { PORTB, 13, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC0_CH1, NOT_ON_TIMER, EXTERNAL_INT_13 },      //STATUS_LED  (30)  PIO_DIGITAL_12
  { PORTB, 17, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC0_CH5, NOT_ON_TIMER, EXTERNAL_INT_1 },       //CHG_LED     (31)  PIO_DIGITAL_13

  // Real-Time Clock Digital I/O
  { PORTD, 21, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },  //RTC_INT     (32)  PIO_DIGITAL_14
  { PORTB, 16, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },   //RTC_EVI     (33)  PIO_DIGITAL_15
  { PORTA, 0,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },   //RTC_CLK     (34)  PIO_DIGITAL_16

  // User Input Touchpads Digital I/O
  { PORTB, 0,  PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC7_CH0, NOT_ON_TIMER, EXTERNAL_INT_0 },        //U_BTN       (35)  PIO_DIGITAL_17
  { PORTB, 1,  PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC7_CH1, NOT_ON_TIMER, EXTERNAL_INT_1 },        //D_BTN       (36)  PIO_DIGITAL_18
  { PORTB, 2,  PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC6_CH0, NOT_ON_TIMER, EXTERNAL_INT_2 },        //U_BTN_LED   (37)  PIO_DIGITAL_19
  { PORTB, 3,  PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC6_CH1, NOT_ON_TIMER, EXTERNAL_INT_3 },        //D_BTN_LED   (38)  PIO_DIGITAL_20
  { PORTA, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },  //S_BTN       (39)  PIO_DIGITAL_21
  { PORTB, 4,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },   //QT_MODE     (40)  PIO_DIGITAL_22

  // VLX53L0X Digital I/O
  { PORTB, 9, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },    //VLX_GPIO    (41)  PIO_DIGITAL_23
  { PORTC, 6, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },    //VLX_XSHUT   (42)  PIO_DIGITAL_24

  // MAX16150 Digital I/O
  { PORTC, 0, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },    //MAX_INT     (43)  PIO_DIGITAL_25
  { PORTC, 2, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },    //MAX_CLR     (44)  PIO_DIGITAL_26

  // BH1749 Digital I/O
  { PORTC, 4, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },    //RGB_INT     (45)  PIO_DIGITAL_27

  // BME688 Digital I/O
  { PORTB, 18, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },   //BME_CS      (46)  PIO_DIGITAL_28

  // BQ27441 Digital I/O
  { PORTD, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },  //BQ_GPO      (47)  PIO_DIGITAL_29

  // LSM303D Digital I/O
  { PORTB, 22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },   //LSM_INT1    (48)  PIO_DIGITAL_30
  { PORTB, 23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },   //LSM_INT2    (49)  PIO_DIGITAL_31
  { PORTB, 28, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },  //LSM_CS      (50)  PIO_DIGITAL_32

  // OV7725 PCC and Digital I/O
  { PORTC, 12, PIO_SERCOM, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH6, NOT_ON_TIMER, EXTERNAL_INT_12 },   //SCCB_SDA    (51)  PIO_SERCOM_10
  { PORTC, 13, PIO_SERCOM, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH7, NOT_ON_TIMER, EXTERNAL_INT_13 },   //SCCB_SCL    (52)  PIO_SERCOM_11
  { PORTA, 12, PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC2_CH0, TC2_CH0, EXTERNAL_INT_12 },            //CAM_VSYNC   (53)  PIO_DIGITAL_33
  { PORTA, 13, PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC2_CH1, TC2_CH1, EXTERNAL_INT_13 },            //CAM_HSYNC   (54)  PIO_DIGITAL_34
  { PORTA, 14, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC2_CH0, NOT_ON_TIMER, EXTERNAL_INT_14 },      //CAM_PCLK    (55)  PIO_DIGITAL_35
  { PORTC, 11, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },  //CAM_XCLK    (56)  PIO_DIGITAL_36
  { PORTC, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },  //CAM_RST     (57)  PIO_DIGITAL_37
  { PORTC, 14, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },  //CAM_PWDN    (58)  PIO_DIGITAL_38
  { PORTA, 16, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },   //CAM_D0      (59)  PIO_DIGITAL_39
  { PORTA, 17, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },   //CAM_D1      (60)  PIO_DIGITAL_40
  { PORTA, 18, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },   //CAM_D2      (61)  PIO_DIGITAL_41
  { PORTA, 19, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },   //CAM_D3      (62)  PIO_DIGITAL_42
  { PORTA, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },   //CAM_D4      (63)  PIO_DIGITAL_43
  { PORTA, 21, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },   //CAM_D5      (64)  PIO_DIGITAL_44
  { PORTA, 22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },   //CAM_D6      (65)  PIO_DIGITAL_45
  { PORTA, 23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },   //CAM_D7      (66)  PIO_DIGITAL_46
  { PORTB, 14, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },  //CAM_D8      (67)  PIO_DIGITAL_47
  { PORTB, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },  //CAM_D9      (68)  PIO_DIGITAL_48

  // Board ID Analog Input
  { PORTC, 30, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel12, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 }, //BOARD_ID    (69)  PIO_ANALOG_01
  
  // Analog Reference
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },      //VREF        (70)  PIO_ANALOG_02

  { PORTA, 27, PIO_COM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },  // USB Host enable (71)
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TCC3, TCC4, TC0, TC1, TC2, TC3, TC4, TC5, TC6, TC7 } ;
const uint32_t GCLK_CLKCTRL_IDs[TCC_INST_NUM+TC_INST_NUM] = { TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID, TCC3_GCLK_ID, TCC4_GCLK_ID, TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID, TC5_GCLK_ID, TC6_GCLK_ID, TC7_GCLK_ID } ;


// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;
SERCOM sercom6( SERCOM6 ) ;
SERCOM sercom7( SERCOM7 ) ;

Uart Serial1( &sercom3, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;
Uart Serial2( &sercom0, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX ) ;

void SERCOM3_0_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM3_1_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM3_2_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM3_3_Handler()
{
  Serial1.IrqHandler();
}


void SERCOM0_0_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM0_1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM0_2_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM0_3_Handler()
{
  Serial2.IrqHandler();
}
