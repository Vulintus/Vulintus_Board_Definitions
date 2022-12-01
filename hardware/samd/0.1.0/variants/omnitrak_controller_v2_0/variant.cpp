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
//{ .ulPort, .ulPin, ??, ??, .ulPWMChannel, .ulTCChannel, ??}

  // SPI Bus (SERCOM1)
  { PORTC, 27, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },   //SPI_COPI   (0)
  { PORTA, 30, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },   //SPI_CIPO   (1)
  { PORTC, 28, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 },   //SPI_SCK    (2)

  // I2C Bus (SERCOM4)
  { PORTA, 13, PIO_SERCOM_ALT, PIN_ATTR_PWM_F, No_ADC_Channel, TCC0_CH1, NOT_ON_TIMER, EXTERNAL_INT_13 },  //I2C_SDA     (3)
  { PORTA, 12, PIO_SERCOM_ALT, PIN_ATTR_PWM_F, No_ADC_Channel, TCC0_CH2, NOT_ON_TIMER, EXTERNAL_INT_12 },  //I2C_SCL     (4)

  // SD Card  
  { PORTD, 12, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },    //nSD_DETECT   (5)
  { PORTD, 11, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },    //nSD_WP       (6)
  { PORTA, 8,  PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI },  //SD_CMD       (7)
  { PORTB, 11, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },   //SD_CLK       (8)
  { PORTA, 9,  PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },    //SD_DAT0      (9)
  { PORTA, 10, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },   //SD_DAT1      (10)
  { PORTA, 11, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },   //SD_DAT2      (11)
  { PORTB, 10, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },   //SD_DAT3      (12)

  // 24VPB
  { PORTB, 2, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },  //CLK_IN_3V3   (13)
  { PORTB, 1, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },  //CLK_OUT_3V3  (14)
  { PORTB, 0, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },  //TRG_3V3      (15)
  { PORTB, 3, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },  //n24VPB_BLOCK (16)

  //NINA W102
  { PORTC, 6, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },  //nNINA_RST    (17)
  { PORTA, 4, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },  //NINA_GPIO_5  (18)  
  { PORTC, 3, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },  //NINA_GPIO_8  (19)
  { PORTA, 6, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },  //nNINA_CS     (20)
  { PORTA, 1, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },  //nNINA_DEBUG  (21)
  { PORTA, 7, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },  //NINA_ACK     (22)
  { PORTC, 7, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },  //nNINA_BOOT   (23)

  // NINA Programming Serial (SERCOM6)
  { PORTC, 5, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },   //SAMD_RX/NINA_TX_PROG  (24)
  { PORTC, 4, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },   //SAMD_TX/NINA_RX_PROG  (25)

  // Programming
  // { PORTA, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },  //SWDIO
  // { PORTA, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },  //SWCLK
  // { PORTB, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },  //SWO
  // { PORTA, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },  //nSAMBA

  // TFT Display
  { PORTC, 13, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 },   //nTFT_RST  (26)
  { PORTC, 11, PIO_DIGITAL, PIN_ATTR_PWM_F,   No_ADC_Channel, TCC0_CH1,   TCC0_CH1, EXTERNAL_INT_11 },       //TFT_LED   (27)
  { PORTC, 12, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 },   //TFT_DC    (28)
  { PORTC, 14, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },   //nTFT_CS   (29) *

  // Status LED (RGB)
  { PORTA, 18, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH6, TCC0_CH6, EXTERNAL_INT_2 },           //LED_R      (30)
  { PORTA, 17, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH5, TCC0_CH5, EXTERNAL_INT_1 },           //LED_G      (31)  
  { PORTA, 16, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH4, TCC0_CH4, EXTERNAL_INT_0 },           //LED_B      (32)

  // RV-3208-C7 Real-Time Clock
  { PORTC, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, //nRTC_INT    (33)
  { PORTA, 14, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 }, //nRTC_EVI    (34)
  { PORTA, 0,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },  //RTC_CLKO    (35)

  // User Input
  { PORTD, 10, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },  //nENC_SW     (36)
  { PORTD, 9,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },  //ENC_A       (37)
  { PORTD, 8,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },  //ENC_B       (38)
  { PORTB, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, TC5_CH1,    TC5_CH1,      EXTERNAL_INT_15 }, //QT_S_BTN    (39)
  { PORTB, 14, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, TC5_CH0,    TC5_CH0,      EXTERNAL_INT_14 }, //QT_WHEEL1   (40)
  { PORTB, 13, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, TC4_CH1,    TC4_CH1,      EXTERNAL_INT_13 }, //QT_WHEEL2   (41)
  { PORTB, 12, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, TC4_CH0,    TC4_CH0,      EXTERNAL_INT_12 }, //QT_WHEEL3   (42)

  // Power Control
  { PORTC, 30, PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel12,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 }, //V_IN_ADC    (43)
  { PORTD, 0,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel14,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },  //I_IN_ADC    (44)
  { PORTB, 5,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel7,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },  //9V_UVLO     (45)
  { PORTB, 4,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel6,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },  //9V_ADC      (46)
  { PORTB, 6,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel8,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },  //5V_ADC      (47)
  { PORTB, 17, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },  //nPWR_SHDN   (48)
  { PORTB, 16, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },  //I_IN_RESET  (49)
  { PORTD, 21, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, //nPWR_FAULT  (50)
  { PORTD, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, //n9V_ON_FLAG (51)

  // OTMP Port 1 (SERCOM0)
  { PORTC, 17, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },  //P1_TX    (52)
  { PORTC, 16, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },  //P1_RX    (53)

  // OTMP Port 2 (SERCOM3)
  { PORTC, 23, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },  //P2_TX    (54)
  { PORTC, 22, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },  //P2_RX    (55)

  // OTMP Port 3 (SERCOM5)
  { PORTA, 23, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },  //P3_TX    (56)
  { PORTA, 22, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },  //P3_RX    (57)

  // OTMP Port 4 (SERCOM2)
  { PORTB, 25, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },  //P4_TX    (58)
  { PORTB, 24, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },  //P4_RX    (59)

  // OTMP Port 5 (SERCOM7)
  { PORTB, 30, PIO_SERCOM,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 }, //P5/QT_TX (60)
  { PORTB, 31, PIO_SERCOM,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, ///QTP5_RX (61)

  // OTMP Power Control
  { PORTB, 19, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },   //P1_24V_EN  (62)
  { PORTB, 21, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },   //P2_24V_EN  (63)
  { PORTA, 21, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5},    //P3_24V_EN  (64)
  { PORTB, 29, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },  //P4_24V_EN  (65)
  { PORTA, 27, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },  //P5_24V_EN  (66)

  { PORTD, 1,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel15, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },      //P1_IOUT    (67)
  { PORTC, 31, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel13, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },     //P2_IOUT    (68)
  { PORTC, 0,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },      //P3_IOUT    (69)
  { PORTC, 1,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel11, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },      //P4_IOUT    (70)
  { PORTC, 2,  PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel4,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },      //P5_IOUT    (71)

  { PORTB, 26, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 },  //nP1_ALERT  (72)
  { PORTB, 27, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 },  //nP2_ALERT  (73)
  { PORTB, 28, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },  //nP3_ALERT  (74)
  { PORTC, 24, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },   //nP4_ALERT  (75)
  { PORTC, 25, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },   //nP5_ALERT  (76)

  { PORTB, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },   //P1_RST     (77)
  { PORTA, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },   //P2_RST     (78)
  { PORTB, 23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },   //P3_RST     (79)
  { PORTC, 26, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },  //P4_RST     (80)
  { PORTA, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },  //P5_RST     (81)

  // BNC I/O
  { PORTA, 2,  PIO_ANALOG,  PIN_ATTR_ANALOG,  DAC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },   //BNC_OUT_1  (82)
  { PORTA, 5,  PIO_ANALOG,  PIN_ATTR_ANALOG,  DAC_Channel1,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },   //BNC_OUT_2  (83)
  { PORTB, 9,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel3,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },   //BNC_IN_1   (84)
  { PORTB, 8,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel2,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },   //BNC_IN_2   (85)
  { PORTC, 18, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },   //BNC_SW_1   (86)
  { PORTC, 19, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },   //BNC_SW_2   (87)
  { PORTC, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },   //BNC_EN_1   (88)
  { PORTC, 21, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },   //BNC_EN_2   (89)

  // Fan Control
  { PORTA, 19, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH7, TCC0_CH7, EXTERNAL_INT_3 },           //FAN        (90)

  // Board ID Analog Input
  { PORTB, 7, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel9, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },        //BOARD_ID   (91)
  
  // USB
  { PORTA, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },  //nSAMBA     (92) >>USB Host Enable
  { PORTB, 22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },   //USB_DETECT (93)
  { PORTA, 24, PIO_COM,     PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },   //USB_D_N    (94)
  { PORTA, 25, PIO_COM,     PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },   //USB_D_P    (95)

  // Analog Reference
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },      //VREF       (96)

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

Uart Serial1( &sercom0, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ; //OTMP Port 1
Uart Serial2( &sercom3, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX ) ; //OTMP Port 2
Uart Serial3( &sercom5, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX ) ; //OTMP Port 3
Uart Serial4( &sercom2, PIN_SERIAL4_RX, PIN_SERIAL4_TX, PAD_SERIAL4_RX, PAD_SERIAL4_TX ) ; //OTMP Port 4
Uart Serial5( &sercom7, PIN_SERIAL5_RX, PIN_SERIAL5_TX, PAD_SERIAL5_RX, PAD_SERIAL5_TX ) ; //OTMP Port 5
Uart Serial6( &sercom6, PIN_SERIAL6_RX, PIN_SERIAL6_TX, PAD_SERIAL6_RX, PAD_SERIAL6_TX ) ; //NINA W102 Programming

//Serial 1 (OTMP Port 1, SERCOM0) handlers.
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

//Serial 2 (OTMP Port 2, SERCOM3) handlers.
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

//Serial 3 (OTMP Port 3, SERCOM5) handlers.
void SERCOM5_0_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM5_1_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM5_2_Handler()
{
  Serial3.IrqHandler();
}
void SERCOM5_3_Handler()
{
  Serial3.IrqHandler();
}

//Serial 4 (OTMP Port 4, SERCOM2) handlers.
void SERCOM2_0_Handler()
{
  Serial4.IrqHandler();
}
void SERCOM2_1_Handler()
{
  Serial4.IrqHandler();
}
void SERCOM2_2_Handler()
{
  Serial4.IrqHandler();
}
void SERCOM2_3_Handler()
{
  Serial4.IrqHandler();
}

//Serial 5 (OTMP Port 5, SERCOM7) handlers.
void SERCOM7_0_Handler()
{
  Serial5.IrqHandler();
}
void SERCOM7_1_Handler()
{
  Serial5.IrqHandler();
}
void SERCOM7_2_Handler()
{
  Serial5.IrqHandler();
}
void SERCOM7_3_Handler()
{
  Serial5.IrqHandler();
}

//Serial 6 (NINA W102 Programming, SERCOM6) handlers.
void SERCOM6_0_Handler()
{
  Serial6.IrqHandler();
}
void SERCOM6_1_Handler()
{
  Serial6.IrqHandler();
}
void SERCOM6_2_Handler()
{
  Serial6.IrqHandler();
}
void SERCOM6_3_Handler()
{
  Serial6.IrqHandler();
}