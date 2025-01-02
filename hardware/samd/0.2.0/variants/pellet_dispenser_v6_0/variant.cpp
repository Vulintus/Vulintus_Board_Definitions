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


#include "variant.h", 

/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[] =
{
//{ _EPortType, [Port Number], _EPioType, [Pin Attributes], _EAnalogChannel, _ETCChannel, _ETCChannel, EExt_Interrupts }

  // I2C Bus (SERCOM3)
  { PORTA, 22, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //SDA          (0)  SERCOM / I2C
  { PORTA, 23, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //SCL          (1)  SERCOM / I2C

  // SPI Bus (SERCOM0)
  { PORTA, 10, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //SPI_PICO     (2)  SERCOM / SPI
  { PORTA, 8,  PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI }, //SPI_POCI     (3)  SERCOM / SPI
  { PORTA, 11, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //SPI_SCK      (4)  SERCOM / SPI

  // Vulintus Peripheral Bus (VPB)
  { PORTA, 14, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //CLK_IN_3V3   (5)  XINT14
  { PORTA, 12, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12  }, //CLK_OUT_3V3  (6)  DIGITAL OUT
  { PORTA, 5,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //TRG_3V3      (7)  XINT5
  { PORTA, 17, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1   }, //nVPB_BLOCK   (8)  DIGITAL OUT

  // Programming
  { PORTA, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //SWDIO        (9)  PROGRAMMING
  { PORTA, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //SWCLK        (10) PROGRAMMING

  // Display / Status
  { PORTB, 8,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8   }, //OLED_RST     (11) DIGITAL OUT
  { PORTA, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //NEOPIX_3V3   (12) DIGITAL OUT
  { PORTA, 27, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //SPKR         (13) DIGITAL OUT
  { PORTB, 23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //PAM_EN_1     (14) DIGITAL OUT
  { PORTB, 22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //PAM_EN_2     (15) DIGITAL OUT

  // User Input
  { PORTB, 3,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3  }, //nBTN          (16) XINT3

  // Power Control
  { PORTA, 2,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //5V_ADC       (17) ADC AIN/0
  { PORTA, 4,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel4,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //15V_ADC      (18) ADC AIN/4
  { PORTA, 6,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel6,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //24V_UVLO     (19) ADC AIN/6
  { PORTB, 9,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel3,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //n15V_ON_FLAG (20) ADC AIN/3

  // DRV8824 Stepper Driver
  { PORTA, 19, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //DRV_DIR      (21) DIGITAL OUT
  { PORTA, 18, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //DRV_STEP     (22) DIGITAL OUT
  { PORTB, 11, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //nDRV_EN      (23) DIGITAL OUT
  { PORTA, 21, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //nDRV_SLP     (24) DIGITAL OUT
  { PORTB, 10, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //nDRV_CS      (25) DIGITAL OUT
  { PORTA, 13, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13  }, //nDRV_FLT     (26) XINT13
  // { PORTA, 5,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel5,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //DRV_VREF     (xx) PWM

  // Pellet Detector.
  { PORTB, 2,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel10,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //SLOT_A       (27) ADC AIN/10
  { PORTA, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //SLOT_D       (28) XINT4

  // Dehumidifier.
  { PORTA, 16, PIO_DIGITAL, PIN_ATTR_ANALOG,  No_ADC_Channel, PWM2_CH0,   TCC2_CH0,     EXTERNAL_INT_0   }, //RDS_ON       (29) PWM TCC2/WO[0]
  { PORTA, 9,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel17,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //PA09         (30) ADC AIN/17

  // Board ID Analog Input
  { PORTA, 7,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel7,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //BOARD_ID     (31) ADC AIN/5
  
  // USB
  { PORTA, 28, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8   }, //USB_DETECT   (32) XINT8 >>USB Host Enable
  { PORTA, 24, PIO_COM,     PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12  }, //USB_D_N      (33) USB DATA-
  { PORTA, 25, PIO_COM,     PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13  }, //USB_D_P      (34) USB DATA+

  // Analog Reference
  { PORTA, 3,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel1,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3  }, //VREF          (35) ANALOG REF

    // Required inclusion of DAC0
  { PORTA, 2,  PIO_ANALOG,  PIN_ATTR_ANALOG,  DAC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2  }, //DAC/VOUT     (26) DAC0

};

extern "C" {
    unsigned int PINCOUNT_fn() {
        return (sizeof(g_APinDescription) / sizeof(g_APinDescription[0]));
    }
}

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM] = { TCC0, TCC1, TCC2, TC3, TC4, TC5 };

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

// Uart Serial1( &sercom0, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;
// Uart Serial( &sercom3, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX ) ;
// void SERCOM0_Handler()
// {
//   Serial1.IrqHandler();
// }

// void SERCOM3_Handler()
// {
//   Serial.IrqHandler();
// }