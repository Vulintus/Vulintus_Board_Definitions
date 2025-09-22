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
  { PORTA, 22, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //SDA             (0)  SERCOM / I2C
  { PORTA, 23, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //SCL             (1)  SERCOM / I2C

  // Vulintus Peripheral Bus (VPB)
  { PORTA, 7,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //CLK_IN_3V3      (2)  XINT
  { PORTA, 6,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //CLK_OUT_3V3     (3)  DIGITAL OUT
  { PORTA, 21, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //TRG_3V3         (4)  XINT
  { PORTA, 28, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8   }, //nVPB_BLOCK      (5)  DIGITAL OUT

  // Programming
  { PORTA, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //SWDIO           (6)  PROGRAMMING / USB_HOST_ENABLE
  { PORTA, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //SWCLK           (7)  PROGRAMMING

  // Display / Status
  { PORTB, 10, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //OLED_RST        (8)  DIGITAL OUT
  { PORTA, 11, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //NEOPIX_3V3      (9)  DIGITAL OUT
  { PORTB, 11, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, PWM0_CH5,   TCC0_CH5,     EXTERNAL_INT_11  }, //SPKR            (10) PWM TCC0 WO/6

  // User Input
  { PORTA, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //BTN_L           (11) XINT
  { PORTB, 22, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //BTN_C           (12) XINT
  { PORTA, 16, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0   }, //BTN_R           (13) XINT

  // Power Control
  { PORTB, 3,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel11,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //VIN             (14) ADC AIN/11

  // DRV8824 Stepper Driver
  { PORTA, 19, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //DRV_DIR         (15) DIGITAL OUT
  { PORTA, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //DRV_STEP        (16) DIGITAL OUT
  { PORTB, 23, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //nDRV_EN         (17) DIGITAL OUT
  { PORTA, 18, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //DRV_MS0         (18) DIGITAL OUT
  { PORTA, 14, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //DRV_MS1         (19) DIGITAL OUT
  { PORTA, 13, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13  }, //nDRV_FLT        (20) XINT
  { PORTA, 5,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel5,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //DRV_VREF        (21) ADC AIN/5

  // Pump Selection (Left/Right).
  { PORTA, 12, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12  }, //PUMP_SEL_3V3    (22) DIGITAL OUT

  // Water Detectors.
  { PORTB, 9,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel3,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //LOADCELL_L      (23) ADC AIN/5
  { PORTA, 9,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel17,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //LC_L_VREF       (24) ADC AIN/5
  { PORTA, 4,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel4,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //LOADCELL_R      (25) ADC AIN/4
  { PORTA, 10, PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel18,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //LC_R_VREF       (26) ADC AIN/5
  { PORTA, 2,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //LIQ_DET_L       (27) ADC AIN/5
  { PORTB, 8,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel19,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //LIQ_DET_R       (28) ADC AIN/5
  { PORTA, 8,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, PWM1_CH2,   TCC1_CH2,     EXTERNAL_INT_NMI }, //LIQ_DET_INT     (29) PWM TCC0 WO/6

  // Board ID Analog Input
  { PORTB, 2,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel10,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //BOARD_ID        (30) ADC AIN/5
  
  // USB
  { PORTA, 17, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1   }, //USB_DETECT      (31) XINT  
  { PORTA, 24, PIO_COM,     PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12  }, //USB_D_N         (32) USB DATA-
  { PORTA, 25, PIO_COM,     PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13  }, //USB_D_P         (33) USB DATA+


  // Analog Reference
  { PORTA, 3,  PIO_ANALOG,  PIN_ATTR_ANALOG,  No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3  }, //VREF             (34) ANALOG

  // Unused pins
  { PORTA, 27, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //               (35) 

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