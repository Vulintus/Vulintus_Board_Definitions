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
 */
const PinDescription g_APinDescription[] =
{
  //{ [PORT], [PIN], ??, ??, [PWM CHANNEL], [TCC CHANNEL], ??}

  // I2C Bus (SERCOM3)
  { PORTA, 22, PIO_SERCOM,  PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6    }, //I2C_SDA       (0)
  { PORTA, 23, PIO_SERCOM,  PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    }, //I2C_SCL       (1)

  // 24VPB
  { PORTA, 7,  PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    },  //CLK_IN_3V3    (2)
  { PORTA, 6,  PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6    }, //CLK_OUT_3V3   (3)
  { PORTB, 23, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    }, //TRG_3V3       (4)
  { PORTA, 28, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8    }, //n24VPB_BLOCK  (5)

  // Programming
  { PORTA, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11   },  //SWDIO      (6)
  { PORTA, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10   },  //SWCLK      (7)

  // { PORTA, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 },  //BTN_C/SAMBA

  // Display / Status
  { PORTB, 10, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10   },  //OLED_RST    (8)
  { PORTA, 9,  PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9    }, //NEOPIX_3V3  (9)
  { PORTA, 12, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),    No_ADC_Channel, PWM2_CH0,   TCC0_CH6,     EXTERNAL_INT_12   }, //SPKR        (10)

  // User Input
  { PORTA, 13, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13   },  //BTN_L      (11)
  { PORTA, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15   },  //BTN_C      (12)
  { PORTA, 14, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14   },  //BTN_R      (13)

  // Power Control
  { PORTB, 8,  PIO_ANALOG,  PIN_ATTR_ANALOG,                    ADC_Channel2,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8    },        //VIN        (14)

  // DRV8824 Stepper Driver
  { PORTA, 18, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    },   //DRV_DIR    (15)
  { PORTA, 17, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1    },   //DRV_STEP   (16)
  { PORTA, 16, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0    },    //nDRV_EN    (17)
  { PORTA, 19, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    },   //DRV_MS0    (18)
  { PORTA, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4    },   //DRV_MS1    (19)
  { PORTA, 21, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5    },   //nDRV_FLT   (20)
  { PORTA, 5,  PIO_ANALOG,  PIN_ATTR_ANALOG,                    ADC_Channel5,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5    },   //DRV_VREF   (21)

  // Pump Selection (Left/Right).
  { PORTB, 11, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11   },  //PUMP_SEL   (22)

  // Water Detectors.
  { PORTA, 4,  PIO_ANALOG,  PIN_ATTR_ANALOG,                    ADC_Channel4,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4    },  //LOADCELL_1       (23)
  { PORTA, 10, PIO_ANALOG,  PIN_ATTR_ANALOG,                    ADC_Channel18,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10   }, //LOADCELL_VREF_1  (24)
  { PORTB, 9,  PIO_ANALOG,  PIN_ATTR_ANALOG,                    ADC_Channel3,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9    },  //LOADCELL_2       (25)
  { PORTB, 3,  PIO_ANALOG,  PIN_ATTR_ANALOG,                    ADC_Channel11,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    },  //LOADCELL_VREF_2  (26)
  { PORTA, 2,  PIO_ANALOG,  PIN_ATTR_ANALOG,                    ADC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    },  //DET_L            (27)
  { PORTA, 11, PIO_ANALOG,  PIN_ATTR_ANALOG,                    ADC_Channel19,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11   }, //DET_R            (28)
  { PORTA, 8,  PIO_ANALOG,  (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),    No_ADC_Channel, PWM0_CH0,   TCC1_CH2,     EXTERNAL_INT_NMI  },      //DET_INT          (29)

  // Board ID Analog Input
  { PORTB, 2,  PIO_ANALOG,  PIN_ATTR_ANALOG,                    ADC_Channel10,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },     //BOARD_ID  (31)
  
  // USB
  { PORTA, 27, PIO_DIGITAL, PIN_ATTR_DIGITAL,                   No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15   }, //USB_DETECT (32) >>USB Host Enable
  { PORTA, 24, PIO_COM,     PIN_ATTR_NONE,                      No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //USB_D_N    (33)
  { PORTA, 25, PIO_COM,     PIN_ATTR_NONE,                      No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //USB_D_P    (34)

  // Analog Reference
  { PORTA, 3,  PIO_ANALOG,  PIN_ATTR_ANALOG,                    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   //VREF       (35)
};

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM] = { TCC0, TCC1, TCC2, TC3, TC4, TC5 };

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

Uart Serial1( &sercom0, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM0_Handler()
{
  Serial1.IrqHandler();
}