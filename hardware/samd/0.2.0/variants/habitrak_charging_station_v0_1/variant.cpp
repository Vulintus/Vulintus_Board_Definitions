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

const PinDescription g_APinDescription[]=
{
//{ _EPortType, [Port Number], _EPioType, [Pin Attributes], _EAnalogChannel, _ETCChannel, _ETCChannel, EExt_Interrupts }

  // I2C Bus #0 (SERCOM2)
  { PORTA, 12, PIO_SERCOM,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12   }, //I2C_SDA0             (0)
  { PORTA, 13, PIO_SERCOM,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13   }, //I2C_SCL0             (1)

  // I2C Bus #1 (SERCOM3)
  { PORTA, 16, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0    }, //I2C_SDA1             (2)
  { PORTA, 17, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1    }, //I2C_SCL1             (3)

  // I2C Bus #2 (SERCOM5)
  { PORTA, 22, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6    }, //I2C_SDA2             (4)
  { PORTA, 23, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    }, //I2C_SCL2             (5)

  // I2C Bus #3 (SERCOM0)
  { PORTA, 8,  PIO_SERCOM,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI  }, //I2C_SDA3             (6)
  { PORTA, 9,  PIO_SERCOM,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9    }, //I2C_SCL3             (7)

  // I2C Bus #4 (SERCOM4)
  { PORTB, 8,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8    }, //I2C_SDA4             (8)
  { PORTB, 9,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9    }, //I2C_SCL4             (9)

  // I2C Bus #5 (SERCOM1)
  { PORTA, 0,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0    }, //I2C_SDA5             (10)
  { PORTA, 1,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1    }, //I2C_SCL5             (11)

  // Charger Enable.
  { PORTA, 14, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14   }, //EN0                  (12)
  { PORTA, 20, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4    }, //EN1                  (13)
  { PORTA, 18, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, //EN2                  (14)
  { PORTB, 11, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11   }, //EN3                  (15)
  { PORTB, 22, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6    }, //EN4                  (16)
  { PORTA, 27, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15   }, //EN5                  (17)

  // MCP Charger Status.
  { PORTA, 15, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15   }, //STATUS0              (18)
  { PORTA, 19, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    }, //STATUS1              (19)
  { PORTA, 21, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5    }, //STATUS2              (20)
  { PORTB, 10, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10   }, //STATUS3              (21)
  { PORTB, 23, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    }, //STATUS4              (22)
  { PORTA, 28, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8    }, //STATUS5              (23)

  // NTC Thermistors.
  { PORTA, 6,  PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel6,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6    }, //NTC0                 (24)
  { PORTA, 10, PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel18,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10   }, //NTC1                 (25)
  { PORTA, 11, PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel19,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11   }, //NTC2                 (26)
  { PORTA, 5,  PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel5,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5    }, //NTC3                 (27)
  { PORTB, 2,  PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel10,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, //NTC4                 (28)
  { PORTA, 2,  PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, //NTC5                 (29)

  // Power Control
  { PORTA, 31, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11   }, //nPGOOD/SWDIO         (30)
  { PORTA, 7,  PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel7,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    }, //VIN_ADC              (31)

  // Board ID
  { PORTA, 4,  PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel4,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4    }, //BOARD_ID             (32)
  
  // USB-C Power Delivery.
  { PORTB, 3,  PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    }, //nUSB_INT             (33)
  { PORTA, 30, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10   }, //mUSB_FAULT_3V3/SWCLK (34)

  //Programming
  // { PORTA, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11   }, //SWDIO
  // { PORTA, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10   }, //SWCLK

  // USB  
  { PORTA, 24, PIO_COM,        PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //USB_D_N              (35)
  { PORTA, 25, PIO_COM,        PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //USB_D_P              (36)

  // Analog Reference
  { PORTA, 3,  PIO_ANALOG,     PIN_ATTR_ANALOG,  No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //VREF                 (37) >> USB Host Enable

  // Required inclusion of DAC0
  { PORTA,  2, PIO_ANALOG,     PIN_ATTR_ANALOG,  DAC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, // DAC/VOUT            (38)

};


extern "C" {
    unsigned int PINCOUNT_fn() {
        return (sizeof(g_APinDescription) / sizeof(g_APinDescription[0]));
    }
}

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;