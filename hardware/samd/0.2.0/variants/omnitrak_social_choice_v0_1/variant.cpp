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

  // OTMP (SERCOM2)
  { PORTA, 14, PIO_SERCOM,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14   }, //OTMP_TX       (0)  SERCOM / SERIAL
  { PORTA, 15, PIO_SERCOM,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15   }, //OTMP_RX       (1)  SERCOM / SERIAL

  // SPI Bus (SERCOM0)
  { PORTA, 6,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6    }, //SPI_PICO      (2)  SERCOM / SPI
  { PORTA, 5,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5    }, //SPI_POCI      (3)  SERCOM / SPI
  { PORTA, 7,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    }, //SPI_SCK       (4)  SERCOM / SPI

  // Display / Status
  { PORTA, 9,  PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9    }, //NEOPIX_3V3    (5)  DIGITAL OUT

  // Speaker
  { PORTA, 31, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11   }, //SPKR/SWDIO    (6)  DIGITAL OUT
  { PORTA, 1,  PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1    }, //PAM_EN_1      (7)  DIGITAL OUT
  { PORTA, 0,  PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0    }, //PAM_EN_2      (8)  DIGITAL OUT

  // User Input
  { PORTA, 30, PIO_EXTINT,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10   }, //nBTN_CW/SWCLK (9)  XINT10
  { PORTA, 11, PIO_EXTINT,     PIN_ATTR_DIGITAL, ADC_Channel5,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    }, //nBTN_CCW      (10) XINT11

  // Power Control
  { PORTA, 10, PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel18,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10   }, //VIN_ADC       (11) ADC AIN/18
  { PORTA, 4,  PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel4,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4    }, //5V_ADC        (12) ADC AIN/4

  // DRV8434S Stepper Driver
  { PORTA, 16, PIO_SERCOM,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0    }, //DRV_DIR       (13) DIGITAL OUT (DUMMY I2C SDA)
  { PORTA, 17, PIO_SERCOM,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1    }, //DRV_STEP      (14) DIGITAL OUT (DUMMY I2C SCL)
  { PORTA, 18, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, //nDRV_EN       (15) DIGITAL OUT
  { PORTA, 23, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    }, //nDRV_SLP      (16) DIGITAL OUT
  { PORTA, 19, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    }, //nDRV_CS       (17) DIGITAL OUT
  { PORTA, 28, PIO_EXTINT,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8    }, //nDRV_FLT      (18) XINT8
  { PORTA, 22, PIO_PWM,        PIN_ATTR_DIGITAL, No_ADC_Channel, PWM0_CH4,   TCC0_CH4,     EXTERNAL_INT_6    }, //DRV_VREF      (19) PWM

  // EEPROM
  { PORTA, 8,  PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI  }, //nEEPROM_CS    (20) DIGITAL OUT

  // Board ID
  { PORTA, 2,  PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, //BOARD_ID      (21) ADC AIN/0
  
  // USB
  { PORTA, 27, PIO_EXTINT,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15   }, //USB_DETECT    (22) XINT15 >> USB Host Enable
  { PORTA, 24, PIO_COM,       PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //USB_D_N       (23) USB DATA-
  { PORTA, 25, PIO_COM,       PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //USB_D_P       (24) USB DATA+

  // Analog Reference
  { PORTA, 3,  PIO_ANALOG,    PIN_ATTR_ANALOG,  No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    }, //VREF          (25) ANALOG REF

} ;


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


Uart Serial1( &sercom2, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX );
void SERCOM2_Handler()
{
  Serial1.IrqHandler();
}