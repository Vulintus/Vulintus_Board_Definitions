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

  // OTMP (SERCOM3)
  { PORTA, 22, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //OTMP_TX       (0)
  { PORTA, 23, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //OTMP_RX       (1)

  // SPI Bus (SERCOM0)
  { PORTA, 10, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10   }, //SPI_PICO     (2)
  { PORTA, 9,  PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9    }, //SPI_POCI     (3)
  { PORTA, 11, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11   }, //SPI_SCK      (4)

  // I2C Bus (SERCOM1)
  { PORTA, 16, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0    }, //I2C_SDA      (5)
  { PORTA, 17, PIO_SERCOM,  PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1    }, //I2C_SCL      (6)

  // Display / Status
  { PORTA, 0,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, TCC2_CH0,     EXTERNAL_INT_0    }, //OLED_RST     (7)
  { PORTA, 1,  PIO_PWM,     (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER),   No_ADC_Channel, PWM2_CH1,   TCC2_CH1,     EXTERNAL_INT_1    }, //LED_R        (8)
  { PORTA, 31, PIO_PWM,     (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER),   No_ADC_Channel, PWM1_CH1,   TCC1_CH1,     EXTERNAL_INT_11   }, //LED_G/SWDIO  (9)
  { PORTA, 30, PIO_PWM,     (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER),   No_ADC_Channel, PWM1_CH0,   TCC1_CH0,     EXTERNAL_INT_10   }, //LED_B/SWCLK  (10)

  // Speaker
  { PORTA, 8,  PIO_PWM,     (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER),   No_ADC_Channel, PWM5_CH1,   TC5_CH1,      EXTERNAL_INT_NMI  }, //SPKR         (11)
  { PORTA, 15, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15   }, //PAM_EN_1     (12)
  { PORTA, 14, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14   }, //PAM_EN_2     (13)

  // User Input
  { PORTA, 7,  PIO_EXTINT,  PIN_ATTR_DIGITAL, ADC_Channel5,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7    }, //BTN          (14)

  // Power Control
  { PORTA, 2,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, //VIN_ADC      (15)

  // PMW3389 Optical Mouse Sensors
  { PORTA, 6,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6    }, //nPMW_RST     (16)
  { PORTA, 27, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15   }, //nPMW_CS_1    (17)
  { PORTA, 18, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, //nPMW_CS_2    (18)
  { PORTA, 19, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3    }, //PMW_MOT_1    (19)
  { PORTA, 4,  PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4    }, //PMW_MOT_2    (20)

  // Board ID
  { PORTA, 5,  PIO_ANALOG,  PIN_ATTR_ANALOG,  ADC_Channel18,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5    }, //BOARD_ID     (21)
  
  // USB
  { PORTA, 28, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8    }, //USB_DETECT   (22)
  { PORTA, 24, PIO_COM,     PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //USB_D_N      (23)
  { PORTA, 25, PIO_COM,     PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //USB_D_P      (24)

  // Analog Reference
  { PORTA, 3,  PIO_ANALOG,  PIN_ATTR_ANALOG,  No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //VREF         (25)

  // Required inclusion of DAC0
  { PORTA, 2,  PIO_ANALOG,  PIN_ATTR_ANALOG,  DAC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2    }, //DAC/VOUT     (26)

} ;


const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;


// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;


Uart Serial1( &sercom3, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM3_Handler()
{
  Serial1.IrqHandler();
}