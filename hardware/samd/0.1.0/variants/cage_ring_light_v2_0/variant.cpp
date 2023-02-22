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
//{ EPortType, [Port Number], EPioType, [Pin Attributes], EAnalogChannel, EPWMChannel, ETCChannel, EExt_Interrupts }

  // Vulintus Peripheral Bus (VPB)
  { PORTA, 30, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //CLK_IN_3V3/SWCLK (0)  XINT
  { PORTA, 14, PIO_SERCOM,     PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI }, //CLK_OUT_3V3      (1)  DIGITAL OUT
  { PORTA, 9,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //TRG_3V3          (2)  XINT
  { PORTA, 2,  PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //nVPB_BLOCK       (3)  DIGITAL OUT

  //LEDs
  { PORTA, 31, PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //NEOPIX_3V3/SWCLK (4)  DIGITAL OUT
  { PORTA, 15, PIO_DIGITAL,    PIN_ATTR_PWM,     No_ADC_Channel, TC1_CH1,   NOT_ON_TIMER, EXTERNAL_INT_1    }, //IR_LEDS          (5)  PWM TC1 WO[1]

  // Power Monitoring 
  { PORTA, 4,  PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel2,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //VIN_ADC          (6)  ADC AIN[2]

  // Board ID
  { PORTA, 5,  PIO_ANALOG,     PIN_ATTR_ANALOG,  ADC_Channel3,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //BOARD_ID         (7)  ADC AIN[3]

  // USB
  { PORTA, 8,  PIO_DIGITAL,    PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //USB_DETECT       (8)  XINT
  { PORTA, 25, PIO_COM,        PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //USB_D_P          (9)  USB DATA+
  { PORTA, 24, PIO_COM,        PIN_ATTR_NONE,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //USB_D_N          (10) USB DATA-
  
};

extern "C" {
    unsigned int PINCOUNT_fn() {
        return (sizeof(g_APinDescription) / sizeof(g_APinDescription[0]));
    }
}

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TC1, TC2 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;


//Serial 1 (Debugging Serial, SERCOM0) handler.
Uart Serial1( &sercom0, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ; //OTMP Port 1
void SERCOM0_Handler()
{
  Serial1.IrqHandler();
}
