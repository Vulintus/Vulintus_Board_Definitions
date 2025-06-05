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

  // SPI Bus (SERCOM1)
  { PORTC, 27, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //SPI_PICO        (0)  SERCOM / SPI *
  { PORTB, 22, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //SPI_POCI        (1)  SERCOM / SPI *
  { PORTC, 28, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12  }, //SPI_SCK         (2)  SERCOM / SPI *

  //NINA W102
  { PORTC, 10,  PIO_DIGITAL,   PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //nNINA_RST       (3)  DIGITAL OUT *
  { PORTA, 6,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //nNINA_CS        (4)  DIGITAL OUT *
  { PORTA, 1,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1   }, //nNINA_DEBUG     (5)  DIGITAL OUT *
  { PORTA, 7,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //NINA_ACK        (6)  XINT7 *
  { PORTC, 7,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //nNINA_BOOT      (7)  DIGITAL OUT *

  // NINA Programming Serial (SERCOM6)
  { PORTC, 4,  PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //SAMD_TX/NINA_RX (8)  SERCOM / SERIAL *
  { PORTC, 5,  PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //SAMD_RX/NINA_TX (9)  SERCOM / SERIAL *
  
  // Status LED (RGB)
  { PORTA, 18, PIO_DIGITAL,    PIN_ATTR_PWM_F,      No_ADC_Channel, TCC1_CH2,   TCC1_CH2,     EXTERNAL_INT_2   }, //LED_R           (10) PWM TCC0 WO/6 *
  { PORTA, 17, PIO_DIGITAL,    PIN_ATTR_PWM_G,      No_ADC_Channel, TCC0_CH5,   TCC0_CH5,     EXTERNAL_INT_1   }, //LED_G           (11) PWM TCC0 WO/5 *
  { PORTA, 16, PIO_DIGITAL,    PIN_ATTR_PWM_G,      No_ADC_Channel, TCC0_CH4,   TCC0_CH4,     EXTERNAL_INT_0   }, //LED_B           (12) PWM TCC0 WO/4 *

  // BNC I/O
  { PORTA, 2,  PIO_ANALOG,     PIN_ATTR_ANALOG,     DAC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //BNC_OUT_1       (13) DAC0 *
  { PORTA, 5,  PIO_ANALOG,     PIN_ATTR_ANALOG,     DAC_Channel1,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //BNC_OUT_2       (14) DAC1 *
  { PORTB, 9,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel1,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //BNC_IN_1        (15) ADC1 AIN/1 *
  { PORTB, 8,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8   }, //BNC_IN_2        (16) ADC1 AIN/0 *

  // USB
  { PORTA, 15, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //nSAMBA          (17) XINT << USB Host Enable *
  { PORTC, 6,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //USB_DETECT      (18) XINT6 *
  { PORTA, 24, PIO_COM,        PIN_ATTR_NONE,       No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8   }, //USB_D_N         (19) USB DATA- *
  { PORTA, 25, PIO_COM,        PIN_ATTR_NONE,       No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //USB_D_P         (20) USB DATA+ *

  // Analog Reference
  { PORTA, 3, PIO_ANALOG,      PIN_ATTR_ANALOG,     ADC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //VREF            (21) ANALOG *

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

//Serial 1 (NINA W102 Programming, SERCOM6) handler.
Uart Serial1( &sercom6, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ; //NINA W102 Programming
void SERCOM6_0_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM6_1_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM6_2_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM6_3_Handler()
{
  Serial1.IrqHandler();
}

//Serial HCI (something to do with the NINA, SERCOM1) handler.
Uart SerialHCI( &sercom1, PIN_SERIALHCI_RX, PIN_SERIALHCI_TX, PAD_SERIALHCI_RX, PAD_SERIALHCI_TX, PIN_SERIALHCI_RTS, PIN_SERIALHCI_CTS);
void SERCOM1_0_Handler()
{
  SerialHCI.IrqHandler();
}
void SERCOM1_1_Handler()
{
  SerialHCI.IrqHandler();
}
void SERCOM1_2_Handler()
{
  SerialHCI.IrqHandler();
}
void SERCOM1_3_Handler()
{
  SerialHCI.IrqHandler();
}