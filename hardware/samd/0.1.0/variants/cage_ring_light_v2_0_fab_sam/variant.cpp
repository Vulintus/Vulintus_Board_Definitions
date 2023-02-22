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

/*
 * Modified 9 June 2018 by Justin Mattair
 *   for MattairTech boards (www.mattairtech.com)
 *
 * See README.md for documentation and pin mapping information
 */


#include "variant.h"

/*
 * Pins descriptions
 */

const PinDescription g_APinDescription[]=
{
//{ EPortType, [Port Number], EPioType, [Pin Attributes], EAnalogChannel, EPWMChannel, ETCChannel, EExt_Interrupts }

  // Vulintus Peripheral Bus (VPB)
  { PORTA, 30, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_STD),                    (PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM),                                             NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE,  GCLK_CCL_NONE }, //CLK_IN_3V3/SWCLK (0)  XINT
  { PORTA, 14, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC1_CH0,      ADC_Channel6,   EXTERNAL_INT_NMI,   GCLK_CCL_NONE }, //CLK_OUT_3V3      (1)  DIGITAL OUT
  { PORTA, 9,  PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT),              TCC0_CH3,     No_ADC_Channel, EXTERNAL_INT_7,     GCLK_CCL_NONE }, //TRG_3V3          (2)  XINT
  { PORTA, 2,  PIO_MULTI, PER_ATTR_DRIVE_STRONG,                                          (PIN_ATTR_ADC|PIN_ATTR_DAC|PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT),                   NOT_ON_TIMER, ADC_Channel0,   EXTERNAL_INT_2,     GCLK_CCL_NONE }, //nVPB_BLOCK       (3)  DIGITAL OUT

  //LEDs
  { PORTA, 31, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_SERCOM_STD),                    (PIN_ATTR_DIGITAL|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT),                             NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_3,     GCLK_CCL_NONE }, //NEOPIX_3V3/SWCLK (4)  DIGITAL OUT
  { PORTA, 15, PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TC1_CH1,      ADC_Channel7,   EXTERNAL_INT_1,     GCLK_CCL_NONE }, //IR_LEDS          (5)  PWM TC1 WO[1]

  // Power Monitoring 
  { PORTA, 4,  PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TCC0_CH0,     ADC_Channel2,   EXTERNAL_INT_4,     GCLK_CCL_NONE }, //VIN_ADC          (6)  ADC AIN[2]

  // Board ID
  { PORTA, 5,  PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_ALT|PER_ATTR_SERCOM_STD), (PIN_ATTR_ADC|PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT), TCC0_CH1,     ADC_Channel3,   EXTERNAL_INT_5,     GCLK_CCL_NONE }, //BOARD_ID         (7)  ADC AIN[3]

  // USB
  { PORTA, 8,  PIO_MULTI, (PER_ATTR_DRIVE_STRONG|PER_ATTR_TIMER_STD|PER_ATTR_SERCOM_STD), (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER|PIN_ATTR_SERCOM|PIN_ATTR_EXTINT),              TCC0_CH2,     No_ADC_Channel, EXTERNAL_INT_6,     GCLK_CCL_NONE }, //USB_DETECT       (8)  XINT
  { PORTA, 25, PIO_MULTI, PER_ATTR_DRIVE_STRONG,                                          (PIN_ATTR_DIGITAL|PIN_ATTR_COM),                                                NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE,  GCLK_CCL_NONE }, //USB_D_P          (9)  USB DATA+
  { PORTA, 24, PIO_MULTI, PER_ATTR_DRIVE_STRONG,                                          (PIN_ATTR_DIGITAL|PIN_ATTR_COM),                                                NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE,  GCLK_CCL_NONE }, //USB_D_N          (10) USB DATA-
  
  // Reset
  { PORTA, 28, PIO_MULTI, PER_ATTR_DRIVE_STRONG,                                          PIN_ATTR_DIGITAL,                                                               NOT_ON_TIMER, No_ADC_Channel, EXTERNAL_INT_NONE,  GCLK_CCL_NONE }, //RESET            (11) *UNUSED*
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TC1, TC2 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;

// #if defined(ONE_UART) || defined(TWO_UART)

// //Serial 1 (Debugging Serial, SERCOM0) handler.
//   Uart Serial1( SERCOM_INSTANCE_SERIAL1, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;
//   void SERCOM0_Handler()
//   {
//     Serial1.IrqHandler();
//   }

// #endif