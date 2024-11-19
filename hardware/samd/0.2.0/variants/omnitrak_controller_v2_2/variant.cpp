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

  // I2C Bus (SERCOM4)
  { PORTA, 13, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13  }, //I2C_SDA         (3)  SERCOM / I2C *
  { PORTA, 12, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12  }, //I2C_SCL         (4)  SERCOM / I2C *

  // SD Card  
  { PORTD, 12, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //nSD_DETECT      (5)  DIGITAL IN *
  { PORTD, 11, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //nSD_WP          (6)  DIGITAL IN
  { PORTA, 8,  PIO_SDHC,       PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI }, //SD_CMD          (7)  SDHC SDCMD *
  { PORTB, 11, PIO_SDHC,       PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //SD_CLK          (8)  SDHC SDCK *
  { PORTA, 9,  PIO_SDHC,       PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //SD_DAT0         (9)  SDHC SDDAT/0 *
  { PORTA, 10, PIO_SDHC,       PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //SD_DAT1         (10) SDHC SDDAT/1 *
  { PORTA, 11, PIO_SDHC,       PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //SD_DAT2         (11) SDHC SDDAT/2 *
  { PORTB, 10, PIO_SDHC,       PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //SD_DAT3         (12) SDHC SDDAT/3 *
  { PORTD, 8,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //SDHC_EN         (13) DIGITAL OUT *

  // Vulintus Peripheral Bus (VPB)
  { PORTB, 2,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //CLK_IN_3V3      (14) XINT2 *
  { PORTB, 1,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1   }, //CLK_OUT_3V3     (15) DIGITAL OUT *
  { PORTB, 0,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0   }, //TRG_3V3         (16) XINT0 *
  { PORTB, 3,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //nVPB_BLOCK      (17) DIGITAL OUT *

  //NINA W102
  { PORTC, 10,  PIO_DIGITAL,   PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //nNINA_RST       (18) DIGITAL OUT *
  { PORTA, 6,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //nNINA_CS        (19) DIGITAL OUT *
  { PORTA, 1,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1   }, //nNINA_DEBUG     (20) DIGITAL OUT *
  { PORTA, 7,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //NINA_ACK        (21) XINT7 *
  { PORTC, 7,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //nNINA_BOOT      (22) DIGITAL OUT *

  // NINA Programming Serial (SERCOM6)
  { PORTC, 5,  PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //SAMD_RX/NINA_TX (23) SERCOM / SERIAL *
  { PORTC, 4,  PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //SAMD_TX/NINA_RX (24) SERCOM / SERIAL *

  // Programming
  { PORTA, 31, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //SWDIO *         (25) DIGITAL I/O
  { PORTA, 30, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //SWCLK *         (26) DIGITAL I/O
  { PORTB, 30, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //SWO             (27) DIGITAL IN I/O
  // { PORTA, 15, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //nSAMBA

  // TFT Display
  { PORTC, 13, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13  }, //nTFT_RST        (28) DIGITAL OUT *
  { PORTC, 11, PIO_DIGITAL,    PIN_ATTR_PWM_F,      No_ADC_Channel, TCC0_CH1,   TCC0_CH1,     EXTERNAL_INT_11  }, //TFT_LED         (29) PWM TCC0 WO/1 *
  { PORTB, 13, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13  }, //TFT_DC          (30) DIGITAL OUT *
  { PORTC, 14, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //nTFT_CS         (31) DIGITAL OUT *

  // Status LED (RGB)
  { PORTA, 18, PIO_DIGITAL,    PIN_ATTR_PWM_F,      No_ADC_Channel, TCC1_CH2,   TCC1_CH2,     EXTERNAL_INT_2   }, //LED_R           (32) PWM TCC0 WO/6 *
  { PORTA, 17, PIO_DIGITAL,    PIN_ATTR_PWM_G,      No_ADC_Channel, TCC0_CH5,   TCC0_CH5,     EXTERNAL_INT_1   }, //LED_G           (33) PWM TCC0 WO/5 *
  { PORTA, 16, PIO_DIGITAL,    PIN_ATTR_PWM_G,      No_ADC_Channel, TCC0_CH4,   TCC0_CH4,     EXTERNAL_INT_0   }, //LED_B           (34) PWM TCC0 WO/4 *

  // RV-3208-C7 Real-Time Clock
  { PORTC, 15, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //nRTC_INT        (35) XINT15 *
  { PORTA, 14, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //nRTC_EVI        (36) DIGITAL OUT *
  { PORTA, 0,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0   }, //RTC_CLKO        (37) DIGITAL IN *

  // QTouch Control
  { PORTB, 12, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12  }, //nQT_RST         (38) DIGITAL OUT *
  { PORTA, 4,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //nQT_INT         (39) XINT4*

  // LED Demultiplexer
  { PORTB, 15, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //DEMUX_A         (40) DIGITAL OUT *
  { PORTB, 18, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //DEMUX_A         (41) DIGITAL OUT *
  { PORTD, 10, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //DEMUX_C         (42) DIGITAL OUT *
  { PORTD, 9,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //DEMUX_D         (43) DIGITAL OUT *

  // Piezo Speaker
  { PORTB, 14, PIO_PWM,        PIN_ATTR_PWM_G,      No_ADC_Channel, TCC0_CH2,   TCC0_CH2,     EXTERNAL_INT_14  }, //SPKR            (44) PWM TCC0 WO/2
  
  // Power Control
  { PORTC, 30, PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel12,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //V_IN_ADC        (45) ADC1 AIN/12 *
  { PORTD, 0,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel14,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0   }, //I_IN_ADC        (46) ADC1 AIN/14 *
  { PORTB, 5,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel7,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //9V_UVLO         (47) ADC1 AIN/7 *
  { PORTB, 4,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel6,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //9V_ADC          (48) ADC1 AIN/6 *
  { PORTB, 6,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel8,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //5V_ADC          (49) ADC1 AIN/8 *
  { PORTB, 17, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1   }, //nPWR_SHDN       (50) XINT1 *
  { PORTB, 16, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0   }, //I_IN_RESET      (51) DIGITAL OUT *
  { PORTD, 21, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //nPWR_FAULT      (52) XINT11 *
  { PORTD, 20, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //n9V_ON_FLAG     (53) XINT10 *

  // OTMP Port 1 (SERCOM0)
  { PORTC, 17, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1   }, //P1_TX           (54) SERCOM / SERIAL *
  { PORTC, 16, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0   }, //P1_RX           (55) SERCOM / SERIAL *

  // OTMP Port 2 (SERCOM3)
  { PORTC, 23, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //P2_TX           (56) SERCOM / SERIAL *
  { PORTC, 22, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //P2_RX           (57) SERCOM / SERIAL *

  // OTMP Port 3 (SERCOM5)
  { PORTA, 23, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //P3_TX           (58) SERCOM / SERIAL *
  { PORTA, 22, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //P3_RX           (59) SERCOM / SERIAL *

  // OTMP Port 4 (SERCOM2)
  { PORTB, 25, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //P4_TX           (60) SERCOM / SERIAL *
  { PORTB, 24, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8   }, //P4_RX           (61) SERCOM / SERIAL *

  // OTMP Port 5 (SERCOM7)
  { PORTC, 12, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12  }, //P5_TX           (62) SERCOM / SERIAL *
  { PORTB, 31, PIO_SERCOM,     PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //P5_RX           (63) SERCOM / SERIAL *

  // OTMP Power Control
  { PORTB, 19, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //P1_24V_EN       (64) DIGITAL OUT *
  { PORTB, 21, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //P2_24V_EN       (65) DIGITAL OUT *
  { PORTA, 21, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //P3_24V_EN       (66) DIGITAL OUT *
  { PORTB, 29, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //P4_24V_EN       (67) DIGITAL OUT *
  { PORTA, 27, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11  }, //P5_24V_EN       (68) DIGITAL OUT *

  { PORTD, 1,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel15,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1   }, //P1_IOUT         (69) ADC1 AIN/15 *
  { PORTC, 31, PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel13,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //P2_IOUT         (70) ADC1 AIN/13 *
  { PORTC, 0,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel10,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0   }, //P3_IOUT         (71) ADC1 AIN/10 *
  { PORTC, 1,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel11,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1   }, //P4_IOUT         (72) ADC1 AIN/11 *
  { PORTC, 2,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel4,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //P5_IOUT         (73) ADC1 AIN/4 *

  { PORTB, 26, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12  }, //nP1_ALERT       (74) XINT12 *
  { PORTB, 27, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13  }, //nP2_ALERT       (75) XINT13 *
  { PORTB, 28, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14  }, //nP3_ALERT       (76) XINT14 *
  { PORTC, 24, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8   }, //nP4_ALERT       (77) XINT8 *
  { PORTC, 25, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //nP5_ALERT       (78) XINT9 *

  { PORTB, 20, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //P1_RST          (79) DIGITAL OUT *
  { PORTA, 20, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //P2_RST          (80) DIGITAL OUT *
  { PORTB, 23, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //P3_RST          (81) DIGITAL OUT *
  { PORTC, 26, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10  }, //P4_RST          (82) DIGITAL OUT *
  { PORTC, 3,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //P5_RST          (83) DIGITAL OUT *

  // BNC I/O
  { PORTA, 2,  PIO_ANALOG,     PIN_ATTR_ANALOG,     DAC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //BNC_OUT_1       (84) DAC0 *
  { PORTA, 5,  PIO_ANALOG,     PIN_ATTR_ANALOG,     DAC_Channel1,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //BNC_OUT_2       (85) DAC1 *
  { PORTB, 9,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel1,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //BNC_IN_1        (86) ADC1 AIN/1 *
  { PORTB, 8,  PIO_ANALOG,     PIN_ATTR_ANALOG_ALT, ADC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8   }, //BNC_IN_2        (87) ADC1 AIN/0 *
  { PORTC, 18, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2   }, //BNC_SW_1        (88) DIGITAL OUT *
  { PORTC, 19, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //BNC_SW_2        (89) DIGITAL OUT *
  { PORTC, 20, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4   }, //BNC_EN_1        (90) DIGITAL OUT *
  { PORTC, 21, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5   }, //BNC_EN_2        (91) DIGITAL OUT *

  // Fan Control
  { PORTA, 19, PIO_PWM,        PIN_ATTR_PWM_G,      No_ADC_Channel, TCC0_CH7,   TCC0_CH7,     EXTERNAL_INT_3   }, //FAN             (92) PWM TC3 WO/1 *

  // Board ID Analog Input
  { PORTB, 7, PIO_ANALOG,      PIN_ATTR_ANALOG_ALT, ADC_Channel9,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7   }, //BOARD_ID        (93) ADC1 AIN/9 *
  
  // USB
  { PORTA, 15, PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, //nSAMBA          (94) XINT << USB Host Enable *
  { PORTC, 6,  PIO_DIGITAL,    PIN_ATTR_DIGITAL,    No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6   }, //USB_DETECT      (95) XINT6 *
  { PORTA, 24, PIO_COM,        PIN_ATTR_NONE,       No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8   }, //USB_D_N         (96) USB DATA- *
  { PORTA, 25, PIO_COM,        PIN_ATTR_NONE,       No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9   }, //USB_D_P         (97) USB DATA+ *

  // Analog Reference
  { PORTA, 3, PIO_ANALOG,      PIN_ATTR_ANALOG,     ADC_Channel0,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3   }, //VREF            (99) ANALOG *

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

//Serial 1 (OTMP Port 1, SERCOM0) handler.
Uart Serial1( &sercom0, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ; //OTMP Port 1
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
Uart Serial2( &sercom3, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX ) ; //OTMP Port 2
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

//Serial 3 (OTMP Port 3, SERCOM5) handler.
Uart Serial3( &sercom5, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX, PAD_SERIAL3_TX ) ; //OTMP Port 3
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

//Serial 4 (OTMP Port 4, SERCOM2) handler.
Uart Serial4( &sercom2, PIN_SERIAL4_RX, PIN_SERIAL4_TX, PAD_SERIAL4_RX, PAD_SERIAL4_TX ) ; //OTMP Port 4
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

//Serial 5 (OTMP Port 5, SERCOM7) handler.
Uart Serial5( &sercom7, PIN_SERIAL5_RX, PIN_SERIAL5_TX, PAD_SERIAL5_RX, PAD_SERIAL5_TX ) ; //OTMP Port 5
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

//Serial 6 (NINA W102 Programming, SERCOM6) handler.
Uart Serial6( &sercom6, PIN_SERIAL6_RX, PIN_SERIAL6_TX, PAD_SERIAL6_RX, PAD_SERIAL6_TX ) ; //NINA W102 Programming
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