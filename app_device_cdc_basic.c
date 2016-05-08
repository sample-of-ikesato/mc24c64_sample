/********************************************************************
 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *******************************************************************/

/** INCLUDES *******************************************************/
#include "system.h"

#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "usb.h"
#include "app_led_usb_status.h"
#include "app_device_cdc_basic.h"
#include "usb_config.h"
#include "queue.h"
#include "accel.h"

/** VARIABLES ******************************************************/

static bool buttonPressed;
static char buttonMessage[] = "Button pressed.\r\n";
static uint8_t readBuffer[CDC_DATA_OUT_EP_SIZE];
static uint8_t writeBuffer[CDC_DATA_IN_EP_SIZE];

/**
 * シリアル・ポートの Ready を待つ
 * timer1 を使ってオーバフローならば 0 を返す
 * Ready になれば 1 を返す
 */
int WaitToReadySerial(void)
{
  PIR1bits.TMR1IF = 0;
  TMR1 = 0;
  PIR1bits.TMR1IF = 0;
  while (PIR1bits.TMR1IF==0) {
    if (mUSBUSARTIsTxTrfReady())
      return 1;
    CDCTxService();
  }
  return 0;
}

void PutsStringCPtr(char *str)
{
  if (!WaitToReadySerial()) return;
  putsUSBUSART(str);
  if (!WaitToReadySerial()) return;
}




unsigned short gcounter = 0;
Accel accel_x;
Accel accel_y;
unsigned short led_x_timer = 0;
unsigned short led_y_timer = 0;
unsigned char debug_buffer[32]; // size needs bigger than queue_buffer
int debug_buffer_size = 0;
unsigned char debug_flag = 0;
unsigned short debug_counter = 0;
unsigned char calc_accel = 0;

#define T0CNT (65536-375)
char led_state = 1;
void interrupt_func(void)
{
  if (INTCONbits.TMR0IF == 1) {
    TMR0 = T0CNT;
    INTCONbits.TMR0IF = 0;
    gcounter++;
    if (gcounter > 8000/120) {
      gcounter = 0;
      led_state = !led_state;
      debug_flag = 1;
      calc_accel = 1;
    }
    if (led_x_timer > 0) led_x_timer--;
    if (led_y_timer > 0) led_y_timer--;
  }

  if (INTCONbits.RABIF == 1) {
    unsigned short now = TMR3;
    //TMR3 = 0;
    //PIR2bits.TMR3IF = 0;
    INTCONbits.RABIF = 0;
    debug_counter++;
    accel_pin_state_changed(&accel_x, PORTBbits.RB5, now);
    accel_pin_state_changed(&accel_y, PORTBbits.RB7, now);

    // ADXL213
    // 10k[ohm] の抵抗をつけた場合
    //   T2 = 10*10**3 / (125*10**6) [s]
    //      = 0.08 / 10**3 [s] = 0.08[ms]
    //   1/T2 = 12.5[kHz]
    //   256 の分解能とすると 12.5 * 256 = 3200[kHz] = 3.2[MHz]
    //
    // 42k[ohm] の抵抗をつけた場合
    //   T2 = 42*10**3 / (125*10**6) [s]
    //      = 0.336 / 10**3 [s] = 0.336[ms]
    //   1/T2 = 2.976[kHz]
    //   256 の分解能とすると 2.976 * 256 = 761.9[kHz] = 0.7[MHz]
    //
    // 125k[ohm] の抵抗をつけた場合
    //   T2 = 125*10**3 / (125*10**6) [s]
    //      = 1 / 10**3 [s] = 1[ms]
    //   1/T2 = 1[kHz]
    //   256 の分解能とすると 1 * 256 = 256[kHz]
    //
    // 126k[ohm] の抵抗をつけた場合
    //   T2 = 126*10**3 / (125*10**6) [s]
    //      = 1.008 / 10**3 [s] = 1.008[ms]
    //   1/T2 = 0.992[kHz]
    //   256 の分解能とすると 1 * 256 = 254[kHz]
    //
    // Timer は 48MHz/4 で prescaler 1:4 なので
    // 1/(48*10**6/4/4) * x = 1/(256*10**3)
    // x = (48*10**6/4/4) / (256*10**3)
    //   = 3 * 10**3 / 256
    //   = 11.719, 約12

    // Timer は 48MHz/4 で prescaler 1:8 なので
    // 1/(48*10**6/4/8) * x = 1/(256*10**3)
    // x = (48*10**6/4/8) / (256*10**3)
    //   = 1.5 * 10**3 / 256
    //   = 11.719, 約12
  }
}

void init(void)
{
  TRISA = 0;
  TRISB = 0b10100000; // input RB5,RB7
  TRISC = 0;
  PORTA = 0;
  PORTB = 0;
  PORTC = 0;
  ANSELH = ANSEL = 0;

  INTCON2bits.RABPU = 0; // enable pull-up
  WPUB7 = WPUB5 = 1;     // pull up pins

  // timer
  // USB Bootloader では 48MHz で動作
  //
  // 8kHz を作るには
  //   48MHz/4 * x = 8kHz としたい
  //   1/x = (48/4)*1000/8 = 1500
  //   prescaler を 1:4 とすると 1500/4 = 375
  //
  T0CONbits.T08BIT = 0;     // 16bit timer
  T0CONbits.T0PS = 0b001;   // prescaler 1:4
  T0CONbits.T0CS = 0;
  T0CONbits.PSA = 0;        // use prescaler
  T0CONbits.TMR0ON = 1;
  TMR0 = T0CNT;
  INTCON2bits.TMR0IP = 1;
  INTCONbits.TMR0IE = 1;
  INTCONbits.TMR0IF = 0;
  INTCONbits.GIEH = 1;
  INTCONbits.GIEL = 1;


  // timer1 (for waiting USB Serial data)
  T1CONbits.TMR1CS = 0;    // 内部クロック (FOSC/4)
  T1CONbits.T1CKPS = 0b11; // prescaler 1:8
  T1CONbits.RD16 = 1;      // 16bit
  T1CONbits.TMR1ON = 1;
  PIR1bits.TMR1IF = 0;

  // PWM settings
  CCP1CONbits.CCP1M = 0b1100; // P1A、P1C をアクティブ High、P1B、P1D をアクティブ High
  CCP1CONbits.DC1B  = 0b11;   // デューティ サイクル値の最下位 2 ビット
  CCP1CONbits.P1M   = 0b00;   // シングル出力
  PSTRCONbits.STRA = 1;
  PSTRCONbits.STRB = 1;
  PSTRCONbits.STRC = 1;
  PSTRCONbits.STRD = 1;
  PSTRCONbits.STRSYNC = 1;

  // 8ビットのデーティー幅とする場合は PR2 が 0x3F となる
  // 16MHz の場合
  //   16MHz/4 = 4MHz
  //   4MHz / (0x3F+1) = 4000kHz/64 = 62.5kHz
  // 48HMz の場合
  //   48MHz/4 = 12MHz
  //   12MHz / (0x3F+1) = 12000kHz/64 = 187.5kHz
  //CCPR1L  = 0x3F;              // デューティ値
  CCPR1L  = 0x00;
  PR2     = 0x3F;            // PWM周期 187.5kHz @48MHz

  // for PWM
  TMR2 = 0;
  T2CONbits.T2CKPS = 0b00;  // prescaler 1:1
  T2CONbits.T2OUTPS = 0;    // postscaler 1:1
  T2CONbits.TMR2ON = 1;     // Timer ON

  // for ADXL213
  TMR3 = 0;
  T3CONbits.RD16 = 1;      // 16bit mode
  //T3CONbits.T3CKPS = 0b10; // prescaler 1:4
  //T3CONbits.T3CKPS = 0b11; // prescaler 1:8
  //T3CONbits.T3CKPS = 0b01; // prescaler 1:2
  T3CONbits.T3CKPS = 0b00; // prescaler 1:1
  T3CONbits.T3CCP1 = 1;    // use Timer3 clock source
  T3CONbits.TMR3CS = 0;    // internal clock
  //T3CONbits.T3SYNC = 1;
  T3CONbits.TMR3ON = 1;    // Timer ON
  PIR2bits.TMR3IF = 0;     // clear overflow
  //PIE2bits.TMR3IE = 1;     // enable interrupt

  // for ADXL213 interrupt pins
  INTCONbits.RABIF = 1;
  INTCONbits.RABIE = 1;
  INTCON2bits.RABIP = 1;   // high level interrupt
  IOCBbits.IOCB5 = 1;
  IOCBbits.IOCB7 = 1;

  // accel
  accel_init(&accel_x);
  accel_init(&accel_y);
}

/*********************************************************************
* Function: void APP_DeviceCDCBasicDemoInitialize(void);
*
* Overview: Initializes the demo code
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceCDCBasicDemoInitialize()
{
    CDCInitEP();

    line_coding.bCharFormat = 0;
    line_coding.bDataBits = 8;
    line_coding.bParityType = 0;
    line_coding.dwDTERate = 9600;

    buttonPressed = false;
}

/*********************************************************************
* Function: void APP_DeviceCDCBasicDemoTasks(void);
*
* Overview: Keeps the demo running.
*
* PreCondition: The demo should have been initialized and started via
*   the APP_DeviceCDCBasicDemoInitialize() and APP_DeviceCDCBasicDemoStart() demos
*   respectively.
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceCDCBasicDemoTasks()
{
    /* If the user has pressed the button associated with this demo, then we
     * are going to send a "Button Pressed" message to the terminal.
     */
    if(BUTTON_IsPressed(BUTTON_DEVICE_CDC_BASIC_DEMO) == true)
    {
        /* Make sure that we only send the message once per button press and
         * not continuously as the button is held.
         */
        if(buttonPressed == false)
        {
            /* Make sure that the CDC driver is ready for a transmission.
             */
            if(mUSBUSARTIsTxTrfReady() == true)
            {
                putrsUSBUSART(buttonMessage);
                buttonPressed = true;
            }
        }
    }
    else
    {
        /* If the button is released, we can then allow a new message to be
         * sent the next time the button is pressed.
         */
        buttonPressed = false;
    }

    /* Check to see if there is a transmission in progress, if there isn't, then
     * we can see about performing an echo response to data received.
     */
    if( USBUSARTIsTxTrfReady() == true)
    {
      uint8_t numBytesRead;
      numBytesRead = getsUSBUSART(readBuffer, sizeof(readBuffer));

      if (debug_flag) {
        debug_flag = 0;
        writeBuffer[0] = 4;
        writeBuffer[1] = 9;
        *((unsigned short *)(&writeBuffer[2])) = accel_x.on;
        *((unsigned short *)(&writeBuffer[4])) = accel_x.off;
        *((unsigned short *)(&writeBuffer[6])) = accel_y.on;
        *((unsigned short *)(&writeBuffer[8])) = accel_y.off;
        writeBuffer[10] = debug_counter;
        putUSBUSART(writeBuffer, writeBuffer[1]+2);
      }
    }
    CDCTxService();

    if (calc_accel) {
      calc_accel = 0;
      accel_apply_filter(&accel_x);
      accel_apply_filter(&accel_y);
      #define THRESHOLD 0.05
      if (accel_x.value < (unsigned short)(4585*(1.0-THRESHOLD)) || accel_x.value > (unsigned short)(4585*(1.0+THRESHOLD)))
        led_x_timer = (unsigned short)(0.5*8000); // 0.5[s]
      if (accel_y.value < (unsigned short)(4585*(1.0-THRESHOLD)) || accel_y.value > (unsigned short)(4585*(1.0+THRESHOLD)))
        led_y_timer = (unsigned short)(0.5*8000); // 0.5[s]

      PORTCbits.RC1 = (led_x_timer > 0 ? 1 : 0);
      PORTCbits.RC7 = (led_y_timer > 0 ? 1 : 0);
    }
}
