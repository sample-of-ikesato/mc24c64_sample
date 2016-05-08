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
#include "i2c.h"
#include "mc24c64.h"

#define _XTAL_FREQ (48000000)

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
unsigned char debug_buffer[32]; // size needs bigger than queue_buffer
int debug_buffer_size = 0;
unsigned char debug_flag = 0;
unsigned char debug_data = 0;
unsigned short debug_counter = 0;
unsigned char debug_flag2 = 0;

#define T0CNT (65536-375)
char led_state = 1;
void interrupt_func(void)
{
  // I2C interrupt handler
  i2c_interrupt();

  if (INTCONbits.TMR0IF == 1) {
    TMR0 = T0CNT;
    INTCONbits.TMR0IF = 0;
    gcounter++;
    if (gcounter > 8000) {
      gcounter = 0;
    }
  }

  if (INTCONbits.RABIF == 1) {
    unsigned short now = TMR3;
    //TMR3 = 0;
    //PIR2bits.TMR3IF = 0;
    INTCONbits.RABIF = 0;
    debug_counter++;
  }
}

void init(void)
{
  TRISA = 0;
  TRISB = 0b11110000; // input RB4(SDA), RB6(SCL), RB5,RB7
  TRISC = 0;
  PORTA = 0;
  PORTB = 0;
  PORTC = 0;
  ANSELH = ANSEL = 0;

  INTCON2bits.RABPU = 0; // enable pull-up
  WPUB = 0b11110000;     // pull up pins: RB4(SDA), RB6(SCL), RB5, RB7

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

  // for i2c
  PORTCbits.RC7 = 1;
  mc24c64_init();

  // write
//  i2c_start(0x50, 0);
//  i2c_send(0x00); // address high
//  i2c_send(0x00); // address low
//  i2c_send(0x12); // data
//  i2c_stop();
//  __delay_ms(10);
//  i2c_start(0x50, 0);
//  i2c_send(0x00); // address high
//  i2c_send(0x01); // address low
//  i2c_send(0x34); // data
//  i2c_stop();
//  __delay_ms(10);
//
//  {
//    i2c_start(0x50, 0);
//    i2c_send(0x00); // address high
//    i2c_send(0x00); // address low
//    //i2c_rstart(0x50, 0);
//    i2c_start(0x50, 1);
//    unsigned char data[2];
//    data[0]  = i2c_receive(ACK);
//    data[1]  = i2c_receive(NOACK);
//    i2c_stop();
//    if (data[0] == 0x12 && data[1] == 0x34)
//      PORTCbits.RC1 = 1;
//    debug_flag = 1;
//    debug_data = data;
//  }
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

      if (numBytesRead > 0) {
        switch(readBuffer[0]) {
        case 0x10:
          {
            unsigned char size = readBuffer[1];
            debug_flag2 = !debug_flag2;
            PORTCbits.RC1 = debug_flag2;
            writeBuffer[0] = 0x90;
            writeBuffer[1] = 4;
            writeBuffer[2] = numBytesRead;
            writeBuffer[3] = readBuffer[1];
            writeBuffer[4] = readBuffer[2];
            writeBuffer[5] = readBuffer[3];
            writeBuffer[1] = 4+size;
            for (unsigned char i = 0; i<size; i++) {
              writeBuffer[i+6] = readBuffer[4 + i];
            }
            if (WaitToReadySerial())
              putUSBUSART(writeBuffer, writeBuffer[1]+2);
            WaitToReadySerial();
          }
          {
            unsigned char size = readBuffer[1];
            i2c_start(0x50, 0);
            // address in big-endian format
            i2c_send(readBuffer[3]); // address MSB
            i2c_send(readBuffer[2]); // address LSB
            for (unsigned char i = 0; i<size; i++) {
              i2c_send(readBuffer[4 + i]);
            }
            i2c_stop();
            __delay_ms(10);
          }
          break;
        case 0x11:
          {
            unsigned char size = readBuffer[1];
            unsigned char data;
            unsigned char i;
            i2c_start(0x50, 0);
            // address in big-endian format
            i2c_send(readBuffer[3]); // address MSB
            i2c_send(readBuffer[2]); // address LSB
            i2c_start(0x50, 1);
            for (i=0; i<size-1; i++) {
              writeBuffer[i+2] = i2c_receive(ACK);
            }
            writeBuffer[i+2] = i2c_receive(NOACK);
            i2c_stop();
            __delay_ms(10);
            writeBuffer[0] = 0x12;
            writeBuffer[1] = size;
            putUSBUSART(writeBuffer, writeBuffer[1]+2);
          }
          break;
        }
      }

      //if (debug_flag) {
      //  debug_flag = 0;
      //  writeBuffer[0] = 9;
      //  writeBuffer[1] = 1;
      //  writeBuffer[2] = debug_data;
      //  putUSBUSART(writeBuffer, writeBuffer[1]+2);
      //}
    }
    CDCTxService();
}
