/**
  EUSART Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    eusart.c

  @Summary
    This is the generated driver implementation file for the EUSART driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This source file provides APIs for EUSART.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC16F1827
        Driver Version    :  2.1.1
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.31 and above
        MPLAB 	          :  MPLAB X 5.45
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

/**
  Section: Included Files
*/
#include "eusart.h"
#include "../Modbus_uart/Modbus.h"
#include "stdlib.h"
/**
  Section: Macro Declarations
*/

#define EUSART_TX_BUFFER_SIZE 8
#define EUSART_RX_BUFFER_SIZE 8

/**
  Section: Global Variables
*/
volatile eusart_status_t eusartRxLastError;

/**
  Section: EUSART APIs
*/
void (*EUSART_TxDefaultInterruptHandler)(void);
void (*EUSART_RxDefaultInterruptHandler)(void);

void (*EUSART_FramingErrorHandler)(void);
void (*EUSART_OverrunErrorHandler)(void);
void (*EUSART_ErrorHandler)(void);

void EUSART_DefaultFramingErrorHandler(void);
void EUSART_DefaultOverrunErrorHandler(void);
void EUSART_DefaultErrorHandler(void);

void EUSART_Initialize(void)
{
    // disable interrupts before changing states
    PIE1bits.RCIE = 0;
    EUSART_SetRxInterruptHandler(EUSART_Receive_ISR);
    PIE1bits.TXIE = 0;
    EUSART_SetTxInterruptHandler(EUSART_Transmit_ISR);
    // Set the EUSART module to the options selected in the user interface.

    // ABDOVF no_overflow; SCKP Non-Inverted; BRG16 16bit_generator; WUE disabled; ABDEN disabled; 
    BAUDCON = 0x08;

    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled; 
    RCSTA = 0x90;

    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave; 
    TXSTA = 0x24;

    // SPBRGL 160; 
    SPBRGL = 0xA0;

    // SPBRGH 1; 
    SPBRGH = 0x01;


    EUSART_SetFramingErrorHandler(EUSART_DefaultFramingErrorHandler);
    EUSART_SetOverrunErrorHandler(EUSART_DefaultOverrunErrorHandler);
    EUSART_SetErrorHandler(EUSART_DefaultErrorHandler);

    eusartRxLastError.status = 0;

    // initializing the driver state

    // enable receive interrupt
    PIE1bits.RCIE = 1;
}

bool EUSART_is_tx_ready(void)
{
    return true;
}

bool EUSART_is_rx_ready(void)
{
    return true;
}


bool EUSART_is_tx_done(void)
{
    return TXSTAbits.TRMT;
}

eusart_status_t EUSART_get_last_status(void){
    return eusartRxLastError;
}

uint8_t EUSART_Read(void)
{
    uint8_t readValue  = 0;
    return readValue;
}

void EUSART_Write(uint8_t txData)
{

    //while((1 == PIE1bits.TXIE)||(EUSART_is_tx_done() == true))
    // while(1 == PIE1bits.TXIE)
    // {

    // }

    TXREG = txData;
    PIE1bits.TXIE = 1;

    while((1 == PIE1bits.TXIE)||(EUSART_is_tx_done() == false))
    {

    }
    
}


void EUSART_Transmit_ISR(void)
{

    // add your EUSART interrupt custom code
        PIE1bits.TXIE = 0;

}

void EUSART_Receive_ISR(void)
{
    
    eusartRxLastError.status = 0;

    if(RCSTAbits.FERR){
        eusartRxLastError.ferr = 1;
        EUSART_FramingErrorHandler();
    }

    if(RCSTAbits.OERR){
        eusartRxLastError.oerr = 1;
        EUSART_OverrunErrorHandler();
    }
    
    if(eusartRxLastError.status){
        EUSART_ErrorHandler();
    } else {
        EUSART_RxDataHandler();
    }
    
    // or set custom function using EUSART_SetRxInterruptHandler()
}

void EUSART_RxDataHandler(void){
    // use this default receive interrupt handler code
    TModbusData * tModbusData = NULL;
    tModbusData = Mos_PtGetModbusObject();

    uint8_t ucReceivedByte = 0;
    ucReceivedByte = RCREG;

    if(tModbusData->u8OneFrameFinish ==1)  //�����ݰ����ڴ���
        {
            return ;
        }

        tModbusData->auReceiveBuff[tModbusData->u8ReceivedByteCount ++] = ucReceivedByte;
        tModbusData->u8TimeoutCount = 0;  //Clean timeout

    if(tModbusData->u8ReceivedByteCount ==1)  //�յ�����������һ֡���ݵĵ�һ�ֽ�
        {
            tModbusData->u8TimeoutStart=1;  //������ʱ
        }

}

void EUSART_DefaultFramingErrorHandler(void)
{

}

void EUSART_DefaultOverrunErrorHandler(void)
{
    // EUSART error - restart

    RCSTAbits.CREN = 0;
    RCSTAbits.CREN = 1;

}

void EUSART_DefaultErrorHandler(void)
{
    EUSART_RxDataHandler();
}

void EUSART_SetFramingErrorHandler(void (* interruptHandler)(void)){
    EUSART_FramingErrorHandler = interruptHandler;
}

void EUSART_SetOverrunErrorHandler(void (* interruptHandler)(void)){
    EUSART_OverrunErrorHandler = interruptHandler;
}

void EUSART_SetErrorHandler(void (* interruptHandler)(void)){
    EUSART_ErrorHandler = interruptHandler;
}

void EUSART_SetTxInterruptHandler(void (* interruptHandler)(void)){
    EUSART_TxDefaultInterruptHandler = interruptHandler;
}

void EUSART_SetRxInterruptHandler(void (* interruptHandler)(void)){
    EUSART_RxDefaultInterruptHandler = interruptHandler;
}

/**
  End of File
*/
