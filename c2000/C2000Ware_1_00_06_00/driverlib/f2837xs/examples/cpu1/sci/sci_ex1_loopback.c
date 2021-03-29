//###########################################################################
//
// FILE:    sci_ex1_loopback.c
//
// TITLE:   SCI FIFO Digital Loop Back.
//
//! \addtogroup driver_example_list
//! <h1>SCI FIFO Digital Loop Back</h1>
//!
//!  This program uses the internal loop back test mode of the peripheral.
//!  Other then boot mode pin configuration, no other hardware configuration
//!  is required.
//!
//!  This test uses the loopback test mode of the SCI module to send
//!  characters starting with 0x00 through 0xFF.  The test will send
//!  a character and then check the receive buffer for a correct match.
//!
//!  \b Watch \b Variables \n
//!  - \b loopCount - Number of characters sent
//!  - \b errorCount - Number of errors detected
//!  - \b sendChar - Character sent
//!  - \b receivedChar - Character received
//!
//
//###########################################################################
// $TI Release: F2837xS Support Library v3.05.00.00 $
// $Release Date: Thu Oct 18 15:50:26 CDT 2018 $
// $Copyright:
// Copyright (C) 2014-2018 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

//
// Globals
//
uint16_t loopCount;
uint16_t errorCount;

//
// Function Prototypes
//
void initSCIALoopback(void);
void initSCIAFIFO(void);
void xmitSCIA(uint16_t a);
void error();

//
// Main
//
void main(void)
{
    uint16_t sendChar;
    uint16_t receivedChar;

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Setup GPIO by disabling pin locks and enabling pullups
    //
    Device_initGPIO();

    // GPIO28 is the SCI Rx pin.
    //
    GPIO_setMasterCore(28, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_28_SCIRXDA);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(28, GPIO_QUAL_ASYNC);

    //
    // GPIO29 is the SCI Tx pin.
    //
    GPIO_setMasterCore(29, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_29_SCITXDA);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(29, GPIO_QUAL_ASYNC);

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Enables CPU interrupts
    //
    Interrupt_enableMaster();

    //
    // Initialize counts
    //
    loopCount = 0;
    errorCount = 0;

    //
    // Initialize SCIA
    //
    initSCIAFIFO();
    initSCIALoopback();

    //
    // Note: Autobaud lock is not required for this example
    //

    //
    // Send a character starting with 0
    //
    sendChar = 0;

    //
    // Send Characters forever starting with 0x00 and going through 0xFF.
    // After sending each, check the receive buffer for the correct value.
    //
    for(;;)
    {
        xmitSCIA(sendChar);

        //
        // Wait for RRDY/RXFFST = 1 for 1 data available in FIFO
        //
        while(SCI_getRxFIFOStatus(SCIA_BASE) == SCI_FIFO_RX0)
        {
            ;
        }

        //
        // Check received character
        //
        receivedChar = SCI_readCharBlockingFIFO(SCIA_BASE);

        //
        // Received character not correct
        //
        if(receivedChar != sendChar)
        {
            error();
        }

        //
        // Move to the next character and repeat the test
        //
        sendChar++;

        //
        // Limit the character to 8-bits
        //
        sendChar &= 0x00FF;
        loopCount++;
    }
}

//
// error - Function to count errors
//
void error()
{
    errorCount++;
    asm("     ESTOP0");  // Uncomment to stop the test here
    for (;;);
}

//
// initSCIALoopback - Configure SCIA settings
//
void initSCIALoopback()
{
    //
    // Note: Clocks were turned on to the SCIA peripheral
    // in the Device_init() function
    //

    //
    // 8 char bits, 1 stop bit, no parity. Baud rate is 9600.
    //
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 9600, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_disableLoopback(SCIA_BASE);

    SCI_enableFIFO(SCIA_BASE);
    SCI_performSoftwareReset(SCIA_BASE);
    SCI_disableInterrupt(SCIA_BASE, SCI_INT_RXERR);

    SCI_enableInterrupt(SCIA_BASE, SCI_INT_TXRDY);
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_RXRDY_BRKDT);

    SCI_enableLoopback(SCIA_BASE);

    //
    // Relinquish SCI from Reset
    //
    SCI_enableModule(SCIA_BASE);
}

//
// xmitSCIA - Transmit a character from the SCI
//
void xmitSCIA(uint16_t a)
{
    SCI_writeCharNonBlocking(SCIA_BASE, a);
}

//
// initSCIAFIFO - Initialize the SCI FIFO
//
void initSCIAFIFO()
{
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_enableFIFO(SCIA_BASE);

    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, SCI_FIFO_RX4);
    SCI_resetChannels(SCIA_BASE);

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
}

//
// End of file
//
