//#############################################################################
//
// FILE:   i2c_ex1_loopback.c
//
// TITLE:  I2C Digital Loopback with FIFO Interrupts
//
//! \addtogroup driver_example_list
//! <h1>I2C Digital Loopback with FIFO Interrupts</h1>
//!
//! This program uses the internal loopback test mode of the I2C module. Both
//! the I2C FIFOs and their interrupts are used.
//!
//! A stream of data is sent and then compared to the received stream.
//! The sent data looks like this: \n
//!  0000 0001 \n
//!  0001 0002 \n
//!  0002 0003 \n
//!  .... \n
//!  00FE 00FF \n
//!  00FF 0000 \n
//!  etc.. \n
//! This pattern is repeated forever.
//!
//! \b External \b Connections \n
//!  - None
//!
//! \b Watch \b Variables \n
//!  - \b sData - Data to send
//!  - \b rData - Received data
//!  - \b rDataPoint - Used to keep track of the last position in the receive
//!    stream for error checking
//!
//
//#############################################################################
// $TI Release: F28004x Support Library v1.05.00.00 $
// $Release Date: Thu Oct 18 15:43:57 CDT 2018 $
// $Copyright:
// Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
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
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

//
// Defines
//
#define SLAVE_ADDRESS   0x3C

//
// Globals
//
uint16_t sData[2];                  // Send data buffer
uint16_t rData[2];                  // Receive data buffer
uint16_t rDataPoint = 0;            // To keep track of where we are in the
                                    // data stream to check received data

//
// Function Prototypes
//
void initI2CFIFO(void);
__interrupt void i2cFIFOISR(void);

//
// Main
//
void main(void)
{
    uint16_t i;

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file.
    //
    Interrupt_register(INT_I2CA_FIFO, &i2cFIFOISR);

    //
    // Set I2C use, initializing it for FIFO mode
    //
    initI2CFIFO();

    //
    // Initialize the data buffers
    //
    for(i = 0; i < 2; i++)
    {
        sData[i] = i;
        rData[i]= 0;
    }

    //
    // Enable interrupts required for this example
    //
    Interrupt_enable(INT_I2CA_FIFO);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Loop forever. Suspend or place breakpoints to observe the buffers.
    //
    while(1)
    {
        ;
    }
}

//
// Function to configure I2C A in FIFO mode.
//
void initI2CFIFO()
{
    //
    // Must put I2C into reset before configuring it
    //
    I2C_disableModule(I2CA_BASE);

    //
    // I2C configuration. Use a 400kHz I2CCLK with a 50% duty cycle.
    //
    I2C_initMaster(I2CA_BASE, DEVICE_SYSCLK_FREQ, 400000, I2C_DUTYCYCLE_50);
    I2C_setConfig(I2CA_BASE, I2C_MASTER_SEND_MODE);
    I2C_setDataCount(I2CA_BASE, 2);
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);

    //
    // Configure for internal loopback mode
    //
    I2C_setSlaveAddress(I2CA_BASE, SLAVE_ADDRESS);
    I2C_setOwnSlaveAddress(I2CA_BASE, SLAVE_ADDRESS);
    I2C_enableLoopback(I2CA_BASE);
    I2C_setEmulationMode(I2CA_BASE, I2C_EMULATION_STOP_SCL_LOW);

    //
    // FIFO and interrupt configuration
    //
    I2C_enableFIFO(I2CA_BASE);
    I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_RXFF | I2C_INT_TXFF);
    I2C_setFIFOInterruptLevel(I2CA_BASE, I2C_FIFO_TX2, I2C_FIFO_RX2);
    I2C_enableInterrupt(I2CA_BASE, I2C_INT_RXFF | I2C_INT_TXFF);

    //
    // Configuration complete. Enable the module.
    //
    I2C_enableModule(I2CA_BASE);
}

//
// I2C A Receive FIFO ISR
//
 __interrupt void i2cFIFOISR(void)
{
    uint16_t i;

    //
    // If receive FIFO interrupt flag is set, read data
    //
    if((I2C_getInterruptStatus(I2CA_BASE) & I2C_INT_RXFF) != 0)
    {
        for(i = 0; i < 2; i++)
        {
            rData[i] = I2C_getData(I2CA_BASE);
        }

        //
        // Check received data
        //
        for(i = 0; i < 2; i++)
        {
            if(rData[i] != ((rDataPoint + i) & 0xFF))
            {
                // Something went wrong. rData doesn't contain expected data.
                ESTOP0;
            }
        }

        rDataPoint = (rDataPoint + 1) & 0xFF;

        //
        // Clear interrupt flag
        //
        I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_RXFF);
    }
    //
    // If transmit FIFO interrupt flag is set, put data in the buffer
    //
    else if((I2C_getInterruptStatus(I2CA_BASE) & I2C_INT_TXFF) != 0)
    {
        for(i = 0; i < 2; i++)
        {
            I2C_putData(I2CA_BASE, sData[i]);
        }

        //
        // Send the start condition
        //
        I2C_sendStartCondition(I2CA_BASE);

        //
        // Increment data for next cycle
        //
        for(i = 0; i < 2; i++)
        {
           sData[i] = (sData[i] + 1) & 0xFF;
        }

        //
        // Clear interrupt flag
        //
        I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_TXFF);
    }

    //
    // Issue ACK
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}
