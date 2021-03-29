//#############################################################################
//
// FILE:    gpio_ex2_toggle.c
//
// TITLE:   GPIO toggle test program.
//
//! \addtogroup driver_example_list
//! <h1> GPIO toggle test program </h1>
//!
//! Three different examples are included. Select the example
//! (data, set/clear or toggle) to execute before compiling using
//! the #define statements found at the top of the code.
//!
//! Toggle all of the GPIO PORT pins
//!
//! The pins can be observed using an oscilloscope.
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
// Select the example to compile in.  Only one example should be set as 1
// the rest should be set as 0.
//
#define EXAMPLE1 1  // Use DATA registers to toggle I/O's
#define EXAMPLE2 0  // Use SET/CLEAR registers to toggle I/O's
#define EXAMPLE3 0  // Use TOGGLE registers to toggle I/O's

//
// Function Prototypes
//
void selectGPIO(void);
void performGPIOExample1(void);
void performGPIOExample2(void);
void performGPIOExample3(void);

//
// Main
//
void main(void)
{
    //
    // Initialize System Control and device clock and peripherals
    //
    Device_init();

    //
    // For this example use the following configuration:
    //
    selectGPIO();

    //
    // Initialize PIE, clear PIE registers, disable and clear all
    // CPU interrupts and flags
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

#if EXAMPLE1

    //
    // This example uses DATA registers to toggle I/O's
    //
    performGPIOExample1();

#endif

#if EXAMPLE2

    //
    // This example uses SET/CLEAR registers to toggle I/O's
    //
    performGPIOExample2();

#endif

#if EXAMPLE3

    //
    // This example uses TOGGLE registers to toggle I/O's
    //
    performGPIOExample3();

#endif
}

//
// performGPIOExample1 - Toggle I/Os using DATA registers
//
void
performGPIOExample1(void)
{
    for(;;)
    {
        GPIO_writePortData(GPIO_PORT_A, 0xAAAAAAAA);
        GPIO_writePortData(GPIO_PORT_B, 0x00000AAA);
        SysCtl_delay(1000);

        GPIO_writePortData(GPIO_PORT_A, 0x55555555);
        GPIO_writePortData(GPIO_PORT_B, 0x00001555);
        SysCtl_delay(1000);
    }
}

//
// performGPIOExample2 - Toggle I/Os using SET/CLEAR registers
//
void
performGPIOExample2(void)
{
    for(;;)
    {
        GPIO_setPortPins(GPIO_PORT_A, 0xAAAAAAAA);
        GPIO_clearPortPins(GPIO_PORT_A, 0x55555555);

        GPIO_setPortPins(GPIO_PORT_B, 0x00000AAA);
        GPIO_clearPortPins(GPIO_PORT_B, 0x00001555);

        SysCtl_delay(1000);

        GPIO_clearPortPins(GPIO_PORT_A, 0xAAAAAAAA);
        GPIO_setPortPins(GPIO_PORT_A, 0x55555555);

        GPIO_clearPortPins(GPIO_PORT_B, 0x00000AAA);
        GPIO_setPortPins(GPIO_PORT_B, 0x00001555);

        SysCtl_delay(1000);
    }
}

//
// performGPIOExample3 - Toggle I/Os using TOGGLE registers
//
void
performGPIOExample3(void)
{
    //
    // Set pins to a known state
    //
    GPIO_setPortPins(GPIO_PORT_A, 0xAAAAAAAA);
    GPIO_clearPortPins(GPIO_PORT_A, 0x55555555);

    GPIO_setPortPins(GPIO_PORT_B, 0x00000AAA);
    GPIO_clearPortPins(GPIO_PORT_B, 0x00001555);

    //
    // Use TOGGLE registers to flip the state of
    // the pins.
    // Any bit set to a 1 will flip state (toggle)
    // Any bit set to a 0 will not toggle.
    //
    for(;;)
    {
        GPIO_togglePortPins(GPIO_PORT_A, 0xFFFFFFFF);
        GPIO_togglePortPins(GPIO_PORT_B, 0x00001FFF);

        SysCtl_delay(1000);
    }
}

//
// selectGPIO - Set up the configuration for specific gpio
//
void selectGPIO(void)
{
    EALLOW;

    //
    // Write value into MUX register and turns on gpios to be set
    //
    HWREG(GPIOCTRL_BASE + GPIO_O_GPAMUX1) &= 0x00000000;  // GPIO0 - GPIO15
    
    //
    // Write value into MUX register and turns on GPIO16 - GPIO31 except GPIO18
    // which is reserved for when the clock source is INTOSC
    //
    HWREG(GPIOCTRL_BASE + GPIO_O_GPAMUX2) &= 0x00000030;

    //
    // Write value into MUX register and turns on GPIO32 - GPIO47 except
    // GPIO35 and GPIO37 which are reserved and mapped to TDI/TDO for JTAG 
    // operation in Debug mode
    //
    HWREG(GPIOCTRL_BASE + GPIO_O_GPBMUX1) &= 0x00000CC0;    

    //
    // Set the data direction to output for port A (GPIO0 to 31)
    //
    HWREG(GPIOCTRL_BASE + GPIO_O_GPADIR) = 0xFFFFFFFF;

    //
    // Set the data direction to output for port B (GPIO32 to 44)
    //
    HWREG(GPIOCTRL_BASE + GPIO_O_GPBDIR) = 0x00001FFF;

    EDIS;
}

//
// End of File
//

