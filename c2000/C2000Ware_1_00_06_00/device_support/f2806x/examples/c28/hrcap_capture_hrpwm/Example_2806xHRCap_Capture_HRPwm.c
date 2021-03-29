//###########################################################################
//
// FILE:	Example_2806xHRCap_Capture_HRPwm.c
//
// TITLE:	HRCAP Capture HRPWM Pulses Example
//
//!  \addtogroup f2806x_example_list
//!  <h1>HRCAP Capture HRPWM Pulses (hrcap_capture_hrpwm)</h1>
//!
//!  This program generates a high-resolution PWM output on ePWM1A
//!  (with falling edge moving by 8 HRPWM MEP steps per period), and
//!  the HRCAP is configured to generate an interrupt on  either rising
//!  edges OR falling edges to continuously capture up to 5 PWM periods
//!  (5 high pulses and 5 low pulses) and calculate the high 
//!  resolution pulse widths in integer + fractional HCCAPCLK cycles
//!  in Q16 format.
//!
//!  Monitor pulsewidthlow and pulsewidthhigh in the Watch Window (pulsewidthlow
//!  gradually decreases and pulsewidthhigh gradually increases as CMPAHR moves
//!  the falling edge MEP steps.) Another option is to monitor periodwidth in
//!  the Watch Window, which should not change much because the period stays
//!  the same.
//!
//! - User must set 
//!   - \#define FALLTEST 1 and RISETEST 0 for falling edge interrupts
//!   - \#define RISETEST 1 and FALLTEST 0 for rising edge interrupts
//! - To measure PERIOD, user must set:
//!  \#define PERIODTEST 1 
//! -  To measure high pulse width or low pulse width, user must set:
//!  \#define PERIODTEST 0 
//!
//!  To determine the actual pulse \f$ \frac{width}{period} \f$ time in 
//!  \f$ \frac{pulsewidthlow}{pulsewidthhigh}\f$ period: \n
//!  \f$ pulse width in seconds = pulsewidth[n] * (1 HCCAPCLK cycle) \f$ \n
//!  (i.e. 1 HCCAPCLK cycle is 8.33 ns for 120 MHz HCCAPCLK)
//!
//!  PLL2 is configured such that:
//!  - PLL2 = 60 MHz using INTOSC1 (10 MHz) as source
//!  - HCCAPCLK = 120 MHz
//!
//! \note 
//! - THE FOLLOWING DEFINITION AND STRUCTURE MUST BE DEFINED IN CODE
//! IN ORDER TO USE THE HCCAL LIBRARY
//!   - \#define NUM_HRCAP 5        // \# of HRCAP modules + 1 (4 HRCAP's on 2806x + 1 = 5)
//!   - volatile struct HRCAP_REGS *HRCAP[NUM_HRCAP] = {0, \&HRCap1Regs, \&HRCap2Regs, \&HRCap3Regs, \&HRCap4Regs};
//!  - Because there is no EMU FREE/STOP support in the HRCAP peripheral,
//!  the HRCAP results cannot easily be seen in "Real-time continuous refresh"
//!  debug mode.  The only way to see accurate results of the capture is to
//!  set a breakpoint in the HRCAP ISR immediately after data has been captured.
//!  Otherwise the results read at any other time in the program via the debugger 
//!  could be mid-capture between one period and the next, and therefore bogus.
//!
//!  \b External \b Connections \n
//!  - HRCAP1 is on GPIO27
//!  - ePWM1A is on GPIO0
//!  - Connect output of ePWM1A to input of HRCAP1 (GPIO0->GPIO27)
//!
//!  \b Watch \b Variables \n
//!  - pulsewidthlow
//!    - Pulse Width of Low Pulses (# of HCCAPCLK cycles - int + frac)
//!      in Q16 format (i.e. upper 16-bits integer, lower 16-bits fraction)
//!  - pulsewidthhigh
//!    - Pulse Width of High Pulses (# of HCCAPCLK cycles - int + frac)
//!      in Q16 format (i.e. upper 16-bits integer, lower 16-bits fraction)
//!  - periodwidth
//!    - Period Width (# of HCCAPCLK cycles - int + frac)
//!      in Q16 format (i.e. upper 16-bits integer, lower 16-bits fraction)
//
//###########################################################################
// $TI Release: F2806x Support Library v2.04.00.00 $
// $Release Date: Thu Oct 18 15:47:20 CDT 2018 $
// $Copyright:
// Copyright (C) 2009-2018 Texas Instruments Incorporated - http://www.ti.com/
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
#include "F2806x_Device.h"         // F2806x Headerfile
#include "F2806x_Examples.h"       // F2806x Examples Headerfile
#include "F2806x_EPwm_defines.h"   // useful defines for initialization
#include "HCCal_Type0_V1.h"

//
// Defines
//
#define HCCAPCLK_PLLCLK 1  // HCCAPCLK = PLL2CLK (CLKIN * PLL2 MULT)

//
// HCCAPCLK = SYSCLK2 (CLKIN * PLL2 MULT / SYSCLK2DIV)
//
#define HCCAPCLK_SYSCLK 0

#define PLL2SRC_INTOSC1 0  // Select INTOSC1 as PLL2 source

//
// Defines for falling edge interrupts or rising edge interrupts
//
#define FALLTEST 0
#define RISETEST 1

//
// Defines to capture period or pulse widths:
//
#define PERIODTEST 0

//
// IMPORTANT - The following definition and struct must be defined in order to 
// use the HCCal library
//
#define NUM_HRCAP 5      // # of HRCAP modules + 1 (4 HRCAP's on 2806x + 1 = 5)
volatile struct HRCAP_REGS *HRCAP[NUM_HRCAP] = {0, &HRCap1Regs, &HRCap2Regs, 
                                                &HRCap3Regs, &HRCap4Regs};

//
// Function Prototypes
//
void HRCAP2_Config(void);
void HRPWM1_Config(Uint16 period);
__interrupt void HRCAP2_Isr (void);

//
// Globals
//

//
// Count and Dly captured by 1st RISE/FALL event after cal, soft reset, 
// or clock enable is invalid and therefore ignored. 
// (equals # cycles from reset, cal, clock enable to edge instead of valid 
// pulse width)
//
Uint16 first;

Uint16 counter;            // Increments CMPAHR by 8 MEP steps with each period
Uint16 datacounter;        // Counts 5 periods then resets.

//
// Stores 5 low pulses in # of HCCAPCLK cycles (Q16 format: int + fraction)
//
Uint32 pulsewidthlow[5];   

//
// Stores 5 high pulses in # of HCCAPCLK cycles (Q16 format: int + fraction)
//
Uint32 pulsewidthhigh[5];

//
// Stores 5 period widths in # of HCCAPCLK cycles (Q16 format: int + fraction)
//
Uint32 periodwidth[5];

//
// Main
//
void main(void)
{
    Uint16 status;

    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // In this example:   
    // PLL2 = INTOSC1 (10 Mhz) * 12 / 2 = 60 MHz 
    // In general, for more accuracy, source PLL2 with a crystal or
    // external clock source.
    //
    InitPll2(PLL2SRC_INTOSC1, 12, PLL2_SYSCLK2DIV2DIS); 

    //
    // Step 2. Initalize GPIO:
    // This example function is found in the F2806x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio();  // Skipped for this example

    //
    // These functions are in the F2806x_EPwm.c file
    //
    InitHRCapGpio();
    InitEPwm1Gpio();

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    // Disable and clear all CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2806x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2806x_DefaultIsr.c.
    // This function is found in F2806x_PieVect.c.
    //
    InitPieVectTable();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW;	// This is needed to write to EALLOW protected registers
    PieVectTable.HRCAP2_INT = &HRCAP2_Isr;
    EDIS;   // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in F2806x_InitPeripherals.c
    //
    // InitPeripherals();  // Not required for this example

    status = 0; 

    HRPWM1_Config(400);	        // EPWM1 output waveform, Period = 400
    
    //
    // While calibration is incomplete
    //
    while (status!=2)
    {
        status = HRCAP_Cal(1, HCCAPCLK_PLLCLK, &EPwm8Regs);
        if (status == HCCAL_ERROR) 
        { 
            //
            // If there is an error on HRCAP, stop and 
            // check 98 MHz < PLL2CLK < 120 MHz .
            //
            ESTOP0;
        }
    }

    HRCAP2_Config();                       // Configure HRCAP2 Module

    //
    // Step 5. User specific code, enable interrupts
    //

    //
    // Initialize variables
    //
    datacounter = 0;  // marks how many pulses have been captured
    first = 0;        // marks first captured data after a SOFTRESET to discard

    //
    // Enable interrupts required for this example
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 8
    IER|=M_INT4;                         // Enable CPU INT4
    EINT;                                // Enable Global Interrupts

    for (;;)
    {
        if (datacounter == 5)
        {
            __asm (" nop");        // Set breakpoint here for debug
        }

        //
        // Run calibration routine periodically in background using HRCAP1 
        // internally tied to HRPWM8 output.
        //
        status = 0;                // New calibration not complete
        
        //
        // While calibration is incomplete
        //
        while (status!=2)
        {
            status = HRCAP_Cal(1,HCCAPCLK_PLLCLK, &EPwm8Regs);
            if (status == HCCAL_ERROR) 
            { 
                //
                // If there is an error on HRCAP, stop and 
                // check 98 MHz < PLL2CLK < 120 MHz .
                //
                ESTOP0;
            }
        }
    }
}

//
// HRPWM1_Config - 
//
void
HRPWM1_Config(Uint16 period)
{
    //
    // ePWM1 register configuration with HRPWM
    // ePWM1A toggle low/high with MEP control on Falling edge
    //
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;       // Disable TBCLKSYNC
    EDIS;

    EPwm1Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;	 // set Immediate load
    EPwm1Regs.TBPRD = period-1;		             // PWM frequency = 1 / period
    EPwm1Regs.CMPA.half.CMPA = period / 2;       // set duty 50% initially
    EPwm1Regs.CMPA.half.CMPAHR = (0 << 8);       // initialize HRPWM extension
    EPwm1Regs.TBPHS.all = 0;
    EPwm1Regs.TBCTR = 0;

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;        // PWM toggle low/high
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;

    EALLOW;
    EPwm1Regs.HRCNFG.all = 0x0;
    EPwm1Regs.HRCNFG.bit.EDGMODE = HR_FEP;    // HR MEP control on Falling edge
    EPwm1Regs.HRCNFG.bit.CTLMODE = HR_CMP;
    EPwm1Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO;
    EDIS;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;    // Enable TBCLKSYNC
    EDIS;
}

//
// HRCAP2_Config -
//
void
HRCAP2_Config(void)
{
    EALLOW;

    HRCap2Regs.HCCTL.bit.SOFTRESET = 1;
    HRCap2Regs.HCCTL.bit.HCCAPCLKSEL = 1;  // HCCAPCLK = PLL2CLK = SYSCLK2 x 2

    #if RISETEST
        //
        // Enable Rising Edge Capture Event Interrupt
        //
        HRCap2Regs.HCCTL.bit.RISEINTE = 1;     
        
        //
        // Disable Falling Edge Capture Event Interrupt
        //
        HRCap2Regs.HCCTL.bit.FALLINTE = 0;
    #elif FALLTEST
        //
        // Enable Falling Edge Capture Event Interrupt
        //
        HRCap2Regs.HCCTL.bit.FALLINTE = 1;
        
        //
        // Disable Rising Edge Capture Event Interrupt
        //
        HRCap2Regs.HCCTL.bit.RISEINTE = 0;
    #endif
    
    //
    // Enable Interrupt on 16-bit Counter Overflow Event
    //
    HRCap2Regs.HCCTL.bit.OVFINTE = 0;
    
    EDIS;
}

//
// HRCAP2_Isr - 
//
__interrupt void
HRCAP2_Isr(void)
{
    EALLOW;
    if (HRCap2Regs.HCIFR.bit.RISEOVF == 1) 
    {
        ESTOP0;         // Another rising edge detected before ISR serviced
    }

    if (first < 1)
    {
        //
        // Discard first data (because first interrupt after reset/clk enable
        // measures time from clock start to edge instead of valid pulse width)
        //
        first++;
    } 
    else
    {
        if (datacounter == 5)
        {
            datacounter = 0;
        }
        #if FALLTEST
        #if PERIODTEST
            periodwidth[datacounter] = PeriodWidthRise0((Uint16 *)&HRCap2Regs);  
        #else
            pulsewidthlow[datacounter] = LowPulseWidth0((Uint16 *)&HRCap2Regs);                  
            pulsewidthhigh[datacounter] = HighPulseWidth0((Uint16 *)&HRCap2Regs);  
        #endif
        #elif RISETEST	   
        #if PERIODTEST
            periodwidth[datacounter] = PeriodWidthRise0((Uint16 *)&HRCap2Regs);  
        #else	        	    
            pulsewidthlow[datacounter] = LowPulseWidth0((Uint16 *)&HRCap2Regs);                  
            pulsewidthhigh[datacounter] = HighPulseWidth0((Uint16 *)&HRCap2Regs);                 
        #endif
        #endif
        
        counter+=8;

        //
        // Increment CMPAHR by 8 for next period.        
        //
        EPwm1Regs.CMPA.half.CMPAHR = counter<<8; 
        datacounter++;
    }

    HRCap2Regs.HCICLR.bit.FALL=1;  // Clear FALL flag
    HRCap2Regs.HCICLR.bit.RISE=1;  // Clear RISE flag

    HRCap2Regs.HCICLR.bit.INT=1;   // Clear HRCAP interrupt flag
    PieCtrlRegs.PIEACK.bit.ACK4=1; // Acknowledge PIE Group 4 interrupts.

    EDIS;
}

//
// End of File
//

