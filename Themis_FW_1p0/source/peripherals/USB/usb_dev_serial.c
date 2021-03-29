//###########################################################################
//
// FILE:   usb_dev_serial.c
//
// TITLE:  USB Serial Device example. 
//
// ASSUMPTIONS:
//
//    This program requires:
//        - the F2806x header files 
//        - Driverlib (/ControlSUITE/lib/MWare/f2806x/driverlib/)
//        - Usblib (/ControlSUITE/lib/MWare/f2806x/usblib/)
//        - USB Capable F2806x
//
// Description:
//
//! \addtogroup f2806x_example_list
//! <h1>USB Serial Device (usb_dev_serial)</h1>
//!
//! This example application turns the evaluation kit into a virtual serial
//! port when connected to the USB host system.  The application supports the
//! USB Communication Device Class, Abstract Control Model to redirect UART0
//! traffic to and from the USB host system.
//!
//! A driver information (INF) file for use with Windows XP, Windows Vista and
//! Windows7 can be found in MWare/f2806x/windows_drivers. For Windows
//! 2000, the required INF file is in /MWare/f2806x/windows_drivers/win2K.
//
//  Section .ebss was initialized to 0x0000 in
//  F2806x_CodeStartBranch_ebss_init.asm. This was to ensure that all global
//  variables of the usblib are initialized to 0x0000 on the F2806x.
//		
//###########################################################################	
// $TI Release: F2806x Support Library v2.02.00.00 $ 
// $Release Date: Sat Sep 16 15:27:43 CDT 2017 $ 
// $Copyright:
// Copyright (C) 2009-2017 Texas Instruments Incorporated - http://www.ti.com/
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
#include "DSP28x_Project.h"
#include <stdbool.h>
#include <stdint.h>
#include <IQmathLib.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/usb.h"
#include "inc/hw_usb.h"
#include "include/usblib.h"
#include "include/usbcdc.h"
#include "driverlib/usb_hal.h"
#include "include/device/usbdevice.h"
#include "include/device/usbdcdc.h"
#include "usb_serial_structs.h"
#include "themis_io_definitions.h"
#include "F2806x_GlobalPrototypes.h"
#include "utils/uartstdio.h"
#include "F2806x_HRCap.h"
#include "HCCal_Type0_V1.h"
#include "FreeRTOS.h"

// #define USB_IO_DBG   0

// #define NO_MAIN (0)

//
// The system tick timer period.
//
#define SYSTICKS_PER_SECOND     100
#define SYSTICK_PERIOD_MS (1000 / SYSTICKS_PER_SECOND)

#define ADC_usDELAY  1000L

//
// Global system tick counter holds elapsed time since the application started
// expressed in 100ths of a second.
//
volatile unsigned long g_ulSysTickCount;

extern void Gpio_select(void);
extern void delay_loop(void);

// Configure if ePWM timer interrupt is enabled at the PIE level:
// 1 = enabled,  0 = disabled
//
 #define PWM7_INT_ENABLE  1 /// 1

// #define PWM7_TIMER_TBPRD   0xAFFF       // Configure the period for the timer ~ 1kHz
#define PWM7_TIMER_TBPRD   0x1FFF       // Configure the period for the timer ~ 5kHz
// #define PWM7_TIMER_TBPRD   0x2500      // Configure the period for the timer
// #define PWM7_TIMER_TBPRD   0x5FFF           // close to 2 kHz
// #define PWM7_TIMER_TBPRD   0xFFF       // Configure the period for the timer ~ 10kHz
// #define PWM7_TIMER_TBPRD   0xAFF
//
// Function Prototypes
//
__interrupt void spiTxFifoIsr(void);
__interrupt void spiRxFifoIsr(void);
// void delay_loop(void);
void spi_fifo_init(void);
extern void error();
//
// Function Prototypes - non-irq
//
void spi_xmit(Uint32 a, Uint8 dac_sel);		// our AD5060 DAC takes 24-bit word
// void spi_fifo_init(void);
void spi_init(void);
// void error(void);

extern Uint16 sdata[2];     // Send data buffer
extern Uint16 rdata[2];     // Receive data buffer
Uint16 rdata_point;
#if 0

void InitAdcLocal(void);

__interrupt void adc_isr(void);

__interrupt void epwm7_timer_isr(void);
void InitEPwmTimer(void);



//
// Keep track of where we are in the data stream to check received data
//


//
// Globals
//
Uint32  ECap1IntCount;
Uint32  ECap1PassCount;
Uint32  ECap2IntCount;
Uint32  ECap2PassCount;
// Uint32  EPwm3TimerDirection;

//
// Function Prototypes
//
__interrupt void ecap1_isr(void);
__interrupt void ecap2_isr(void);
void InitECapture(void);
// void InitEPwmTimer(void);
void Fail(void);



//
// Globals
//
Uint32  EPwm7TimerIntCount;     //counts entries into PWM1 Interrupt

typedef struct
{
    volatile struct EPWM_REGS *EPwmRegHandle;
    Uint16 EPwm_CMPA_Direction;
    Uint16 EPwm_CMPB_Direction;
    Uint16 EPwmTimerIntCount;
    Uint16 EPwmMaxCMPA;
    Uint16 EPwmMinCMPA;
    Uint16 EPwmMaxCMPB;
    Uint16 EPwmMinCMPB;
} EPWM_INFO;

//
// Globals
//
EPWM_INFO epwm7_info;
//
// Defines to keep track of which way the compare value is moving
//

#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0
#define DAC_PWM       1
#define USE_USB       1
// #define USE_SPI_IRQ   1
#define USE_NORM      1

//
//
// Defines that configure the period for each timer
//
#define EPWM7_TIMER_TBPRD  1000000  // Period register
#define EPWM7_MAX_CMPA     1950
#define EPWM7_MIN_CMPA       50
#define EPWM7_MAX_CMPB     1950
#define EPWM7_MIN_CMPB       50


//
// Flag indicating whether or not a Break condition is currently being sent.
//
static tBoolean g_bSendingBreak = false;

//
// __error__ - The error routine that is called if the driver library 
// encounters an error.
//
#ifdef NO_MAIN
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
	__asm(" ESTOP0");
}
#endif
#endif

#if 1
// HRCAP:
#define ENABLE_HRCAP_IRQ1	1
#define ENABLE_HRCAP_IRQ2	1
#define ENABLE_HRCAP_IRQ3	1
#define ENABLE_HRCAP_IRQ4	1
#define ENABLE_SCITXRXINT_IRQS      1
#define USE_HRCAP1	1
#define USE_HRCAP2	1
#define USE_HRCAP3	1
#define USE_HRCAP4	1
#endif
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
#if 1
#define NUM_HRCAP 5      // # of HRCAP modules + 1 (4 HRCAP's on 2806x + 1 = 5)
volatile struct HRCAP_REGS *HRCAP[NUM_HRCAP] = {0, &HRCap1Regs, &HRCap2Regs, &HRCap3Regs, &HRCap4Regs};
#endif

//! ...and calculate the high
//!  resolution pulse widths in integer + fractional HCCAPCLK cycles
//!  in Q16 format.


#ifdef USE_HRCAP1
void HRCAP1_Config(void);
__interrupt void HRCAP1_Isr (void);
Uint16 first1;
Uint16 counter1;            // Increments CMPAHR by 8 MEP steps with each period
Uint16 datacounter1;        // Counts 5 periods then resets.
// Uint32 pulsewidthlow0[5];
_iq pulsewidthlow0[5];
unsigned long int_pulsewidthlow0[5];
unsigned long frac_pulsewidthlow0[5];
// Uint32 pulsewidthhigh0[5];
_iq pulsewidthhigh0[5];
unsigned long int_pulsewidthhigh0[5];
unsigned long frac_pulsewidthhigh0[5];
unsigned char do_cal_1 = 0;

Uint32 periodwidth0[5];
Uint32 periodwidthhigh0[5];
#endif

#ifdef USE_HRCAP2
void HRCAP2_Config(void);
__interrupt void HRCAP2_Isr (void);
Uint16 first2;
Uint16 counter2;            // Increments CMPAHR by 8 MEP steps with each period
Uint16 datacounter2;        // Counts 5 periods then resets.
_iq pulsewidthlow1[5];
unsigned long int_pulsewidthlow1[5];
unsigned long frac_pulsewidthlow1[5];
_iq pulsewidthhigh1[5];
unsigned long int_pulsewidthhigh1[5];
unsigned long frac_pulsewidthhigh1[5];
Uint32 periodwidthhigh1[5];
Uint32 periodwidth1[5];
#endif

#ifdef USE_HRCAP3
void HRCAP3_Config(void);
__interrupt void HRCAP3_Isr (void);
Uint16 first3;

Uint16 counter3;            // Increments CMPAHR by 8 MEP steps with each period
Uint16 datacounter3;        // Counts 5 periods then resets.
_iq pulsewidthlow2[5];
unsigned long int_pulsewidthlow2[5];
unsigned long frac_pulsewidthlow2[5];
_iq pulsewidthhigh2[5];
unsigned long int_pulsewidthhigh2[5];
unsigned long frac_pulsewidthhigh2[5];
Uint32 periodwidthhigh2[5];
Uint32 periodwidth2[5];
#endif

#ifdef USE_HRCAP4
void HRCAP4_Config(void);
__interrupt void HRCAP4_Isr (void);
Uint16 first4;
Uint16 counter4;            // Increments CMPAHR by 8 MEP steps with each period
Uint16 datacounter4;        // Counts 5 periods then resets.
// Uint32 pulsewidthlow3[5];
_iq pulsewidthlow3[5];
unsigned long int_pulsewidthlow3[5];
unsigned long frac_pulsewidthlow3[5];
// Uint32 pulsewidthhigh3[5];
_iq pulsewidthhigh3[5];
unsigned long int_pulsewidthhigh3[5];
unsigned long frac_pulsewidthhigh3[5];
Uint32 periodwidth3[5];
Uint32 periodwidthhigh3[5];
#endif

#endif

//
// Count and Dly captured by 1st RISE/FALL event after cal, soft reset, 
// or clock enable is invalid and therefore ignored. 
// (equals # cycles from reset, cal, clock enable to edge instead of valid 
// pulse width)
//

//
// Stores 5 low pulses in # of HCCAPCLK cycles (Q16 format: int + fraction)
//

//
// Stores 5 high pulses in # of HCCAPCLK cycles (Q16 format: int + fraction)
//

//
// Stores 5 period widths in # of HCCAPCLK cycles (Q16 format: int + fraction)
//

//
// SysTickIntHandler - This is the interrupt handler for the SysTick interrupt.
// It is used to update our local tick count which, in turn, is used to check 
// for transmit timeouts.
//
#ifdef NO_MAIN
__interrupt void
SysTickIntHandler(void)
{
    g_ulSysTickCount++;
//    PieCtrlRegs.PIEACK.all |= 1;
    PieCtrlRegs.PIEACK.bit.ACK1=1;
}


//
// CheckForSerialStateChange - This function is called whenever serial data is 
// received from the UART. It is passed the accumulated error flags from each 
// character received in this interrupt and determines from them whether or not
// an interrupt notification to the host is required.
//
// If a notification is required and the control interrupt endpoint is idle,
// send the notification immediately.  If the endpoint is not idle, accumulate
// the errors in a global variable which will be checked on completion of the
// previous notification and used to send a second one if necessary.
//
static void
CheckForSerialStateChange(const tUSBDCDCDevice *psDevice, uint32_t lErrors)
{
    unsigned short usSerialState;

    //
    // Clear the USB serial state.  Since handshakes are being faked, always
    // set the TXCARRIER (DSR) and RXCARRIER (DCD) bits.
    //
    usSerialState = USB_CDC_SERIAL_STATE_TXCARRIER |
                    USB_CDC_SERIAL_STATE_RXCARRIER;

    //
    // Are any error bits set?
    //
    if(lErrors & (UART_RXST_BRKDT | UART_RXST_FE | UART_RXST_OE | UART_RXST_PE))
    {
        //
        // At least one error is being notified so translate from the hardware
        // error bits into the correct state markers for the USB notification.
        //
        if(lErrors & UART_RXST_OE)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_OVERRUN;
        }

        if(lErrors & UART_RXST_PE)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_PARITY;
        }

        if(lErrors & UART_RXST_FE)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_FRAMING;
        }

        if(lErrors & UART_RXST_BRKDT)
        {
            usSerialState |= USB_CDC_SERIAL_STATE_BREAK;
        }

        //
        // Call the CDC driver to notify the state change.
        //
        USBDCDCSerialStateChange((void *)psDevice, usSerialState);
    }
}

//
// ReadUARTData - Read as many characters from the UART FIFO as possible and 
// move them into the CDC transmit buffer.
//
// \return Returns UART error flags read during data reception.
//
static long
ReadUARTData(void)
{
    int32_t i32Char, i32Errors;
    uint8_t ui8Char;
    uint32_t ui32Space;

    //
    // Clear the error indicator.
    //
    i32Errors = 0;

    //
    // How much space is available in the buffer?
    //
    ui32Space = USBBufferSpaceAvailable(&g_sTxBuffer);

    //
    // Read data from the UART FIFO until there is none left or there is no
    // more space in the receive buffer.
    //
    while(ui32Space && UARTCharsAvail(UART1_BASE))
    {
        //
        // Read a character from the UART FIFO into the ring buffer if no
        // errors are reported.
        //
        i32Char = UARTCharGetNonBlocking(UART1_BASE);

        //
        // If the character did not contain any error notifications,
        // copy it to the output buffer.
        //
        if(!(i32Char & ~0xFF))
        {
            ui8Char = (unsigned char)(i32Char & 0xFF);
            USBBufferWrite(&g_sTxBuffer, &ui8Char, 1);

            //
            // Decrement the number of bytes the buffer can accept.
            //
            ui32Space--;
        }
        else
        {
            //
            // Update the error accumulator.
            //
            i32Errors |= i32Char;
        }
    }

    //
    // Pass back the accumulated error indicators.
    //
    return(i32Errors);
}

//
// USBUARTPrimeTransmit - Take as many bytes from the transmit buffer as there
// is space for and move them into the USB UART's transmit FIFO.
//
static void
USBUARTPrimeTransmit(void)
{
    uint32_t ui32Read;
    uint8_t ui8Char;

    //
    // If a break condition is currently being sent, don't receive any more
    // data.  Transmission will resume once the break is turned off.
    //
    if(g_bSendingBreak)
    {
        return;
    }

    //
    // If there is space in the UART FIFO, try to read some characters
    // from the receive buffer to fill it again.
    //
    while(UARTSpaceAvail(UART1_BASE))
    {
        //
        // Get a character from the buffer.
        //
        ui32Read = USBBufferRead(&g_sRxBuffer, &ui8Char, 1);

        //
        // Was a character read?
        //
        if(ui32Read)
        {
            //
            // Place the character in the UART transmit FIFO.
            //
            UARTCharPutNonBlocking(UART1_BASE, ui8Char);
        }
        else
        {
            //
            // There are no more characters so exit the function.
            //
            return;
        }
    }
}

//
// USBUARTTXIntHandler- Interrupt handler for the UART TX which is being 
// redirected via USB.
//
__interrupt void
USBUARTTXIntHandler(void)
{
    uint32_t ui32Ints;

    ui32Ints = UARTIntStatus(UART1_BASE, true);
    //
    // Handle transmit interrupts.
    //
    if(ui32Ints & UART_INT_TXRDY)
    {
        //
        // Move as many bytes as possible into the transmit FIFO.
        //
        USBUARTPrimeTransmit();

        //
        // If the output buffer is empty, turn off the transmit interrupt.
        //
        if(!USBBufferDataAvailable(&g_sRxBuffer))
        {
            UARTIntDisable(UART1_BASE, UART_INT_TXRDY);
        }
    }

    PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}

//
// USBUARTRXIntHandler - Interrupt handler for the UART RX which is being 
// redirected via USB.
//
__interrupt void
USBUARTRXIntHandler(void)
{
    uint32_t ui32Ints;

    ui32Ints = UARTIntStatus(UART1_BASE, true);
    
    //
    // Handle receive interrupts.
    //
    if(ui32Ints & UART_INT_RXRDY_BRKDT)
    {
        //
        // Read the UART's characters into the buffer.
        //
        ReadUARTData();

    }
    else if(ui32Ints & UART_INT_RXERR)
    {
        //
        //Notify Host of our error
        //
        CheckForSerialStateChange(&g_sCDCDevice, UARTRxErrorGet(UART1_BASE));

        //
        //Clear the error and continue
        //
        UARTRxErrorClear(UART1_BASE);
    }

    PieCtrlRegs.PIEACK.bit.ACK9 = 1;

}

//
// SetLineCoding - Set the communication parameters to use on the UART.
//
static tBoolean
SetLineCoding(tLineCoding *psLineCoding)
{
    uint32_t ui32Config;
    tBoolean bRetcode;

    //
    // Assume everything is OK until a problem is detected.
    //
    bRetcode = true;

    //
    // Word length.  For invalid values, the default is to set 8 bits per
    // character and return an error.
    //
    switch(psLineCoding->ui8Databits)
    {
        case 5:
        {
            ui32Config = UART_CONFIG_WLEN_5;
            break;
        }

        case 6:
        {
            ui32Config = UART_CONFIG_WLEN_6;
            break;
        }

        case 7:
        {
            ui32Config = UART_CONFIG_WLEN_7;
            break;
        }

        case 8:
        {
            ui32Config = UART_CONFIG_WLEN_8;
            break;
        }

        default:
        {
            ui32Config = UART_CONFIG_WLEN_8;
            bRetcode = false;
            break;
        }
    }

    //
    // Parity.  For any invalid values, set no parity and return an error.
    //
    switch(psLineCoding->ui8Parity)
    {
        case USB_CDC_PARITY_NONE:
        {
            ui32Config |= UART_CONFIG_PAR_NONE;
            break;
        }

        case USB_CDC_PARITY_ODD:
        {
            ui32Config |= UART_CONFIG_PAR_ODD;
            break;
        }

        case USB_CDC_PARITY_EVEN:
        {
            ui32Config |= UART_CONFIG_PAR_EVEN;
            break;
        }

        default:
        {
            ui32Config |= UART_CONFIG_PAR_NONE;
            bRetcode = false;
            break;
        }
    }

    //
    // Stop bits.  The hardware only supports 1 or 2 stop bits whereas CDC
    // allows the host to select 1.5 stop bits.  If passed 1.5 (or any other
    // invalid or unsupported value of ucStop, set up for 1 stop bit but return
    // an error in case the caller needs to Stall or otherwise report this back
    // to the host.
    //
    switch(psLineCoding->ui8Stop)
    {
        //
        // One stop bit requested.
        //
        case USB_CDC_STOP_BITS_1:
        {
            ui32Config |= UART_CONFIG_STOP_ONE;
            break;
        }

        //
        // Two stop bits requested.
        //
        case USB_CDC_STOP_BITS_2:
        {
            ui32Config |= UART_CONFIG_STOP_TWO;
            break;
        }

        //
        // Other cases are either invalid values of ucStop or values that are
        // not supported, so set 1 stop bit but return an error.
        //
        default:
        {
            ui32Config = UART_CONFIG_STOP_ONE;
            bRetcode = false;
            break;
        }
    }

    //
    // Set the UART mode appropriately.
    //
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED)/4,
    		            readusb32_t(&(psLineCoding->ui32Rate)), ui32Config);

    //
    // Let the caller know if a problem was encountered.
    //
    return(bRetcode);
}

//
// GetLineCoding - Get the communication parameters in use on the UART.
//
static void
GetLineCoding(tLineCoding *psLineCoding)
{
    uint32_t ui32Config;
    uint32_t ui32Rate;

    //
    // Get the current line coding set in the UART.
    //
    UARTConfigGetExpClk(UART1_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED)/4,
                        &ui32Rate, &ui32Config);
    writeusb32_t(&(psLineCoding->ui32Rate), ui32Rate);

    //
    // Translate the configuration word length field into the format expected
    // by the host.
    //
    switch(ui32Config & UART_CONFIG_WLEN_MASK)
    {
        case UART_CONFIG_WLEN_8:
        {
            psLineCoding->ui8Databits = 8;
            break;
        }

        case UART_CONFIG_WLEN_7:
        {
            psLineCoding->ui8Databits = 7;
            break;
        }

        case UART_CONFIG_WLEN_6:
        {
            psLineCoding->ui8Databits = 6;
            break;
        }

        case UART_CONFIG_WLEN_5:
        {
            psLineCoding->ui8Databits = 5;
            break;
        }
    }

    //
    // Translate the configuration parity field into the format expected
    // by the host.
    //
    switch(ui32Config & UART_CONFIG_PAR_MASK)
    {
        case UART_CONFIG_PAR_NONE:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_NONE;
            break;
        }

        case UART_CONFIG_PAR_ODD:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_ODD;
            break;
        }

        case UART_CONFIG_PAR_EVEN:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_EVEN;
            break;
        }
    }

    //
    // Translate the configuration stop bits field into the format expected
    // by the host.
    //
    switch(ui32Config & UART_CONFIG_STOP_MASK)
    {
        case UART_CONFIG_STOP_ONE:
        {
            psLineCoding->ui8Stop = USB_CDC_STOP_BITS_1;
            break;
        }

        case UART_CONFIG_STOP_TWO:
        {
            psLineCoding->ui8Stop = USB_CDC_STOP_BITS_2;
            break;
        }
    }
}

//
// SendBreak - This function sets or clears a break condition on the redirected
// UART RX line.  A break is started when the function is called with \e bSend 
// set to \b true and persists until the function is called again with \e bSend
// set to \b false.
//
static void
SendBreak(tBoolean bSend)
{
    //
    //C28x SCI cannot send break conditions
    //
    return;
}

//
// ControlHandler - Handles CDC driver notifications related to control and 
// setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the notification event.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
unsigned long
ControlHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
               void *pvMsgData)
{
    //
    // Which event was sent?
    //
    switch(ui32Event)
    {
        //
        // The host has connected.
        //
        case USB_EVENT_CONNECTED:
        {
            //
            // Flush the buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            break;
        }

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
        {
            break;
        }

        //
        // Return the current serial communication parameters.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
        {
            GetLineCoding(pvMsgData);
            break;
        }

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_LINE_CODING:
        {
            SetLineCoding(pvMsgData);
            break;
        }

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
        {
            break;
        }

        //
        // Send a break condition on the serial line.
        //
        case USBD_CDC_EVENT_SEND_BREAK:
        {
            SendBreak(true);
            break;
        }

        //
        // Clear the break condition on the serial line.
        //
        case USBD_CDC_EVENT_CLEAR_BREAK:
        {
            SendBreak(false);
            break;
        }

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        {
            break;
        }

        //
        // Other events can be safely ignored.
        //
        default:
        {
            break;
        }
    }
    return(0);
}

//
// TxHandler - Handles CDC driver notifications related to the transmit channel
// (data to the USB host).
//
// \param ui32CBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the notification event.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
unsigned long
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // Which event was sent?
    //
    switch(ui32Event)
    {
        case USB_EVENT_TX_COMPLETE:
        {
            //
            // There is nothing to do here since it is handled by the
            // USBBuffer.
            //
            break;
        }

        //
        // Other events can be safely ignored.
        //
        default:
        {
            break;
        }
    }

    return(0);
}

//
// RxHandler - Handles CDC driver notifications related to the receive channel 
// (data from the USB host).
//
// \param ui32CBData is the client-supplied callback data value for this channel.
// \param ui32Event identifies the notification event.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
unsigned long
RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    uint32_t ui32Count;

    //
    // Which event was sent?
    //
    switch(ui32Event)
    {
        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            //
            // Feed some characters into the UART TX FIFO and enable the
            // interrupt.
            //
            USBUARTPrimeTransmit();
            UARTIntEnable(UART1_BASE, UART_INT_TXRDY);
            break;
        }

        //
        // This is a request for how much unprocessed data is still waiting to
        // be processed.  Return 0 if the UART is currently idle or 1 if it is
        // in the process of transmitting something.  The actual number of
        // bytes in the UART FIFO is not important here, merely whether or
        // not everything previously sent to us has been transmitted.
        //
        case USB_EVENT_DATA_REMAINING:
        {
            //
            // Get the number of bytes in the buffer and add 1 if some data
            // still has to clear the transmitter.
            //
            ui32Count = UARTBusy(UART1_BASE) ? 1 : 0;
            return(ui32Count);
        }

        //
        // This is a request for a buffer into which the next packet can be
        // read.  This mode of receiving data is not supported so let the
        // driver know by returning 0.  The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        //
        // Other events can be safely ignored.
        //
        default:
        {
            break;
        }
    }

    return(0);
}

//
// SysCtrlInit - 
//
void
SysCtrlInit(void)
{
    EALLOW;
    
    //
    // Disable Watchdog
    //
    SysCtrlRegs.WDCR = 0x68;

    //
    // Setup Clock
    // 20MHz ->PLL->80MHz->C28
    //      ->PLL2->120MHz->USB
    //
     SysCtrlRegs.CLKCTL.bit.XCLKINOFF = 1;
///    SysCtrlRegs.CLKCTL.bit.XTALOSCOFF = 1;                          // MUST be set to 1 if XCLKIN used
///    SysCtrlRegs.CLKCTL.bit.XCLKINOFF = 0;
    SysCtrlRegs.CLKCTL.bit.XTALOSCOFF = 0;
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 0;
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;

    while(SysCtrlRegs.PLLSTS.bit.MCLKSTS);
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
///    SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
///    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
    SysCtrlRegs.PLLCR.bit.DIV = 4;
///    SysCtrlRegs.PLLCR.bit.DIV = 1;
    while(!SysCtrlRegs.PLLSTS.bit.PLLLOCKS);
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;


    SysCtrlRegs.PLL2CTL.bit.PLL2CLKSRCSEL = 2;                    // 20 MHz XTAL
//    SysCtrlRegs.PLL2CTL.bit.PLL2CLKSRCSEL = 3;                      // 90 MHz MEMS
    SysCtrlRegs.PLL2CTL.bit.PLL2EN = 1;
    SysCtrlRegs.PLL2MULT.bit.PLL2MULT = 6;                        // 20 * 6 = 120 MHz
//    SysCtrlRegs.PLL2MULT.bit.PLL2MULT = 1;
    while(!SysCtrlRegs.PLL2STS.bit.PLL2LOCKS);

#ifdef USE_USB

    //
    // Enable USB Clock
    //
    SysCtrlRegs.PCLKCR3.bit.USB0ENCLK = 1;
    
    //
    // Enable PHY
    //
    GpioCtrlRegs.GPACTRL2.bit.USB0IOEN = 1;

#endif
    EDIS;
}
#endif

//
// main - This is the main application entry function.
//
#ifdef NO_MAIN
int main(void)
{
	unsigned char f_f = 0;
	Uint16 i;
	Uint32 qdataL;
	Uint32 sdataL;  // send data
	Uint32 rdataL;  // received data
	Uint16 xmit_fmt;
	Uint16 rcv_fmt;
	Uint16 status1;
	Uint16 status2;
	Uint16 status3;
	Uint16 status4;

	//
    // Section .ebss was initialized to 0x0000 in
	// F2806x_CodeStartBranch_ebss_init.asm. This was to ensure that all global
	// variables of the usblib are initialized to 0x0000 on the F2806x.
	//
    DisableDog();
    //
    // Set the clocking to run from the PLL
    //
    DINT;

    //
    // Disable CPU interrupts and clear all CPU interrupt flags
    //
    IER = 0x0000;
    IFR = 0x0000;
    SysCtrlInit();
 //    InitAdc();         // For this example, init the ADC
 //    AdcOffsetSelfCal();
    themis_gpio_init();   // initialize themis gpio
   
    themis_usb_gpio_init();
    themis_uart_gpio_init();

//    InitSysCtrl();

    InitSpiaGpio();
    InitECap1Gpio();

//    InitScibGpio();
    InitPieCtrl();
    InitPieVectTable();

    //
     // Interrupts that are used in this example are re-mapped to
     // ISR functions found within this file.
     //
     EALLOW;  // This is needed to write to EALLOW protected register
     PieVectTable.ADCINT1 = &adc_isr;
     EDIS;    // This is needed to disable write to EALLOW protected registers
     //
       // Interrupts that are used in this example are re-mapped to
       // ISR functions found within this file.
       //
       EALLOW;    // This is needed to write to EALLOW protected registers
       PieVectTable.ECAP1_INT = &ecap1_isr;
       PieVectTable.ECAP2_INT = &ecap2_isr;
       EDIS;      // This is needed to disable write to EALLOW protected registers

       //
       // Step 4. Initialize all the Device Peripherals:
       // This function is found in F2806x_InitPeripherals.c
       //
       // InitPeripherals();  // Not required for this example

       InitECapture();


    InitPeripheralClocks();
    InitAdcLocal();
    InitAdcAio();


//    GpioCtrlRegs.AIOMUX1.bit. = DISABLE;

//    AdcOffsetSelfCal();



#ifdef DAC_PWM
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM7_INT = &epwm7_timer_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers
#endif

    InitHRCapGpio();

#ifdef USE_HRCAP1
    EALLOW;
    PieVectTable.HRCAP1_INT = &HRCAP1_Isr;
    EDIS;
#endif
#ifdef USE_HRCAP2
    EALLOW;
    PieVectTable.HRCAP2_INT = &HRCAP2_Isr;
    EDIS;
#endif
#ifdef USE_HRCAP3
    EALLOW;
    PieVectTable.HRCAP3_INT = &HRCAP3_Isr;
    EDIS;
#endif
#ifdef USE_HRCAP4
    EALLOW;
    PieVectTable.HRCAP4_INT = &HRCAP4_Isr;
    EDIS;
#endif

   //
   // While calibration is incomplete
   //
#if 0
#ifdef USE_HRCAP1
    status1 = 0;

    while (status1!=2)
    {
        status1 = HRCAP_Cal(1, HCCAPCLK_PLLCLK, &EPwm8Regs);
        if (status1 == HCCAL_ERROR)
        {
       //
       // If there is an error on HRCAP, stop and
       // check 98 MHz < PLL2CLK < 120 MHz .
       //
            ESTOP0;
        }
    }
#endif
#endif    
//// - Note - due to UG - one channel has to be used for calibration -
//// not any others.

#ifdef USE_HRCAP2
    status2 = 0;
    
    while (status2!=2)
    {
	    status2 = HRCAP_Cal(2, HCCAPCLK_PLLCLK, &EPwm8Regs);
//	    status2 = HRCAP_Cal(4, HCCAPCLK_PLLCLK, &EPwm8Regs);
	    if (status2 == HCCAL_ERROR)
	    {
       //
       // If there is an error on HRCAP, stop and
       // check 98 MHz < PLL2CLK < 120 MHz .
       //
		    ESTOP0;
	    }
    }
#endif
#if 0
#ifdef USE_HRCAP3
    status3 = 0;

    while (status3!=2)
    {
	    status3 = HRCAP_Cal(3, HCCAPCLK_PLLCLK, &EPwm8Regs);
	    if (status3 == HCCAL_ERROR)
	    {
       //
       // If there is an error on HRCAP, stop and
       // check 98 MHz < PLL2CLK < 120 MHz .
       //
		    ESTOP0;
	    }
    }
#endif
#endif
#if 0
#ifdef USE_HRCAP4
    status4 = 0;

    while (status4!=2)
    {
	    status4 = HRCAP_Cal(4, HCCAPCLK_PLLCLK, &EPwm8Regs);
	    if (status4 == HCCAL_ERROR)
	    {
       //
       // If there is an error on HRCAP, stop and
       // check 98 MHz < PLL2CLK < 120 MHz .
       //
		    ESTOP0;
	    }
    }  
#endif
#endif
#ifdef USE_HRCAP1  
    HRCAP1_Config();                       // Configure HRCAP1 Module
    datacounter1 = 0;  // marks how many pulses have been captured
    first1 = 0;        // marks first captured data after a SOFTRESET to discard    
#endif
#if 1
#ifdef USE_HRCAP2
    HRCAP2_Config();                       // Configure HRCAP2 Module
    datacounter2 = 0;  // marks how many pulses have been captured
    first2 = 0;        // marks first captured data after a SOFTRESET to discard
#endif
#endif    
#ifdef USE_HRCAP3
    HRCAP3_Config();                       // Configure HRCAP3 Module
    datacounter3 = 0;  // marks how many pulses have been captured
    first3 = 0;        // marks first captured data after a SOFTRESET to discard
#endif
#ifdef USE_HRCAP4
    HRCAP4_Config();                       // Configure HRCAP4 Module
    datacounter4 = 0;  // marks how many pulses have been captured
    first4 = 0;        // marks first captured data after a SOFTRESET to discard
#endif

#ifdef DAC_PWM
      EALLOW;
      SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
      EDIS;

      InitEPwmTimer();    // For this example, only initialize the ePWM Timer();

      EALLOW;
      SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
      EDIS;
#endif

#ifdef DAC_PWM
     IER |= M_INT3;

     //
      // Enable EPWM INTn in the PIE: Group 3 interrupt 1-6
      //
      EALLOW;
      PieCtrlRegs.PIEIER3.bit.INTx7 = PWM7_INT_ENABLE;
      EDIS;
    //
    // Initalize counters
    //
     EPwm7TimerIntCount = 0;
      
     //
     // Initially disable time-critical interrupts
     //
     SetDBGIER(0x0000);          // PIE groups time-critical designation

     //
     // Enable global Interrupts and higher priority real-time debug events
     //
//     EINT;   // Enable Global interrupt INTM
//     ERTM;   // Enable Global realtime interrupt DBGM
#endif

#ifdef USE_SPI_IRQ    
      EALLOW;    // This is needed to write to EALLOW protected registers
      PieVectTable.SPIRXINTA = &spiRxFifoIsr;
      PieVectTable.SPITXINTA = &spiTxFifoIsr;
      EDIS;      // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in F2806x_InitPeripherals.c
    //
    //InitPeripherals(); // Not required for this example
      spi_fifo_init();   // Initialize the SPI only
    //
    // Step 5. User specific code, enable interrupts
    //

    //
    // Initalize the send data buffer
    //
      for(i=0; i<2; i++)
      {
	      sdata[i] = i;
      }
      rdata_point = 0;


#else
      
      spi_fifo_init();   // Initialize the SPI only
      spi_init();		  // init SPI
#if 0
      sdataL = 0x0000ffff;		// our initial DAC setting
      rdataL = 0x00ffffff;      // our initial PGA setting
      qdataL = 0x0000ffff;
#endif
      sdataL = 0x0;
      rdataL = 0x0;
      qdataL = 0x0;


      EALLOW;
//             GpioDataRegs.GPBTOGGLE.bit.GPIO51;
#if 1
      GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
      GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
      GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
      GpioDataRegs.GPBDAT.bit.GPIO51 = DISABLE;
//            GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
//             GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

//             GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
#endif
      EDIS;
      EALLOW;
//		       GpioDataRegs.GPBTOGGLE.bit.GPIO51;
#if 1
      GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
      GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
      GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
      GpioDataRegs.GPBDAT.bit.GPIO51 = DISABLE;
//		                    GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
		      //             GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

		      //             GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
#endif
      EDIS;

      DSP28x_usDelay(2);

      
      spi_xmit(0x0000ffff, 'l');

      DSP28x_usDelay(20);
		       EALLOW;
//		       GpioDataRegs.GPBTOGGLE.bit.GPIO51;
#if 1
		       GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
		       GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
		       GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
		       GpioDataRegs.GPBDAT.bit.GPIO51 = ENABLE;
//		                    GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
		      //             GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

		      //             GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
#endif
		       EDIS;

      
//      spi_xmit(0xff);
//      spi_xmit(0xff);
#endif


#if 0
// EN 10MHz_CLK_EN
                                             EALLOW;

                                             GpioCtrlRegs.GPAMUX1.bit.GPIO10 = DISABLE;              //
                                             GpioDataRegs.GPASET.bit.GPIO10 = ENABLE;
                                             GpioCtrlRegs.GPADIR.bit.GPIO10 = ENABLE;
                                             GpioDataRegs.GPADAT.bit.GPIO10 = DISABLE;
                                            //GpioCtrlRegs.GPAPUD.bit.GPIO10 = ENABLE;                // this means NO pullup is enabled
                                            // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

                                             GpioCtrlRegs.GPAMUX1.bit.GPIO1 = DISABLE;                // 10MHZ_6TIN_EN
                                             GpioDataRegs.GPASET.bit.GPIO1 = ENABLE;
                                             GpioCtrlRegs.GPADIR.bit.GPIO1 = ENABLE;                 // active low
                                             GpioDataRegs.GPADAT.bit.GPIO1 = DISABLE;
                                             // GpioCtrlRegs.GPAPUD.bit.GPIO1 = ENABLE;

                                             GpioCtrlRegs.GPBMUX2.bit.GPIO58 = DISABLE;                // 10MHZ_OUT6T_EN
                                             GpioDataRegs.GPBSET.bit.GPIO58 = ENABLE;
                                             // GpioCtrlRegs.GPBPUD.bit.GPIO58 = ENABLE;
                                             GpioCtrlRegs.GPBDIR.bit.GPIO58 = ENABLE;                // mux - this selects A
                                             GpioDataRegs.GPBDAT.bit.GPIO58 = ENABLE;

                                             GpioCtrlRegs.GPBMUX2.bit.GPIO52 = DISABLE;              // OCXO CLOCK SEL
                                             GpioDataRegs.GPBSET.bit.GPIO52 = ENABLE;
                                             GpioCtrlRegs.GPBDIR.bit.GPIO52 = ENABLE;
                                             GpioDataRegs.GPBDAT.bit.GPIO52 = DISABLE;
                                          //   GpioCtrlRegs.GPBPUD.bit.GPIO52 = ENABLE;                // this means NO pullup is enabled
                                          // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

                                             EDIS;
#endif
					     DSP28x_usDelay(2);
    EALLOW;
    HWREG(USBMODESEL) = USBMODESEL_DEV;
    HWREG(USB0_BASE + USB_O_GPCS) = USBGPCS_DEV;
    EDIS;
    
 //   InitEPwmTimer();    // For this example, only initialize the ePWM Timer
    // Enable interrupts required for this example
                         EALLOW;
                         PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
                         PieCtrlRegs.PIEIER6.bit.INTx1=1;     // Enable PIE Group 6, INT 1
                         PieCtrlRegs.PIEIER6.bit.INTx2=1;     // Enable PIE Group 6, INT 2


                         //
                          // Enable ADCINT1 in PIE
                          //
                          PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
//                          IER |= M_INT1;                     // Enable CPU Interrupt 1
//                          EINT;                              // Enable Global interrupt INTM
//                          ERTM;                              // Enable Global realtime interrupt DBGM


//                         EDIS;

 //   InitAdc();         // For this example, init the ADC
 //   AdcOffsetSelfCal();
                         IER|= M_INT1 | M_INT5 | M_INT9 | M_INT3;                         // Enable CPU INT4

                          IER|=0x20;                            // Enable CPU INT6
//                          EINT;
                          EDIS;

    //
    // Enable Device Mode
    //

 //   Gpio_select();                          // init GPIO
    IntRegister(INT_TINT0, SysTickIntHandler);
    //
    // Set the system tick to fire 100 times per second.
    //
    SysTickPeriodSet(SysCtlClockGet(SYSTEM_CLOCK_SPEED) / SYSTICKS_PER_SECOND);
    SysTickIntEnable();
    SysTickEnable();


    //
    // Enable the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

//    UARTStdioInit(1);
    //
    // Enable and configure the UART RX and TX pins
    //
//          USBGPIOEnable();
#ifdef USE_USB
        //
        // Register interrupt handlers
        //
        IntRegister(INT_SCIRXINTB, USBUARTRXIntHandler);
//        IntRegister(USBUARTRXIntHandler,INT_SCIRXINTB);
        IntRegister(INT_SCITXINTB, USBUARTTXIntHandler);
//        IntRegister(INT_SCITXINTB, USBUARTRXIntHandler);

        //
        // Configure the required pins for USB operation.
        //
        USBGPIOEnable();
        USBIntRegister(USB0_BASE, f28x_USB0DeviceIntHandler);

        //
        // Set the default UART configuration.
        //
#if 1
        UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED)/4,
                            115200, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE |
                            UART_CONFIG_STOP_ONE);
        UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
#endif
        //
        // Configure and enable UART interrupts.
        //
#if 1
        UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, false));
        UARTIntEnable(UART1_BASE, (UART_INT_RXERR | UART_INT_RXRDY_BRKDT |
                      UART_INT_TXRDY ));
#endif
        //
        // Initialize the transmit and receive buffers.
        //
        USBBufferInit(&g_sTxBuffer);
        USBBufferInit(&g_sRxBuffer);

        //
        // Set the USB stack mode to Device mode with VBUS monitoring.
        //
        USBStackModeSet(0, eUSBModeForceDevice, 0);

        //
        // Pass the device information to the USB library and place the device
        // on the bus.
        //
        USBDCDCInit(0, &g_sCDCDevice);

        //
        // Enable interrupts now that the application is ready to start.
        //
        IntEnable(INT_SCITXINTB);
        IntEnable(INT_SCIRXINTB);

        IntMasterEnable();
        // Enable CPU INT3 which is connected to EPWM1-6 INT
         //

        //


#ifdef ENABLE_SCITXRXINT_IRQS
	EALLOW;
	PieCtrlRegs.PIEIER9.bit.INTx4=1;     // Enable PIE Group 9, INT 4
	PieCtrlRegs.PIEIER9.bit.INTx3=1;     // Enable PIE Group 9, INT 3
	EDIS;
#endif

	
        // Initialize variables
        //
	 //

#endif

    EALLOW;
#ifdef ENABLE_HRCAP_IRQ2
                         PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 8
#endif
#ifdef ENABLE_HRCAP_IRQ1
                         PieCtrlRegs.PIEIER4.bit.INTx7=1;     // Enable PIE Group 4, INT 7
#endif
#ifdef ENABLE_HRCAP_IRQ4
                         PieCtrlRegs.PIEIER5.bit.INTx5=1;     // Enable PIE Group 4, INT 8
#endif
#ifdef ENABLE_HRCAP_IRQ3
                         PieCtrlRegs.PIEIER5.bit.INTx4=1;     // Enable PIE Group 4, INT 8
#endif

                         EDIS;

          // Enable Global Interrupts


//       UARTprintf("\033[2JUSB device application\n");
	// EN 10MHz_CLK_EN
	                                             EALLOW;

	                                             GpioCtrlRegs.GPAMUX1.bit.GPIO10 = DISABLE;              //  10MHz_CLK_EN
	                                             // GpioDataRegs.GPASET.bit.GPIO10 = ENABLE;
	                                             GpioDataRegs.GPASET.bit.GPIO10 = DISABLE;
	                                             GpioCtrlRegs.GPADIR.bit.GPIO10 = ENABLE;
	                                             // GpioCtrlRegs.GPADIR.bit.GPIO10 = DISABLE;
	                                              GpioDataRegs.GPADAT.bit.GPIO10 = DISABLE;              // HRCAP2 CLK INPUT
	                                              GpioCtrlRegs.GPAPUD.bit.GPIO10 = ENABLE;                // (DISABLE) this means NO pullup is enabled
	                                            // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

	                                             GpioCtrlRegs.GPAMUX1.bit.GPIO1 = DISABLE;                // 10MHZ_6TIN_EN
	                                             GpioDataRegs.GPASET.bit.GPIO1 = ENABLE;
	                                             // GpioDataRegs.GPASET.bit.GPIO1 = DISABLE;
	                                             // GpioCtrlRegs.GPADIR.bit.GPIO1 = DISABLE;
	                                             GpioCtrlRegs.GPADIR.bit.GPIO1 = ENABLE;                 // active low
	                                             GpioDataRegs.GPADAT.bit.GPIO1 = DISABLE;
	                                             GpioCtrlRegs.GPAPUD.bit.GPIO1 = ENABLE;

	                                             GpioCtrlRegs.GPBMUX2.bit.GPIO58 = DISABLE;                // 10MHZ_OUT6T_EN
	                                             GpioDataRegs.GPBSET.bit.GPIO58 = ENABLE;
	                                             // GpioDataRegs.GPBSET.bit.GPIO58 = DISABLE;
	                                             GpioCtrlRegs.GPBPUD.bit.GPIO58 = ENABLE;
	                                             // GpioCtrlRegs.GPBDIR.bit.GPIO58 = DISABLE;                // mux - this selects A
	                                             GpioCtrlRegs.GPBDIR.bit.GPIO58 = ENABLE;
	                                             GpioDataRegs.GPBDAT.bit.GPIO58 = DISABLE;

	                                             GpioCtrlRegs.GPBMUX2.bit.GPIO52 = DISABLE;              // OCXO CLOCK SEL
	                                             GpioDataRegs.GPBSET.bit.GPIO52 = DISABLE;
	                                             // GpioDataRegs.GPBSET.bit.GPIO52 = ENABLE;
	                                             GpioCtrlRegs.GPBDIR.bit.GPIO52 = ENABLE;
	                                             // GpioCtrlRegs.GPBDIR.bit.GPIO52 = DISABLE;
	                                             GpioDataRegs.GPBDAT.bit.GPIO52 = DISABLE;
	                                          //   GpioCtrlRegs.GPBPUD.bit.GPIO52 = ENABLE;                // this means NO pullup is enabled
	                                          // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;


	                                             GpioCtrlRegs.GPBMUX2.bit.GPIO53 = DISABLE;              //  OCXO_CLK_EN
	                                                                                              // GpioDataRegs.GPASET.bit.GPIO10 = ENABLE;
	                                                                                              GpioDataRegs.GPBSET.bit.GPIO53 = DISABLE;
	                                                                                              GpioCtrlRegs.GPBDIR.bit.GPIO53 = ENABLE;
	                                                                                              // GpioCtrlRegs.GPADIR.bit.GPIO10 = DISABLE;
	                                                                                               GpioDataRegs.GPBDAT.bit.GPIO53 = DISABLE;              // HRCAP2 CLK INPUT
	                                                                                               GpioCtrlRegs.GPBPUD.bit.GPIO53 = ENABLE;                // (DISABLE) this means NO pullup is enabled
	                                                                                             // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;

	                                             //
	                                             // Enable CPU INT4 which is connected to ECAP1-4 INT
	                                             //
	                                             IER |= M_INT4;

	                                             //
	                                             // Enable eCAP INTn in the PIE: Group 3 interrupt 1-6
	                                             //
	                                             PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
	                                             PieCtrlRegs.PIEIER4.bit.INTx2 = 1;
	                                             //
	                                             // Enable global Interrupts and higher priority real-time debug events


                                        // EINT;
	                                             EDIS;
	                                             EINT;
	                                             ERTM;   // Enable Global realtime interrupt DBGM
	                                             //
	                                                                                              // Initalize counters
	                                                                                              //
	                                                                                              ECap1IntCount = 0;
	                                                                                              ECap1PassCount = 0;

    //
    // Main application loop.
    //
    while(1)
    {
#if 0
            EALLOW;
            GpioCtrlRegs.GPAMUX1.bit.GPIO8 = ENABLE;                // ENABLE MEMS OSC - 90 MHz
            GpioCtrlRegs.GPAPUD.bit.GPIO8 = DISABLE;
            GpioCtrlRegs.GPADIR.bit.GPIO8 = ENABLE;
            EDIS;
#endif



#ifndef USB_IO_DBG
#if 0
#ifdef USE_HRCAP1

//	    if (datacounter1 == 5)
        if ( do_cal_1 )
	    {
		    __asm (" nop");        // Set breakpoint here for debug

	//
	// Run calibration routine periodically in background using HRCAP1 
	// internally tied to HRPWM8 output.
	//
	    status1 = 0;                // New calibration not complete

	//
	// While calibration is incomplete
	//
	    while (status1!=2)
	    {
		    status1 = HRCAP_Cal(2,HCCAPCLK_PLLCLK, &EPwm8Regs);			// Ch. 2 is connected to 'Cal'
//		    status1 = HRCAP_Cal(1,HCCAPCLK_PLLCLK, &EPwm8Regs);
		    if (status1 == HCCAL_ERROR) 
		    { 
		//
		// If there is an error on HRCAP, stop and 
		// check 98 MHz < PLL2CLK < 120 MHz .
		//
			    ESTOP0;
		    }
	    }
#if 1
        EALLOW;
#if 0
#ifdef USE_HRCAP1
        HRCap1Regs.HCCTL.bit.RISEINTE = 1;
#endif
#endif
#if 1
#ifdef USE_HRCAP2
        HRCap2Regs.HCCTL.bit.RISEINTE = 1;
#endif
#endif
#if 0
#ifdef USE_HRCAP3
        HRCap3Regs.HCCTL.bit.RISEINTE = 1;
#endif
#endif
#if 0
#ifdef USE_HRCAP4
        HRCap4Regs.HCCTL.bit.RISEINTE = 1;
#endif
#endif
        EDIS;
#endif


//	    if (datacounter1 == 5)
//	            {
//	                datacounter1 = 0;
//	            }
        do_cal_1 = 0;
	    }

////	    HRCAP1_Config();
#endif
#endif
#if 0	    
#ifdef USE_HRCAP2
	    
	    if (datacounter2 == 5)
	    {
		    __asm (" nop");        // Set breakpoint here for debug


	//
	// Run calibration routine periodically in background using HRCAP1 
	// internally tied to HRPWM8 output.
	//
	    status2 = 0;                // New calibration not complete

	//
	// While calibration is incomplete
	//
	    while (status2!=2)
	    {
		    status2 = HRCAP_Cal(2,HCCAPCLK_PLLCLK, &EPwm8Regs);
		    if (status2 == HCCAL_ERROR) 
		    { 
		//
		// If there is an error on HRCAP, stop and 
		// check 98 MHz < PLL2CLK < 120 MHz .
		//
			    ESTOP0;
		    }
	    }
	    }
#endif
#endif
#if 0
#ifdef USE_HRCAP3
	    
	    if (datacounter3 == 5)
	    {
		    __asm (" nop");        // Set breakpoint here for debug


	//
	// Run calibration routine periodically in background using HRCAP1 
	// internally tied to HRPWM8 output.
	//
	    status3 = 0;                // New calibration not complete

	//
	// While calibration is incomplete
	//
	    while (status3!=2)
	    {
		    status3 = HRCAP_Cal(2,HCCAPCLK_PLLCLK, &EPwm8Regs);
		    if (status3 == HCCAL_ERROR) 
		    { 
		//
		// If there is an error on HRCAP, stop and 
		// check 98 MHz < PLL2CLK < 120 MHz .
		//
			    ESTOP0;
		    }
	    }
	    }	    
#endif
#endif
#if 0
#ifdef USE_HRCAP4
	    
	    if (datacounter4 == 5)
	    {
		    __asm (" nop");        // Set breakpoint here for debug


	//
	// Run calibration routine periodically in background using HRCAP1 
	// internally tied to HRPWM8 output.
	//
	    status4 = 0;                // New calibration not complete

	//
	// While calibration is incomplete
	//
	    while (status4!=2)
	    {
		    status4 = HRCAP_Cal(2,HCCAPCLK_PLLCLK, &EPwm8Regs);
		    if (status4 == HCCAL_ERROR) 
		    { 
		//
		// If there is an error on HRCAP, stop and 
		// check 98 MHz < PLL2CLK < 120 MHz .
		//
			    ESTOP0;
		    }
	    }
	    }	    

////	    HRCAP4_Config();
#endif
#endif	    
            EALLOW;
 //           GpioDataRegs.GPASET.bit.GPIO8 = ENABLE;
 //           GpioCtrlRegs.GPADIR.bit.GPIO8 = ENABLE;
 //           EDIS;

//            GpioCtrlRegs.GPACTRL.bit.GPIO4 = ENABLE;
//            GpioCtrlRegs.GPACTRL |= 0x10;
            GpioCtrlRegs.GPAMUX1.bit.GPIO4 = DISABLE;              //
            GpioDataRegs.GPASET.bit.GPIO4 =  DISABLE;
//            GpioCtrlRegs.GPADIR.bit.GPIO4 =  ENABLE;
            GpioDataRegs.GPADAT.bit.GPIO4 = ENABLE;

            if ( f_f == 1 )
            {
                GpioCtrlRegs.GPADIR.bit.GPIO4 =  ENABLE;
                f_f = 0;
//                spi_xmit(rdataL, 'p');      // set PGA
            } else {
                GpioCtrlRegs.GPADIR.bit.GPIO4  = DISABLE;
                f_f++;
//                spi_xmit(0x00000000, 'p');      // set PGA
            }
        //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
         //   GpioCtrlRegs.GPAPUD.bit.GPIO3 = ENABLE;                // this means NO pullup is enabled
         // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
            EDIS;
////            UARTprintf("\033[2JBulk device application\n");
	    EALLOW;
            GpioCtrlRegs.GPAMUX1.bit.GPIO4 = DISABLE;              //
            GpioDataRegs.GPASET.bit.GPIO4 = ENABLE;
            GpioCtrlRegs.GPADIR.bit.GPIO4 = ENABLE;
	    GpioDataRegs.GPADAT.bit.GPIO4 = DISABLE;
	    EDIS;
	    DSP28x_usDelay(1);
         //   GpioCtrlRegs.GPAPUD.bit.GPIO4 = ENABLE;                // this means NO pullup is enabled
         // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
//            delay_loop();
	    DINT;
	    spi_xmit(sdataL, 'l');		// set 'L' DAC
	    spi_xmit(qdataL, 'h');		// set 'H' DAC
	    spi_xmit(rdataL, 'p');      // set PGA
	    EINT;

#if 0
            EALLOW;
                       GpioCtrlRegs.GPAMUX1.bit.GPIO8 = ENABLE;                // ENABLE MEMS OSC - 90 MHz
                       GpioCtrlRegs.GPAPUD.bit.GPIO8 = DISABLE;
                       GpioCtrlRegs.GPADIR.bit.GPIO8 = DISABLE;
                       EDIS;
#endif
//                       delay_loop();
      //        
	// Transmit data
	//
// #ifndef USE_SPI_IRQ
#if 0
		       xmit_fmt = ((sdataL & 0xffff0000) >> 16);
		       // set 'H' CS low:
		       // gpio51 is H
		       EALLOW;
//		       GpioDataRegs.GPBTOGGLE.bit.GPIO51;

		       GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
		       GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
		       GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
		       GpioDataRegs.GPBDAT.bit.GPIO51 = DISABLE;
//		      GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
//		       GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

//		       GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
#endif
		       EDIS;

// #ifdef USE_NORM
#if 0
		       spi_xmit(xmit_fmt);

		       EALLOW;
//		       GpioDataRegs.GPBTOGGLE.bit.GPIO51;

		                    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
		                    GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
		                    GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
		                    GpioDataRegs.GPBDAT.bit.GPIO51 = ENABLE;
//		                    GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
		     //             GpioDataRegs.GPBDAT.bit.GPIO51 = ENABLE;
//		                    GpioDataRegs.GPASET.bit.GPIO17 = ENABLE;
// #endif
		                    EDIS;
// #else
		       xmit_fmt = (sdataL & 0x0000ffff);

		       spi_xmit(xmit_fmt);
#endif		       
		      
#if 0		       
	//
	// Wait until data is received
	//
		       while(SpiaRegs.SPIFFRX.bit.RXFFST !=1)
		       {

		       }

	//        
	// Check against sent data
	//
		       rcv_fmt = SpiaRegs.SPIRXBUF;				
		       if(rcv_fmt != xmit_fmt)
		       {
//			       error();
		       }

#endif
// #ifdef USE_NORM
#if 0
		       EALLOW;
//		       GpioDataRegs.GPBTOGGLE.bit.GPIO51;

		                     GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
		                     GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
		                     GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
		                     GpioDataRegs.GPBDAT.bit.GPIO51 = DISABLE;
//		                    GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
		      //             GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

		      //             GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
// #endif
		                     EDIS;

		       xmit_fmt = (sdataL & 0x0000ffff);
		       
		       spi_xmit(xmit_fmt);
// #else
		       xmit_fmt = ((sdataL & 0xffff0000) >> 16);
		       spi_xmit(xmit_fmt);
#endif		       
		       

#if 0		       
//
	// Wait until data is received
	//
		       while(SpiaRegs.SPIFFRX.bit.RXFFST !=1)
		       {

		       }

	//        
	// Check against sent data
	//
		       rcv_fmt = SpiaRegs.SPIRXBUF;				
		       if(rcv_fmt != xmit_fmt)
		       {
//			       error();
		       }		       
#endif

#if 0
		       sdataL--;
		       rdataL--;
		       qdataL--;
#endif

		       if ( sdataL < 0x0000ffff )
		       {
		       sdataL++;
		       qdataL++;
		       }

		       if ( rdataL < 0x00ffffff )
		           rdataL++;



	       EALLOW;      
               GpioCtrlRegs.GPAMUX1.bit.GPIO4 = DISABLE;              //
               GpioDataRegs.GPASET.bit.GPIO4 = ENABLE;
               GpioCtrlRegs.GPADIR.bit.GPIO4 = ENABLE;
	       GpioDataRegs.GPADAT.bit.GPIO4 = ENABLE;
	       EDIS;
	       DSP28x_usDelay(1);
#if 1
		       if (sdataL == 0x0000ffff)
		       {
		   EALLOW;
                   GpioCtrlRegs.GPAMUX1.bit.GPIO4 = DISABLE;              //
                   GpioDataRegs.GPASET.bit.GPIO4 = ENABLE;
                   GpioCtrlRegs.GPADIR.bit.GPIO4 = ENABLE;
                   GpioDataRegs.GPADAT.bit.GPIO4 = DISABLE;
                 //  GpioCtrlRegs.GPAPUD.bit.GPIO4 = ENABLE;                // this means NO pullup is enabled
                // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
		   EDIS;

		           sdataL = 0x0;
		           qdataL = 0x0;
		           rdataL = 0x0;


		       }
#endif
#if 0
		       EALLOW;
//		       GpioDataRegs.GPBTOGGLE.bit.GPIO51;

		       GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
		       GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
		       GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
		       GpioDataRegs.GPBDAT.bit.GPIO51 = ENABLE;
//		       GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
//		       GpioDataRegs.GPBDAT.bit.GPIO51 = ENABLE;
//		       GpioDataRegs.GPASET.bit.GPIO17 = ENABLE;
// #endif
		       EDIS;

#endif
//		       UARTCharPutNonBlocking(UART1_BASE, "?");
//		       UARTwrite("ATDT?", 5);
#endif
		       delay_loop();
#if 0
#ifdef ENABLE_HRCAP_IRQ2
               EALLOW;
                         PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 8
                         EDIS;
#endif
#ifdef ENABLE_HRCAP_IRQ1
                         EALLOW;
                         PieCtrlRegs.PIEIER4.bit.INTx7=1;     // Enable PIE Group 4, INT 7
                         EDIS;
#endif
#endif
#if 0
#ifdef ENABLE_HRCAP_IRQ4
                         EALLOW;
                         PieCtrlRegs.PIEIER5.bit.INTx5=1;     // Enable PIE Group 4, INT 8
                         EDIS;
#endif
#ifdef ENABLE_HRCAP_IRQ3
                         EALLOW;
                         PieCtrlRegs.PIEIER5.bit.INTx4=1;     // Enable PIE Group 4, INT 8
                         EDIS;
#endif
#endif



    }
}

#endif // NO_MAIN

//
// InitEPwmTimer -
//
#if 0
void
InitEPwmTimer()
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;

    //
    // Disable Sync
    //
    EPwm7Regs.TBCTL.bit.SYNCOSEL = 11;  // Pass through

    //
    // Initally disable Free/Soft Bits
    //
    EPwm7Regs.TBCTL.bit.FREE_SOFT = 0;

    EPwm7Regs.TBPRD = PWM7_TIMER_TBPRD;          // Set up PWM1 Period
    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;   // Count up mode
    EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm7Regs.ETSEL.bit.INTEN = PWM7_INT_ENABLE; // Enable INT
    EPwm7Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 1st event
    EPwm7Regs.TBCTR = 0x0000;                    // Clear timer counter

    //
    // CompareA event at half of period
    //
    EPwm7Regs.CMPA.half.CMPA = PWM7_TIMER_TBPRD/2;

    //
    // Action-qualifiers, Set on CMPA, Clear on PRD
    //
    EPwm7Regs.AQCTLA.all = 0x0024;

    EALLOW;
    SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 1;    // ePWM7
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;       // Start all the timers synced
    EDIS;
}
#endif

//
// InitEPwm1Example -
//
#if 0
void
InitEPwmTimer()
{
    //
    // Setup TBCLK
    //

    EPwm7Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;     // set Immediate load
    EPwm7Regs.TBPRD = EPWM7_TIMER_TBPRD - 1;                   // PWM frequency = 1 / period
    EPwm7Regs.CMPA.half.CMPA = EPWM7_TIMER_TBPRD / 2;        // set duty 50% initially
    EPwm7Regs.CMPA.half.CMPAHR = (1 << 8);        // initialize HRPWM extension
    EPwm7Regs.CMPB = EPWM7_TIMER_TBPRD / 2;                  // set duty 50% initially
    EPwm7Regs.TBPHS.all = 0;
    EPwm7Regs.TBCTR = 0;


    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm7Regs.TBPRD = EPWM7_TIMER_TBPRD;       // Set timer period
    EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm7Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm7Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
    EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV2;

    //
    // Setup shadow register load on ZERO
    //
    EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
    EPwm7Regs.CMPA.half.CMPA = EPWM7_MIN_CMPA;    // Set compare A value
    EPwm7Regs.CMPB = EPWM7_MIN_CMPB;              // Set Compare B value

    //
    // Set actions
    //
    EPwm7Regs.AQCTLA.bit.ZRO = AQ_CLEAR;      // Set PWM1A on Zero
    EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;    // Clear PWM1A on event A, up count

    EPwm7Regs.AQCTLB.bit.ZRO = AQ_CLEAR;      // Set PWM1B on Zero
    EPwm7Regs.AQCTLB.bit.CBU = AQ_SET;    // Clear PWM1B on event B, up count

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm7Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm7Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //

    //
    // Start by increasing CMPA & CMPB
    //
    epwm7_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
    epwm7_info.EPwm_CMPB_Direction = EPWM_CMP_UP;

    epwm7_info.EPwmTimerIntCount = 0;         // Zero the interrupt counter
    epwm7_info.EPwmRegHandle = &EPwm7Regs;// Set the pointer to the ePWM module
    epwm7_info.EPwmMaxCMPA = EPWM7_MAX_CMPA;  // Setup min/max CMPA/CMPB values
    epwm7_info.EPwmMinCMPA = EPWM7_MIN_CMPA;
    epwm7_info.EPwmMaxCMPB = EPWM7_MAX_CMPB;
    epwm7_info.EPwmMinCMPB = EPWM7_MIN_CMPB;
}

//
// epwm1_timer_isr - Interrupt routines uses in this example
//
__interrupt void
epwm7_timer_isr(void)
{
    EPwm7TimerIntCount++;
//    LEDcount++;

    //
    // Clear INT flag for this timer
    //
    EPwm7Regs.ETCLR.bit.INT = 1;
#if 0
    if (LEDcount==500)
    {
        //
        // turn on/off LED3 on the controlCARD
        //
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
        LEDcount=0;
    }
#endif
    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// error -
//
void
   error(void)
{
	__asm("     ESTOP0");  //Test failed!! Stop!
	for (;;);
}
#endif
#ifdef USE_SPI_IRQ
//
// spi_fifo_init - Initialize SPI FIFO registers
//
void spi_fifo_init()
{
	SpiaRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI

	SpiaRegs.SPICCR.all=0x001F;      // 8-bit character, Loopback mode
	SpiaRegs.SPICTL.all=0x0017; // Interrupt enabled, Master/Slave XMIT enabled
	SpiaRegs.SPISTS.all=0x0000;
	SpiaRegs.SPIBRR=0x0063;           // Baud rate
	SpiaRegs.SPIFFTX.all=0xC022;      // Enable FIFO's, set TX FIFO level to 4
	SpiaRegs.SPIFFRX.all=0x0022;      // Set RX FIFO level to 4
	SpiaRegs.SPIFFCT.all=0x00;
	SpiaRegs.SPIPRI.all=0x0010;

	SpiaRegs.SPICCR.bit.SPISWRESET=1;  // Enable SPI

	SpiaRegs.SPIFFTX.bit.TXFIFO=1;
	SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;
}
#else
//
// spi_fifo_init - 
//
void spi_fifo_init(void)										
{
    //
    // Initialize SPI FIFO registers
    //

	SpiaRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI
	
	SpiaRegs.SPICCR.all=0x0007;      // 8-bit character, CLOCK POLARITY = 0
	
	SpiaRegs.SPICTL.all=0x0006; // Interrupt DISabled, Master/Slave XMIT DISabled
	SpiaRegs.SPISTS.all=0x0000;
	SpiaRegs.SPIBRR=0x0063;           // Baud rate
	SpiaRegs.SPIFFTX.all=0xE040;
	SpiaRegs.SPIFFRX.all=0x2044;
	SpiaRegs.SPIFFCT.all=0x0;
	SpiaRegs.SPICCR.bit.SPISWRESET=1;  // Enable SPI
}  
#endif
//
// spi_init -
//
void spi_init()
{
//    SpiaRegs.SPICCR.all =0x000F;  // Reset on, rising edge, 16-bit char bits

    //
    // Enable master mode, normal phase, enable talk, and SPI int disabled.
    //
//    SpiaRegs.SPICTL.all =0x0006;

//    SpiaRegs.SPIBRR =0x007F;
//    SpiaRegs.SPICCR.all =0x009F;   // Relinquish SPI from Reset
    SpiaRegs.SPIPRI.bit.FREE = 1;  // Set so breakpoints don't disturb xmission
    SpiaRegs.SPIPRI.bit.TRIWIRE = 1; // three wire SPI mode
}
//
// spiTxFifoIsr - 
//
#if 0
__interrupt void
   spiTxFifoIsr(void)
{
	Uint16 i;
	for(i=0;i<2;i++)
	{
		SpiaRegs.SPITXBUF=sdata[i];      // Send data
	}

	for(i=0;i<2;i++)                    // Increment data for next cycle
	{
		sdata[i] = sdata[i] + 1;
	}

	SpiaRegs.SPIFFTX.bit.TXFFINTCLR=1;  // Clear Interrupt flag
	PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ACK
}

//
// spiRxFifoIsr - 
//
__interrupt void
   spiRxFifoIsr(void)
{
	Uint16 i;
	for(i=0;i<2;i++)
	{
		rdata[i]=SpiaRegs.SPIRXBUF;     // Read data
	}
	for(i=0;i<2;i++)                    // Check received data
	{
		if(rdata[i] != rdata_point+i) error();
	}
	rdata_point++;
	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
	SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
	PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
}
#endif
//
// spi_xmit - for AD5060
//


void spi_xmit(Uint32 a, Uint8 dac_sel)
{

	static Uint16 xmit_val = 0;

//	return;

	a &= 0x00ffffff;		// 24-bits are valid


	switch (dac_sel)
	{

	case 'h':
	    portENTER_CRITICAL();
		       EALLOW;
//		       GpioDataRegs.GPBTOGGLE.bit.GPIO51;
#if 1
		                     GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
		                     GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
		                     GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
		                     GpioDataRegs.GPBDAT.bit.GPIO51 = DISABLE;
//		                    GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
		      //             GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

		      //             GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
#endif
				     EDIS;

			    DSP28x_usDelay(2);
#if 1
		       xmit_val = (a >> 16) & 0x0003;                       // 24-bits xmit_val = (a >> 16) & 0x00ff;
		       xmit_val <<= 8;
		       xmit_val |= (a & 0x0000ffff);
		       SpiaRegs.SPITXBUF = xmit_val;
		       xmit_val = a & 0x0000ff00;
		       SpiaRegs.SPITXBUF = xmit_val;
		       xmit_val = a & 0x000000ff;
		       xmit_val <<= 8;
		       xmit_val |= (a & 0x0000ffff);
		       SpiaRegs.SPITXBUF = xmit_val;
#else
//		       xmit_val = (a >> 16) & 0x00ff;                       // 24-bits
		       xmit_val = (a & 0x00ff);                       // 24-bits
		       xmit_val <<= 8;
//		       xmit_val |= (a & 0x0000ff00);
		       SpiaRegs.SPITXBUF = xmit_val;
		       xmit_val = a & 0xff00;
//		       xmit_val >>= 8;
		       SpiaRegs.SPITXBUF = xmit_val;
		       xmit_val = a & 0xff0000;
		       xmit_val >>= 8;
		       SpiaRegs.SPITXBUF = xmit_val;

#endif
		       


//		       DELAY_US(20);
////		       DSP28x_usDelay(360);
		       DSP28x_usDelay(2250);
		       EALLOW;
//		       GpioDataRegs.GPBTOGGLE.bit.GPIO51;

#if 1
		       GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
		       GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
		       GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
		       GpioDataRegs.GPBDAT.bit.GPIO51 = ENABLE;
//		                    GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
		      //             GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

		      //             GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
#endif
		       EDIS;
		       portEXIT_CRITICAL();
		       break;

	case 'l':
	           portENTER_CRITICAL();
		       EALLOW;
//		       GpioDataRegs.GPBTOGGLE.bit.GPIO51;
#if 1
		                     GpioCtrlRegs.GPAMUX2.bit.GPIO17 = DISABLE;              // AD SPI CSL
		                     GpioDataRegs.GPASET.bit.GPIO17 = ENABLE;
		                     GpioCtrlRegs.GPADIR.bit.GPIO17 = ENABLE;
		                     GpioDataRegs.GPADAT.bit.GPIO17 = DISABLE;
//		                    GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
		      //             GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

		      //             GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
#endif
				     EDIS;

			    DSP28x_usDelay(2);
#if 1
	               xmit_val = (a >> 16) & 0x0003;                       // 24-bits xmit_val = (a >> 16) & 0x00ff;
	               xmit_val <<= 8;
	               xmit_val |= (a & 0x0000ffff);
	               SpiaRegs.SPITXBUF = xmit_val;
	               xmit_val = a & 0x0000ff00;
	               SpiaRegs.SPITXBUF = xmit_val;
	               xmit_val = a & 0x000000ff;
	               xmit_val <<= 8;
	               xmit_val |= (a & 0x0000ffff);
	               SpiaRegs.SPITXBUF = xmit_val;
#if 0

		       xmit_val = (a >> 16) & 0x0003;
		       xmit_val <<= 8;
		       xmit_val |= (a & 0x0000ffff);
		       SpiaRegs.SPITXBUF = xmit_val;
		       xmit_val = a & 0xff00;
		       SpiaRegs.SPITXBUF = xmit_val;
		       xmit_val = a & 0x00ff;
		       xmit_val <<= 8;
		       SpiaRegs.SPITXBUF = xmit_val;
#endif
#else
		       //             xmit_val = (a >> 16) & 0x00ff;                       // 24-bits
		                      xmit_val = (a & 0x00ff);                       // 24-bits
		                      xmit_val <<= 8;
		       //             xmit_val |= (a & 0x0000ff00);
		                      SpiaRegs.SPITXBUF = xmit_val;
		                      xmit_val = a & 0xff00;
		       //             xmit_val >>= 8;
		                      SpiaRegs.SPITXBUF = xmit_val;
		                      xmit_val = a & 0xff0000;
		                      xmit_val >>= 8;
		                      SpiaRegs.SPITXBUF = xmit_val;
#if 0
               xmit_val = (a >> 16) & 0x00ff;                       // 24-bits
//             xmit_val <<= 8;
//             xmit_val |= (a & 0x0000ff00);
               SpiaRegs.SPITXBUF = xmit_val;
               xmit_val = a & 0xff00;
               xmit_val >>= 8;
               SpiaRegs.SPITXBUF = xmit_val;
               xmit_val = a & 0x00ff;
//             xmit_val <<= 8;
               SpiaRegs.SPITXBUF = xmit_val;
#endif
#endif
		       
//		       DELAY_US(20);
////		       DSP28x_usDelay(360);
		       DSP28x_usDelay(2250);
		       EALLOW;
//		       GpioDataRegs.GPBTOGGLE.bit.GPIO51;

#if 1
		       GpioCtrlRegs.GPAMUX2.bit.GPIO17 = DISABLE;              // AD SPI CSH
		       GpioDataRegs.GPASET.bit.GPIO17 = ENABLE;
		       GpioCtrlRegs.GPADIR.bit.GPIO17 = ENABLE;
		       GpioDataRegs.GPADAT.bit.GPIO17 = ENABLE;
//		                    GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
		      //             GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

		      //             GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
#endif

		       EDIS;
        portEXIT_CRITICAL();
		break;

// set the PGA level

    case 'p':
               portENTER_CRITICAL();
               EALLOW;
//             GpioDataRegs.GPBTOGGLE.bit.GPIO51;
#if 1
                             GpioCtrlRegs.GPBMUX2.bit.GPIO50 = DISABLE;              // PGA SPI CS
                             GpioDataRegs.GPBSET.bit.GPIO50 = ENABLE;
                             GpioCtrlRegs.GPBDIR.bit.GPIO50 = ENABLE;
                             GpioDataRegs.GPBDAT.bit.GPIO50 = DISABLE;
//                          GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
              //             GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

              //             GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
#endif
                     EDIS;

               DSP28x_usDelay(1);
#if 0
               xmit_val = (a >> 24) & 0x00ff;
               xmit_val <<= 8;
               xmit_val |= (a & 0x0000ff00);
               SpiaRegs.SPITXBUF = xmit_val;
               xmit_val = (a >> 16) & 0x00ff;
               xmit_val <<= 8;
               xmit_val |= (a & 0x0000ff00);
               SpiaRegs.SPITXBUF = xmit_val;
               xmit_val = a & 0xff00;
               SpiaRegs.SPITXBUF = xmit_val;
               xmit_val = a & 0x00ff;
               xmit_val <<= 8;
               SpiaRegs.SPITXBUF = xmit_val;
#endif
#if 1
               xmit_val = (a >> 16) & 0xffff;
               xmit_val <<= 8;
               xmit_val |= (a & 0x0000ffff);                    // ffff
               SpiaRegs.SPITXBUF = xmit_val;                    // just 8 bits for the PGA
#endif
#if 0
               xmit_val = a & 0xff00;
               SpiaRegs.SPITXBUF = xmit_val;
               xmit_val = a & 0x00ff;
               xmit_val <<= 8;
               SpiaRegs.SPITXBUF = xmit_val;

//             DELAY_US(20);
#endif
////               DSP28x_usDelay(360);
               DSP28x_usDelay(1750);                        // 2050
               EALLOW;
//             GpioDataRegs.GPBTOGGLE.bit.GPIO51;

#if 1
               GpioCtrlRegs.GPBMUX2.bit.GPIO50 = DISABLE;              // PGA SPI CS
               GpioDataRegs.GPBSET.bit.GPIO50 = ENABLE;
               GpioCtrlRegs.GPBDIR.bit.GPIO50 = ENABLE;
               GpioDataRegs.GPBDAT.bit.GPIO50 = ENABLE;
//                          GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
              //             GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

              //             GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
#endif
               EDIS;
               portEXIT_CRITICAL();
               break;




	default:			// err_ctr
		break;
	}
		       
}

#if 0

#ifdef USE_HRCAP1
//
// HRCAP1_Config -
//
void HRCAP1_Config(void)
{
	EALLOW;

	HRCap1Regs.HCCTL.bit.SOFTRESET = 1;
	HRCap1Regs.HCCTL.bit.HCCAPCLKSEL = 1;  // HCCAPCLK = PLL2CLK = SYSCLK2 x 2

#if RISETEST
	//
	// Enable Rising Edge Capture Event Interrupt
	//
	HRCap1Regs.HCCTL.bit.RISEINTE = 1;     

	//
	// Disable Falling Edge Capture Event Interrupt
	//
	HRCap1Regs.HCCTL.bit.FALLINTE = 0;
#elif FALLTEST
	//
	// Enable Falling Edge Capture Event Interrupt
	//
	HRCap1Regs.HCCTL.bit.FALLINTE = 1;

	//
	// Disable Rising Edge Capture Event Interrupt
	//
	HRCap1Regs.HCCTL.bit.RISEINTE = 0;
#endif

    //
    // Enable Interrupt on 16-bit Counter Overflow Event
    //
	HRCap1Regs.HCCTL.bit.OVFINTE = 0;

	EDIS;
}

//
// HRCAP1_Isr - 
//
// extern Uint32 HighPulseWidth0 (Uint16 *ptrHRCAPmodule);
// extern Uint32 HighPulseWidth1 (Uint16 *ptrHRCAPmodule);
// extern Uint32 LowPulseWidth0 (Uint16 *ptrHRCAPmodule);
// extern Uint32 PeriodWidthRise0 (Uint16 *ptrHRCAPmodule);
// extern Uint32 PeriodWidthFall0 (Uint16 *ptrHRCAPmodule);
// extern Uint16 HRCAP_Cal(Uint16 HRCapModule, Uint16 PLLClk, volatile struct EPWM_REGS *ePWMModule);       // HRCAP Calibration Function

__interrupt void HRCAP1_Isr(void)
{
	EALLOW;
	
	if (HRCap1Regs.HCIFR.bit.RISEOVF == 1) 
	{
//	    periodwidth0[datacounter1] = PeriodWidthRise0((Uint16 *)&HRCap1Regs);
//        HRCap1Regs.HCICLR.bit.RISEOVF=1;
//	    HRCap1Regs.HCICLR.bit.RISE=1;
	    EDIS;
//		ESTOP0;         // Another rising edge detected before ISR serviced
		EALLOW;
	}

	if (first1 < 1)
	{
	//
	// Discard first data (because first interrupt after reset/clk enable
	// measures time from clock start to edge instead of valid pulse width)
	//
		first1++;
	} 
	else
	{
#if 1
		if (datacounter1 == 5)
		{
			datacounter1 = 0;
			do_cal_1 = 1;
		}
#endif
#if FALLTEST
#if PERIODTEST
		periodwidth0[datacounter1] = PeriodWidthFall0((Uint16 *)&HRCap1Regs);
#else
		pulsewidthlow0[datacounter1] = LowPulseWidth0((Uint16 *)&HRCap1Regs);                  
		pulsewidthhigh0[datacounter1] = HighPulseWidth0((Uint16 *)&HRCap1Regs);  
#endif
#elif RISETEST	   
#if PERIODTEST
		periodwidth0[datacounter1] = PeriodWidthRise0((Uint16 *)&HRCap1Regs);
//		periodwidth0[datacounter1] = PeriodWidthFall0((Uint16 *)&HRCap1Regs);
#else	        	    
		pulsewidthlow0[datacounter1] = LowPulseWidth0((Uint16 *)&HRCap1Regs);
		int_pulsewidthlow0[datacounter1] = _IQ16int(pulsewidthlow0[datacounter1]);
		frac_pulsewidthlow0[datacounter1] = _IQ16frac(pulsewidthlow0[datacounter1]);
		pulsewidthhigh0[datacounter1] = HighPulseWidth0((Uint16 *)&HRCap1Regs);
		int_pulsewidthhigh0[datacounter1] = _IQ16int(pulsewidthhigh0[datacounter1]);
		frac_pulsewidthhigh0[datacounter1] = _IQ16frac(pulsewidthhigh0[datacounter1]);
#endif
#endif

//		counter1+=8;

	//
	// Increment CMPAHR by 8 for next period.        
	//
///		EPwm1Regs.CMPA.half.CMPAHR = counter<<8; 
		datacounter1++;
	}

	HRCap1Regs.HCICLR.bit.FALL=1;  // Clear FALL flag
	HRCap1Regs.HCICLR.bit.RISE=1;  // Clear RISE flag

	HRCap1Regs.HCICLR.bit.INT=1;   // Clear HRCAP interrupt flag
	PieCtrlRegs.PIEACK.bit.ACK4=1; // Acknowledge PIE Group 4 interrupts.
	   //
	    // Acknowledge this interrupt to receive more interrupts from group 3
	    //
//	    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;



#if 1
#ifdef ENABLE_HRCAP_IRQ1
                         PieCtrlRegs.PIEIER4.bit.INTx7=0;     // Enable PIE Group 4, INT 7
                         __asm("   NOP");
                         PieCtrlRegs.PIEIER4.bit.INTx8=1;
#endif
#if 1
#ifdef ENABLE_HRCAP_IRQ4
                         __asm("   NOP");
                         PieCtrlRegs.PIEIER5.bit.INTx5=0;     // Enable PIE Group 4, INT 8
#endif
#endif
#endif

	EDIS;
}
#endif

#ifdef USE_HRCAP2
//
// HRCAP2_Config -
//
void HRCAP2_Config(void)
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
__interrupt void HRCAP2_Isr(void)
{
	EALLOW;
	if (HRCap2Regs.HCIFR.bit.RISEOVF == 1) 
	{
	    EDIS;
//		ESTOP0;         // Another rising edge detected before ISR serviced
//	    HRCap2Regs.HCICLR.bit.RISE=1;  // Clear RISE flag
		EALLOW;
	}

	if (first2 < 1)
	{
	//
	// Discard first data (because first interrupt after reset/clk enable
	// measures time from clock start to edge instead of valid pulse width)
	//
		first2++;
	} 
	else
	{
		if (datacounter2 == 5)
		{
			datacounter2 = 0;
		}
#if FALLTEST
#if PERIODTEST
		periodwidth1[datacounter2] = PeriodWidthRise0((Uint16 *)&HRCap2Regs);  
#else
		pulsewidthlow1[datacounter2] = LowPulseWidth0((Uint16 *)&HRCap2Regs);                  
		pulsewidthhigh1[datacounter2] = HighPulseWidth0((Uint16 *)&HRCap2Regs);  
#endif
#elif RISETEST	   
#if PERIODTEST
//		periodwidth1[datacounter2] = PeriodWidthRise1((Uint16 *)&HRCap2Regs);
		periodwidth1[datacounter2] = PeriodWidthRise0((Uint16 *)&HRCap2Regs);
#else	        	    
		pulsewidthlow1[datacounter2] = LowPulseWidth0((Uint16 *)&HRCap2Regs);
		int_pulsewidthlow1[datacounter2] = _IQ16int(pulsewidthlow1[datacounter2]);
		frac_pulsewidthlow1[datacounter2] = _IQ16frac(pulsewidthlow1[datacounter2]);

		pulsewidthhigh1[datacounter2] = HighPulseWidth0((Uint16 *)&HRCap2Regs);
		int_pulsewidthhigh1[datacounter2] = _IQ16int(pulsewidthhigh1[datacounter2]);
		frac_pulsewidthhigh1[datacounter2] = _IQ16frac(pulsewidthhigh1[datacounter2]);

#endif
#endif

//		counter2+=8;

	//
	// Increment CMPAHR by 8 for next period.        
	//
///		EPwm1Regs.CMPA.half.CMPAHR = counter<<8; 
		datacounter2++;
	}

	HRCap2Regs.HCICLR.bit.FALL=1;  // Clear FALL flag
	HRCap2Regs.HCICLR.bit.RISE=1;  // Clear RISE flag

	HRCap2Regs.HCICLR.bit.INT=1;   // Clear HRCAP interrupt flag
	PieCtrlRegs.PIEACK.bit.ACK4=1; // Acknowledge PIE Group 4 interrupts.

    PieCtrlRegs.PIEIER4.bit.INTx8=0;
    PieCtrlRegs.PIEIER4.bit.INTx7=0;
    __asm("   NOP");
    PieCtrlRegs.PIEIER5.bit.INTx4=1;
	EDIS;
}
#endif

#ifdef USE_HRCAP3
//
// HRCAP3_Config -
//
void HRCAP3_Config(void)
{
	EALLOW;

	HRCap3Regs.HCCTL.bit.SOFTRESET = 1;
	HRCap3Regs.HCCTL.bit.HCCAPCLKSEL = 1;  // HCCAPCLK = PLL2CLK = SYSCLK2 x 2

#if RISETEST
	//
	// Enable Rising Edge Capture Event Interrupt
	//
	HRCap3Regs.HCCTL.bit.RISEINTE = 1;     

	//
	// Disable Falling Edge Capture Event Interrupt
	//
	HRCap3Regs.HCCTL.bit.FALLINTE = 0;
#elif FALLTEST
	//
	// Enable Falling Edge Capture Event Interrupt
	//
	HRCap3Regs.HCCTL.bit.FALLINTE = 1;

	//
	// Disable Rising Edge Capture Event Interrupt
	//
	HRCap3Regs.HCCTL.bit.RISEINTE = 0;
#endif

    //
    // Enable Interrupt on 16-bit Counter Overflow Event
    //
	HRCap3Regs.HCCTL.bit.OVFINTE = 0;

	EDIS;
}

//
// HRCAP3_Isr - 
//
__interrupt void HRCAP3_Isr(void)
{
	EALLOW;
	if (HRCap3Regs.HCIFR.bit.RISEOVF == 1) 
	{
		EDIS;
	    ESTOP0;         // Another rising edge detected before ISR serviced
        EALLOW;
	}

	if (first3 < 1)
	{
	//
	// Discard first data (because first interrupt after reset/clk enable
	// measures time from clock start to edge instead of valid pulse width)
	//
		first3++;
	} 
	else
	{
		if (datacounter3 == 5)
		{
			datacounter3 = 0;
		}
#if FALLTEST
#if PERIODTEST
		periodwidth2[datacounter3] = PeriodWidthRise0((Uint16 *)&HRCap3Regs);  
#else
		pulsewidthlow2[datacounter3] = LowPulseWidth0((Uint16 *)&HRCap3Regs);                  
		pulsewidthhigh2[datacounter3] = HighPulseWidth0((Uint16 *)&HRCap3Regs);  
#endif
#elif RISETEST	   
#if PERIODTEST
		periodwidth2[datacounter3] = PeriodWidthRise0((Uint16 *)&HRCap3Regs);  
#else	        	    
		pulsewidthlow2[datacounter3] = LowPulseWidth0((Uint16 *)&HRCap3Regs);
        int_pulsewidthlow2[datacounter3] = _IQ16int(pulsewidthlow2[datacounter3]);
        frac_pulsewidthlow2[datacounter3] = _IQ16frac(pulsewidthlow2[datacounter3]);

		pulsewidthhigh2[datacounter3] = HighPulseWidth0((Uint16 *)&HRCap3Regs);
	    int_pulsewidthhigh2[datacounter3] = _IQ16int(pulsewidthhigh2[datacounter3]);
	    frac_pulsewidthhigh2[datacounter3] = _IQ16frac(pulsewidthhigh2[datacounter3]);
#endif
#endif

		counter3+=8;

	//
	// Increment CMPAHR by 8 for next period.        
	//
///		EPwm1Regs.CMPA.half.CMPAHR = counter<<8; 
		datacounter3++;
	}

	HRCap3Regs.HCICLR.bit.FALL=1;  // Clear FALL flag
	HRCap3Regs.HCICLR.bit.RISE=1;  // Clear RISE flag

	HRCap3Regs.HCICLR.bit.INT=1;   // Clear HRCAP interrupt flag
	PieCtrlRegs.PIEACK.bit.ACK5=1; // Acknowledge PIE Group 5 interrupts.

	PieCtrlRegs.PIEIER5.bit.INTx4=0;     // Enable PIE Group 4, INT 8
	__asm("   NOP");
	PieCtrlRegs.PIEIER5.bit.INTx5=1;

	EDIS;
}
#endif

#ifdef USE_HRCAP4
//
// HRCAP4_Config -
//
void HRCAP4_Config(void)
{
	EALLOW;

	HRCap4Regs.HCCTL.bit.SOFTRESET = 1;
	HRCap4Regs.HCCTL.bit.HCCAPCLKSEL = 1;  // HCCAPCLK = PLL2CLK = SYSCLK2 x 2

#if RISETEST
	//
	// Enable Rising Edge Capture Event Interrupt
	//
	HRCap4Regs.HCCTL.bit.RISEINTE = 1;     

	//
	// Disable Falling Edge Capture Event Interrupt
	//
	HRCap4Regs.HCCTL.bit.FALLINTE = 0;
#elif FALLTEST
	//
	// Enable Falling Edge Capture Event Interrupt
	//
	HRCap4Regs.HCCTL.bit.FALLINTE = 1;

	//
	// Disable Rising Edge Capture Event Interrupt
	//
	HRCap4Regs.HCCTL.bit.RISEINTE = 0;
#endif

    //
    // Enable Interrupt on 16-bit Counter Overflow Event
    //
	HRCap4Regs.HCCTL.bit.OVFINTE = 0;
	EDIS;
}
//
// HRCAP4_Isr - 
//
__interrupt void HRCAP4_Isr(void)
{
	EALLOW;
	if (HRCap4Regs.HCIFR.bit.RISEOVF == 1) 
	{
	    periodwidth3[datacounter4] = PeriodWidthRise0((Uint16 *)&HRCap4Regs);
//	    EDIS;
	    ESTOP0;         // Another rising edge detected before ISR serviced
	}

	if (first4 < 1)
	{
	//
	// Discard first data (because first interrupt after reset/clk enable
	// measures time from clock start to edge instead of valid pulse width)
	//
		first4++;
	} 
	else
	{
		if (datacounter4 == 5)
		{
			datacounter4 = 0;
		}
#if FALLTEST
#if PERIODTEST
		periodwidth3[datacounter4] = PeriodWidthRise0((Uint16 *)&HRCap4Regs);  
#else
		pulsewidthlow3[datacounter4] = LowPulseWidth0((Uint16 *)&HRCap4Regs);                  
		pulsewidthhigh3[datacounter4] = HighPulseWidth0((Uint16 *)&HRCap4Regs);  
#endif
#elif RISETEST	   
#if PERIODTEST
		periodwidth3[datacounter4] = PeriodWidthRise0((Uint16 *)&HRCap4Regs);  
#else	        	    
		pulsewidthlow3[datacounter4] = LowPulseWidth0((Uint16 *)&HRCap4Regs);
		int_pulsewidthlow3[datacounter4] = _IQ16int(pulsewidthlow3[datacounter4]);
		frac_pulsewidthlow3[datacounter4] = _IQ16frac(pulsewidthlow3[datacounter4]);

		pulsewidthhigh3[datacounter4] = HighPulseWidth0((Uint16 *)&HRCap4Regs);
		int_pulsewidthhigh3[datacounter4] = _IQ16int(pulsewidthhigh3[datacounter4]);
		frac_pulsewidthhigh3[datacounter4] = _IQ16frac(pulsewidthhigh3[datacounter4]);

//        pulsewidthlow0[datacounter1] = LowPulseWidth0((Uint16 *)&HRCap1Regs);
//        pulsewidthhigh0[datacounter1] = HighPulseWidth0((Uint16 *)&HRCap1Regs);



#endif
#endif

//		counter4+=8;

	//
	// Increment CMPAHR by 8 for next period.        
	//
///		EPwm1Regs.CMPA.half.CMPAHR = counter<<8; 
		datacounter4++;
	}

	HRCap4Regs.HCICLR.bit.FALL=1;  // Clear FALL flag
	HRCap4Regs.HCICLR.bit.RISE=1;  // Clear RISE flag

	HRCap4Regs.HCICLR.bit.INT=1;   // Clear HRCAP interrupt flag
	PieCtrlRegs.PIEACK.bit.ACK5=1; // Acknowledge PIE Group 4 interrupts.
	   //
	    // Acknowledge this interrupt to receive more interrupts from group 3
	    //
///	    PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
#if 1
#ifdef ENABLE_HRCAP_IRQ1
                         PieCtrlRegs.PIEIER4.bit.INTx7=0;     // Enable PIE Group 4, INT 7
#endif
#ifdef ENABLE_HRCAP_IRQ4
                         PieCtrlRegs.PIEIER5.bit.INTx5=0;     // Enable PIE Group 4, INT 8

                         __asm("   NOP");                       // for debug breakpoint

                         PieCtrlRegs.PIEIER4.bit.INTx7=1;
#endif
#endif
	EDIS;
}
#endif

//
// adc_isr -
//
__interrupt void adc_isr(void)
{
#if 0
    Voltage1[ConversionCount] = AdcResult.ADCRESULT0;
    Voltage2[ConversionCount] = AdcResult.ADCRESULT1;

    //
    // If 20 conversions have been logged, start over
    //
    if(ConversionCount == 9)
    {
        ConversionCount = 0;
    }
    else
    {
        ConversionCount++;
    }
#endif
    //
    // Clear ADCINT1 flag reinitialize for next SOC
    //
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

///    return;
}
void InitAdcLocal(void)
{
    extern void DSP28x_usDelay(Uint32 Count);

    // *IMPORTANT*
    // The Device_cal function, which copies the ADC calibration values from TI reserved
    // OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
    // Boot ROM. If the boot ROM code is bypassed during the debug process, the
    // following function MUST be called for the ADC to function according
    // to specification. The clocks to the ADC MUST be enabled before calling this
    // function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.

        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
        (*Device_cal)();
        EDIS;

    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
    // after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
    // CPU_RATE define statement in the F2806x_Examples.h file must
    // contain the correct CPU clock period in nanoseconds.
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;      // Power ADC BG 1
    AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      // Power reference 1
    AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;      // Power ADC 1
    AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC 1
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      // Select internal BG 0
    EDIS;

    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels

    EALLOW;
    AdcRegs.ADCCTL2.bit.CLKDIV2EN = 1;
    EDIS;

    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels
}

//
// InitECapture -
//
void
InitECapture()
{
    ECap1Regs.ECEINT.all = 0x0000;          // Disable all capture interrupts
    ECap1Regs.ECCLR.all = 0xFFFF;           // Clear all CAP interrupt flags
    ECap1Regs.ECCTL1.bit.CAPLDEN = 0;       // Disable CAP1-CAP4 register loads
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Make sure the counter is stopped

    //
    // Configure peripheral registers
    //
    ECap1Regs.ECCTL2.bit.CONT_ONESHT = 1;   // One-shot

    ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;     // Stop at 4 events
    ECap1Regs.ECCTL1.bit.CAP1POL = 1;       // Falling edge
    ECap1Regs.ECCTL1.bit.CAP2POL = 0;       // Rising edge
    ECap1Regs.ECCTL1.bit.CAP3POL = 1;       // Falling edge
    ECap1Regs.ECCTL1.bit.CAP4POL = 0;       // Rising edge
    ECap1Regs.ECCTL1.bit.CTRRST1 = 1;       // Difference operation
    ECap1Regs.ECCTL1.bit.CTRRST2 = 1;       // Difference operation
    ECap1Regs.ECCTL1.bit.CTRRST3 = 1;       // Difference operation
    ECap1Regs.ECCTL1.bit.CTRRST4 = 1;       // Difference operation
    ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;      // Enable sync in
    ECap1Regs.ECCTL2.bit.SYNCO_SEL = 0;     // Pass through
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable capture units

    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
    ECap1Regs.ECCTL2.bit.REARM = 1;         // arm one-shot
//    ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable CAP1-CAP4 register loads
    ECap1Regs.ECEINT.bit.CEVT4 = 1;         // 4 events = interrupt

    ECap2Regs.ECEINT.all = 0x0000;          // Disable all capture interrupts
    ECap2Regs.ECCLR.all = 0xFFFF;           // Clear all CAP interrupt flags
    ECap2Regs.ECCTL1.bit.CAPLDEN = 0;       // Disable CAP1-CAP4 register loads
    ECap2Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Make sure the counter is stopped

    //
    // Configure peripheral registers
    //
    ECap2Regs.ECCTL2.bit.CONT_ONESHT = 1;   // One-shot

    ECap2Regs.ECCTL2.bit.STOP_WRAP = 3;     // Stop at 4 events
    ECap2Regs.ECCTL1.bit.CAP1POL = 1;       // Falling edge
    ECap2Regs.ECCTL1.bit.CAP2POL = 0;       // Rising edge
    ECap2Regs.ECCTL1.bit.CAP3POL = 1;       // Falling edge
    ECap2Regs.ECCTL1.bit.CAP4POL = 0;       // Rising edge
    ECap2Regs.ECCTL1.bit.CTRRST1 = 1;       // Difference operation
    ECap2Regs.ECCTL1.bit.CTRRST2 = 1;       // Difference operation
    ECap2Regs.ECCTL1.bit.CTRRST3 = 1;       // Difference operation
    ECap2Regs.ECCTL1.bit.CTRRST4 = 1;       // Difference operation
    ECap2Regs.ECCTL2.bit.SYNCI_EN = 1;      // Enable sync in
    ECap2Regs.ECCTL2.bit.SYNCO_SEL = 0;     // Pass through
    ECap2Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable capture units

    ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
    ECap2Regs.ECCTL2.bit.REARM = 1;         // arm one-shot
    ECap2Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable CAP1-CAP4 register loads
    ECap2Regs.ECEINT.bit.CEVT4 = 1;         // 4 events = interrupt



}

//
// ecap1_isr -
//
__interrupt void
ecap1_isr(void)
{
    //
    // Cap input is syc'ed to SYSCLKOUT so there may be
    // a +/- 1 cycle variation
    //
    if(ECap1Regs.CAP2 > EPwm3Regs.TBPRD*2+1 ||
       ECap1Regs.CAP2 < EPwm3Regs.TBPRD*2-1)
    {
//        Fail();
    }

    if(ECap1Regs.CAP3 > EPwm3Regs.TBPRD*2+1 ||
       ECap1Regs.CAP3 < EPwm3Regs.TBPRD*2-1)
    {
//        Fail();
    }

    if(ECap1Regs.CAP4 > EPwm3Regs.TBPRD*2+1 ||
       ECap1Regs.CAP4 < EPwm3Regs.TBPRD*2-1)
    {
//        Fail();
    }

    ECap1IntCount++;
#if 0
    if(EPwm3TimerDirection == EPWM_TIMER_UP)
    {
        if(EPwm3Regs.TBPRD < PWM3_TIMER_MAX)
        {
            EPwm3Regs.TBPRD++;
        }
        else
        {
            EPwm3TimerDirection = EPWM_TIMER_DOWN;
            EPwm3Regs.TBPRD--;
        }
    }
    else
    {
        if(EPwm3Regs.TBPRD > PWM3_TIMER_MIN)
        {
            EPwm3Regs.TBPRD--;
        }
        else
        {
            EPwm3TimerDirection = EPWM_TIMER_UP;
            EPwm3Regs.TBPRD++;
        }
    }
#endif

    ECap1PassCount++;

    ECap1Regs.ECCLR.bit.CEVT4 = 1;
    ECap1Regs.ECCLR.bit.INT = 1;
    ECap1Regs.ECCTL2.bit.REARM = 1;
    ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;
    //
    // Acknowledge this interrupt to receive more interrupts from group 4
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

//
// ecap2_isr -
//
__interrupt void ecap2_isr(void)
{
    //
    // Cap input is syc'ed to SYSCLKOUT so there may be
    // a +/- 1 cycle variation
    //
    if(ECap1Regs.CAP2 > EPwm3Regs.TBPRD*2+1 ||
       ECap1Regs.CAP2 < EPwm3Regs.TBPRD*2-1)
    {
        Fail();
    }

    if(ECap1Regs.CAP3 > EPwm3Regs.TBPRD*2+1 ||
       ECap1Regs.CAP3 < EPwm3Regs.TBPRD*2-1)
    {
        Fail();
    }

    if(ECap1Regs.CAP4 > EPwm3Regs.TBPRD*2+1 ||
       ECap1Regs.CAP4 < EPwm3Regs.TBPRD*2-1)
    {
        Fail();
    }

    ECap2IntCount++;
#if 0
    if(EPwm3TimerDirection == EPWM_TIMER_UP)
    {
        if(EPwm3Regs.TBPRD < PWM3_TIMER_MAX)
        {
            EPwm3Regs.TBPRD++;
        }
        else
        {
            EPwm3TimerDirection = EPWM_TIMER_DOWN;
            EPwm3Regs.TBPRD--;
        }
    }
    else
    {
        if(EPwm3Regs.TBPRD > PWM3_TIMER_MIN)
        {
            EPwm3Regs.TBPRD--;
        }
        else
        {
            EPwm3TimerDirection = EPWM_TIMER_UP;
            EPwm3Regs.TBPRD++;
        }
    }
#endif

    ECap2PassCount++;

    ECap2Regs.ECCLR.bit.CEVT4 = 1;
    ECap2Regs.ECCLR.bit.INT = 1;
    ECap2Regs.ECCTL2.bit.REARM = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 4
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}



//
// Fail -
//
void Fail()
{
    __asm("   ESTOP0");
}

void
InitECap1Gpio(void)
{
    EALLOW;
    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
//    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;      // Enable pull-up on GPIO5 (CAP1)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;   // Enable pull-up on GPIO11 (CAP1)
    //GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pull-up on GPIO19 (CAP1)
    //GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;   // Enable pull-up on GPIO24 (CAP1)
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;
    //
    // Inputs are synchronized to SYSCLKOUT by default.
    // Comment out other unwanted lines.
    //
//    GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 0;    // Synch to SYSCLKOUT GPIO5 (CAP1)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 0; // Synch to SYSCLKOUT GPIO11 (CAP1)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 0; // Synch to SYSCLKOUT GPIO19 (CAP1)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0; // Synch to SYSCLKOUT GPIO24 (CAP1)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 0;
    //
    // Configure eCAP-1 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be eCAP1 functional
    // pins.
    // Comment out other unwanted lines.
    //
//    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 3;     // Configure GPIO5 as CAP1
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 3;  // Configure GPIO11 as CAP1
    //GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 3;  // Configure GPIO19 as CAP1
    //GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 1;  // Configure GPIO24 as CAP1
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 1;
    EDIS;
}

#endif

//
// End of File
//
