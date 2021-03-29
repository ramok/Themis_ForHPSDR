 

// hrcap related functions --- jcw


#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"


// #include "DSP28x_Project.h"
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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#if 1
// HRCAP:
#define ENABLE_HRCAP_IRQ1	1
#define ENABLE_HRCAP_IRQ2	1
// #define ENABLE_HRCAP_IRQ3	1
// #define ENABLE_HRCAP_IRQ4	1
#define ENABLE_SCITXRXINT_IRQS      1
#define USE_HRCAP1	1
#define USE_HRCAP2	1
// #define USE_HRCAP3	1
// #define USE_HRCAP4	1
#endif

#define DEBUG_RUN_PID_IN_HRCAP_IRQ (1)		// 'post'

extern Uint8 * debug_string;
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

#define FALLTESTHRCAP1 0
#define RISETESTHRCAP1 1

//
// Defines to capture period or pulse widths:
//
#define PERIODTEST 0
#define PERIODTESTHRCAP1 0

extern SemaphoreHandle_t xHRCAP1Semaphore;
extern SemaphoreHandle_t xHRCAP2Semaphore;
extern SemaphoreHandle_t xHRCAP3Semaphore;
extern SemaphoreHandle_t xHRCAP4Semaphore;

extern SemaphoreHandle_t xControl1Semaphore;
extern SemaphoreHandle_t xControl4Semaphore;
extern SemaphoreHandle_t xHRCAP1DataDebugSemaphore;

#if 1
#define NUM_HRCAP 5      // # of HRCAP modules + 1 (4 HRCAP's on 2806x + 1 = 5)
volatile struct HRCAP_REGS *HRCAP[NUM_HRCAP] = {0, &HRCap1Regs, &HRCap2Regs, &HRCap3Regs, &HRCap4Regs};
#endif

//! ...and calculate the high
//!  resolution pulse widths in integer + fractional HCCAPCLK cycles
//!  in Q16 format.

/*
0x00006AE0  HRCAP2_HCCTL, HRCap2Regs
0x00006AE0  0102
0x00006AE1  HRCAP2_HCIFR
0x00006AE1  0017
0x00006AE2  HRCAP2_HCICLR
0x00006AE2  0000
0x00006AE3  HRCAP2_HCIFRC
0x00006AE3  0000
0x00006AE4  HRCAP2_HCCOUNTER
0x00006AE4  0041    0000    0000    0000    0000    0000    0000    0000    0000    0000    0000    0000
0x00006AF0  HRCAP2_HCCAPCNTRISE0
0x00006AF0  0042    0000
0x00006AF2  HRCAP2_HCCAPCNTFALL0
0x00006AF2  001B    0000
0x00006AF4  HRCAP2_HCCAPDLYRISE0
0x00006AF4  0052
0x00006AF5  HRCAP2_HCCAPDLYFALL0
0x00006AF5  0049    0000    0000
0x00006AF8  HRCAP2_HCCAPCNTRISE1
0x00006AF8  0041    0000
0x00006AFA  HRCAP2_HCCAPCNTFALL1
0x00006AFA  001B    0000
0x00006AFC  HRCAP2_HCCAPDLYRISE1
0x00006AFC  0043
0x00006AFD  HRCAP2_HCCAPDLYFALL1
0x00006AFD  004C    0000    0000
0x00006B00  EQep1Regs, eQEP1_QPOSCNT
*/


#ifdef USE_HRCAP1
extern QueueHandle_t xHRCAP1Queue;
// extern struct HRCAP1_Queue HRCAP1Data;

typedef struct HRCAP1_Queue
{

    unsigned long int_HRC1pulsewidthlow0[5];
    unsigned long frac_HRC1pulsewidthlow0[5];
    unsigned long int_HRC1pulsewidthhigh0[5];
    unsigned long frac_HRC1pulsewidthhigh0[5];
    unsigned long int_HRC1pulsewidthhigh1[5];
    unsigned long frac_HRC1pulsewidthhigh1[5];
    unsigned long int_HRC1periodwidthRise0[5];
    unsigned long frac_HRC1periodwidthRise0[5];

} HRCAP1Queue;

HRCAP1Queue HRCAP1Data;


void HRCAP1_Config(void);
__interrupt void HRCAP1_Isr (void);
Uint16 first1;
Uint16 counter1;            // Increments CMPAHR by 8 MEP steps with each period
Uint16 datacounter1;        // Counts 5 periods then resets.
// Uint32 pulsewidthlow0[5];
_iq pulsewidthlow0[5];
// unsigned long int_pulsewidthlow0[5];
// unsigned long frac_pulsewidthlow0[5];
// Uint32 pulsewidthhigh0[5];
_iq pulsewidthhigh0[5];
_iq pulsewidthhigh1[5];
// unsigned long int_pulsewidthhigh0[5];
// unsigned long frac_pulsewidthhigh0[5];
unsigned char do_cal_1 = 0;

// Uint32 periodwidth0[5];
_iq periodwidthRise0[5];
Uint32 periodwidthhigh0[5];
#endif

#ifdef USE_HRCAP2
extern QueueHandle_t xHRCAP2Queue;
typedef struct HRCAP2_Queue
{

    unsigned long int_HRC2pulsewidthlow1[5];
    unsigned long frac_HRC2pulsewidthlow1[5];
    unsigned long int_HRC2pulsewidthhigh1[5];
    unsigned long frac_HRC2pulsewidthhigh1[5];
    Uint16 datacounter2;
    Uint16 HRC2_HCCAPDLYRISE0[5];
    Uint16 HRC2_HCCAPDLYFALL0[5];
    bool runECTask;

} HRCAP2Queue;



HRCAP2Queue HRCAP2Data;

void HRCAP2_Config(void);
__interrupt void HRCAP2_Isr (void);
Uint16 first2;
Uint16 counter2;            // Increments CMPAHR by 8 MEP steps with each period
// Uint16 datacounter2;        // Counts 5 periods then resets.
_iq pulsewidthlow1[5];
// unsigned long int_pulsewidthlow1[5];
// unsigned long frac_pulsewidthlow1[5];
_iq pulsewidthhigh1[5];
// unsigned long int_pulsewidthhigh1[5];
// unsigned long frac_pulsewidthhigh1[5];
Uint32 periodwidthhigh1[5];
Uint32 periodwidth1[5];
#endif

#ifdef USE_HRCAP3
extern QueueHandle_t xHRCAP3Queue;
typedef struct HRCAP3_Queue
{

    unsigned long int_HRC3pulsewidthlow2[5];
    unsigned long frac_HRC3pulsewidthlow2[5];
    unsigned long int_HRC3pulsewidthhigh2[5];
    unsigned long frac_HRC3pulsewidthhigh2[5];

} HRCAP3Queue;

HRCAP3Queue HRCAP3Data;
void HRCAP3_Config(void);
__interrupt void HRCAP3_Isr (void);
Uint16 first3;

Uint16 counter3;            // Increments CMPAHR by 8 MEP steps with each period
Uint16 datacounter3;        // Counts 5 periods then resets.
_iq pulsewidthlow2[5];
// unsigned long int_pulsewidthlow2[5];
// unsigned long frac_pulsewidthlow2[5];
_iq pulsewidthhigh2[5];
// unsigned long int_pulsewidthhigh2[5];
// unsigned long frac_pulsewidthhigh2[5];
Uint32 periodwidthhigh2[5];
Uint32 periodwidth2[5];
#endif

#ifdef USE_HRCAP4

extern QueueHandle_t xHRCAP4Queue;
typedef struct HRCAP4_Queue
{

    unsigned long int_HRC4pulsewidthlow3[5];
    unsigned long frac_HRC4pulsewidthlow3[5];
    unsigned long int_HRC4pulsewidthhigh3[5];
    unsigned long frac_HRC4pulsewidthhigh3[5];

} HRCAP4Queue;

HRCAP4Queue HRCAP4Data;

void HRCAP4_Config(void);
__interrupt void HRCAP4_Isr (void);
Uint16 first4;
Uint16 counter4;            // Increments CMPAHR by 8 MEP steps with each period
Uint16 datacounter4;        // Counts 5 periods then resets.
// Uint32 pulsewidthlow3[5];
_iq pulsewidthlow3[5];
// unsigned long int_pulsewidthlow3[5];
// unsigned long frac_pulsewidthlow3[5];
// Uint32 pulsewidthhigh3[5];
_iq pulsewidthhigh3[5];
// unsigned long int_pulsewidthhigh3[5];
// unsigned long frac_pulsewidthhigh3[5];
Uint32 periodwidth3[5];
Uint32 periodwidthhigh3[5];
#ifndef USE_HRCAP1
unsigned char do_cal_1 = 0;
#endif
#endif


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
// extern Uint32 LowPulseWidth0 (Uint16 *ptrHRCAPmodule);    LowPulseWidth0
// extern Uint32 PeriodWidthRise0 (Uint16 *ptrHRCAPmodule);
// extern Uint32 PeriodWidthFall0 (Uint16 *ptrHRCAPmodule);
// extern Uint16 HRCAP_Cal(Uint16 HRCapModule, Uint16 PLLClk, volatile struct EPWM_REGS *ePWMModule);       // HRCAP Calibration Function

__interrupt void HRCAP1_Isr(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
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

	if (HRCap1Regs.HCIFR.bit.RISE == 1)
	{

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
		if (datacounter1 >= 5)
		{
			datacounter1 = 0;
			do_cal_1 = 1;
		}
#endif
#if FALLTESTHRCAP1
#if PERIODTESTHRCAP1
		periodwidth0[datacounter1] = PeriodWidthFall0((Uint16 *)&HRCap1Regs);
#else
		pulsewidthlow0[datacounter1] = LowPulseWidth0((Uint16 *)&HRCap1Regs);                  
		pulsewidthhigh0[datacounter1] = HighPulseWidth0((Uint16 *)&HRCap1Regs);  
#endif
#elif RISETESTHRCAP1
#if PERIODTESTHRCAP1
		periodwidthRise0[datacounter1] = PeriodWidthRise0((Uint16 *)&HRCap1Regs);

		HRCAP1Data.int_HRC1periodwidthRise0[datacounter1] = _IQ16int(periodwidthRise0[datacounter1]);
		HRCAP1Data.frac_HRC1periodwidthRise0[datacounter1] = _IQ16frac(periodwidthRise0[datacounter1]);

//		periodwidth0[datacounter1] = PeriodWidthFall0((Uint16 *)&HRCap1Regs);
#else	        	    
		pulsewidthlow0[datacounter1] = LowPulseWidth0((Uint16 *)&HRCap1Regs);           // maybe just Q these
//		pulsewidthlow0[datacounter1] = LowPulseWidth1((Uint16 *)&HRCap1Regs);


//		int_pulsewidthlow0[datacounter1] = _IQ16int(pulsewidthlow0[datacounter1]);
//		HRCAP1Data.int_HRC1pulsewidthlow0[datacounter1] = int_pulsewidthlow0[datacounter1];
		HRCAP1Data.int_HRC1pulsewidthlow0[datacounter1] = _IQ16int(pulsewidthlow0[datacounter1]);

//		frac_pulsewidthlow0[datacounter1] = _IQ16frac(pulsewidthlow0[datacounter1]);
//		HRCAP1Data.frac_HRC1pulsewidthlow0[datacounter1] = frac_pulsewidthlow0[datacounter1];
		HRCAP1Data.frac_HRC1pulsewidthlow0[datacounter1] = _IQ16frac(pulsewidthlow0[datacounter1]);

		pulsewidthhigh0[datacounter1] = HighPulseWidth0((Uint16 *)&HRCap1Regs);       // maybe just Q these
		pulsewidthhigh1[datacounter1] = HighPulseWidth1((Uint16 *)&HRCap1Regs);

//		int_pulsewidthhigh0[datacounter1] = _IQ16int(pulsewidthhigh0[datacounter1]);
//		HRCAP1Data.int_HRC1pulsewidthhigh0[datacounter1] = int_pulsewidthhigh0[datacounter1];
		HRCAP1Data.int_HRC1pulsewidthhigh0[datacounter1] = _IQ16int(pulsewidthhigh0[datacounter1]);
		HRCAP1Data.int_HRC1pulsewidthhigh1[datacounter1] = _IQ16int(pulsewidthhigh1[datacounter1]);

//		frac_pulsewidthhigh0[datacounter1] = _IQ16frac(pulsewidthhigh0[datacounter1]);
//		HRCAP1Data.frac_HRC1pulsewidthhigh0[datacounter1] = frac_pulsewidthhigh0[datacounter1];
		HRCAP1Data.frac_HRC1pulsewidthhigh0[datacounter1] = _IQ16frac(pulsewidthhigh0[datacounter1]);
		HRCAP1Data.frac_HRC1pulsewidthhigh1[datacounter1] = _IQ16frac(pulsewidthhigh1[datacounter1]);
#if 0
		sprintf((uint8_t *)&debug_string[0], "HRC1: H%lu, %lu, L%lu, %lu\r\n", HRCAP1Data.int_HRC1pulsewidthhigh0[datacounter1], HRCAP1Data.frac_HRC1pulsewidthhigh0[datacounter1], HRCAP1Data.int_HRC1pulsewidthlow0[datacounter1], HRCAP1Data.frac_HRC1pulsewidthlow0[datacounter1] );
//		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (uint8_t *)&debug_string[0], strlen(debug_string));
#endif
//		sprintf((uint8_t *)&debug_string[0], "%lu", HRCAP1Data.int_HRC1pulsewidthhigh0[datacounter1]);
//	    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (uint8_t *)&debug_string[0], strlen(debug_string));

//		extern QueueHandle_t xHRCAP1Queue;
//		extern struct HRCAP1_Queue HRCAP1Data;

		xQueueSendToBackFromISR(xHRCAP1Queue, &HRCAP1Data, &xHigherPriorityTaskWoken );

//		xQueueSendToBackFromISR(xHRCAP1Queue, int_pulsewidthhigh0[datacounter1],  &xHigherPriorityTaskWoken );
//		xQueueSendToBackFromISR(xHRCAP1Queue, frac_pulsewidthlow0[datacounter1], &xHigherPriorityTaskWoken );
//		xQueueSendToBackFromISR(xHRCAP1Queue, int_pulsewidthlow0[datacounter1],  &xHigherPriorityTaskWoken );
#endif
#endif

//		counter1+=8;

	//
	// Increment CMPAHR by 8 for next period.        
	//
///		EPwm1Regs.CMPA.half.CMPAHR = counter<<8; 
		datacounter1++;
	}
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
//                         PieCtrlRegs.PIEIER4.bit.INTx8=0;
#endif
#if 0
#ifdef ENABLE_HRCAP_IRQ4
                         __asm("   NOP");
//                         PieCtrlRegs.PIEIER5.bit.INTx5=0;     // Enable PIE Group 4, INT 8
#endif
#endif
#endif

	EDIS;
//	xSemaphoreGiveFromISR( xHRCAP1DataDebugSemaphore, &xHigherPriorityTaskWoken );
	xSemaphoreGiveFromISR( xHRCAP1Semaphore, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
			 
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
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool run_5_then_1 = pdFALSE;
#ifdef HRCAP2_DELAY_REGISTERS
    Uint16 * HR2_DLYRISE0; // = (Uint16*) (0x00006af4);         // HR2 delay rise 0
    Uint16 * HR2_DLYFALL0; // = (Uint16*) (0x00006af5);         // HR2 delay fall 0

    HR2_DLYRISE0 = (Uint16*) (0x00006af4);         // HR2 delay rise 0
    HR2_DLYFALL0 = (Uint16*) (0x00006af5);         // HR2 delay fall 0
#endif
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
		if (HRCAP2Data.datacounter2 >= 5)
		{
//		    if (HRCAP2Data.runECTask == pdFALSE)
		    HRCAP2Data.datacounter2 = 0;
//			HRCAP2Data.runECTask = false;
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
//		if (HRCAP2Data.runECTask == pdFALSE)
		{
//		HRCAP2Data.datacounter2 = 0;
		pulsewidthlow1[HRCAP2Data.datacounter2] = LowPulseWidth0((Uint16 *)&HRCap2Regs);
//		int_pulsewidthlow1[datacounter2] = _IQ16int(pulsewidthlow1[datacounter2]);
//		frac_pulsewidthlow1[datacounter2] = _IQ16frac(pulsewidthlow1[datacounter2]);
		HRCAP2Data.int_HRC2pulsewidthlow1[HRCAP2Data.datacounter2] = _IQ16int(pulsewidthlow1[HRCAP2Data.datacounter2]);
		HRCAP2Data.frac_HRC2pulsewidthlow1[HRCAP2Data.datacounter2] = _IQ16frac(pulsewidthlow1[HRCAP2Data.datacounter2]);

		pulsewidthhigh1[HRCAP2Data.datacounter2] = HighPulseWidth0((Uint16 *)&HRCap2Regs);
//		int_pulsewidthhigh1[datacounter2] = _IQ16int(pulsewidthhigh1[datacounter2]);
//		frac_pulsewidthhigh1[datacounter2] = _IQ16frac(pulsewidthhigh1[datacounter2]);
		HRCAP2Data.int_HRC2pulsewidthhigh1[HRCAP2Data.datacounter2] = _IQ16int(pulsewidthhigh1[HRCAP2Data.datacounter2]);
		HRCAP2Data.frac_HRC2pulsewidthhigh1[HRCAP2Data.datacounter2] = _IQ16frac(pulsewidthhigh1[HRCAP2Data.datacounter2]);

#ifdef HRCAP2_DELAY_REGISTERS
		HRCAP2Data.HRC2_HCCAPDLYRISE0[HRCAP2Data.datacounter2] = *HR2_DLYRISE0;
		HRCAP2Data.HRC2_HCCAPDLYFALL0[HRCAP2Data.datacounter2] = *HR2_DLYFALL0;
#endif
//		xQueueSendToBackFromISR(xHRCAP2Queue, &HRCAP2Data, &xHigherPriorityTaskWoken );
		}

#endif
#endif

//		counter2+=8;

	//
	// Increment CMPAHR by 8 for next period.        
	//
///		EPwm1Regs.CMPA.half.CMPAHR = counter<<8; 
//	    if (HRCAP2Data.runECTask == pdFALSE)
		HRCAP2Data.datacounter2++;

#ifdef USE_HRCAP2_AVERAGES
    if ( HRCAP2Data.datacounter2 >= 5 )
        {
            HRCAP2Data.runECTask = pdTRUE;
//          run_5_then_1 = pdTRUE;
        }
#else
         HRCAP2Data.runECTask = pdTRUE;
#endif
	}

#if 0
	if ( datacounter2 >= 5 && run_5_then_1 == pdFALSE)
	{
	    HRCAP2Data.runECTask = pdTRUE;
	    run_5_then_1 = pdTRUE;
	}


	if ( run_5_then_1 == pdTRUE)
	    HRCAP2Data.runECTask = pdTRUE;
#endif

#if DEBUG_RUN_PID_IN_HRCAP_IRQ



	
#endif
	HRCap2Regs.HCICLR.bit.FALL=1;  // Clear FALL flag
	HRCap2Regs.HCICLR.bit.RISE=1;  // Clear RISE flag

	HRCap2Regs.HCICLR.bit.INT=1;   // Clear HRCAP interrupt flag
	PieCtrlRegs.PIEACK.bit.ACK4=1; // Acknowledge PIE Group 4 interrupts.

    PieCtrlRegs.PIEIER4.bit.INTx8=0;
//    PieCtrlRegs.PIEIER4.bit.INTx7=0;
    __asm("   NOP");
//    PieCtrlRegs.PIEIER5.bit.INTx4=0;
	    EDIS;
//    if (HRCAP2Data.runECTask == pdFALSE)
	xSemaphoreGiveFromISR( xHRCAP2Semaphore, &xHigherPriorityTaskWoken );
	xQueueSendToBackFromISR(xHRCAP2Queue, &HRCAP2Data, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

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
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    EALLOW;
	if (HRCap3Regs.HCIFR.bit.RISEOVF == 1) 
	{
		EDIS;
///	    ESTOP0;         // Another rising edge detected before ISR serviced
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
//        int_pulsewidthlow2[datacounter3] = _IQ16int(pulsewidthlow2[datacounter3]);
//        frac_pulsewidthlow2[datacounter3] = _IQ16frac(pulsewidthlow2[datacounter3]);
		HRCAP3Data.int_HRC3pulsewidthlow2[datacounter3] = _IQ16int(pulsewidthlow2[datacounter3]);
		HRCAP3Data.frac_HRC3pulsewidthlow2[datacounter3] = _IQ16frac(pulsewidthlow2[datacounter3]);

		pulsewidthhigh2[datacounter3] = HighPulseWidth0((Uint16 *)&HRCap3Regs);
//	    int_pulsewidthhigh2[datacounter3] = _IQ16int(pulsewidthhigh2[datacounter3]);
//	    frac_pulsewidthhigh2[datacounter3] = _IQ16frac(pulsewidthhigh2[datacounter3]);
		HRCAP3Data.int_HRC3pulsewidthhigh2[datacounter3] = _IQ16int(pulsewidthhigh2[datacounter3]);
		HRCAP3Data.frac_HRC3pulsewidthhigh2[datacounter3] = _IQ16frac(pulsewidthhigh2[datacounter3]);

		xQueueSendToBackFromISR(xHRCAP3Queue, &HRCAP3Data, &xHigherPriorityTaskWoken );

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
//	PieCtrlRegs.PIEIER5.bit.INTx5=0;
	EDIS;
	
	xSemaphoreGiveFromISR( xHRCAP3Semaphore, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

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

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	EALLOW;
	if (HRCap4Regs.HCIFR.bit.RISEOVF == 1) 
	{
//	    periodwidth3[datacounter4] = PeriodWidthRise0((Uint16 *)&HRCap4Regs);
	    EDIS;
///	    ESTOP0;         // Another rising edge detected before ISR serviced
	    EALLOW;
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
#ifndef USE_HRCAP1			
			do_cal_1 = 1;
#endif			
		}
#if FALLTEST
#if PERIODTEST
		periodwidth3[datacounter4] = PeriodWidthFall0((Uint16 *)&HRCap4Regs);  
#else
		pulsewidthlow3[datacounter4] = LowPulseWidth0((Uint16 *)&HRCap4Regs);                  
		pulsewidthhigh3[datacounter4] = HighPulseWidth0((Uint16 *)&HRCap4Regs);  
#endif
#elif RISETEST	   
#if PERIODTEST
		periodwidth3[datacounter4] = PeriodWidthRise0((Uint16 *)&HRCap4Regs);  
#else	        	    
		pulsewidthlow3[datacounter4] = LowPulseWidth0((Uint16 *)&HRCap4Regs);
//		int_pulsewidthlow3[datacounter4] = _IQ16int(pulsewidthlow3[datacounter4]);
//		frac_pulsewidthlow3[datacounter4] = _IQ16frac(pulsewidthlow3[datacounter4]);
		HRCAP4Data.int_HRC4pulsewidthlow3[datacounter4] = _IQ16int(pulsewidthlow3[datacounter4]);
		HRCAP4Data.frac_HRC4pulsewidthlow3[datacounter4] = _IQ16frac(pulsewidthlow3[datacounter4]);

		pulsewidthhigh3[datacounter4] = HighPulseWidth0((Uint16 *)&HRCap4Regs);
//		int_pulsewidthhigh3[datacounter4] = _IQ16int(pulsewidthhigh3[datacounter4]);
//		frac_pulsewidthhigh3[datacounter4] = _IQ16frac(pulsewidthhigh3[datacounter4]);
		HRCAP4Data.int_HRC4pulsewidthhigh3[datacounter4] = _IQ16int(pulsewidthhigh3[datacounter4]);
		HRCAP4Data.frac_HRC4pulsewidthhigh3[datacounter4] = _IQ16frac(pulsewidthhigh3[datacounter4]);

//        pulsewidthlow0[datacounter1] = LowPulseWidth0((Uint16 *)&HRCap1Regs);
//        pulsewidthhigh0[datacounter1] = HighPulseWidth0((Uint16 *)&HRCap1Regs);

//		xQueueSendToBackFromISR(xHRCAP4Queue, &HRCAP4Data, &xHigherPriorityTaskWoken );
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
#if 0
#ifdef ENABLE_HRCAP_IRQ1
                         PieCtrlRegs.PIEIER4.bit.INTx7=0;     // Enable PIE Group 4, INT 7
#endif
#endif

#ifdef ENABLE_HRCAP_IRQ4
                         PieCtrlRegs.PIEIER5.bit.INTx5=0;     // Enable PIE Group 4, INT 8

                         __asm("   NOP");                       // for debug breakpoint

//                       PieCtrlRegs.PIEIER4.bit.INTx7=1;
#endif
// #endif

	EDIS;
			 
//	xSemaphoreGiveFromISR( xControl4Semaphore, &xHigherPriorityTaskWoken );
	xSemaphoreGiveFromISR( xHRCAP4Semaphore, &xHigherPriorityTaskWoken );
    xQueueSendToBackFromISR(xHRCAP4Queue, &HRCAP4Data, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}
#endif
