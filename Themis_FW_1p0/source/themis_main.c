// source file for themis - need to rename this --- jcw
// main() in this file.
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
// #include "F2806x_Device.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
// #include "DSP28x_Project.h"
#include <stdbool.h>
#include <stdint.h>
#include "math.h"
#include "C28x_FPU_FastRTS.h"
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
#include "SFO_V6.h"
#include "F2806x_XIntrupt.h"
#include "F2806x_SWPrioritizedIsrLevels.h"

#include "DCL.h"
#include "DCLF32.h"
#include "DCL_fdlog.h"
#include "DCL_NLPID.h"
#include "control-pid.h"

#include "sntp-alpha.h"

extern Uint16 powfStart;
extern Uint16 powfEnd;
extern Uint16 powfRunStart;
extern Uint16 powfLoadSize;
/*
LOAD_START(_fpuTablesSStart),
LOAD_END(_fpuTablessEnd),
RUN_START(_fpuRunStart),
LOAD_SIZE(_fpuLoadSize)
*/
extern Uint16 fpuTablesSStart;
extern Uint16 fpuTablessEnd;
extern Uint16 fpuRunStart;
extern Uint16 fpuLoadSize;

/// #define    INT1PL      1        // Group1 Interrupts (PIEIER1)

/// #define    G14PL       2        // XINT1       (External)

#pragma CODE_SECTION(powf_special, "ramfuncs");

extern float powf_special(float,float);
void USB_Print_Task(void * pvParameters);
__interrupt void XINT1_ISR(void);
uint32_t PID_ID = 0x00;

// PID ID's:
// Linear PID C1 : 0x004C5031   -> LP1
// Linear PID C4 : 0x004C5034   -> LP4
// Non-Linear PID C1: 0x4E4C5031 -> NLP1
// Non-Linear PID C2: 0x4E4C5032 -> NLP2
// Alternate PID:     0x00004150 -> AP

#define LINEAR_PID_C1 (uint32_t) 0x004C5031
#define LINEAR_PID_C4 (uint32_t) 0x004C5034
#define NON_LINEAR_PID_C1 (uint32_t) 0x4E4C5031
#define NON_LINEAR_PID_C2 (uint32_t) 0x4E4C5032
#define ALTERNATE_PID (uint32_t) 0x00004150

#define XINT_RISING (1)
#define XINT_FALLING (2)

static Uint8 XINT_polarity_state = XINT_RISING;

#define LAST_STATE_LOCKED (1)
#define LAST_STATE_UNLOCKED (2)

static Uint8 digitalLockState = LAST_STATE_UNLOCKED;

// #define NO_USB_TASK_FOR_OUTPUT (1)

#define POLARITY_CHANGE (1)                 // 1 - trigger on rising and falling edge of 1PPS
// #define TRACK_DIGITAL_PHASE_LOCK  (1)

#define USE_ALTERNATE_PID (1)                            // define to use grass roots PID
#ifdef USE_ALTERNATE_PID
pid *controlPID;		// alternate PID structure
/// float calcStartHRCAPCount0 = 48.0f;                     // assume good starting count - PID will adjust
// int8_t heater_on;
// int8_t heater_state;
// PID_ID = ALTERNATE_PID;
static float PWM_X;
static float HRCAP_Diff = 0.0f;
static float PID_Adjust;
#endif

#define NO_INTEGRAL_SATURATION (1)
#define NO_USB_PRINTING (1)
// #define RUN_OPEN_LOOP_TESTING (1)
#ifndef RUN_OPEN_LOOP_TESTING
#define NOT_DOING_OPEN_LOOP_TESTING (1)
#endif
#define RK_IS_SET   (1)
// #define USE_SYNTHETIC_UK (1)
#ifdef USE_SYNTHETIC_UK
#define UK_MAX (float) 100.0
#define UK_MIN (float) -100.0
#endif

// #define RUN_MAN_CONTROL (1)
#define RUN_NON_LINEAR_PID (1)
#ifndef RUN_NON_LINEAR_PID
#define RUN_LINEAR_PID  (1)
#define NO_SP_WEIGHT    (1)                     // C4 vs C1 (C4 means no SP weighting)
#else
#define RUN_NON_LINEAR_PID_C2 (1)
// #define RUN_NON_LINEAR_PID_C1 (1)
#endif

// #define PGA_DEBUG            (1)                // check PGA gains/voltages
static bool edge_flag = pdFALSE;
// global  variables
// long IdleLoopCount = 0;
// long IsrCount = 0;
#if 0                                           // put into structure for debug purposes (CCS DBG BUG)
volatile float rk = 0.50f;
float yk;
float lk;
float uk;
#endif
#if RUN_LINEAR_PID
DCL_PID *pid1;
// DCL_PID pid1 = PID_DEFAULTS;
DCL_PID_SPS sps1 = PID_SPS_DEFAULTS;
DCL_CSS css1 = DCL_CSS_DEFAULTS;
#else
// DCL_NLPID nlpid1 = NLPID_DEFAULTS;
DCL_NLPID *nlpid1;
DCL_NLPID_SPS sps1 = NLPID_SPS_DEFAULTS;
// DCL_NLPID_SPS *sps1;
DCL_CSS css1 = DCL_CSS_DEFAULTS;
// DCL_CSS *css1;
volatile uint16_t calFlag = 0;
#endif
static float Duty;
unsigned long control_valL;
unsigned long control_valL_min;
unsigned long control_valH;
unsigned long control_valH_min;
float HRCAP_Delta 	= 0.0;
float HRCAP_OldData = 0.0;
float PID_Delta 	= 0.0;
float PID_OldData = 0.0;
uint8_t *ptr;
// #define COPY_TO_USB_SERIAL_FROM_UBLOX 1
//
//                      *!!IMPORTANT!!*
// UPDATE NUMBER OF HRPWM CHANNELS + 1  USED IN SFO_V6.H
//
// Configured for 4 HRPWM channels, but F2806x has a maximum of 7
// HRPWM channels (8=7+1)
//
// i.e.: #define PWM_CH 5

//
// Function Prototypes
void vConfigTimer0( void );
__interrupt void timer0_ISR( void );
__interrupt void timer1_ISR( void );
/// #define PWM_CH 2
#define RUN_EC_TASK (1)
void HRPWM_ConfigLocal(int);
void HRPWM_ConfigLocal2(Uint16 period);
__interrupt void FreqCtlISR(void);  // frequency modulation & phase sync ISR

//
// Global variable used by teh SFO library. Result can be used for all HRPWM
// channels. This variable is also copied to HRMSTEP register by SFO(0)
// function.
//
int MEP_ScaleFactor;

//
// Array of pointers to EPwm register structures:
// *ePWM[0] is defined as dummy value not used in the example
//
#if 1
volatile struct EPWM_REGS *ePWM[PWM_CH] = {&EPwm1Regs, &EPwm7Regs};
#endif
//
// Main
//

void SysCtrlInit(void);
void SysCtrlInitA(void);
void error();
// #define STACK_SIZE 	128U
#define STACK_SIZE  256U
#define USE_USB       (1)
#define ControlSemaphore_DELAY ( TickType_t ) 0x00001000UL
#define EVER ;;

extern volatile unsigned long g_ulSysTickCount;
static uint32_t control1, control2, control3, control4, ECcontrol = 0;
extern __interrupt void USBUARTTXIntHandler(void);
extern __interrupt void USBUARTRXIntHandler(void);

void EC_Task(void * pvParameters);
void USB_Print_Task(void * pvParameters);
#define ENABLE_SCITXRXINT_IRQS      1

// #define UBLOX_USB_TEST_ONLY         1
extern volatile uint32_t g_ui32UARTTxCount;
extern volatile uint32_t g_ui32UARTRxCount;
extern volatile uint32_t g_ui32UARTRxIntEntryCount;
extern volatile uint32_t g_ui32UARTRxIntErrorCount;
#ifdef DEBUG
extern uint32_t g_ui32UARTRxErrors;
#endif
// The system tick timer period.
//
#define SYSTICKS_PER_SECOND     100
#define SYSTICK_PERIOD_MS (1000 / SYSTICKS_PER_SECOND)

Uint8 * debug_string;               // for debug strings - default is 128 bytes

#define ADC_usDELAY  1000L
// Configure if ePWM timer interrupt is enabled at the PIE level:
// 1 = enabled,  0 = disabled
//
#ifndef UBLOX_USB_TEST_ONLY
#define PWM7_INT_ENABLE  1 /// 0  <=== NOT ENABLED!
void InitEPwmTimer(void);
// #define PWM7_TIMER_TBPRD   0xAFFF       // Configure the period for the timer ~ 1kHz
// #define PWM7_TIMER_TBPRD   0x1FFF       // Configure the period for the timer ~ 5kHz
// #define PWM7_TIMER_TBPRD   0x2500      // Configure the period for the timer
// #define PWM7_TIMER_TBPRD   0x5FFF           // close to 2 kHz
// #define PWM7_TIMER_TBPRD   0xFFF       // Configure the period for the timer ~ 10kHz
#define PWM7_TIMER_TBPRD   0xAFF          // ~ 15kHz
// #define PWM7_TIMER_TBPRD   0x08           // on 280 MHz 'Synth' - ~ 5MHz Out
// #define PWM7_TIMER_TBPRD   0x04              // approx 9MHz
// #define PWM7_TIMER_TBPRD   0x0F                 // approx 2.8 MHz

#endif

// #define SYSTEM_CLOCK_SPEED_LOCAL   80000000UL
#define SYSTEM_CLOCK_SPEED_LOCAL   90000000UL

// Function Prototypes
#define CPU_FREQ    90E6
// #define CPU_FREQ    80E6
// #define LSPCLK_FREQ CPU_FREQ/4.5
#define LSPCLK_FREQ 22500000UL
//#define SCI_FREQ    100E3
#define SCI_FREQ    115200
#define SCI_PRD     (LSPCLK_FREQ/(SCI_FREQ*8))-1
//
// Function Prototypes
//
__interrupt void sciaTxFifoIsr(void);
__interrupt void sciaRxFifoIsr(void);
// __interrupt void scibTxFifoIsr(void);
__interrupt void scibRxFifoIsr(void);
void scia_fifo_init(void);
void scib_fifo_init(void);
//
// Globals
//
Uint16 sdataA[2];    // Send data for SCI-A
Uint16 rdataA[2];    // Received data for SCI-A
Uint16 rdata_pointA; // Used for checking the received data
Uint16 rdataB[2];

//
// Global system tick counter holds elapsed time since the application started
//
// Defines to keep track of which way the compare value is moving
//

#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0
#define DAC_PWM       1
// #define USE_USB       1
// #define USE_SPI_IRQ   1
#define USE_NORM      1

void InitEPwm7Gpio(void);

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

#ifndef UBLOX_USB_TEST_ONLY
#if 1
// HRCAP:
#define ENABLE_HRCAP_IRQ1	1
#define ENABLE_HRCAP_IRQ2	1
// #define ENABLE_HRCAP_IRQ3	1
// #define ENABLE_HRCAP_IRQ4	1
// #define ENABLE_SCITXRXINT_IRQS      1
#define USE_HRCAP1	1
#define USE_HRCAP2	1
// #define USE_HRCAP3	1
// #define USE_HRCAP4	1
#endif
#endif
// #define USE_STATIC_TASK_CREATION    (1)

static SemaphoreHandle_t xSemaphore = NULL;

// static StaticTask_t idleTaskBuffer;
// static StackType_t  idleTaskStack[STACK_SIZE];

SemaphoreHandle_t xHRCAP1Semaphore = NULL;
SemaphoreHandle_t xHRCAP2Semaphore = NULL;
SemaphoreHandle_t xHRCAP3Semaphore = NULL;
SemaphoreHandle_t xHRCAP4Semaphore = NULL;

SemaphoreHandle_t xControl1Semaphore = NULL;
SemaphoreHandle_t xControl2Semaphore = NULL;
SemaphoreHandle_t xControl3Semaphore = NULL;
SemaphoreHandle_t xControl4Semaphore = NULL;
SemaphoreHandle_t xControlECTaskSemaphore = NULL;
SemaphoreHandle_t EC_TaskSemaphore = NULL;
SemaphoreHandle_t xHRCAP1DataDebugSemaphore = NULL;

SemaphoreHandle_t xUSBPrintSemaphore = NULL;

#ifdef USE_STATIC_TASK_CREATION

static StaticSemaphore_t xSemaphoreBuffer;
static StaticSemaphore_t xHRCAP1SemaphoreBuffer;

static StaticTask_t redTaskBuffer;
static StackType_t  redTaskStack[STACK_SIZE];

static StaticTask_t blueTaskBuffer;
static StackType_t  blueTaskStack[STACK_SIZE];

static StaticTask_t idleTaskBuffer;
static StackType_t  idleTaskStack[STACK_SIZE];

static StaticTask_t HRCAP1TaskBuffer;
static StackType_t  HRCAP1TaskStack[STACK_SIZE];

static StaticTask_t ControlTaskBuffer;
static StackType_t  ControlTaskStack[STACK_SIZE];

static StaticTask_t ECTaskBuffer;
static StackType_t  ECTaskStack[STACK_SIZE];

#else

static TaskHandle_t redTaskHandle;
static TaskHandle_t blueTaskHandle;
static StaticTask_t idleTaskBuffer;
static StackType_t  idleTaskStack[STACK_SIZE];
static TaskHandle_t HRCAP1TaskHandle;
static TaskHandle_t HRCAP2TaskHandle;
static TaskHandle_t HRCAP3TaskHandle;
static TaskHandle_t HRCAP4TaskHandle;
static TaskHandle_t ControlTaskHandle;
static TaskHandle_t ECTaskHandle;
static TaskHandle_t StartupTaskHandle;

static TaskHandle_t USBPrintTaskHandle;

QueueHandle_t xHRCAP1Queue;
QueueHandle_t xHRCAP2Queue;
QueueHandle_t xHRCAP3Queue;
QueueHandle_t xHRCAP4Queue;

#endif

int aHeap = 0;
#if 0
extern void HRCAP1_Config(void);
extern void HRCAP2_Config(void);
extern void HRCAP3_Config(void);
extern void HRCAP4_Config(void);
extern __interrupt void HRCAP1_Isr (void);
extern __interrupt void HRCAP2_Isr (void);
extern __interrupt void HRCAP3_Isr (void);
extern __interrupt void HRCAP4_Isr (void);
#endif


Uint16 LoopCount;
Uint16 ConversionCount;

Uint16 Voltage1[10];
Uint16 Voltage2[10];
Uint16 Voltage3[10];
Uint16 Voltage4[10];
Uint16 Voltage5[10];
Uint16 Voltage6[10];
Uint16 Voltage7[10];
Uint16 Voltage8[10];

//
// Globals
//
Uint16 status;
Uint16 CMP_Reg = 20;
Uint16 CMP_HR = 0;
Uint16 PRD_Reg = 40;
Uint16 PRD_HR = 0;
Uint16 update = 0;
Uint16 isr_cnt = 0;
Uint16 change_dir = 1;
Uint16 update_rate = 10000;
Uint16 CMP_Inc = 0;
Uint16 CMP_HR_INC = 0;
Uint32 InputCMPInc = 2621;
int32 CMP_HR_temp = 0;
// Uint16 Period = 50;
Uint16 Period = 8;

// #define PGA_DEBUG   1

typedef struct PID_Vars
{
    volatile float rk;
    float yk;
    float lk;
    float uk;
}   S_PID_Vars;

S_PID_Vars  S_PID_VDATA;

#define PID_Vars_LENGTH 1
#define PID_Vars_ITEM_SIZE    sizeof (S_PID_Vars)

#ifdef USE_HRCAP1

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

typedef struct Duty_Log
{

   float Duty_data[10];

} DutyLog;

DutyLog *DutyData;

ntp_pkt *SNTPpkt;

extern HRCAP1Queue HRCAP1Data;

#define HRCAP1_QUEUE_LENGTH 1
#define HRCAP1Q_ITEM_SIZE    sizeof (HRCAP1Queue)

extern void HRCAP1_Config(void);
extern __interrupt void HRCAP1_Isr (void);
extern Uint16 first1;
extern Uint16 counter1;            // Increments CMPAHR by 8 MEP steps with each period
extern Uint16 datacounter1;        // Counts 5 periods then resets.
// Uint32 pulsewidthlow0[5];
extern _iq pulsewidthlow0[5];
// extern unsigned long int_pulsewidthlow0[5];
// extern unsigned long frac_pulsewidthlow0[5];
// Uint32 pulsewidthhigh0[5];
extern _iq pulsewidthhigh0[5];
extern _iq pulsewidthhigh1[5];
// extern unsigned long int_pulsewidthhigh0[5];
// extern unsigned long frac_pulsewidthhigh0[5];
extern unsigned char do_cal_1;

extern Uint32 periodwidth0[5];
extern Uint32 periodwidthhigh0[5];

#define PULSE_COUNT_HIGH_HRC1    48                  // this is the HRCAP count for our HRC1 config
#define PULSE_COUNT_LOW_HRC1    48
Uint32 error_count_hrc1_high = 0;
Uint32 error_count_hrc1_low = 0;

// after rework - proto #1 - SYNC_IN routed to HRCAP2:
#define PULSE_COUNT_HIGH_HRC2    48                  // this is the HRCAP count for our HRC1 config
#define PULSE_COUNT_LOW_HRC2    47


#endif

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
#ifdef USE_HRCAP2
typedef struct HRCAP2_Queue
{

    unsigned long int_HRC2pulsewidthlow1[5];
    unsigned long frac_HRC2pulsewidthlow1[5];
    unsigned long int_HRC2pulsewidthhigh1[5];
    unsigned long frac_HRC2pulsewidthhigh1[5];
    Uint16 datacounter2;
    Uint16 HRC2_HCCAPDLYRISE0[5];
    Uint16 HRC2_HCCAPDLYFALL0[5];
//    Uint16 HRC2_HCCAPCNTRISE0[5];
//    Uint16 HRC2_HCCAPCNTFALL0[5];
    bool runECTask;

} HRCAP2Queue;

extern HRCAP2Queue HRCAP2Data;

#define HRCAP2_QUEUE_LENGTH 1
#define HRCAP2Q_ITEM_SIZE    sizeof (HRCAP2Queue)

extern void HRCAP2_Config(void);
extern __interrupt void HRCAP2_Isr (void);
extern Uint16 first2;
extern Uint16 counter2;            // Increments CMPAHR by 8 MEP steps with each period
// extern Uint16 datacounter2;        // Counts 5 periods then resets.
extern _iq pulsewidthlow1[5];
// extern unsigned long int_pulsewidthlow1[5];
// extern unsigned long frac_pulsewidthlow1[5];
extern _iq pulsewidthhigh1[5];
// extern unsigned long int_pulsewidthhigh1[5];
// extern unsigned long frac_pulsewidthhigh1[5];
extern Uint32 periodwidthhigh1[5];
extern Uint32 periodwidth1[5];
#endif

#ifdef USE_HRCAP3
// extern QueueHandle_t xHRCAP3Queue;
typedef struct HRCAP3_Queue
{

    unsigned long int_HRC3pulsewidthlow2[5];
    unsigned long frac_HRC3pulsewidthlow2[5];
    unsigned long int_HRC3pulsewidthhigh2[5];
    unsigned long frac_HRC3pulsewidthhigh2[5];

} HRCAP3Queue;

// HRCAP3Queue HRCAP3Data;
#define HRCAP3_QUEUE_LENGTH 1
#define HRCAP3Q_ITEM_SIZE    sizeof (HRCAP3Queue)

extern void HRCAP3_Config(void);
extern __interrupt void HRCAP3_Isr (void);
extern Uint16 first3;

extern Uint16 counter3;            // Increments CMPAHR by 8 MEP steps with each period
extern Uint16 datacounter3;        // Counts 5 periods then resets.
extern _iq pulsewidthlow2[5];
// extern unsigned long int_pulsewidthlow2[5];
// extern unsigned long frac_pulsewidthlow2[5];
extern _iq pulsewidthhigh2[5];
// extern unsigned long int_pulsewidthhigh2[5];
// extern unsigned long frac_pulsewidthhigh2[5];
extern Uint32 periodwidthhigh2[5];
extern Uint32 periodwidth2[5];
#endif

#ifdef USE_HRCAP4

typedef struct HRCAP4_Queue
{

    unsigned long int_HRC4pulsewidthlow3[5];
    unsigned long frac_HRC4pulsewidthlow3[5];
    unsigned long int_HRC4pulsewidthhigh3[5];
    unsigned long frac_HRC4pulsewidthhigh3[5];

} HRCAP4Queue;
// HRCAP3Queue HRCAP3Data;
#define HRCAP4_QUEUE_LENGTH 1
#define HRCAP4Q_ITEM_SIZE    sizeof (HRCAP4Queue)

extern void HRCAP4_Config(void);
extern __interrupt void HRCAP4_Isr (void);
extern Uint16 first4;
extern Uint16 counter4;            // Increments CMPAHR by 8 MEP steps with each period
extern Uint16 datacounter4;        // Counts 5 periods then resets.
// Uint32 pulsewidthlow3[5];
extern _iq pulsewidthlow3[5];
// extern unsigned long int_pulsewidthlow3[5];
// extern unsigned long frac_pulsewidthlow3[5];
// Uint32 pulsewidthhigh3[5];
extern _iq pulsewidthhigh3[5];
// extern unsigned long int_pulsewidthhigh3[5];
// extern unsigned long frac_pulsewidthhigh3[5];
extern Uint32 periodwidth3[5];
extern Uint32 periodwidthhigh3[5];
#endif

//
// __error__ - The error routine that is called if the driver library
// encounters an error.
//
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    __asm(" ESTOP0");
}
#endif

void InitAdcLocal(void);

__interrupt void adc_isr(void);

__interrupt void epwm7_timer_isr(void);

#if 1
__interrupt void spiTxFifoIsr(void);
__interrupt void spiRxFifoIsr(void);
// void delay_loop(void);
void spi_fifo_init(void);
void spi_xmit(Uint32 a, Uint8 dac_sel);		// our AD5060 DAC takes 24-bit word
// void spi_fifo_init(void);
void spi_init(void);
// void error(void);
#endif

//
// Function Prototypes - non-irq
//

Uint16 sdata[2];     // Send data buffer
Uint16 rdata[2];     // Receive data buffer
//
// Keep track of where we are in the data stream to check received data
//
/// Uint16 rdata_point;

//
// Globals
//
Uint32  ECap1IntCount;
Uint32  ECap1PassCount;
Uint32  ECap2IntCount;
Uint32  ECap2PassCount;
Uint32  ECap3IntCount;
Uint32  ECap3PassCount;
// Uint32  EPwm3TimerDirection;

//
// Function Prototypes
//
__interrupt void ecap1_isr(void);
__interrupt void ecap2_isr(void);
__interrupt void ecap3_isr(void);
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
// IMPORTANT - The following definition and struct must be defined in order to 
// use the HCCal library
//

//
//
// Globals
//
EPWM_INFO epwm7_info;
//
// Defines to keep track of which way the compare value is moving
//
#if 1
__interrupt void SysTickIntHandler(void)
{
    g_ulSysTickCount++;
//    PieCtrlRegs.PIEACK.all |= 1;
    PieCtrlRegs.PIEACK.bit.ACK1=1;
}
#endif

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    for( EVER );
}
void vConfigTimer0( void )
{
    // Start the timer than activate timer interrupt to switch into first task.
    EALLOW;
    PieVectTable.TINT0 = &timer0_ISR;
//  PieVectTable.RTOSINT = &portTICK_ISR;
    EDIS;

    ConfigCpuTimer(&CpuTimer0,
                   configCPU_CLOCK_HZ / 1000000,  // CPU clock in MHz
//                   1000000 / configTICK_RATE_HZ); // Timer period in uS
//                   1000000); // Timer period in uS    -> 1s
                     500000); // Timer period in uS    -> 500ms
//                     750000); // Timer period in uS    -> 750ms
//                   250000); // Timer period in uS    -> 250ms
    CpuTimer0Regs.TCR.all = 0x4000;               // Enable interrupt and start timer
    IntRegister(INT_TINT0, timer0_ISR);
//    IER |= M_RTOS;
    IER |= M_INT1;
}


//-------------------------------------------------------------------------------------------------
void vApplicationSetupTimerInterrupt( void )
{
	// Start the timer than activate timer interrupt to switch into first task.
	EALLOW;
	PieVectTable.TINT2 = &portTICK_ISR;
//	PieVectTable.RTOSINT = &portTICK_ISR;
	EDIS;

	ConfigCpuTimer(&CpuTimer2,
	               configCPU_CLOCK_HZ / 1000000,  // CPU clock in MHz
//				   1000000 / configTICK_RATE_HZ); // Timer period in uS
	               1000000 / configTICK_RATE_HZ); // Timer period in uS  -> 1ms
	CpuTimer2Regs.TCR.all = 0x4000;               // Enable interrupt and start timer
//	IntRegister(INT_TINT0, portTICK_ISR);
//    IER |= M_RTOS;
	IER |= M_INT14;
}
//-------------------------------------------------------------------------------------------------
static void testLedToggle(void)
{
	static uint32_t counter = 0;

	counter++;
	if(counter & 1)
	{
		GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
	}
	else
	{
		GpioDataRegs.GPASET.bit.GPIO4 = 1;
	}
}
//-------------------------------------------------------------------------------------------------
static void blueLedToggle(void)
{
	static uint32_t counter = 0;

	counter++;
	if(counter & 1)
	{
		GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
	}
	else
	{
		GpioDataRegs.GPASET.bit.GPIO3 = 1;
	}
}

//-------------------------------------------------------------------------------------------------
static void redLedToggle(void)
{
	static uint32_t counter = 0;

	counter++;
	if(counter & 1)
	{
		GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;
	}
	else
	{
		GpioDataRegs.GPBSET.bit.GPIO39 = 1;
	}
}

//-------------------------------------------------------------------------------------------------
__interrupt void timer1_ISR( void )
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

__interrupt void timer0_ISR( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#if 0
#if 0
                EALLOW;
                if ( PieCtrlRegs.PIEIER4.bit.INTx8 == 0 )
                {
                if (HRCAP2Data.runECTask==pdFALSE)
                PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 8 - Enable HRCAP2 IRQ
                }
                EDIS;
#else
                EALLOW;
                if ( PieCtrlRegs.PIEIER1.bit.INTx4 == 0 )
                {
                if (HRCAP2Data.runECTask==pdFALSE)
                PieCtrlRegs.PIEIER1.bit.INTx4=1;     // Enable PIE Group 4, INT 8 - Enable XINT1
                }
                EDIS;
#endif
#endif

//    xSemaphoreGiveFromISR( EC_TaskSemaphore, &xHigherPriorityTaskWoken );
//    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


//-------------------------------------------------------------------------------------------------
static void setupTimer1( void )
{
	// Start the timer than activate timer interrupt to switch into first task.
	EALLOW;
	PieVectTable.TINT1 = &timer1_ISR;

	EDIS;

	ConfigCpuTimer(&CpuTimer1,
	               configCPU_CLOCK_HZ / 1000000,  // CPU clock in MHz
	               1000000); 					  // Timer period in uS
	CpuTimer1Regs.TCR.all = 0x4000;               // Enable interrupt and start timer

	IER |= M_INT13;
}

//-------------------------------------------------------------------------------------------------
void LED_TaskRed(void * pvParameters)
{
	for(;;)
	{
		if(xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE)
		{
			blueLedToggle();
			vTaskDelay(500 / portTICK_PERIOD_MS);
		}
	}
}
#ifdef USE_HRCAP1
void HRCAP1_Task(void * pvParameters)
{
    HRCAP1Queue xMessage;
    QueueHandle_t xQueue;
    Uint16 status1;
    static Uint8 ui8Char, i;

    i = 0;

    xQueue = ( QueueHandle_t ) pvParameters;


	for(EVER)
	{
//	    vTaskSuspend(HRCAP1TaskHandle);

		if(xSemaphoreTake( xHRCAP1Semaphore, portMAX_DELAY ) == pdTRUE)
		{
////			testLedToggle();


#ifdef USE_HRCAP1_FOO

//      if (datacounter1 == 5)
        if ( do_cal_1 )
        {
#if 0
#ifdef USE_HRCAP2
            EALLOW;
            while ( PieCtrlRegs.PIEIER4.bit.INTx8 != 0 )    // wait for IRQ clear

//            PieCtrlRegs.PIEIER4.bit.INTx8=0;     // disable PIE Group 4, INT 7
            EDIS;

#endif
#endif
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
            status1 = HRCAP_Cal(2,HCCAPCLK_PLLCLK, &EPwm8Regs);         // Ch. 2 is connected to 'Cal'
//          status1 = HRCAP_Cal(1,HCCAPCLK_PLLCLK, &EPwm8Regs);
            if (status1 == HCCAL_ERROR)
            {
        //
        // If there is an error on HRCAP, stop and
        // check 98 MHz < PLL2CLK < 120 MHz .
        //
                ESTOP0;
            }
        }
#if 0
        EALLOW;
#if 1
#ifdef USE_HRCAP1
        HRCap1Regs.HCCTL.bit.RISEINTE = 1;
#endif
#endif
#if 0
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


//      if (datacounter1 == 5)
//              {
//                  datacounter1 = 0;
//              }
        do_cal_1 = 0;
#if 0
#ifdef USE_HRCAP2
            EALLOW;
            if ( PieCtrlRegs.PIEIER4.bit.INTx8 == 0 )    //

            PieCtrlRegs.PIEIER4.bit.INTx8=1;     // disable PIE Group 4, INT 8
            EDIS;

#endif
#endif

        }

////        HRCAP1_Config();
#endif
#if 0       /// OOPS!!!
         USBBufferFlush((tUSBBuffer *)&g_sTxBuffer);
#endif
        if( xQueueReceive( xQueue, &xMessage, portMAX_DELAY ) != pdPASS )
        {
        /* Nothing was received from the queue – even after blocking to wait
        for data to arrive. */
            DINT;
            while(1){;}
        }
        else
        {
        /* xMessage now contains the received data. */

            if ( xMessage.int_HRC1pulsewidthhigh0[0] != PULSE_COUNT_HIGH_HRC1 )
                error_count_hrc1_high++;

            if ( xMessage.int_HRC1pulsewidthlow0[0] != PULSE_COUNT_LOW_HRC1 )
                           error_count_hrc1_low++;
#if 0
//            memset(debug_string, '\0', sizeof(debug_string));
            sprintf((Uint8 *)&debug_string[0], "HRC1: H%lu, %lu, L%lu, %lu\r\n", xMessage.int_HRC1pulsewidthhigh0[i], xMessage.frac_HRC1pulsewidthhigh0[i], xMessage.int_HRC1pulsewidthlow0[i], xMessage.frac_HRC1pulsewidthlow0[i] );
//            sprintf((Uint8 *)&debug_string[0], "H%lu, %lu\r\n", xMessage.int_HRC1pulsewidthhigh1[i], xMessage.frac_HRC1pulsewidthhigh1[i]);
//            sprintf((Uint8 *)&debug_string[0], ""H%lu, %lu, L%lu, %lu\r\n", xMessage.int_HRC1pulsewidthlow0[i], xMessage.frac_HRC1pulsewidthlow0[i]);
            USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));
#endif
//            memset(debug_string, '\0', sizeof(debug_string));
//            vTaskDelay(1000 / portTICK_PERIOD_MS);
#if 0
            ui8Char = 'A';

            USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (uint8_t *)&ui8Char, 1);
#endif
//            i++;
            if ( ++i >= 5)
                i = 0;
        }



#if USE_HRCAP1
//			EALLOW;


#ifdef ENABLE_HRCAP_IRQ1
//                             PieCtrlRegs.PIEIER4.bit.INTx7=1;     // Enable PIE Group 4, INT 7
			if( xSemaphoreGive( xControl1Semaphore ) != pdTRUE )
			{
			    error();

			} else {
			    ;
			}
#endif
//            EDIS;
//			PieCtrlRegs.PIEIER4.bit.INTx7=1;     // Enable PIE Group 4, INT 7
///			PieCtrlRegs.PIEIER5.bit.INTx5=1;

#endif
            vTaskDelay(100 / portTICK_PERIOD_MS);
//            vTaskResume(ControlTaskHandle);
		}
	}
}
#endif
void Startup_Task(void * pvParameters)
{

    for (EVER)
    {
        vConfigTimer0();
//        XIntruptRegs.XINT1CR.= 1;
//        XINT1CR = 0x01;                 // enable XINT1
        vTaskResume(ECTaskHandle);

        EALLOW;
        XIntruptRegs.XINT1CR.all |= 1;
//        XIntruptRegs.
//        XIntruptRegs.XINT1CR.bit.ENABLE = 1;

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

        vTaskResume(ControlTaskHandle);


        vTaskSuspend(NULL);                     // suspend after initial start

    }
}

void Control_Task(void * pvParameters)
{
#ifdef USE_HRCAP1
    HRCAP1Queue xMessage;
#endif
    static Uint8 i;
    Uint16 status4;

	for(EVER)
	{

//	    vTaskSuspend(NULL);

#if USE_HRCAP1
#if 0
//		if(xSemaphoreTake( xControl1Semaphore, portMAX_DELAY ) == pdTRUE)
        sprintf((Uint8 *)&debug_string[0], "HRC1: H%lu, %lu, L%lu, %lu\r\n", xMessage.int_HRC1pulsewidthhigh0[i], xMessage.frac_HRC1pulsewidthhigh0[i], xMessage.int_HRC1pulsewidthlow0[i], xMessage.frac_HRC1pulsewidthlow0[i] );
//            sprintf((Uint8 *)&debug_string[0], "H%lu, %lu\r\n", xMessage.int_HRC1pulsewidthhigh1[i], xMessage.frac_HRC1pulsewidthhigh1[i]);
          USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));
#endif
		if(xSemaphoreTake( xControl1Semaphore, ControlSemaphore_DELAY ) == pdTRUE)
		{
		    EALLOW;
		    if ( PieCtrlRegs.PIEIER4.bit.INTx7 == 0 )
		    PieCtrlRegs.PIEIER4.bit.INTx7=1;     // Enable PIE Group 4, INT 7
		    EDIS;
//		    vTaskResume (HRCAP1TaskHandle);
		    control1++;
		} else {

		    EALLOW;
		    if ( PieCtrlRegs.PIEIER4.bit.INTx7 == 0 )
		    {
		    PieCtrlRegs.PIEIER4.bit.INTx7=1;     // Enable PIE Group 4, INT 7
		    } else {
		        PieCtrlRegs.PIEIER4.bit.INTx7=0;
		    }

		    EDIS;
#if 0
		    sprintf(debug_string, "HRC1: %lu, %lu, %lu, %lu\n\d", xMessage.int_HRC1pulsewidthhigh0[0], xMessage.frac_HRC1pulsewidthhigh0[0], xMessage.int_HRC1pulsewidthlow0[0], xMessage.frac_HRC1pulsewidthlow0[0] );
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (uint8_t *)&debug_string[0], strlen(debug_string));
#endif

		}
#if 0
		if ( i++ >= 5)
		                        i = 0;
#endif
#endif
//		vTaskDelay( 500 / portTICK_PERIOD_MS);
//		vTaskSuspend(NULL);
#if USE_HRCAP2
//          if(xSemaphoreTake( xControl1Semaphore, portMAX_DELAY ) == pdTRUE)
        if(xSemaphoreTake( xControl2Semaphore, ControlSemaphore_DELAY ) == pdTRUE)
        {
#if 1
            if ( do_cal_1 )
              {
                  __asm (" nop");        // Set breakpoint here for debug
      // #ifndef USE_HRCAP1
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
                  status4 = HRCAP_Cal(4,HCCAPCLK_PLLCLK, &EPwm8Regs);         // Ch. 2 is connected to 'Cal'
      //          status1 = HRCAP_Cal(1,HCCAPCLK_PLLCLK, &EPwm8Regs);
                  if (status4 == HCCAL_ERROR)
                  {
              //
              // If there is an error on HRCAP, stop and
              // check 98 MHz < PLL2CLK < 120 MHz .
              //
                      ESTOP0;
                  }
              }

              do_cal_1 = 0;
              }

#endif
#if 0
            EALLOW;
            if ( PieCtrlRegs.PIEIER4.bit.INTx8 == 0 )
            {
            if (HRCAP2Data.runECTask==pdFALSE)
            PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 7
            }
            EDIS;
#endif
//          vTaskResume (HRCAP1TaskHandle);
#if 0
// for HRCAP1 - OOB of it's timeslot
//            memset(debug_string, '\0', sizeof(debug_string));
            sprintf((Uint8 *)&debug_string[0], "HRC1: H%lu, %lu, L%lu, %lu\r\n", xMessage.int_HRC1pulsewidthhigh0[i], xMessage.frac_HRC1pulsewidthhigh0[i], xMessage.int_HRC1pulsewidthlow0[i], xMessage.frac_HRC1pulsewidthlow0[i] );
//            sprintf((Uint8 *)&debug_string[0], "H%lu, %lu\r\n", xMessage.int_HRC1pulsewidthhigh1[i], xMessage.frac_HRC1pulsewidthhigh1[i]);
              USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));
#endif
            control2++;
        } else {
            if ( do_cal_1 )
              {
                  __asm (" nop");        // Set breakpoint here for debug
      // #ifndef USE_HRCAP1
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
                  status4 = HRCAP_Cal(4,HCCAPCLK_PLLCLK, &EPwm8Regs);         // Ch. 2 is connected to 'Cal'
      //          status1 = HRCAP_Cal(1,HCCAPCLK_PLLCLK, &EPwm8Regs);
                  if (status4 == HCCAL_ERROR)
                  {
              //
              // If there is an error on HRCAP, stop and
              // check 98 MHz < PLL2CLK < 120 MHz .
              //
                      ESTOP0;
                  }
              }

              do_cal_1 = 0;
              }

#if 0
            EALLOW;
            if ( PieCtrlRegs.PIEIER4.bit.INTx8 == 0 )
            {  if (HRCAP2Data.runECTask==pdFALSE)
               {
            PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 8
               }
            } else {
                PieCtrlRegs.PIEIER4.bit.INTx8=0;
            }
            EDIS;
#endif
        }
#if 0
        if ( i++ >= 5)
                        i = 0;
#endif
#endif
#if USE_HRCAP3
//        if(xSemaphoreTake( xControl4Semaphore, portMAX_DELAY ) == pdTRUE)
        if(xSemaphoreTake( xControl3Semaphore, ControlSemaphore_DELAY ) == pdTRUE)
        {
            EALLOW;
            if ( PieCtrlRegs.PIEIER5.bit.INTx4 == 0)
            PieCtrlRegs.PIEIER5.bit.INTx4=1;     // Enable PIE Group 4, INT 8
//            vTaskResume (HRCAP4TaskHandle);
            EDIS;
            control3++;
        } else {
            EALLOW;
            if ( PieCtrlRegs.PIEIER5.bit.INTx4 == 0 )
            {
            PieCtrlRegs.PIEIER5.bit.INTx4=1;     // Enable PIE Group 4, INT 7
            } else {
                PieCtrlRegs.PIEIER5.bit.INTx4=0;
            }
            EDIS;
        }
#endif
#if USE_HRCAP4
//        if(xSemaphoreTake( xControl4Semaphore, portMAX_DELAY ) == pdTRUE)
        if(xSemaphoreTake( xControl4Semaphore, ControlSemaphore_DELAY ) == pdTRUE)
        {
            EALLOW;
            if ( PieCtrlRegs.PIEIER5.bit.INTx5 == 0)
            PieCtrlRegs.PIEIER5.bit.INTx5=1;     // Enable PIE Group 5, INT 5
//            vTaskResume (HRCAP4TaskHandle);
            EDIS;
            control4++;
        } else {
            EALLOW;
            if ( PieCtrlRegs.PIEIER5.bit.INTx5 == 0 )
            {
            PieCtrlRegs.PIEIER5.bit.INTx5=1;     // Enable PIE Group 5, INT 5
            } else {
                PieCtrlRegs.PIEIER5.bit.INTx5=0;
            }

            EDIS;
        }
#endif
#if 0
        if( xSemaphoreGive( EC_TaskSemaphore ) != pdTRUE )
        {
        /* This call should fail because the semaphore has not yet been
        ‘taken’. */
         DINT;
         while(1);;
        } else
            ECcontrol++;

        if(xSemaphoreTake( xControlECTaskSemaphore, ControlSemaphore_DELAY ) == pdTRUE)
        {
            ;
        }
#endif

//        UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, false));
//        USBBufferInit((tUSBBuffer *)&g_sTxBuffer);
        testLedToggle();
		vTaskDelay( 10 / portTICK_PERIOD_MS);

	}
}
#ifdef USE_HRCAP4
void HRCAP4_Task(void * pvParameters)
{
     HRCAP4Queue xMessage;
     QueueHandle_t xQueue;
     Uint16 status4;

     xQueue = ( QueueHandle_t ) pvParameters;

	for(EVER)
	{
///	    vTaskSuspend(HRCAP4TaskHandle);

		if(xSemaphoreTake( xHRCAP4Semaphore, portMAX_DELAY ) == pdTRUE)
		{
			testLedToggle();


#ifdef USE_HRCAP4

//      if (datacounter1 == 5)
//      if ( do_cal_1 )
        {
            __asm (" nop");        // Set breakpoint here for debug
#ifndef USE_HRCAP1
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
            status4 = HRCAP_Cal(2,HCCAPCLK_PLLCLK, &EPwm8Regs);         // Ch. 2 is connected to 'Cal'
//          status1 = HRCAP_Cal(1,HCCAPCLK_PLLCLK, &EPwm8Regs);
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
#if 0
        EALLOW;
#if 1
#ifdef USE_HRCAP1
        HRCap1Regs.HCCTL.bit.RISEINTE = 1;
#endif
#endif
#if 0
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

//      if (datacounter1 == 5)
//              {
//                  datacounter1 = 0;
//              }
//        do_cal_1 = 0;
        }

////        HRCAP1_Config();
#endif
        if( xQueueReceive( xQueue, &xMessage, portMAX_DELAY ) != pdPASS )
         {
         /* Nothing was received from the queue – even after blocking to wait
         for data to arrive. */
             DINT;
             while(1){;}
         }
         else
         {
         /* xMessage now contains the received data. */
             ;
         }

#if USE_HRCAP4	

//            EALLOW;
#ifdef ENABLE_HRCAP_IRQ4

//                             PieCtrlRegs.PIEIER5.bit.INTx5=1;     // Enable PIE Group 4, INT 7
            if( xSemaphoreGive( xControl4Semaphore ) != pdTRUE )
                        {
                            error();

                        } else {
                            ;
                        }
#endif
//            EDIS;
//			PieCtrlRegs.PIEIER5.bit.INTx5=1;
//			PieCtrlRegs.PIEIER4.bit.INTx7=1;     // Enable PIE Group 4, INT 7

#endif
            vTaskDelay(100 / portTICK_PERIOD_MS);
///			vTaskResume(ControlTaskHandle);
		}
	}
}
#endif
#ifdef ENABLE_HRCAP_IRQ3
void HRCAP3_Task(void * pvParameters)
{
     HRCAP3Queue xMessage;
     QueueHandle_t xQueue;
     Uint16 status4;

     xQueue = ( QueueHandle_t ) pvParameters;

	for(EVER)
	{
///	    vTaskSuspend(HRCAP4TaskHandle);

		if(xSemaphoreTake( xHRCAP3Semaphore, portMAX_DELAY ) == pdTRUE)
		{
//			testLedToggle();


#ifdef USE_HRCAP3

//      if (datacounter1 == 5)
//      if ( do_cal_1 )
        {
            __asm (" nop");        // Set breakpoint here for debug
#ifndef USE_HRCAP1
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
            status4 = HRCAP_Cal(2,HCCAPCLK_PLLCLK, &EPwm8Regs);         // Ch. 2 is connected to 'Cal'
//          status1 = HRCAP_Cal(1,HCCAPCLK_PLLCLK, &EPwm8Regs);
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
#if 0
        EALLOW;
#if 1
#ifdef USE_HRCAP1
        HRCap1Regs.HCCTL.bit.RISEINTE = 1;
#endif
#endif
#if 0
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


//      if (datacounter1 == 5)
//              {
//                  datacounter1 = 0;
//              }
//        do_cal_1 = 0;
        }

////        HRCAP1_Config();
#endif
        if( xQueueReceive( xQueue, &xMessage, portMAX_DELAY ) != pdPASS )
          {
          /* Nothing was received from the queue – even after blocking to wait
          for data to arrive. */
              DINT;
              while(1){;}
          }
          else
          {
          /* xMessage now contains the received data. */
              ;
          }

#if USE_HRCAP3

//            EALLOW;
#ifdef ENABLE_HRCAP_IRQ3
//                             PieCtrlRegs.PIEIER5.bit.INTx4=1;     // Enable PIE Group 4, INT 7

            if( xSemaphoreGive( xControl3Semaphore ) != pdTRUE )
                        {
                            error();

                        } else {
                            ;
                        }


#endif
//            EDIS;
//			PieCtrlRegs.PIEIER5.bit.INTx5=1;
//			PieCtrlRegs.PIEIER4.bit.INTx7=1;     // Enable PIE Group 4, INT 7

#endif
            vTaskDelay(1000 / portTICK_PERIOD_MS);



///			vTaskResume(ControlTaskHandle);
		}
	}
}
#endif
#ifdef USE_HRCAP2
void HRCAP2_Task(void * pvParameters)
{
// #ifdef USE_HRCAP2
    HRCAP2Queue xMessage;           // pass as arg...
// #endif
    QueueHandle_t xQueue;

    Uint16 status4;
    static Uint8 i;

    xQueue = ( QueueHandle_t ) pvParameters;

    i = 0;
    xMessage.runECTask = pdFALSE;         // init to false

	for(EVER)
	{
///	    vTaskSuspend(HRCAP4TaskHandle);

		if(xSemaphoreTake( xHRCAP2Semaphore, portMAX_DELAY ) == pdTRUE)
		{
///			testLedToggle();

#if 0                                           // HRCAP cal function is in control task
#ifdef USE_HRCAP2

//      if (datacounter1 == 5)
// #if 0
      if ( do_cal_1 )
        {
            __asm (" nop");        // Set breakpoint here for debug
// #ifndef USE_HRCAP1
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
            status4 = HRCAP_Cal(4,HCCAPCLK_PLLCLK, &EPwm8Regs);         // Ch. 2 is connected to 'Cal'
//          status1 = HRCAP_Cal(1,HCCAPCLK_PLLCLK, &EPwm8Regs);
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
#if 0
        EALLOW;
#if 1
#ifdef USE_HRCAP1
        HRCap1Regs.HCCTL.bit.RISEINTE = 1;
#endif
#endif
#if 0
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


//      if (datacounter1 == 5)
//              {
//                  datacounter1 = 0;
//              }
        do_cal_1 = 0;
        }

////        HRCAP1_Config();
#endif
#if 0
        USBBufferFlush((tUSBBuffer *)&g_sTxBuffer);
        memset(debug_string, '\0', sizeof(debug_string));
#endif
        if( xQueueReceive( xQueue, &xMessage, portMAX_DELAY ) != pdPASS )
           {
           /* Nothing was received from the queue – even after blocking to wait
           for data to arrive. */
               DINT;
               while(1){;}
           }
           else
           {
           /* xMessage now contains the received data. */
               /* xMessage now contains the received data. */
#if 0
                    if ( xMessage.int_HRC2pulsewidthhigh1[i] < PULSE_COUNT_LOW_HRC2 || xMessage.int_HRC2pulsewidthhigh1[i] > PULSE_COUNT_HIGH_HRC2 )
                    {
                        sprintf((Uint8 *)&debug_string[0], "NS:H%lu,L%lu,H%lu,L%lu,H%lu,L%lu,H%lu,L%lu,H%lu,L%lu ", xMessage.int_HRC2pulsewidthhigh1[0], xMessage.int_HRC2pulsewidthlow1[0], xMessage.int_HRC2pulsewidthhigh1[1], xMessage.int_HRC2pulsewidthlow1[1], xMessage.int_HRC2pulsewidthhigh1[2], xMessage.int_HRC2pulsewidthlow1[2], xMessage.int_HRC2pulsewidthhigh1[3], xMessage.int_HRC2pulsewidthlow1[3], xMessage.int_HRC2pulsewidthhigh1[4], xMessage.int_HRC2pulsewidthlow1[4] );
//                        sprintf((Uint8 *)&debug_string[0], "NS:H%lu,L%lu ", xMessage.int_HRC2pulsewidthhigh1[i], xMessage.int_HRC2pulsewidthlow1[i] );//  , xMessage.frac_HRC1pulsewidthhigh0[i], xMessage.int_HRC1pulsewidthlow0[i], xMessage.frac_HRC1pulsewidthlow0[i] );
                                //            sprintf((Uint8 *)&debug_string[0], "H%lu, %lu\r\n", xMessage.int_HRC1pulsewidthhigh1[i], xMessage.frac_HRC1pulsewidthhigh1[i]);
                                //            sprintf((Uint8 *)&debug_string[0], ""H%lu, %lu, L%lu, %lu\r\n", xMessage.int_HRC1pulsewidthlow0[i], xMessage.frac_HRC1pulsewidthlow0[i]);

                    }  else {
                        sprintf((Uint8 *)&debug_string[0], "<*PHASE LOCK*>:S:H%lu,L%lu,H%lu,L%lu,H%lu,L%lu,H%lu,L%lu,H%lu,L%lu ", xMessage.int_HRC2pulsewidthhigh1[0], xMessage.int_HRC2pulsewidthlow1[0], xMessage.int_HRC2pulsewidthhigh1[1], xMessage.int_HRC2pulsewidthlow1[1], xMessage.int_HRC2pulsewidthhigh1[2], xMessage.int_HRC2pulsewidthlow1[2], xMessage.int_HRC2pulsewidthhigh1[3], xMessage.int_HRC2pulsewidthlow1[3], xMessage.int_HRC2pulsewidthhigh1[4], xMessage.int_HRC2pulsewidthlow1[4] );
//                        sprintf((Uint8 *)&debug_string[0], "S:H%lu,L%lu ", xMessage.int_HRC2pulsewidthhigh1[i], xMessage.int_HRC2pulsewidthlow1[i] );
//                        error_count_hrc1_high++;
                    }

                        USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));
#endif

//                    if ( xMessage.int_HRC2pulsewidthlow0[i] != PULSE_COUNT_LOW_HRC1 )
//                                   error_count_hrc1_low++;
        #if 0
        //            memset(debug_string, '\0', sizeof(debug_string));
                    sprintf((Uint8 *)&debug_string[0], "HRC1: H%lu, %lu, L%lu, %lu\r\n", xMessage.int_HRC1pulsewidthhigh0[i], xMessage.frac_HRC1pulsewidthhigh0[i], xMessage.int_HRC1pulsewidthlow0[i], xMessage.frac_HRC1pulsewidthlow0[i] );
        //            sprintf((Uint8 *)&debug_string[0], "H%lu, %lu\r\n", xMessage.int_HRC1pulsewidthhigh1[i], xMessage.frac_HRC1pulsewidthhigh1[i]);
        //            sprintf((Uint8 *)&debug_string[0], ""H%lu, %lu, L%lu, %lu\r\n", xMessage.int_HRC1pulsewidthlow0[i], xMessage.frac_HRC1pulsewidthlow0[i]);
                    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));
        #endif

//               ;

                    if ( ++i >= 5)
                       i = 0;

                    if ( xMessage.runECTask == pdTRUE )                                                     // boolean
                    {

//                        vTaskResume(ECTaskHandle);

                     // ok - safe to do average here now - 5 values taken per average.
#ifdef USE_HRCAP2_AVERAGES
                     control_valL = (( xMessage.int_HRC2pulsewidthlow1[0] + xMessage.int_HRC2pulsewidthlow1[1] + xMessage.int_HRC2pulsewidthlow1[2] + xMessage.int_HRC2pulsewidthlow1[3] + xMessage.int_HRC2pulsewidthlow1[4] ) / 5) ;
                     control_valH = (( xMessage.int_HRC2pulsewidthhigh1[0] + xMessage.int_HRC2pulsewidthhigh1[1] + xMessage.int_HRC2pulsewidthhigh1[2] + xMessage.int_HRC2pulsewidthhigh1[3] + xMessage.int_HRC2pulsewidthhigh1[4] ) / 5) ;
#else
                     control_valL = (( xMessage.int_HRC2pulsewidthlow1[HRCAP2Data.datacounter2-1] )); //  + xMessage.int_HRC2pulsewidthlow1[1] + xMessage.int_HRC2pulsewidthlow1[2] + xMessage.int_HRC2pulsewidthlow1[3] + xMessage.int_HRC2pulsewidthlow1[4] ) / 5) ;
                     control_valH = (( xMessage.int_HRC2pulsewidthhigh1[HRCAP2Data.datacounter2-1] )); //  + xMessage.int_HRC2pulsewidthhigh1[1] + xMessage.int_HRC2pulsewidthhigh1[2] + xMessage.int_HRC2pulsewidthhigh1[3] + xMessage.int_HRC2pulsewidthhigh1[4] ) / 5) ;
#endif
#if 1
                     if( xSemaphoreGive( EC_TaskSemaphore ) != pdTRUE )
                     {
                            /* This call should fail because the semaphore has not yet been
                            ‘taken’. */
                             DINT;
                             while(1);
                     } else  {
                                ECcontrol++;

                     }
#endif

                            if(xSemaphoreTake( xControlECTaskSemaphore, portMAX_DELAY ) == pdTRUE)
//                             if(xSemaphoreTake( xControlECTaskSemaphore, ControlSemaphore_DELAY ) == pdTRUE)             // portMAX_DELAY
                            {
//                                xMessage.runECTask = pdFALSE;
                                 ;
                            }
                    }
           }

#if USE_HRCAP2	

//            EALLOW;
#ifdef ENABLE_HRCAP_IRQ2


//                             PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 7
#if 1
       if ( xMessage.runECTask == pdFALSE ) {

            if( xSemaphoreGive( xControl2Semaphore ) != pdTRUE )
                        {
                            error();

                        } else {
                            ;
                        }
       }
#endif
#endif
//            EDIS;
//			PieCtrlRegs.PIEIER5.bit.INTx5=1;
//			PieCtrlRegs.PIEIER4.bit.INTx7=1;     // Enable PIE Group 4, INT 7

#endif
#if 0
            if ( xMessage.runECTask == pdTRUE ){
                xMessage.runECTask = pdFALSE;
            }
#endif
// #ifdef DELAY_MAIN_HRCAP_CONTROL_LOOP
           vTaskDelay(10 / portTICK_PERIOD_MS);
// #endif
///			vTaskResume(ControlTaskHandle);
		}            // semaphore take
	}   // end (ever)
}       // end task hrcap2
#endif
//-------------------------------------------------------------------------------------------------
void LED_TaskBlue(void * pvParameters)
{
	for(;;)
	{
		redLedToggle();
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

//-------------------------------------------------------------------------------------------------
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
	*ppxIdleTaskTCBBuffer = &idleTaskBuffer;
	*ppxIdleTaskStackBuffer = idleTaskStack;
	*pulIdleTaskStackSize = STACK_SIZE;
}
//-------------------------------------------------------------------------------------------------
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{

    DINT;
    taskDISABLE_INTERRUPTS();
    while(1);
}


//-------------------------------------------------------------------------------------------------
void main(void)
{

    Uint16 status2;
    Uint16 status3;
    Uint16 status4;
    unsigned char i;

#ifdef FLASH
         // Copy time critical code and Flash setup code to RAM
        // This includes the following ISR functions: epwm1_timer_isr(),
        // epwm2_timer_isr(), epwm3_timer_isr and and InitFlash();
        // The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
        // symbols are created by the linker. Refer to the F2808.cmd file.
        //
        memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
        memcpy(&powfRunStart, &powfStart, (Uint32)&powfLoadSize);
        memcpy(&fpuRunStart, &fpuTablesSStart, (Uint32)&fpuLoadSize);
        //
        // Call Flash Initialization to setup flash waitstates
        // This function must reside in RAM
        //
        InitFlash();
#endif //(FLASH)

// RTOS reserves the following memory space:  (all PAGE = 1)
//    RAML5                 0000c000   00002000  00000000  00002000  RWIX
//    RAML6                 0000e000   00002000  00000000  00002000  RWIX
//    RAML7                 00010000   00002000  00000000  00002000  RWIX

    HeapRegion_t xHeapRegions[] =
    {
    { ( uint8_t * ) 0x0000c000UL, 0x06000 }, // << Defines a block of 0x10000 bytes starting at address 0x80000000
    // { ( uint8_t * ) 0x90000000UL, 0xa0000 }, // << Defines a block of 0xa0000 bytes starting at address of 0x90000000
    { NULL, 0 }              //   << Terminates the array.
    };
    DINT;
    DisableDog();
    vPortDefineHeapRegions( xHeapRegions );

    SysCtrlInit();
    InitCpuTimers();
    themis_gpio_init();   // initialize themis gpio

#ifdef    USE_USB
    themis_usb_gpio_init();
#endif

    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = DISABLE;                // ENABLE MEMS OSC - 90 MHz
      GpioDataRegs.GPASET.bit.GPIO8 = ENABLE;
    // GpioCtrlRegs.GPADIR.bit.GPIO8 = DISABLE;
    //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
     GpioCtrlRegs.GPAPUD.bit.GPIO8 = ENABLE;

    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = DISABLE;                  // XCLKIN
   // GpioDataRegs.GPASET.bit.GPIO8 = DISABLE;
        GpioCtrlRegs.GPADIR.bit.GPIO19 = DISABLE;
      //  GpioDataRegs.GPADAT.bit.GPIO8 = ENABLE;
        GpioCtrlRegs.GPAPUD.bit.GPIO19 = ENABLE;                // this means NO pullup is enabled
        GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
   EDIS;
   SysCtrlInitA();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
//    UARTEnable(UART1_BASE);
//    themis_uart_gpio_init();
    InitScibGpio();

//    Uint8 * debug_string;

//    memcpy(&powfRunStart, &powfStart, (Uint32)&powfLoadSize);
//    memcpy(&fpuRunStart, &fpuTablesSStart, (Uint32)&fpuLoadSize);

     debug_string = (Uint8 *)pvPortMalloc ( sizeof(char) * 128 );       // allocate 128 bytes

     if ( debug_string == NULL )
         error();

     memset((Uint8 *)debug_string, '\0', sizeof(debug_string));

//     *addr_a = 0xaa55;

/*
     typedef struct Duty_Log
     {

        float Duty_data[10];

     } DutyLog;

     DutyLog DutyData;
*/
     DutyData = (struct Duty_Log * ) pvPortMalloc ( sizeof (struct Duty_Log) );

     if ( DutyData == NULL )
              error();

          memset((struct Duty_Log * )DutyData, '\0', sizeof(struct Duty_Log));

    SNTPpkt = ( struct pkt *) pvPortMalloc ( sizeof (struct pkt) );

    if ( SNTPpkt == NULL )
            error();

    memset((struct pkt *)SNTPpkt, '\0', sizeof(struct pkt));


#ifdef USE_ALTERNATE_PID
     
     controlPID = (struct _pid *) pvPortMalloc ( sizeof(struct _pid));
//       controlPID = (struct _pid *) malloc (sizeof(struct _pid));

     if ( controlPID == NULL )
	     error();

     memset((struct _pid *)controlPID, '\0', sizeof(struct _pid));

     (controlPID->pv) = (float *) pvPortMalloc (sizeof(float *));
     (controlPID->sp) = (float *) pvPortMalloc (sizeof(float *));

     if ( (controlPID->pv) == NULL )
              error();
     if ( (controlPID->sp) == NULL )
                   error();
#endif     
#ifdef RUN_NON_LINEAR_PID
//     DCL_NLPID nlpid1 = NLPID_DEFAULTS;
     nlpid1 =  (struct _nlpid *) pvPortMalloc (sizeof(struct _nlpid));

     if ( nlpid1 == NULL )
              error();

     memset((struct _nlpid *)nlpid1, '\0', sizeof(struct _nlpid));

//     nlpid1 = NLPID_DEFAULTS;

//     DCL_NLPID_SPS *sps1;
     // DCL_CSS css1 = DCL_CSS_DEFAULTS;
//     DCL_CSS *css1;

     (nlpid1->sps) = (struct _nlpid_sps *) pvPortMalloc(sizeof (struct _nlpid_sps));

     if (nlpid1->sps == NULL)
           error();

     (nlpid1->css) = (struct _dcl_css *) pvPortMalloc(sizeof (struct _dcl_css));

     if (nlpid1->css == NULL)
           error();
#endif
//     nlpid1 = NLPID_DEFAULTS;
#ifdef RUN_LINEAR_PID
//     DCL_NLPID nlpid1 = NLPID_DEFAULTS;
     pid1 =  (struct _lpid *) pvPortMalloc (sizeof(struct _lpid));

     if ( pid1 == NULL )
              error();

     memset((struct _lpid *)pid1, '\0', sizeof(struct _lpid));

//     nlpid1 = NLPID_DEFAULTS;

//     DCL_NLPID_SPS *sps1;
     // DCL_CSS css1 = DCL_CSS_DEFAULTS;
//     DCL_CSS *css1;

     (pid1->sps) = (struct _lpid_sps *) pvPortMalloc(sizeof (struct _lpid_sps));

     if (pid1->sps == NULL)
           error();

     (pid1->css) = (struct _dcl_css *) pvPortMalloc(sizeof (struct _dcl_css));

     if (pid1->css == NULL)
           error();

#endif


    //
      // Disable CPU interrupts and clear all CPU interrupt flags
      //
      InitPieCtrl();
      IER = 0x0000;
      IFR = 0x0000;
      //
      // Initially disable time-critical interrupts
      //
      SetDBGIER(0x0000);          // PIE groups time-critical designation

      //
      InitPieVectTable();

///      InitSysCtrl();
//      SysCtrlInit();
#if 0
#ifdef USE_USB
     EALLOW;
    //
    // Enable USB Clock
    //
    SysCtrlRegs.PCLKCR3.bit.USB0ENCLK = 1;

    //
    // Enable PHY
    //
    GpioCtrlRegs.GPACTRL2.bit.USB0IOEN = 1;
    EDIS;
#endif
#endif
    //    InitAdc();         // For this example, init the ADC
    //    AdcOffsetSelfCal();

	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the F2837xS_SysCtrl.c file.

	// Step 2. Initialize GPIO:
	// This example function is found in the F2837xS_Gpio.c file and
	// illustrates how to set the GPIO to it's default state.

//    InitGpio();

#if 0
    EALLOW;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;
    EDIS;
#endif

	// Step 3. Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
///    DINT;

	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the F2837xS_PieCtrl.c file.
    InitSpiaGpio();                 //
#ifndef UBLOX_USB_TEST_ONLY
    InitSciaGpio();                 // to/from Bluetooth Radio
#endif
//    InitScibGpio();
    InitECap1Gpio();

	// Disable CPU interrupts and clear all CPU interrupt flags:
//    IER = 0x0000;
//    IFR = 0x0000;
    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
#ifndef UBLOX_USB_TEST_ONLY
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.SCIRXINTA = &sciaRxFifoIsr;
    PieVectTable.SCITXINTA = &sciaTxFifoIsr;
#if 1
    PieVectTable.SCIRXINTB = &scibRxFifoIsr;
#endif
    EDIS;   // This is needed to disable write to EALLOW prote
#endif
    //
    // Init send data.  After each transmission this data
    // will be updated for the next transmission
    //
    for(i = 0; i<2; i++)
    {
        sdataA[i] = i;
    }

    rdata_pointA = sdataA[0];
	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in F2837xS_DefaultIsr.c.
	// This function is found in F2837xS_PieVect.c.
///    InitPieVectTable();
///    InitPeripheralClocks();
    // Enable global Interrupts and higher priority real-time debug events:
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
    PieVectTable.ECAP3_INT = &ecap3_isr;
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

    LoopCount = 0;
    ConversionCount = 0;

    //
    // Configure ADC
    //
    EALLOW;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1; // Enable non-overlap mode

// ADC channels:
//    0h ADCINA0 - OA_IN_MEASURE
//    1h ADCINA1 - SUMMER_IN
//    2h ADCINA2 - OCXO_OUT_MEASURE
//    3h ADCINA3 - 1PPS_IN_MEASURE
//    4h ADCINA4 - RES PACK
//    5h ADCINA5 - RES PACK
//    6h ADCINA6 - RES PACK - TP34
//    7h ADCINA7 - RES PACK
//    8h ADCINB0 - SUMMER_OUT
//    9h ADCINB1 - TP36
//    Ah ADCINB2 - 10MHZ_CLKIN_MEAS
//    Bh ADCINB3 - TUNE_VOLTAGE_IN
//    Ch ADCINB4 - RES PACK
//    Dh ADCINB5 - RES PACK
//    Eh ADCINB6 - RES PACK - TP35
//    Fh ADCINB7 - RES PACK

    //
    // ADCINT1 trips after AdcResults latch
    //
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    AdcRegs.INTSEL1N2.bit.INT1E     = 1;  // Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;  // Disable ADCINT1 Continuous mode

    //
    // setup EOC1 to trigger ADCINT1 to fire
    //
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 1;

    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 0x02;  // set SOC0 channel select to ADCINA2 - OCXO_OUT_MEASURE
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 0x0b;  // set SOC1 channel select to ADCINB3 - TUNE_VOLTAGE_IN  <HERE!!!>

    AdcRegs.ADCSOC2CTL.bit.CHSEL = 0x00;
    AdcRegs.ADCSOC3CTL.bit.CHSEL = 0x01;
    AdcRegs.ADCSOC4CTL.bit.CHSEL = 0x03;
    AdcRegs.ADCSOC5CTL.bit.CHSEL = 0x08;
    AdcRegs.ADCSOC6CTL.bit.CHSEL = 0x09;
    AdcRegs.ADCSOC7CTL.bit.CHSEL = 0x0a;

    //    0h ADCINA0 - OA_IN_MEASURE
    //    1h ADCINA1 - SUMMER_IN

    //
    // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1
    //
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 0x0;       //  5;
    //
    // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1
    //
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 0x00;    // 5;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 0x00;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 0x00;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL  = 0x00;
    AdcRegs.ADCSOC6CTL.bit.TRIGSEL  = 0x00;
    AdcRegs.ADCSOC7CTL.bit.TRIGSEL  = 0x00;

    //
    // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 0x3f;

    //
    // set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 0x3f; // 6;

    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 0x3f;
    AdcRegs.ADCSOC3CTL.bit.ACQPS    = 0x3f;
    AdcRegs.ADCSOC4CTL.bit.ACQPS    = 0x3f;
    AdcRegs.ADCSOC5CTL.bit.ACQPS    = 0x3f;
    AdcRegs.ADCSOC6CTL.bit.ACQPS    = 0x3f;
    AdcRegs.ADCSOC7CTL.bit.ACQPS    = 0x3f;
    EDIS;

    //
     // Enable the UART.
     //
//     SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    InitEPwm7Gpio();

    // For this example, only initialize the ePWM
    // Step 5. User specific code, enable interrupts:
    //
#if 1
#ifdef DAC_PWM
    EALLOW;
    PieVectTable.EPWM7_INT = &FreqCtlISR;
    EDIS;
#endif
    status = SFO_INCOMPLETE;
    //
    // Calling SFO() updates the HRMSTEP register with calibrated
    // MEP_ScaleFactor. HRMSTEP must be populated with a scale factor value
    // prior to enabling high resolution period control.
    //
    while  (status== SFO_INCOMPLETE)
    {
        //
        // Call until complete
        //
        status = SFO();
        if (status == SFO_ERROR)
        {
            //
            // SFO function returns 2 if an error occurs & # of MEP
            // steps/coarse step exceeds maximum of 255.
            //
            error();
        }
    }

    //
    // ePWM and HRPWM register initialization
    //
    HRPWM_ConfigLocal(Period);

    //
    // Configure ePWM1 to generate interrupts on period match
    //
    EPwm7Regs.ETSEL.bit.INTSEL = 1;     // interrupt on counter zero match
    EPwm7Regs.ETSEL.bit.INTEN = 1;      // enable peripheral interrupt
    EPwm7Regs.ETPS.bit.INTPRD = 1;      // generate interrupt on every event

#endif
    HRPWM_ConfigLocal2(9);          // approx. 10 MHz using SYSCLK of 30*3=90MHz
//      HRPWM_ConfigLocal2(6000);       // 15kHz
//        HRPWM_ConfigLocal2(90);         // 1MHz
//          HRPWM_ConfigLocal2(90000);      // ~ 1kHz
//            HRPWM_ConfigLocal2(9000);       // 10kHz

#if 0
#ifdef DAC_PWM
     EALLOW;  // This is needed to write to EALLOW protected registers
     PieVectTable.EPWM7_INT = &epwm7_timer_isr;
     EDIS;    // This is needed to disable write to EALLOW protected registers
#endif
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
     HRCAP2Data.datacounter2 = 0;  // marks how many pulses have been captured
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
#if 0
#ifndef UBLOX_USB_TEST_ONLY
     InitEPwmTimer();    // For this example, only initialize the ePWM Timer();
#endif
#endif
     EALLOW;
     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
     EDIS;
#endif
     EALLOW;
     PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
     EDIS;
#ifdef DAC_PWM
     IER |= M_INT3;
     //
      // Enable EPWM INTn in the PIE: Group 3 interrupt 1-6
      //
///
#if 0
#ifndef UBLOX_USB_TEST_ONLY
     EALLOW;
     PieCtrlRegs.PIEIER3.bit.INTx7 = PWM7_INT_ENABLE;
     EDIS;
#endif
#endif
    //
    // Initalize counters
    //
     EPwm7TimerIntCount = 0;


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
      spi_init();         // init SPI
#ifndef UBLOX_USB_TEST_ONLY
      scia_fifo_init();  // Init SCI-A
#if 0
      scib_fifo_init();  // Init SCI-B
#endif
#endif
 //    UARTStdioInit(1);
     //
     // Enable and configure the UART RX and TX pins
     //
 //          USBGPIOEnable();

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
//      EALLOW;
//		       GpioDataRegs.GPBTOGGLE.bit.GPIO51;
#if 0
      GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
      GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
      GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
      GpioDataRegs.GPBDAT.bit.GPIO51 = DISABLE;
//		                    GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
		      //             GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

		      //             GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
#endif
//      EDIS;

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
//      DSP28x_usDelay(2);
 //   InitEPwmTimer();    // For this example, only initialize the ePWM Timer
    // Enable interrupts required for this example
//      EALLOW;
//      PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
//      PieCtrlRegs.PIEIER6.bit.INTx1=1;     // Enable PIE Group 6, INT 1
//      PieCtrlRegs.PIEIER6.bit.INTx2=1;     // Enable PIE Group 6, INT 2
//      EDIS;

			 //
			  // Enable ADCINT1 in PIE
			  //
//      PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
//                          IER |= M_INT1;                     // Enable CPU Interrupt 1
//                          EINT;                              // Enable Global interrupt INTM
//                          ERTM;                              // Enable Global realtime interrupt DBGM


//                         EDIS;

 //   InitAdc();         // For this example, init the ADC
 //   AdcOffsetSelfCal();
//      IER|= M_INT1 | M_INT5 | M_INT9 | M_INT3;                         // Enable CPU INT4

//      IER|=0x20;                            // Enable CPU INT6
//                          EINT;
//      EDIS;

    //
    // Enable Device Mode
    //
#if 0
 //   Gpio_select();                          // init GPIO
      IntRegister(INT_TINT0, SysTickIntHandler);
    //
    // Set the system tick to fire 100 times per second.
    //

      SysTickPeriodSet(SysCtlClockGet(SYSTEM_CLOCK_SPEED) / SYSTICKS_PER_SECOND);
      SysTickIntEnable();
      SysTickEnable();
#endif
    //
    // Enable the UART.
    //

///    UARTStdioInit(1);
    //
    // Enable and configure the UART RX and TX pins
    //
//          USBGPIOEnable();
      
#ifdef USE_USB
       DSP28x_usDelay(2);
       EALLOW;
       HWREG(USBMODESEL) = USBMODESEL_DEV;
       HWREG(USB0_BASE + USB_O_GPCS) = USBGPCS_DEV;
       EDIS;
       DSP28x_usDelay(2);

         //
         // Register interrupt handlers
         //
//         IntRegister(INT_SCIRXINTB, USBUARTRXIntHandler);
 //        IntRegister(USBUARTRXIntHandler,INT_SCIRXINTB);
 //        IntRegister(INT_SCIRXINTB, USBUARTTXIntHandler);
         IntRegister(INT_SCITXINTB, USBUARTTXIntHandler);

         //
         // Configure the required pins for USB operation.
         //
         USBGPIOEnable();
         USBIntRegister(USB0_BASE, f28x_USB0DeviceIntHandler);

         //
         // Set the default UART configuration.   SYSTEM_CLOCK_SPEED
         //
#if 0
         UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED_LOCAL)/4.5,             // changed to MEMs - to maintain ratio
                             19200, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE |                    // 115200
                             UART_CONFIG_STOP_ONE);
#endif

#if 1
         UARTConfigSetExpClk(UART1_BASE, LSPCLK_FREQ,                            // #define LSPCLK_FREQ 20E6
                             115200, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE |                    // 115200
                             UART_CONFIG_STOP_ONE);
#endif
#if 0
      scib_fifo_init();  // Init SCI-B
#endif
//         UARTFIFOEnable(UART1_BASE);
         UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
//         UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
// #endif
         //
         // Configure and enable UART interrupts.
         //
 #if 1
         UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, false));
         UARTIntEnable(UART1_BASE, (UART_INT_RXERR | UART_INT_RXRDY_BRKDT |
                       UART_INT_TXRDY ));
//         UARTRXIntRegister(UART1_BASE, USBUARTRXIntHandler);      -> these only work with UART0 for the included lib - easy to fix.
//         UARTTXIntRegister(UART1_BASE, USBUARTTXIntHandler);
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
#endif

#if 1
 #ifdef ENABLE_SCITXRXINT_IRQS
     EALLOW;
     PieCtrlRegs.PIEIER9.bit.INTx4=1;     // Enable PIE Group 9, INT 4
     PieCtrlRegs.PIEIER9.bit.INTx3=1;     // Enable PIE Group 9, INT 3
     EDIS;
 #endif
#endif

#if 0
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
                                                     GpioCtrlRegs.GPAPUD.bit.GPIO1 = ENABLE;

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



                                            // EINT;
                                                     EDIS;
#endif

	// Initialize variables
	//
//						     InitHRCapGpio();
///                             InitEPwm7Gpio();
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


						     
//                                                     EALLOW;
//                                                      PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
                                                      PieCtrlRegs.PIEIER6.bit.INTx1=1;     // Enable PIE Group 6, INT 1
                                                      PieCtrlRegs.PIEIER6.bit.INTx2=1;     // Enable PIE Group 6, INT 2
                                                      //
//                                                      PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
                                                      PieCtrlRegs.PIEIER9.bit.INTx1=1;     // PIE Group 9, INT1
                                                      PieCtrlRegs.PIEIER9.bit.INTx2=1;     // PIE Group 9, INT2



                                                      //
                                                       // Enable ADCINT1 in PIE
                                                       //
                                                       PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
                             //                          IER |= M_INT1;                     // Enable CPU Interrupt 1
                             //                          EINT;                              // Enable Global interrupt INTM
                             //                          ERTM;                              // Enable Global realtime interrupt DBGM
                                                       PieCtrlRegs.PIEIER1.bit.INTx7 = 1;           // TINT0
                                                       PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
                                                       PieCtrlRegs.PIEIER4.bit.INTx2 = 1;
                                                       PieCtrlRegs.PIEIER4.bit.INTx3 = 1;

                             //                         EDIS;
                                                                                 // Enable CPU INT6
                                                  //                          EINT;
                              //   InitAdc();         // For this example, init the ADC
                              //   AdcOffsetSelfCal();
#ifndef UBLOX_USB_TEST_ONLY
                                                      IER|= M_INT1 | M_INT5 | M_INT9 | M_INT3 | M_INT4 | M_INT6;                         // Enable CPU INT4

                                                       IER|= 0x20 | 0x100;                         // Enable CPU INT;Enable CPU INT6
                             //                          EINT;
#else
                                                       IER|= M_INT9;
#endif
                                                       EDIS;
                                                       //
                                                       // Enable interrupts now that the application is ready to start.
                                                       //

#ifdef USE_STATIC_TASK_CREATION
                                                      xSemaphore = xSemaphoreCreateBinaryStatic( &xSemaphoreBuffer );
                                                      if ( xSemaphore == NULL)
                                                         { while(1); }
                                                      xHRCAP1Semaphore = xSemaphoreCreateBinaryStatic( &xHRCAP1SemaphoreBuffer );
                                                      if ( xHRCAP1Semaphore == NULL)
                                                        { while(1); }
#else
                                                        xSemaphore = xSemaphoreCreateBinary();
                                                        if ( xSemaphore == NULL)
								                           { while(1); }
#ifndef UBLOX_USB_TEST_ONLY
							xHRCAP1Semaphore = xSemaphoreCreateBinary();
							if ( xHRCAP1Semaphore == NULL)
								{ while(1); }
							xHRCAP2Semaphore = xSemaphoreCreateBinary();
							if ( xHRCAP2Semaphore == NULL)
								{ while(1); }
							xHRCAP3Semaphore = xSemaphoreCreateBinary();
							if ( xHRCAP3Semaphore == NULL)
								{ while(1); }
							xHRCAP4Semaphore = xSemaphoreCreateBinary();
							if ( xHRCAP4Semaphore == NULL)
								{ while(1); }
							
							xControl1Semaphore = xSemaphoreCreateBinary();
							if ( xControl1Semaphore == NULL)
							{ while(1); }
							xControl2Semaphore = xSemaphoreCreateBinary();
						   if ( xControl2Semaphore == NULL)
							{ while(1); }

						xControl3Semaphore = xSemaphoreCreateBinary();
						   if ( xControl3Semaphore == NULL)
							  { while(1); }

							xControl4Semaphore = xSemaphoreCreateBinary();
							if ( xControl4Semaphore == NULL)
							{ while(1); }

							EC_TaskSemaphore = xSemaphoreCreateBinary();
							if ( EC_TaskSemaphore == NULL)
							{ while(1); }

							xControlECTaskSemaphore = xSemaphoreCreateBinary();
							if ( xControlECTaskSemaphore == NULL)
							{ while(1); }

							xHRCAP1DataDebugSemaphore = xSemaphoreCreateBinary();
							if ( xHRCAP1DataDebugSemaphore == NULL)
						   { while(1); }

							xUSBPrintSemaphore = xSemaphoreCreateBinary();
							if ( xUSBPrintSemaphore == NULL)
							{ while(1); }

#ifdef USE_HRCAP1
							xHRCAP1Queue = xQueueCreate ( HRCAP1_QUEUE_LENGTH, HRCAP1Q_ITEM_SIZE );
							if ( xHRCAP1Queue == NULL)
							 { while(1); }
#endif
#ifdef USE_HRCAP2
							xHRCAP2Queue = xQueueCreate ( HRCAP2_QUEUE_LENGTH, HRCAP2Q_ITEM_SIZE );
							if ( xHRCAP2Queue == NULL)
							{ while(1); }
#endif
#ifdef ENABLE_HRCAP_IRQ3
							xHRCAP3Queue = xQueueCreate ( HRCAP3_QUEUE_LENGTH, HRCAP3Q_ITEM_SIZE );
							if ( xHRCAP3Queue == NULL)
							{ while(1); }
#endif
#ifdef USE_HRCAP4
							xHRCAP4Queue = xQueueCreate ( HRCAP4_QUEUE_LENGTH, HRCAP4Q_ITEM_SIZE );
							if ( xHRCAP4Queue == NULL)
						    { while(1); }
#endif

							/* The created queue needs to be viewable in a kernel aware debugger, so
							add it to the registry. */

							vQueueAddToRegistry( xHRCAP1Queue, "HRCap1Q" );
							vQueueAddToRegistry( xHRCAP2Queue, "HRCap2Q" );
							vQueueAddToRegistry( xHRCAP3Queue, "HRCap3Q" );
							vQueueAddToRegistry( xHRCAP4Queue, "HRCap4Q" );
#endif
#endif

							
/// #endif


#ifdef USE_STATIC_TASK_CREATION
    // Create the task without using any dynamic memory allocation.
    xTaskCreateStatic(LED_TaskRed,          // Function that implements the task.
                      "Red LED task",       // Text name for the task.
                      STACK_SIZE,           // Number of indexes in the xStack array.
                      ( void * ) 1,         // Parameter passed into the task.
                      tskIDLE_PRIORITY + 2, // Priority at which the task is created.
                      redTaskStack,         // Array to use as the task's stack.
                      &redTaskBuffer );     // Variable to hold the task's data structure.

    xTaskCreateStatic(LED_TaskBlue,         // Function that implements the task.
                      "Blue LED task",      // Text name for the task.
                      STACK_SIZE,           // Number of indexes in the xStack array.
                      ( void * ) 2,         // Parameter passed into the task.
                      tskIDLE_PRIORITY + 1, // Priority at which the task is created.
                      blueTaskStack,        // Array to use as the task's stack.
                      &blueTaskBuffer );    // Variable to hold the task's data structure.
#if 0
    xTaskCreateStatic(HRCAP1_Task,         // Function that implements the task.
         "HRCAP1 task",      // Text name for the task.
         STACK_SIZE,           // Number of indexes in the xStack array.
         ( void * ) 2,         // Parameter passed into the task.
         tskIDLE_PRIORITY + 3, // Priority at which the task is created.
         HRCAP1TaskStack,        // Array to use as the task's stack.
         &HRCAP1TaskBuffer );    // Variable to hold the task's data structure.

     xTaskCreateStatic(Control_Task,         // Function that implements the task.
         "ctrl task",      // Text name for the task.
         STACK_SIZE,           // Number of indexes in the xStack array.
         ( void * ) 2,         // Parameter passed into the task.
         tskIDLE_PRIORITY + 2, // Priority at which the task is created.
         ControlTaskStack,        // Array to use as the task's stack.
         &ControlTaskBuffer );    // Variable to hold the task's data structure.

     xTaskCreateStatic(EC_Task,         // Function that implements the task.
         "ec task",      // Text name for the task.
         STACK_SIZE,           // Number of indexes in the xStack array.
         ( void * ) 2,         // Parameter passed into the task.
         tskIDLE_PRIORITY + 2, // Priority at which the task is created.
         ECTaskStack,        // Array to use as the task's stack.
         &ECTaskBuffer );    // Variable to hold the task's data structure.

#endif

#else
    // Create the task without using any dynamic memory allocation.
#ifndef UBLOX_USB_TEST_ONLY
#if 1
#ifdef USE_HRCAP1
     xTaskCreate(HRCAP1_Task,         // Function that implements the task.
         "HRCAP1 task",      // Text name for the task.
         STACK_SIZE*2,           // Number of indexes in the xStack array. *5 w/usb print enabled
         ( void * ) xHRCAP1Queue,         // Parameter passed into the task.
         tskIDLE_PRIORITY + 3, // Priority at which the task is created.
//                       blueTaskStack,        // Array to use as the task's stack.
         &HRCAP1TaskHandle );    // Variable to hold the task's data structure.

//     vTaskSuspend(HRCAP1TaskHandle);
#endif
#endif
#if 1
#ifdef USE_HRCAP2
     xTaskCreate(HRCAP2_Task,         // Function that implements the task.
		 "HRCAP2 task",      // Text name for the task.
		 STACK_SIZE*6,           // Number of indexes in the xStack array.
		 ( void * ) xHRCAP2Queue,         // Parameter passed into the task.
		 tskIDLE_PRIORITY + 3, // Priority at which the task is created.
//                       blueTaskStack,        // Array to use as the task's stack.
		 &HRCAP2TaskHandle );    // Variable to hold the task's data structure.

//     vTaskSuspend(HRCAP1TaskHandle);
#endif
#endif
#ifdef ENABLE_HRCAP_IRQ3
#if 1
     xTaskCreate(HRCAP3_Task,         // Function that implements the task.
		 "HRCAP3 task",      // Text name for the task.
		 STACK_SIZE*1,           // Number of indexes in the xStack array.
		 ( void * ) xHRCAP3Queue,         // Parameter passed into the task.
		 tskIDLE_PRIORITY + 3, // Priority at which the task is created.
//                       blueTaskStack,        // Array to use as the task's stack.
		 &HRCAP3TaskHandle );    // Variable to hold the task's data structure.

//     vTaskSuspend(HRCAP1TaskHandle);
#endif
#endif
#ifdef USE_HRCAP4
     xTaskCreate(HRCAP4_Task,         // Function that implements the task.
		 "HRCAP4 task",      // Text name for the task.
		 STACK_SIZE*5,           // Number of indexes in the xStack array.
		 ( void * ) xHRCAP4Queue,         // Parameter passed into the task.
		 tskIDLE_PRIORITY + 3, // Priority at which the task is created.
//                       blueTaskStack,        // Array to use as the task's stack.
		 &HRCAP4TaskHandle );    // Variable to hold the task's data structure.
//     vTaskSuspend(HRCAP4TaskHandle);
#endif
#endif
#if 1

     xTaskCreate(LED_TaskRed,          // Function that implements the task.
                       "Red LED task",       // Text name for the task.
                       STACK_SIZE,           // Number of indexes in the xStack array.
                       ( void * ) 1,         // Parameter passed into the task.
                       tskIDLE_PRIORITY + 2, // Priority at which the task is created.
 //                      redTaskStack,         // Array to use as the task's stack.
                       &redTaskHandle );     // Variable to hold the task's data structure.

     xTaskCreate(LED_TaskBlue,         // Function that implements the task.
                       "Blue LED task",      // Text name for the task.
                       STACK_SIZE,           // Number of indexes in the xStack array.
                       ( void * ) 2,         // Parameter passed into the task.
                       tskIDLE_PRIORITY + 1, // Priority at which the task is created.
//                       blueTaskStack,        // Array to use as the task's stack.
		 &blueTaskHandle );    // Variable to hold the task's data structure.
#endif
#ifndef UBLOX_USB_TEST_ONLY
#if 1
     xTaskCreate(Control_Task,         // Function that implements the task.
		 "ctrl task",      // Text name for the task.
		 STACK_SIZE*3,           // Number of indexes in the xStack array.
		 ( void * ) 2,         // Parameter passed into the task.
		 tskIDLE_PRIORITY + 2, // Priority at which the task is created.
//                       blueTaskStack,        // Array to use as the task's stack.
		 &ControlTaskHandle );    // Variable to hold the task's data structure.
     
        vTaskSuspend(ControlTaskHandle);
#endif
#endif
#if 1
#ifdef RUN_EC_TASK
     xTaskCreate(EC_Task,         // Function that implements the task.
		 "ec task",      // Text name for the task.
		 STACK_SIZE*7,           // Number of indexes in the xStack array.
#ifdef USE_ALTERNATE_PID
		 ( void * ) (struct _pid *)controlPID,         // Parameter passed into the task.
#else
		 ( void * ) 2,
#endif
		 tskIDLE_PRIORITY + 4, // Priority at which the task is created.
//                       blueTaskStack,        // Array to use as the task's stack.
		 &ECTaskHandle );    // Variable to hold the task's data structure. 

     vTaskSuspend(ECTaskHandle);
#endif
#ifndef UBLOX_USB_TEST_ONLY
     xTaskCreate(Startup_Task,         // Function that implements the task.
         "start task",      // Text name for the task.
         STACK_SIZE,           // Number of indexes in the xStack array.
         ( void * ) 2,         // Parameter passed into the task.
         tskIDLE_PRIORITY + 4, // Priority at which the task is created.
//                       blueTaskStack,        // Array to use as the task's stack.
         &StartupTaskHandle );    // Variable to hold the task's data structure.

//      vTaskSuspend(StartupTaskHandle);
#endif
#ifndef  NO_USB_TASK_FOR_OUTPUT
         xTaskCreate(USB_Print_Task,         // Function that implements the task.
         "usb prt tsk",      // Text name for the task.
         STACK_SIZE*6,           // Number of indexes in the xStack array.
         ( void * ) 2,         // Parameter passed into the task.
         tskIDLE_PRIORITY + 3, // Priority at which the task is created.
//                       blueTaskStack,        // Array to use as the task's stack.
         &USBPrintTaskHandle );    // Variable to hold the task's data structure.

//      vTaskSuspend(StartupTaskHandle);
#endif


#endif
#endif
#if 0
     EALLOW;
#ifdef ENABLE_HRCAP_IRQ2
     PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 8
#endif
#ifdef ENABLE_HRCAP_IRQ1
     PieCtrlRegs.PIEIER4.bit.INTx7=0;     // Enable PIE Group 4, INT 7
#endif
#ifdef ENABLE_HRCAP_IRQ4
     PieCtrlRegs.PIEIER5.bit.INTx5=0;     // Enable PIE Group 4, INT 8
#endif
#ifdef ENABLE_HRCAP_IRQ3
     PieCtrlRegs.PIEIER5.bit.INTx4=1;     // Enable PIE Group 4, INT 8
#endif

     EDIS;
#endif
     ////                                                     SysCtrlInit();

     //
         // Make GPIO24 the input source for XINT1
         //




//         XINT1CR = 0x4;                       // IRQ Disabled; IRQ on Rising Edge; | with 0x1 to enable IRQ
         IER |= M_INT1;                      // Set "global" priority
//         PieCtrlRegs.PIEIER1.all &= MG14;   // Set "group"  priority
//         PieCtrlRegs.PIEACK.all = 0xFFFF;   // Enable PIE inter
         EALLOW;
         GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0; // GPIO24 = GPIO24
         GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;  // GPIO24 = input
         GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 3;
         GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 24;   // XINT1 connected to GPIO24
         XIntruptRegs.XINT1CR.bit.POLARITY = 1;
         XINT_polarity_state = XINT_RISING;
//         XIntruptRegs.XINT1CR.all |= 1;
         PieCtrlRegs.PIEIER1.bit.INTx4 = 1;   // ENABLE PIE
         PieVectTable.XINT1 = &XINT1_ISR;
 //        GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 24;
         EDIS;


                                                                IntEnable(INT_SCITXINTB);
                                                                IntEnable(INT_SCIRXINTB);
//                                                               USBIntEnable(USB0_BASE, USB_INT_DEV_IN | USB_INT_DEV_OUT );
                                                             //  IntEnable(INT_USB0INT);
                                                             // IntEnable(INT_TINT0);
                                                             //  IntMasterEnable();


                                                          EINT;  // Enable Global interrupt INTM
                                                           ERTM;  // Enable Global realtime interrupt DBGM
     //                                                      SysTickIntEnable();
     //                                                      SysTickEnable();

                                                          // Enable CPU INT3 which is connected to EPWM1-6 INT
                                                           setupTimer1();

                                                           DSP28x_usDelay(2);

                                                           // spi_xmit(0x77777777, 'p');
                                                            spi_xmit(0x22222222, 'p');       // ISOTEMP - set gain to 1 since currently using Vref from OCXO
                                                          //  spi_xmit(0x11111111, 'p');

                                                           DSP28x_usDelay(20);

                                                          // spi_xmit(0xffffffff, 'l');
                                                             spi_xmit(0xf0f0f0f0, 'l');       // ISOTEMP - set to max.... XXX:::XXX 0xf0f0f0f0
                                                           // spi_xmit(0x70707070, 'l');
                                                          //  spi_xmit(0x8f8f8f8f, 'l');
                                                           DSP28x_usDelay(20);

                                                          // spi_xmit(0xffffffff, 'h');
                                                             spi_xmit(0xf0f0f0f0, 'h');       // ISOTEMP - set to max.... XXX:::XXX 0xf0f0f0f0
                                                          //  spi_xmit(0x70707070, 'h');
                                                           // spi_xmit(0x8f8f8f8f, 'h');
#ifndef USE_ALTERNATE_PID

							   /* initialize controller variables */
#if RUN_LINEAR_PID
                                                           pid1->sps = &sps1;
                                                           pid1->css = &css1;

                                                              //  pid1->Kp = 9.0f;                  // proportional gain
                                                               pid1->Kp = 3.0f;                     // 2.5
                                                                pid1->Ki = 0.015f;    // 0.015           // integral gain
                                                                pid1->Kd = 0.35f;   // 0.35             // derivative gain

                                                                pid1->sps->Kp = 3.0f;                  // proportional gain

                                                                pid1->sps->Ki = 0.015f;               // integral gain
                                                                pid1->sps->Kd = 0.35f;

                                                             //  pid1.Ki = 1.0f;
                                                             //  pid1.Kd = 1.0f;
                                                               pid1->Kr = 1.0f;                  // set point weight
                                                               pid1->sps->Kr = 2.0f;                  // set point weight
                                                               pid1->c1 = 188.0296600613396f;    // D-term filter coefficient 1
                                                               pid1->c2 = 0.880296600613396f;    //!< D-term filter coefficient 2
                                                               pid1->d2 = 0.0f;                  //!< D-term filter intermediate storage 1
                                                               pid1->d3 = 0.0f;                  //!< D-term filter intermediate storage 2
                                                               pid1->i10 = 0.0f;                 //!< I-term intermediate storage
                                                               pid1->i14 = 1.0f;                 //!< Intermediate saturation storage

                                                                 pid1->Umax = 1.0f;
                                                              //   pid1->Umax = 0.50f;            //!< Upper saturation limit
                                                                 pid1->Umin  = 0.0f;           //!< Lower saturation limit
                                                                 pid1->sps->Umax = 1.0f;
                                                                 pid1->sps->Umin = 0.0f;
                                                                 pid1->sps->c1 = 188.0296600613396f;    // D-term filter coefficient 1
                                                                 pid1->sps->c2 = 0.880296600613396f;    //!< D-term filter coefficient 2
                                                               //  pid1->Umax = 0.75f;
                                                               // pid1->Umin  = 0.25f;

                                                                // pid1.Umax = 360.0f;
                                                               // pid1.Umax = 510.0f;
                                                               // pid1.Umin = 440.0f;                        // 360 -> 50%, 540 -> 25%, 630 -> 12.5%, 450 -> 37.5%
								 DCL_REQUEST_UPDATE(pid1);
								 DCL_fupdatePID(pid1);
#else
                                                                 /* initialize controller variables */
								                                    nlpid1->sps = &sps1;
								                                    nlpid1->css = &css1;

                                                                 /* initialize controller variables */
                                                                     nlpid1->sps->Kp = 3.5f;
                                                                //     nlpid1->sps->Kp = 10.0f;
                                                                     nlpid1->sps->Ki = 0.004f;
                                                                   //  nlpid1.sps->Ki = 0.0f;
                                                                     nlpid1->sps->Kd = 0.15f;

                                                                      nlpid1->Ki = 0.004f;
                                                                  //   nlpid1.Ki = 0.0f;
                                                                 //    nlpid1->Kp = 3.5f;
                                                                     nlpid1->Kp = 3.5f;
                                                                     nlpid1->Kd = 0.15f;

                                                                 //    nlpid1.Kp = 1.0f;
                                                                 //    nlpid1.Kd = 0.0f;
                                                                     nlpid1->sps->alpha_p = 0.8f;
                                                                     nlpid1->sps->alpha_i = 0.95f;
                                                                     nlpid1->sps->alpha_d = 1.0f;
                                                                     nlpid1->sps->delta_p = 0.15f;
                                                                     nlpid1->sps->delta_i = 0.15f;
                                                                     nlpid1->sps->delta_d = 0.15f;

                                                                     nlpid1->alpha_p = 0.8f;
                                                                     nlpid1->alpha_i = 0.95f;
                                                                     nlpid1->alpha_d = 1.0f;
                                                                     nlpid1->delta_p = 0.15f;
                                                                     nlpid1->delta_i = 0.15f;
                                                                     nlpid1->delta_d = 0.15f;
#if 1
                                                                     nlpid1->gamma_p = 0.15f;                //
                                                                     nlpid1->gamma_i = 0.15f;                //
                                                                     nlpid1->gamma_d = 0.15f;                //

                                                                     nlpid1->sps->gamma_p = 0.15f;                //
                                                                     nlpid1->sps->gamma_i = 0.15f;                //
                                                                     nlpid1->sps->gamma_d = 0.15f;
#endif
                                                                     nlpid1->i7 = 0.0f;
                                                                     nlpid1->i16 = 0.0f;
                                                                     nlpid1->i18 = 0.0f;                     //
                                                                     nlpid1->d2 = 0.0f;
                                                                     nlpid1->d3 = 0.0f;
                                                                     nlpid1->sps->c1 = 151.7093985989552f;
                                                                     nlpid1->sps->c2 = 0.517093985989552f;
                                                                     nlpid1->c1 = 151.7093985989552f;
                                                                     nlpid1->c2 = 0.517093985989552f;
                                                                     // nlpid1.Umax = 0.31f;
                                                                     // nlpid1.Umin = -0.27f;
#if 1
#ifdef USE_SYNTHETIC_UK
                                                                     nlpid1.sps->Umax = UK_MAX / 100.0f;
                                                                     nlpid1.sps->Umin = UK_MIN / 100.0f;

                                                                     nlpid1.Umax = UK_MAX / 100.0f;
                                                                     nlpid1.Umin = UK_MIN / 100.0f;


#else
#if 0
                                                                     nlpid1.sps->Umax = 0.70f;
                                                                     nlpid1.sps->Umin = -0.50f;

                                                                     nlpid1.Umax = 0.70f;
                                                                     nlpid1.Umin = -0.50f;
#endif
                                                                     nlpid1->sps->Umax = 1.0f;
                                                                     nlpid1->sps->Umin = 0.0f;

                                                                     nlpid1->Umax = 1.00f;
                                                                     nlpid1->Umin = 0.0f;

#endif
#endif
#if 0
                                                                     nlpid1.sps->Umax = 100.0f;
                                                                     nlpid1.sps->Umin = -70.0f;

                                                                     nlpid1.Umax = 100.0f;
                                                                    nlpid1.Umin = -70.00f;
#endif
                                                                    //  DCL_setGamma(&nlpid1);          // -> 1.04.00 version
                                                                    DCL_resetNLPID(nlpid1);
                                                                    DCL_setNLPIDgamma(nlpid1);
                                                                    DCL_REQUEST_UPDATE(nlpid1);
                                                                    DCL_updateNLPID(nlpid1);


#endif
                                                                  //   S_PID_VDATA.rk = 48.0f;
                                                                 //    rk = 1.0f;
								//    rk = 0.0f;
							    //      rk = 0.48f;                             // initial value for control reference
                                      //                            rk = 0.96f;
                                                               //      rk = 0.72f;
                                                              //  rk = 0.50f;
                                                                    S_PID_VDATA.rk = 0.24f;         // 24 - 24 is 1/2 our normal set point
                                                                     S_PID_VDATA.lk = 0.480f;        // 48                      // control loop not saturated
                                                             //        lk = 0.48f;

								control_valH_min = 1000;
								control_valL_min = 1000;
/*
								uint32_t PID_ID = 0x00;

								// PID ID's:
								// Linear PID C1 : 0x004C5031   -> LP1
								// Linear PID C4 : 0x004C5034   -> LP4
								// Non-Linear PID C1: 0x4E4C5031 -> NLP1
								// Non-Linear PID C2: 0x4E4C5032 -> NLP2
								// Alternate PID:     0x00004150 -> AP

*/
#if 1                           // USB driver not running yet...
#ifndef RUN_NON_LINEAR_PID
#ifdef RUN_LINEAR_PID
#ifdef NO_SP_WEIGHT
//							      sprintf((Uint8 *)&debug_string[0], "Running Linear PID vC4\r\n");
								PID_ID = LINEAR_PID_C4;
#else
//							      sprintf((Uint8 *)&debug_string[0], "Running Linear PID vC1\r\n");
								PID_ID = LINEAR_PID_C1;
#endif
#endif
#else
#ifdef RUN_NON_LINEAR_PID_C1
//							      sprintf((Uint8 *)&debug_string[0], "Running Non-Linear PID vC1\r\n");
								PID_ID = NON_LINEAR_PID_C1;
#else
//							      sprintf((Uint8 *)&debug_string[0], "Running Non-Linear PID vC2\r\n");
								PID_ID = NON_LINEAR_PID_C2;
#endif
#endif
//							      USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));
#endif
#else

							      // initialize alternate PID:
							     *(controlPID->pv) = (float) 0.0f;	      // just init
							     *(controlPID->sp) = (float) 36.0f;       // set point is 48 - this is HRCAP CLK counts -> 24.0 is good test to see how regulation works

							      controlPID->pgain = 5.0f;			// 5
							      controlPID->igain = 0.1f;			// .1
							      controlPID->dgain = 0.2f;			// .2

							      controlPID->last_error = 0;        //  Last calculated error
							      controlPID->prev_error = 0;          //  Last -1 Error
							      controlPID->error_sum = 0;           //  Sums of Errors

							    controlPID->integral_error_max = 100.;          //   Sum max limit
							    controlPID->integral_error_min = -100.;            //   Sum min limit
							    controlPID->deadband = 0;             //   set this to zero for now
//							    controlPID->integral = 10.0;
							    controlPID->integral = 20.0;

							    pid_init((struct _pid *)(controlPID), &(*(controlPID->pv)), &(*(controlPID->sp)));
							    pid_tune((struct _pid *)(controlPID), controlPID->pgain, controlPID->igain, controlPID->dgain, 0);

							    pid_setinteg((struct _pid *)controlPID,controlPID->integral);
							/// pid_setinteg(&tecControlPID,tecControlPID->igain);
							    pid_bumpless((struct _pid *)controlPID);
							    PID_ID = ALTERNATE_PID;
#if 0
							    sprintf((Uint8 *)&debug_string[0], "Running Alternate PID \r\n");
							    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));
#endif
#endif

							    // fill up (s)NTP test packet
							    // note - this is test packet only

							    SNTPpkt->li_vn_mode = 0x1c;     // leap indicator, version number, mode:server
							    SNTPpkt->stratum = 0x01;
							    SNTPpkt->ppoll = 0x00;          // peer polling
							    SNTPpkt->precision = 0xed;      // 0.000002 sec
							    SNTPpkt->rootdelay = 0x0000;    // root delay
							    SNTPpkt->rootdisp = 0x0010;     // root dispersion
							    SNTPpkt->refid = 0x47505300;    // GPS
//							    SNTPpkt->reftime = 0xdfa4c4e3658f0962;
							    SNTPpkt->reftime.Ul_i.Xl_ui = 0xdfa4c4e3;
							    SNTPpkt->reftime.l_uf = 0x658f0962;
							    SNTPpkt->org.Ul_i.Xl_ui =  0xdfa4c4e3;
							    SNTPpkt->org.l_uf = 0x658f0962;
							    SNTPpkt->rec.Ul_i.Xl_ui =  0xdfa4c4e3;
							    SNTPpkt->rec.l_uf = 0x658f0962;
							    SNTPpkt->xmt.Ul_i.Xl_ui =  0xdfa4c4e3;
							    SNTPpkt->xmt.l_uf = 0x658f0962;

//							    uint8_t *ptr;

							    ptr = ( uint8_t *) &(SNTPpkt->li_vn_mode);
 //                                                          while (1) {;}

//						        Uint16 (*) HR2_DLYRISE0 = (Uint16*) (0x00006af4);         // HR2 delay rise 0
//						        Uint16 (*) HR2_DLYFALL0  = (Uint16*) (0x00006af5);         // HR2 delay fall 0

///							      vConfigTimer0();        // mainly for PID IRQ rate control

	vTaskStartScheduler();

    while (1) {;}

}
void SysCtrlInit(void)
{
    EALLOW;
    //
    // Disable Watchdog
    //
    SysCtrlRegs.WDCR = 0x68;

    //
    // Setup Clock
    // 20MHz ->PLL->80MHz->C28 - this is using 20MHz XTAL
    //      ->PLL2->120MHz->USB
    //
    SysCtrlRegs.CLKCTL.bit.XCLKINOFF = 1;
    SysCtrlRegs.CLKCTL.bit.XTALOSCOFF = 0;
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 0;
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;

    while(SysCtrlRegs.PLLSTS.bit.MCLKSTS);
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
    SysCtrlRegs.PLLCR.bit.DIV = 4;
    while(!SysCtrlRegs.PLLSTS.bit.PLLLOCKS);
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;

    SysCtrlRegs.PLL2CTL.bit.PLL2CLKSRCSEL = 2;
    SysCtrlRegs.PLL2CTL.bit.PLL2EN = 1;
    SysCtrlRegs.PLL2MULT.bit.PLL2MULT = 6;
    while(!SysCtrlRegs.PLL2STS.bit.PLL2LOCKS);
}

//
// SysCtrlInit -
//
void SysCtrlInitA(void)
{
    EALLOW;
    //
    // Disable Watchdog
    //
    SysCtrlRegs.WDCR = 0x68;
#if 0
    //
    // Setup Clock
    // 20MHz ->PLL->80MHz->C28 - this is using 20MHz XTAL
    //      ->PLL2->120MHz->USB
    //
    SysCtrlRegs.CLKCTL.bit.XCLKINOFF = 1;
    SysCtrlRegs.CLKCTL.bit.XTALOSCOFF = 0;
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 0;
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;

    while(SysCtrlRegs.PLLSTS.bit.MCLKSTS);
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
    SysCtrlRegs.PLLCR.bit.DIV = 4;
    while(!SysCtrlRegs.PLLSTS.bit.PLLLOCKS);
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;

    SysCtrlRegs.PLL2CTL.bit.PLL2CLKSRCSEL = 2;
    SysCtrlRegs.PLL2CTL.bit.PLL2EN = 1;
    SysCtrlRegs.PLL2MULT.bit.PLL2MULT = 6;
    while(!SysCtrlRegs.PLL2STS.bit.PLL2LOCKS);
#else
    //
    // Setup Clock
    // 30MHz ->PLL->90MHz->C28               - this is using 30MHz MEMS Extclk - XCLKIN GPIO19
    //      ->PLL2->120MHz->USB
    //

    SysCtrlRegs.XCLK.bit.XCLKINSEL = 1;             // GPIO19 is XCLKIN input source
    SysCtrlRegs.CLKCTL.bit.XTALOSCOFF = 1;          // Turn off 20MHz XTAL
    DSP28x_usDelay(1000);
    SysCtrlRegs.CLKCTL.bit.XCLKINOFF = 0;          // Turn on XCLKIN
    DSP28x_usDelay(2);
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 0;     // Switch to external clock
    //
    // Switch from INTOSC1 to INTOSC2/ext clk    //

    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;

    //
    // Clock Watchdog off of INTOSC1 always
    //
     SysCtrlRegs.CLKCTL.bit.WDCLKSRCSEL = 0;

     SysCtrlRegs.CLKCTL.bit.INTOSC2OFF = 1;    // Turn off INTOSC2
     SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 0;    // Leave INTOSC1 on

    while(SysCtrlRegs.PLLSTS.bit.MCLKSTS);
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
    SysCtrlRegs.PLLCR.bit.DIV = 3;
    while(!SysCtrlRegs.PLLSTS.bit.PLLLOCKS);
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;
 ;;   while(!SysCtrlRegs.PLLSTS.bit.PLLLOCKS);

    SysCtrlRegs.PLL2CTL.bit.PLL2CLKSRCSEL = 3;
    SysCtrlRegs.PLL2CTL.bit.PLL2EN = 1;
    SysCtrlRegs.PLL2MULT.bit.PLL2MULT = 4;

    while(!SysCtrlRegs.PLL2STS.bit.PLL2LOCKS);

#endif

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

//
// adc_isr -
//
__interrupt void adc_isr(void)
{
#if 1
    Voltage1[ConversionCount] = AdcResult.ADCRESULT0;
    Voltage2[ConversionCount] = AdcResult.ADCRESULT1;

    Voltage3[ConversionCount] = AdcResult.ADCRESULT2;
    Voltage4[ConversionCount] = AdcResult.ADCRESULT3;

    Voltage5[ConversionCount] = AdcResult.ADCRESULT4;
    Voltage6[ConversionCount] = AdcResult.ADCRESULT5;

    Voltage7[ConversionCount] = AdcResult.ADCRESULT6;
    Voltage8[ConversionCount] = AdcResult.ADCRESULT7;

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
void InitECapture()
{
    ECap1Regs.ECEINT.all = 0x0000;          // Disable all capture interrupts
    ECap1Regs.ECCLR.all = 0xFFFF;           // Clear all CAP interrupt flags
    ECap1Regs.ECCTL1.bit.CAPLDEN = 0;       // Disable CAP1-CAP4 register loads
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Make sure the counter is stopped

    //
    // Configure peripheral registers
    //
    ECap1Regs.ECCTL2.bit.CONT_ONESHT = 1;   // 1 - One-shot - 0 continuous

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
    ECap1Regs.ECCTL2.bit.SYNCI_EN = 1;
    ECap1Regs.ECCTL1.bit.PRESCALE = 5;      // divide by 10
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable CAP1-CAP4 register loads
    ECap1Regs.ECEINT.bit.CEVT4 = 1;         // 4 events = interrupt
//    ECap1Regs.ECEINT.bit.CEVT1 = 1;         // 4 events = interrupt


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

    ECap3Regs.ECEINT.all = 0x0000;          // Disable all capture interrupts
    ECap3Regs.ECCLR.all = 0xFFFF;           // Clear all CAP interrupt flags
    ECap3Regs.ECCTL1.bit.CAPLDEN = 0;       // Disable CAP1-CAP4 register loads
    ECap3Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Make sure the counter is stopped

    //
    // Configure peripheral registers
    //
    ECap3Regs.ECCTL2.bit.CONT_ONESHT = 1;   // 1 - One-shot - 0 continuous

    ECap3Regs.ECCTL2.bit.STOP_WRAP = 3;     // Stop at 4 events
    ECap3Regs.ECCTL1.bit.CAP1POL = 1;       // Falling edge
    ECap3Regs.ECCTL1.bit.CAP2POL = 0;       // Rising edge
    ECap3Regs.ECCTL1.bit.CAP3POL = 1;       // Falling edge
    ECap3Regs.ECCTL1.bit.CAP4POL = 0;       // Rising edge
    ECap3Regs.ECCTL1.bit.CTRRST1 = 1;       // Difference operation
    ECap3Regs.ECCTL1.bit.CTRRST2 = 1;       // Difference operation
    ECap3Regs.ECCTL1.bit.CTRRST3 = 1;       // Difference operation
    ECap3Regs.ECCTL1.bit.CTRRST4 = 1;       // Difference operation
    ECap3Regs.ECCTL2.bit.SYNCI_EN = 1;      // Enable sync in
    ECap3Regs.ECCTL2.bit.SYNCO_SEL = 0;     // Pass through
    ECap3Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable capture units

    ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
    ECap3Regs.ECCTL2.bit.REARM = 1;         // arm one-shot
    ECap3Regs.ECCTL2.bit.SYNCI_EN = 1;
    ECap3Regs.ECCTL1.bit.PRESCALE = 5;      // divide by 10
    ECap3Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable CAP1-CAP4 register loads
    ECap3Regs.ECEINT.bit.CEVT4 = 1;         // 4 events = interrupt
//    ECap1Regs.ECEINT.bit.CEVT1 = 1;         // 4 events = interrupt

}

//
// ecap1_isr -
//
__interrupt void ecap1_isr(void)
{
    //
    // Cap input is syc'ed to SYSCLKOUT so there may be
    // a +/- 1 cycle variation
    //
    if(ECap1Regs.CAP1 > EPwm3Regs.TBPRD*2+1 ||
        ECap1Regs.CAP1 < EPwm3Regs.TBPRD*2-1)
     {
 //        Fail();
     }


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

    ECap1Regs.ECCTL1.bit.CAPLDEN = 1;       // Enable CAP1-CAP4 register loads
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
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
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
    ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
    ECap2Regs.ECCLR.bit.CEVT4 = 1;
    ECap2Regs.ECCLR.bit.INT = 1;
    ECap2Regs.ECCTL2.bit.REARM = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 4
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
//
// ecap3_isr -
//
__interrupt void ecap3_isr(void)
{
    //
    // Cap input is syc'ed to SYSCLKOUT so there may be
    // a +/- 1 cycle variation
    //
    if(ECap3Regs.CAP1 > EPwm3Regs.TBPRD*2+1 ||
        ECap3Regs.CAP1 < EPwm3Regs.TBPRD*2-1)
     {
 //        Fail();
     }


    if(ECap3Regs.CAP2 > EPwm3Regs.TBPRD*2+1 ||
       ECap3Regs.CAP2 < EPwm3Regs.TBPRD*2-1)
    {
//        Fail();
    }

    if(ECap3Regs.CAP3 > EPwm3Regs.TBPRD*2+1 ||
       ECap3Regs.CAP3 < EPwm3Regs.TBPRD*2-1)
    {
//        Fail();
    }

    if(ECap3Regs.CAP4 > EPwm3Regs.TBPRD*2+1 ||
       ECap3Regs.CAP4 < EPwm3Regs.TBPRD*2-1)
    {
//        Fail();
    }

    ECap3IntCount++;
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

    ECap3PassCount++;
    ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;     // Start Counter
    ECap3Regs.ECCLR.bit.CEVT4 = 1;
    ECap3Regs.ECCLR.bit.INT = 1;
    ECap3Regs.ECCTL2.bit.REARM = 1;
    ECap3Regs.ECCTL2.bit.SYNCI_EN = 1;
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
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;
    //
    // Inputs are synchronized to SYSCLKOUT by default.
    // Comment out other unwanted lines.
    //
//    GpioCtrlRegs.GPAQSEL1.bit.GPIO5 = 0;    // Synch to SYSCLKOUT GPIO5 (CAP1)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 0; // Synch to SYSCLKOUT GPIO11 (CAP1)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 0; // Synch to SYSCLKOUT GPIO19 (CAP1)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0; // Synch to SYSCLKOUT GPIO24 (CAP1)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 0;
    GpioCtrlRegs.GPAQSEL1.bit.GPIO9 = 0;
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
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 3;
    EDIS;
}

//
// InitEPwmTimer -
//
#ifndef UBLOX_USB_TEST_ONLY
#if 1
void InitEPwmTimer()
{
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
	EDIS;

    //
    // Disable Sync
    //
//	EPwm7Regs.TBCTL.bit.SYNCOSEL = 11;  // Pass through
	EPwm7Regs.TBCTL.bit.SYNCOSEL = 3;
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
#endif
//
// epwm1_timer_isr - Interrupt routines uses in this example
//
__interrupt void epwm7_timer_isr(void)
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


void vApplicationIdleHook(void * pvParameters)
{
#ifdef USE_HRCAP1
    HRCAP1Queue xMessage;
#endif
    unsigned char f_f = 0;
    Uint16 i;
    uint8_t ui8Char = 0x20;
    Uint16 DutyFine, LocalPeriod = 0;

    static unsigned char once0, once3 = 0;
    static unsigned long counter_hrc1 = 0;

    Uint16 xmit_fmt;
    Uint16 rcv_fmt;
#if 0    
    unsigned char f_f = 0;
    Uint16 i;
    Uint32 qdataL;
    Uint32 sdataL;  // send data
    Uint32 rdataL;  // received data
    Uint16 xmit_fmt;
    Uint16 rcv_fmt;
    Uint16 status1;
    sdataL = 0x0;
    rdataL = 0x0;
    qdataL = 0x0;
#endif

    LocalPeriod = 9;

    for ( EVER )
    {

              /* Yield in case cooperative scheduling is being used. */
      #if configUSE_PREEMPTION == 0
      {
          taskYIELD();
      }
      #endif

#ifndef USE_STATIC_TASK_CREATION
              aHeap = ( int ) xPortGetFreeHeapSize();
#endif

              // Force start of conversion on SOC0, S0C1
              //
              AdcRegs.ADCSOCFRC1.bit.SOC0 = 0x01;
              AdcRegs.ADCSOCFRC1.bit.SOC1 = 0x01;
              AdcRegs.ADCSOCFRC1.bit.SOC2 = 0x01;
              AdcRegs.ADCSOCFRC1.bit.SOC3 = 0x01;
              AdcRegs.ADCSOCFRC1.bit.SOC4 = 0x01;
              AdcRegs.ADCSOCFRC1.bit.SOC5 = 0x01;
              AdcRegs.ADCSOCFRC1.bit.SOC6 = 0x01;
              AdcRegs.ADCSOCFRC1.bit.SOC7 = 0x01;

              // update linear gains if new NL parameters
#ifdef  USE_VERSION_1PZERO4
                      if (calFlag == 1)
                      {
                          DCL_setGamma(&nlpid1);
                          calFlag = 0;
                      }
#endif
#if 0
#if 1
#if RUN_NON_LINEAR_PID
// #ifndef RUN_OPEN_LOOP_TESTING
                      // update linear gains if new NL parameters
                               DCL_updateNLPID(nlpid1);                     // new in 1.05
                      //        DCL_fupdateNLPID(&nlpid1);                     // new in 1.05
// #endif
#else
                               DCL_fupdatePID(pid1);
#endif
#endif
#endif

//               UARTIntEnable(UART1_BASE, UART_INT_RXRDY_BRKDT);
#if 0
                USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (uint8_t *)&ui8Char, 1);
                if ( ui8Char++ > 0x7e )
                    ui8Char = 0x20;
#endif
//                UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, false));
//                      UARTIntEnable(UART1_BASE, (UART_INT_RXERR | UART_INT_RXRDY_BRKDT |
//                                    UART_INT_TXRDY ));
//              IntEnable(INT_SCITXINTB);
//              IntEnable(INT_SCIRXINTB);
#if 0
                if ( counter_hrc1++ > 1024 )
                {
                    sprintf(debug_string, "HRC1: %lu, %lu, %lu, %lu\n", xMessage.int_HRC1pulsewidthhigh0[0], xMessage.frac_HRC1pulsewidthhigh0[0], xMessage.int_HRC1pulsewidthlow0[0], xMessage.frac_HRC1pulsewidthlow0[0] );
                    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (uint8_t *)&debug_string[0], strlen(debug_string));

                    counter_hrc1 = 0;

                }
#endif

#if 0
                if (isr_cnt >= update_rate)
                {
                    if (CMP_Reg < (PRD_Reg-4) && change_dir == 1)   // Increment CMP
                    {
                        CMP_Inc = InputCMPInc >> 16;
                        CMP_HR_INC = (Uint16) InputCMPInc;
                        CMP_Reg = CMP_Reg+CMP_Inc;
                        CMP_HR_temp = (Uint32)CMP_HR + (Uint32)CMP_HR_INC;

                        if(CMP_HR_temp>=0x10000)
                        {
                            CMP_HR_temp = CMP_HR_temp-0x10000;
                            CMP_Reg = CMP_Reg+1;
                        }

                        CMP_HR = (Uint16) CMP_HR_temp;
                    }

                    else
                    {
                        change_dir = 0;         // Decrement CMP
                    }

                    if (CMP_Reg > 4 && change_dir == 0) // Decrement CMP
                    {
                        CMP_Inc = InputCMPInc >> 16;
                        CMP_HR_INC = (Uint16) InputCMPInc;
                        CMP_Reg = CMP_Reg-CMP_Inc;
                        CMP_HR_temp = (int32)CMP_HR - (int32)CMP_HR_INC;

                        if(CMP_HR_temp < 0)
                        {
                            CMP_HR_temp = 0x10000 + CMP_HR_temp;
                            CMP_Reg = CMP_Reg-1;
                        }

                        CMP_HR = (Uint16) CMP_HR_temp;
                    }

                    else
                    {
                        change_dir = 1;                             // Increment CMP
                    }

                    update = 1;
                    isr_cnt = 0;
                }
#endif
                    //
                    // Example, write to the HRPWM extension of CMPA
                    //
                    //
                    // Left shift by 8 to write into MSB bits
                    //
#if 0
                    EPwm7Regs.CMPA.half.CMPAHR = 0;

                    EPwm7Regs.CMPA.half.CMPAHR = DutyFine << 8;
                    DutyFine++;
                    if ( DutyFine > 8 )
                    {
                        DutyFine = 0;
                        if (LocalPeriod > 10)
                          LocalPeriod = 8;

//                        HRPWM_ConfigLocal2(LocalPeriod);
                        LocalPeriod++;
                    }
#endif
#if 0
#ifdef ENABLE_HRCAP_IRQ1
                              if (!once0)
                              {
                              EALLOW;
                             if (PieCtrlRegs.PIEIER4.bit.INTx7 == 0)
                                 PieCtrlRegs.PIEIER4.bit.INTx7 = 1;     // Enable PIE Group 4, INT 7
                              EDIS;
                              once0++;
                              }
#endif
#endif
#if 0
#ifdef ENABLE_HRCAP_IRQ4
//                              if (!once3)
                              {
                              EALLOW;
                             if (PieCtrlRegs.PIEIER5.bit.INTx5 == 0)
                                 PieCtrlRegs.PIEIER5.bit.INTx5 = 1;     // Enable PIE Group 4, INT 7
                              EDIS;
                              once3++;
                              }
#endif
#endif
              //
              // Main application loop.
              //
//              while(1)
              {
          #if 0
                      EALLOW;
                      GpioCtrlRegs.GPAMUX1.bit.GPIO8 = ENABLE;                // ENABLE MEMS OSC - 90 MHz
                      GpioCtrlRegs.GPAPUD.bit.GPIO8 = DISABLE;
                      GpioCtrlRegs.GPADIR.bit.GPIO8 = ENABLE;
                      EDIS;
          #endif
#if 0
        EALLOW;
#if 1
#ifdef USE_HRCAP1
        HRCap1Regs.HCCTL.bit.RISEINTE = 1;
#endif
#endif
#if 1
#ifdef USE_HRCAP2
        HRCap2Regs.HCCTL.bit.RISEINTE = 1;
#endif
#endif
#if 1
#ifdef USE_HRCAP3
        HRCap3Regs.HCCTL.bit.RISEINTE = 1;
#endif
#endif
#if 1
#ifdef USE_HRCAP4
        HRCap4Regs.HCCTL.bit.RISEINTE = 1;
#endif
#endif
        EDIS;
#endif
//        EPwm2Regs.TBCTL.bit.SWFSYNC = 0;

          #ifndef USB_IO_DBG
          #if 0
          #ifdef USE_HRCAP1

          //      if (datacounter1 == 5)
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
                      status1 = HRCAP_Cal(2,HCCAPCLK_PLLCLK, &EPwm8Regs);         // Ch. 2 is connected to 'Cal'
          //          status1 = HRCAP_Cal(1,HCCAPCLK_PLLCLK, &EPwm8Regs);
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


          //      if (datacounter1 == 5)
          //              {
          //                  datacounter1 = 0;
          //              }
                  do_cal_1 = 0;
                  }

          ////        HRCAP1_Config();
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

          ////        HRCAP4_Config();
#endif
#endif
#if 0		  
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
#endif		  
///                  DSP28x_usDelay(1);
                   //   GpioCtrlRegs.GPAPUD.bit.GPIO4 = ENABLE;                // this means NO pullup is enabled
                   // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
          //            delay_loop();
///                  DINT;
///                  EINT;

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
          //             GpioDataRegs.GPBTOGGLE.bit.GPIO51;

                         GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
                         GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
                         GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
                         GpioDataRegs.GPBDAT.bit.GPIO51 = DISABLE;
          //            GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
          //             GpioDataRegs.GPBSET.bit.GPIO51 = DISABLE;

          //             GpioDataRegs.GPASET.bit.GPIO17 = DISABLE;
          #endif
//                         EDIS;

          // #ifdef USE_NORM
          #if 0
                         spi_xmit(xmit_fmt);

                         EALLOW;
          //             GpioDataRegs.GPBTOGGLE.bit.GPIO51;

                                      GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
                                      GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
                                      GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
                                      GpioDataRegs.GPBDAT.bit.GPIO51 = ENABLE;
          //                          GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
                       //             GpioDataRegs.GPBDAT.bit.GPIO51 = ENABLE;
          //                          GpioDataRegs.GPASET.bit.GPIO17 = ENABLE;
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
          //                 error();
                         }

          #endif
          // #ifdef USE_NORM
          #if 0
                         EALLOW;
          //             GpioDataRegs.GPBTOGGLE.bit.GPIO51;

                                       GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
                                       GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
                                       GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
                                       GpioDataRegs.GPBDAT.bit.GPIO51 = DISABLE;
          //                          GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
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
          //                 error();
                         }
          #endif

          #if 0
                         sdataL--;
                         rdataL--;
                         qdataL--;
          #endif


#if 0
                     EALLOW;
                         GpioCtrlRegs.GPAMUX1.bit.GPIO4 = DISABLE;              //
                         GpioDataRegs.GPASET.bit.GPIO4 = ENABLE;
                         GpioCtrlRegs.GPADIR.bit.GPIO4 = ENABLE;
                     GpioDataRegs.GPADAT.bit.GPIO4 = ENABLE;
		     EDIS;
#endif		     
 ////                    DSP28x_usDelay(1);
 
          #endif
          #if 0
                         EALLOW;
          //             GpioDataRegs.GPBTOGGLE.bit.GPIO51;

                         GpioCtrlRegs.GPBMUX2.bit.GPIO51 = DISABLE;              // AD SPI CSH
                         GpioDataRegs.GPBSET.bit.GPIO51 = ENABLE;
                         GpioCtrlRegs.GPBDIR.bit.GPIO51 = ENABLE;
                         GpioDataRegs.GPBDAT.bit.GPIO51 = ENABLE;
          //             GpioCtrlRegs.GPBPUD.bit.GPIO51 = ENABLE;
          //             GpioDataRegs.GPBDAT.bit.GPIO51 = ENABLE;
          //             GpioDataRegs.GPASET.bit.GPIO17 = ENABLE;
          // #endif
                         EDIS;

          #endif
//                       UARTCharPutNonBlocking(UART1_BASE, "?");
//                       UARTwrite("ATDT?", 5);
//          #endif
 ////                        delay_loop();
          #if 0
          #ifdef ENABLE_HRCAP_IRQ2
                         EALLOW;
                                   PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 8
                                   EDIS;
          #endif
          #ifdef ENABLE_HRCAP_IRQ1
                                   EALLOW;
                                   PieCtrlRegs.PIEIER4.bit.INTx7=0;     // Enable PIE Group 4, INT 7
                                   EDIS;
          #endif
          #endif
          #if 0
          #ifdef ENABLE_HRCAP_IRQ4
                                   EALLOW;
                                   PieCtrlRegs.PIEIER5.bit.INTx5=0;     // Enable PIE Group 4, INT 8
                                   EDIS;
          #endif
          #ifdef ENABLE_HRCAP_IRQ3
                                   EALLOW;
                                   PieCtrlRegs.PIEIER5.bit.INTx4=;     // Enable PIE Group 4, INT 8
                                   EDIS;
          #endif
          #endif

	      }

      } // end for
}

// electronic control task - mostly test now; need to dial in --- jcw
void EC_Task(void * pvParameters)
{

//    HRCAP2Queue xMessage;

static Uint32 qdataL;
static Uint32 sdataL;  // send data
static Uint32 rdataL;  // received dat
static Uint8 counter, counterp, window_ctr, min_ctr, max_ctr = 0;
static Uint16 i, duty_inc, duty_save;
static bool max_min_flag = pdFALSE;
static bool phase_lock_flag = pdFALSE;
static Uint8 task_suspend_counter = 0;
static Uint8 l = 0;
static Uint8 lock_counter = 0;

static float x, y, z, yp_k = 0.0f;
static float saturationLevel = 0.0;
static float rk_pl, rk_ll = 0.f;            // rk phase lock, rk last lock
///static float HRCAP_Diff = 0;
static float HRCAP_Adjust = 0;

	//TODO, Calculate ???
static float PWMX_LAST;                // PWM_X
// static float PID_Adjust;
static float PID_Composite;
static float HRCAP_BAND;
// static float HRCAP_Diff = 0.0f;

#ifdef USE_ALTERNATE_PID
    pid *p;
    p = ( struct _pid * ) (&pvParameters);
    p = controlPID;
#endif
    HRCAP_BAND = (float) 48.0f;
    sdataL = 0xf0f0f0f0;
//	rdataL = 0x11111111;
    rdataL = 0xffffffff;
	qdataL = 0xf0f0f0f0;
//	duty_inc = 255;
//	duty_inc = 575;
	duty_inc = 1100;
	x = 100.0f;
	y = 100.0f;
#ifndef	NO_INTEGRAL_SATURATION
	saturationLevel = 1.0f;
#endif
#if USE_SYNTHETIC_UK
	z = UK_MAX;
#endif
//	rk = 0.50f;
	for(EVER)
	{
//	    vTaskSuspend(NULL);

        if(xSemaphoreTake( EC_TaskSemaphore, portMAX_DELAY ) == pdTRUE)
        {
		USBBufferFlush((tUSBBuffer *)&g_sTxBuffer);


#ifndef USE_ALTERNATE_PID
#if 0		
		if(xSemaphoreTake( xHRCAP1Semaphore, portMAX_DELAY ) == pdTRUE)
		{
			testLedToggle();
			vTaskDelay(500 / portTICK_PERIOD_MS);
		}		
#endif
#if 1
#if PGA_DEBUG
#if 1
		  if ( counterp > 2 && phase_lock_flag != pdTRUE )
		  {
#endif
#if 1
		  DINT;
		  DSP28x_usDelay(10);
          spi_xmit(rdataL, 'p');      // set PGA
          DSP28x_usDelay(100);
          EINT;
#endif
#if 1
          counterp = 0;
          rdataL += 0x11111111;
          if ( rdataL >= 0xffffffff)
          {
//              rdataL = 0x11111111;
              rdataL = 0x00000000;
          }
		  } else {
		      counterp++;
		      rdataL = 0x77777777;
		  }
#endif
#endif
//		  counter = 5;
#if 0
		  switch ( counter++ )

		  {

		  case 0:
		      sdataL = 0xffff0000;
		      qdataL = 0xffff0000;
		      break;

		  case 1:
		      sdataL = 0x00ffff00;
		      qdataL = 0x00ffff00;
		      break;

		  case 2:
		      sdataL = 0x0000ffff;
		      qdataL = 0x0000ffff;
		      break;

		  case 3:
		      sdataL = 0xff00ff00;
		      qdataL = 0xff00ff00;
		      break;

		  case 4:
		      sdataL = 0x00ff00ff;
		      qdataL = 0x00ff00ff;
		      break;

		  case 5:
		      sdataL = 0xf0f0f0f0;
		      qdataL = 0xf0f0f0f0;
		      break;

		  case 6:
		      sdataL = 0x0f0f0f0f;
		      qdataL = 0x0f0f0f0f;
		      break;

		  case 7:
		      sdataL = 0x07070707;
		      qdataL = 0x07070707;
		      break;

		  case 8:
		      counter = 0;
		      break;

		  default:
		      counter = 0;
		      break;

		  }

#endif
#if 1
		  if ( sdataL == 0xffffffff)
		  {
		      sdataL = 0x10101010;
		      qdataL = 0x10101010;
		  }
//		  sdataL++;
//		  qdataL++;
		  sdataL += 0x01010101;
		  qdataL += 0x01010101;
#endif
#if 0


	          if ( sdataL <= 0x00000000)
	          {
	              sdataL = 0xf0f0f0f0;
	              qdataL = 0xf0f0f0f0;
	          }

	          sdataL -= 0x10101010;
	          qdataL -= 0x10101010;
#endif
//		  DINT;

////          spi_xmit(sdataL, 'l');      // set 'L' DAC
//          EINT;
//          vTaskDelay(200 / portTICK_PERIOD_MS);
//          DINT;
//          vTaskDelay(100 / portTICK_PERIOD_MS);
#if 1
////          DSP28x_usDelay(100);
////          spi_xmit(qdataL, 'h');      // set 'H' DAC
//          EINT;
//          vTaskDelay(200 / portTICK_PERIOD_MS);
//          vTaskDelay(100 / portTICK_PERIOD_MS);
//          DINT;
#if 0
          DSP28x_usDelay(100);
		  spi_xmit(rdataL, 'p');      // set PGA
#endif
#endif
//		  EINT;
#endif
//		  vTaskDelay(200 / portTICK_PERIOD_MS);
//		  ERTM;
#if 0
		  if ( sdataL < 0x0000ffff )
		  {
			  sdataL++;
			  qdataL++;
		  }
#endif
#if 0
		  if ( rdataL < 0x00ffffff )
			  rdataL++;
		  else
		      rdataL = 0;
#endif
#if 0
		  if (sdataL >= 0x0000ffff)
		  {
#if 0			  
			  EALLOW;
			  GpioCtrlRegs.GPAMUX1.bit.GPIO4 = DISABLE;              //
			  GpioDataRegs.GPASET.bit.GPIO4 = ENABLE;
			  GpioCtrlRegs.GPADIR.bit.GPIO4 = ENABLE;
			  GpioDataRegs.GPADAT.bit.GPIO4 = DISABLE;
			   //  GpioCtrlRegs.GPAPUD.bit.GPIO4 = ENABLE;                // this means NO pullup is enabled
			  // GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = ENABLE;
			  EDIS;
#endif
			  sdataL = 0x0;
			  qdataL = 0x0;
//			  rdataL = 0x0;
		  }
#endif
		  // read ADC channel
//		      yk = ((float) AdcResult.ADCRESULT0 - 2048.0f) / 2047.0f;
//		      lk = (float) AdcResult.ADCRESULT1;

//		  yk = ((float)xMessage.int_HRC2pulsewidthlow1[0] / 100.);
//		  yk = ((float)xMessage.int_HRC2pulsewidthlow1[0] / 100.);
// #ifdef NTEST_DACS
#if 1
		  if (control_valH < 52 && control_valL >= 47)

		  {
#endif
#ifndef RK_IS_SET
		      if ( phase_lock_flag != pdTRUE)
		      {
		      rk = y/100.0f;

		     if ( y == 0.0f)
		          y = 100.0f;
		      y--;
		      }
#endif

#if RUN_LINEAR_PID
#if 0

		      if ( pid1->Kp < 1.0f )
		      {
		          pid1->Kp = 4.5f;
		          pid1->sps->Kp = 4.5f;
		      }   else {
		          pid1->Kp -= 0.01f;
		          pid1->sps->Kp -= 0.01f;
		      }
		      if ( pid1->Kd < 0.01f )
		      {
		          pid1->Kd = 0.2f;
		          pid1->sps->Kd = 0.2f;
		       }   else {
		          pid1->Kd -= 0.01f;
		          pid1->sps->Kd -= 0.01f;
		       }
#else
//		     pid1->Kd = 0.01f;
//		      pid1->sps->Kp = 2.1f;
//		      pid1->sps->Kd = 0.01f;

#if 1
		      pid1->Kp = 3.0f;
		      pid1->sps->Kp = 3.0f;
		      pid1->Kd = 0.15f;
		      pid1->sps->Kd = 0.15f;

//		      pid1->Ki = 0.00f;
//		      pid1->sps->Ki = 0.00f;
#endif

#if 0
		      if ( pid1->Ki < 0.01f )
		      {
		           pid1->Ki = 0.1f;
		           pid1->sps->Ki = 0.1f;
		      }   else {
		           pid1->Ki -= 0.01f;
		           pid1->sps->Ki -= 0.01f;
		      }
#endif

#endif

#else
#if 1
		      nlpid1->sps->Kp = 3.5f;     // 1.0           // NOTE:  If this isn't done - the PIDs will just output Umax and Umin...
		      nlpid1->sps->Kd = 0.01f;     // 0.0
		      nlpid1->Kp = 3.5f;          // 1.0
		      nlpid1->Kd = 0.01f;          // 0.0

#endif
#define FOO_TUNE (1)
#endif
//		   yk = (((float)control_valH + (float)control_valL) / 200.);
#ifndef RUN_NON_LINEAR_PID_C2

#ifdef RUN_OPEN_LOOP_TESTING
		  yk = x/100.0f;

		  if ( x == 0.0f)
		      x = 100.0f;
		  x--;
#else
//		      yk = ((float)control_valH / 100.0f);
		  S_PID_VDATA.yk = fminf((float)control_valH,(float)control_valL) / 100.0f;
//		  S_PID_VDATA.yk = (float)(control_valH / 100.0f);
//		  S_PID_VDATA.yk = fminf((float)control_valH,(float)control_valL);
//		  S_PID_VDATA.yk = fminf((float)control_valH,(float)control_valL) / 1000.0f;
//		      yk *= 2.0f;
#endif
//		  yk = (((float)control_valL + (float)control_valH) / 200.0f );                           /// remember we are using an AND gate here right now - this is measured process variable
/////		  yk = ((float)control_valH / 100. );
		  ///		  yk = ((float)(fabs( 0.50f - (float)control_valH / 100.)));
//		  lk = ((float)xMessage.int_HRC2pulsewidthlow1[0]);
//		      lk = ((float)control_valL / 100.0f);
//		  S_PID_VDATA.lk = fmaxf((float)control_valH,(float)control_valL);
///		  S_PID_VDATA.lk = fmaxf((float)control_valH,(float)control_valL) / 100.0f;
//		  S_PID_VDATA.lk = fmaxf((float)control_valH,(float)control_valL) / 1000.0f;
//		   lk = 0.0f;
//		  lk = ((float)control_valL );
		  S_PID_VDATA.lk = 1.0f;
#else
#if RUN_NON_LINEAR_PID_C2
//		  S_PID_VDATA.yk = ((float)control_valH / 100.0f );                           /// remember we are using an AND gate here right now - this is measured process variable
		  /////         yk = ((float)control_valH / 100. );
//          S_PID_VDATA.yk = ((float)(fabs( 1.00f - (float)control_valH / 100.)));
		  //        lk = ((float)xMessage.int_HRC2pulsewidthlow1[0]);

#ifdef RUN_OPEN_LOOP_TESTING
		  yk = x/100.0f;

		          if ( x == 0.0f) {
		              x = 100.0f;
		          //    rk = 0.96f;           // 2*0.48
		          __asm("   NOP"); // for BP
		          }
		          x--;

		          yk = 0.480f;
#else
//			  yk = fabsf((float)control_valH / 100.0f );
///		          yk = ((float)control_valL / 100.0f );
//		          yk *= 2.0f;
//		          yk = (fmaxf((float)control_valH,(float)control_valL) - fminf((float)control_valH,(float)control_valL)) / 100.0f ;
//			  yk =  fabsf(fabsf(rk/2.0f - ((float)control_valH) / 100.0f ) + fabsf(rk/2.0f - ((float)control_valL) / 100.0f));
//			  yk =  fabsf(fabsf(rk/2.0f - ((float)control_valH) / 100.0f ) + fabsf(rk/2.0f - ((float)control_valL) / 100.0f)) / 2.0f;
//			  yk = fabsf((fmaxf((float)control_valH,(float)control_valL) - fminf((float)control_valH,(float)control_valL))) / 100.0f ;
//			  yk = 2.0f * fminf((float)control_valH,(float)control_valL) / 100.0f;
		      S_PID_VDATA.yk = fminf((float)control_valH,(float)control_valL) / 100.0f;
//		      S_PID_VDATA.yk = fminf((float)control_valH,(float)control_valL);
//		      S_PID_VDATA.yk = fmaxf((float)control_valH,(float)control_valL);
//		      yk = fminf((float)control_valH,(float)control_valL) / 100.0f;
//		          yk = fmaxf((float)control_valH,(float)control_valL) / 100.0f;
//			  yk =  fabsf(rk - (fmaxf((float)control_valH,(float)control_valL) / 100.0f));
//			  yk =  fabsf(rk - (fminf((float)control_valH,(float)control_valL) / 100.0f));
//			  yk =  (fmaxf(fabsf(rk - ((float)control_valH) / 100.0f ),fabsf(rk - ((float)control_valL) / 100.0f)) + fminf(fabsf(rk - ((float)control_valH) / 100.0f ),fabsf(rk - ((float)control_valL) / 100.0f))) / 2.0f;
//			  yk = rk - yk;
#endif
//		      S_PID_VDATA.lk = fmaxf((float)control_valH,(float)control_valL) / 100.0f;
//		      S_PID_VDATA.lk = fmaxf((float)control_valH,(float)control_valL);
//		      lk = fmaxf((float)control_valH,(float)control_valL) / 100.0f;
		      S_PID_VDATA.lk = 1.0f;
//		      lk = 1.0f;
//		      lk = 0.0f;
#ifdef FP_DEBUG_DBGTRC_4145			  
		         sprintf((Uint8 *)&debug_string[0], "lk:%f, yk:%f\r\n", lk, yk);  /* (100 - (Duty/1440.)*100.)*/

		         USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));
#endif
//          lk = ((float)(fabs( 1.00f - (float)control_valL / 100.)));
#endif
#endif
/////		  lk = ((float)control_valL / 100.);
//		  lk = ((float)(fabs( 0.48f - (float)control_valL / 100.)));    // ok to bias?  saturation limit
//		  rk = (float) 0.48;
		  // set point in %
#if 1
#ifndef USE_SYNTHETIC_UK
		  if ( S_PID_VDATA.yk > 1.0f)
//		      if ( yk > 1.0f)
		  {
//		      yk = 096.0f;
		      edge_flag = pdTRUE;
		  } // else {
		    //  edge_flag = pdFALSE;
		 //  }
#endif
		  if ( S_PID_VDATA.lk > 1.0f) {
//		      if ( lk > 1.0f) {
//		      lk = 096.0f;
		      edge_flag = pdTRUE;
		  }
#endif
//		  pid1.Ki = 0.015f;
//		  pid1.Kp = 9.0f;
//		  pid1.Kd = 0.35f;
		      // external clamp for anti-windup reset
//		     DCL_runClamp_C1(&lk, 0.56f, 0.420f);
//		     DCL_runClamp_C1(&lk, 1.0f, 0.0f);
#ifndef USE_SYNTHETIC_UK
		  if (edge_flag != pdTRUE)                                    // ignore readings 'made on the edge' or corner cases...
#endif
		  {
#if RUN_LINEAR_PID
#ifndef	NO_INTEGRAL_SATURATION			  
			  lk = saturationLevel;
#endif
taskENTER_CRITICAL();
#pragma FUNC_ALWAYS_INLINE(DCL_runClamp_C1)

		     DCL_runClamp_C1(&(S_PID_VDATA.lk), 1.0f, 0.0f);                     // 0.48 is ideal, so 0.96 is 2xIdeal - Umax and Umin
taskEXIT_CRITICAL();
		     // run PID controller
#ifdef NO_SP_WEIGHT
taskENTER_CRITICAL();
#pragma FUNC_ALWAYS_INLINE(DCL_runPID_C4)
                        S_PID_VDATA.uk = DCL_runPID_C4(pid1, S_PID_VDATA.rk, S_PID_VDATA.yk, S_PID_VDATA.lk);    // Does not use SP weighting
taskEXIT_CRITICAL();
#else
// taskENTER_CRITICAL();
#pragma FUNC_ALWAYS_INLINE(DCL_runPID_C1)
                        S_PID_VDATA.uk = DCL_runPID_C1(pid1, S_PID_VDATA.rk, S_PID_VDATA.yk, S_PID_VDATA.lk);    // Uses SP weighting
// taskEXIT_CRITICAL();
#endif
#else
		     // external clamp for anti-windup reset
#ifndef NO_INTEGRAL_SATURATION
		         lk = saturationLevel;
#endif
taskENTER_CRITICAL();
		         DCL_runClamp_C2(&(S_PID_VDATA.lk), 1.0f, 0.0f);
//		         DCL_runClamp_C2(&lk, 1.0f, 0.0f);
taskEXIT_CRITICAL();
		         // run non-linear PID controller
#if RUN_NON_LINEAR_PID_C1
//		         lk = 1.0f;                                         // manually saturate this now
// taskENTER_CRITICAL();
                                S_PID_VDATA.uk = DCL_runNLPID_C1(nlpid1, S_PID_VDATA.rk, S_PID_VDATA.yk, S_PID_VDATA.lk);
// taskEXIT_CRITICAL();
#endif
#if RUN_NON_LINEAR_PID_C2
/*
#define USE_SYNTHETIC_UK (1)
#ifdef USE_SYNTHETIC_UK
#define UK_MAX (float) 1.0
#define UK_MIN (float) -1.0
#endif
*/
#if USE_SYNTHETIC_UK
		         uk = z / 100.0f;

		                 if ( z == UK_MIN )
		                     z = UK_MAX;

		                 z = z - 1.0f;
#else
//		                 The controller ‘run’ functions are not re-entrant, since they
//		                 rely on a global variable (in this case a structure).
taskENTER_CRITICAL();
                    S_PID_VDATA.uk = DCL_runNLPID_C2(nlpid1, S_PID_VDATA.rk, S_PID_VDATA.yk, S_PID_VDATA.lk);
//                    uk = DCL_runNLPID_C2(&nlpid1, rk, yk, lk);
taskEXIT_CRITICAL();
#endif
#endif
#endif
//		      DCL_runClamp_C1(&lk, 270.0f, 45.0f);
		      // write u(k) to PWM
	//	      Duty = (uk / 2.0f + 0.5f) * (float) EPwm7Regs.TBPRD;                      // EPWM7A
	//	      Duty = (uk / 2.0f + 0.5f) * (float) 360.0f;                               // EPWM7A
	//	      Duty = (uk / 2.0f + 0.5f) * (float) 1440.0f;
	//	      Duty = (fabsf(uk) / 2.0f + 0.5f) * (float) 1440.0f;
#if RUN_LINEAR_PID
		      Duty = (S_PID_VDATA.uk / 2.0f + 0.5f) * (float) 1440.0f;
#else
              Duty = (S_PID_VDATA.uk / 2.0f + 0.5f) * (float) 1440.0f;
//              Duty = (S_PID_VDATA.uk / 2.0f + 0.5f) * (float) 2880.0f;
#endif
//              Duty = (S_PID_VDATA.uk / 2.0f + 0.5f) * (float) 1440.0f;
//		      Duty = (uk / 2.0f + 0.5f) * (float) 1440.0f;
//		      Duty = (1.0f - uk) * (float) 1440.0f;
//	      Duty = (uk) * (float) 1440.0f;
	//	      Duty = (uk) * (float) 1440.0f;
//		      Duty = (((100.0f - (uk / 20.0f + 050.0f)) / 100.0f ) * (float) 1440.0f);
///		      Duty = (1.0f - (uk / 2.0f + 0.50f)) * (float) 1440.0f;                    // normalized version for duty cycle - positive
//		      Duty = (100.0f - (uk / 200.0f + 50.0f)) / 100.0f   * (float) 1440.0f;
		      // period - 360 - 360 / 2 -> 180 is approx 50%		      
#if 0
//		      Duty =  ( 1. / Duty ) * 100.0;
//		      Duty =  ( 1. / uk ) * 100.0;

		      if (Duty < 1.)          // set min
		      {
		          Duty = 0.;
		      }
#if 0
		      else {

		          Duty *= 0.5;
		      }
#endif

//		      EPwm7Regs.CMPA.half.CMPA = (float) EPwm7Regs.TBPRD - i++;
//		      EPwm7Regs.CMPA.half.CMPA = (float) EPwm7Regs.TBPRD / (Uint16) Duty;
//		      EPwm7Regs.CMPA.half.CMPA = (float) 360.0f / (Uint16) Duty;
		      EPwm7Regs.CMPA.half.CMPA = (Uint16) (Duty);
// 		      EPwm7Regs.CMPA.half.CMPA = (float) 57;
#endif
#if 0
		  } else {
		               sprintf((Uint8 *)&debug_string[0], "<LOCK>" );
		               USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));

		  }
#endif
//		  control_valL = HRCAP2Data.int_HRC2pulsewidthlow1[2];
//		  control_valH = HRCAP2Data.int_HRC2pulsewidthhigh1[2];
		 // check for control_val min		 
		  // pos/neg slope - check CL vs CH
#ifdef RUN_MAN_CONTROL
		  if ( control_valL < control_valL_min )
		  {
			  control_valL_min = control_valL;
			  max_min_flag = pdTRUE;
			  min_ctr++;
		  } else {
			  max_min_flag = pdFALSE;
			  max_ctr++;
		  }				 
			  
                  window_ctr++;

//		  if ( max_min_flag == pdFALSE && min_ctr >= 1 )
//	         if ( max_min_flag == pdTRUE )
//		  {
		  // good sample
#if 1
		  if ( control_valL <= 56 && control_valH <= 51 ) {
			  duty_save = duty_inc;					// save
			  phase_lock_flag = pdTRUE;
#if 1
			  duty_inc -= 2;
#endif
		  } else {
#if 1
		      if ( phase_lock_flag == pdTRUE)
		          duty_inc += 1;
#endif
		      phase_lock_flag = pdFALSE;
		  }
#endif
//		  if ( (control_valL > control_valH > 92) && (control_valL + control_valH < 101) )
//		  {

		  if ( phase_lock_flag != pdTRUE )
		  {
		  if ( (control_valL + control_valH > 92) && (control_valL + control_valH < 101) )
		  {
		  
		  if ( control_valL > (48 + 5 ) && control_valH < ( 47 - 5 ) )
		  {
//			  duty_inc++;
			  duty_inc--;
		  } else
		  { if ( control_valL < ( 47 - 5 )) {
//		      duty_inc--;
		      duty_inc++;
		  }
		  }

//		  }
#if 0		  
		  if ( control_valL < 47 && control_valH > 48 )
		  {
			  duty_inc++;

		  }
		  else { if ( control_valH < 47 ) {
			  duty_inc--;
		  }  
		  }
#endif		  

//		  }
#if 1		  
		  } else { if ( control_valL < 47 && control_valH > 48 )
			  {
//				  duty_inc++;
				  duty_inc--;
			  }
			  else { if ( control_valL > 48 ) {
//				  duty_inc--;
			      duty_inc++;
			  }  
			  }
		  }

		  }
#endif
		  if ( window_ctr >= 5 ) {

			  window_ctr = 0;
			  control_valL_min = 1000;
			  control_valH_min = 1000;
			  max_min_flag = pdFALSE;
			  min_ctr = 0;
			  max_ctr = 0;
		  }
		  
#endif
//		  EPwm7Regs.CMPA.half.CMPA = (Uint16) (duty_inc);
//		  if ( S_PID_VDATA.rk != S_PID_VDATA.yk && S_PID_VDATA.uk != 0.0f && S_PID_VDATA.uk != 100.0f )
		  if ( S_PID_VDATA.rk != S_PID_VDATA.yk )               // damp this with phase_lock_flag counter
		  EPwm7Regs.CMPA.half.CMPA = (Uint16) (Duty);
		  }
	}
//		  sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, D%f, uk:%f, lk:%f, PDC:%f, di:%d, ds:%d\r\n", control_valH, control_valL, Duty, uk, lk, (100 - (Duty/1440.)*100.),duty_inc, duty_save );
// #ifndef NO_USB_PRINTING
#ifdef NOT_DOING_OPEN_LOOP_TESTING
		  if ( control_valL >=45 && control_valL <=50 && control_valH >= 45 && control_valH <= 50) {
//			  sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, D%f, uk:%f, lk:%f, PDC:%f, DC:%d, HR:%d, HF:%d <:!:=>DPHASE LOCK<=:!:>\r\n", control_valH, control_valL, Duty, S_PID_VDATA.uk, S_PID_VDATA.lk, (100 -(Duty/1440.)*100.), HRCAP2Data.datacounter2, HRCAP2Data.HRC2_HCCAPDLYRISE0[HRCAP2Data.datacounter2], HRCAP2Data.HRC2_HCCAPDLYFALL0[HRCAP2Data.datacounter2]);  /* (100 - (Duty/1440.)*100.)*/
#ifndef NO_USB_PRINTING
		      sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, D%f, uk:%f, lk:%f, PDC:%f, yk:%f, DC:%d <:!:=>DPHASE LOCK<=:!:>\r\n", control_valH, control_valL, Duty, S_PID_VDATA.uk, S_PID_VDATA.lk, (100 -(Duty/1440.)*100.), S_PID_VDATA.yk, HRCAP2Data.datacounter2 );  /* (100 - (Duty/1440.)*100.)*/
#endif
		      phase_lock_flag = pdTRUE;
		      edge_flag = pdFALSE;
		      rk_pl = S_PID_VDATA.rk;
		      rk_ll = S_PID_VDATA.rk;
		      DutyData->Duty_data[l] = Duty;              // keep track of the last duty cycle calculations in a lock state
//		      l++;
		      if (l++ > 9)
		      l = 0;

		  } else
#endif
		  {                                                                                                                           // (100 -(Duty/1440.)*100.)
//			  sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, D%f, uk:%f, lk:%f, PDC:%f, rk:%f, yk:%f, HR:%d, HF:%d, DC:%d, EF:%d\r\n", control_valH, control_valL, Duty, S_PID_VDATA.uk, S_PID_VDATA.lk, (100 - (Duty/1440.)*100.), S_PID_VDATA.rk, S_PID_VDATA.yk, HRCAP2Data.HRC2_HCCAPDLYRISE0[HRCAP2Data.datacounter2], HRCAP2Data.HRC2_HCCAPDLYFALL0[HRCAP2Data.datacounter2], HRCAP2Data.datacounter2, edge_flag);  /* (100 - (Duty/1440.)*100.)*/
//			  sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, D%f, uk:%f, lk:%f, PDC:%f, rk:%f, yk:%f, DC:%d, EF:%d, P:%lx\r\n", control_valH, control_valL, Duty, S_PID_VDATA.uk, S_PID_VDATA.lk, (100 - (Duty/1440.)*100.), S_PID_VDATA.rk, S_PID_VDATA.yk, HRCAP2Data.datacounter2, edge_flag, PID_ID);  /* (100 - (Duty/1440.)*100.)*/
#ifndef NO_USB_PRINTING
		      sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, D%f, uk:%f, lk:%f, PDC:%f, rk:%f, yk:%f, DC:%d, EF:%d\r\n", control_valH, control_valL, Duty, S_PID_VDATA.uk, S_PID_VDATA.lk, (100 - (Duty/1440.)*100.), S_PID_VDATA.rk, S_PID_VDATA.yk, HRCAP2Data.datacounter2, edge_flag);  /* (100 - (Duty/1440.)*100.)*/
#endif
		      phase_lock_flag = pdFALSE;
		      edge_flag = pdFALSE;
		      rk_pl = 0.0f;
#if 0
		      if (rk_ll != 0.f)
		          S_PID_VDATA.rk = rk_ll;
#endif
		  }
#ifndef NO_USB_PRINTING
		  USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));
#endif


//
//		  memset(HRCAP2Data.int_HRC2pulsewidthlow1[0], '/0', sizeof(HRCAP2Data));
#if 0
		  if ( duty_inc > 260 || duty_inc < 230)
		                duty_inc = 260;

		  if ( duty_inc > 600 || duty_inc < 400)
		                          duty_inc = 575;
#endif
		  if ( duty_inc > 1400 || duty_inc < 700)
		                duty_inc = 1100;

		  HRCAP2Data.runECTask = pdFALSE;

#ifndef NO_USB_TASK_FOR_OUTPUT
		  if( xSemaphoreGive( xUSBPrintSemaphore ) != pdTRUE )
		                 {
		                                    DINT;
		                                    while(1);
		                 }
#endif

#if RUN_NON_LINEAR_PID
// #ifndef RUN_OPEN_LOOP_TESTING
                      // update linear gains if new NL parameters
taskENTER_CRITICAL();
                               DCL_updateNLPID(nlpid1);                     // new in 1.05
//		  ;
                      //        DCL_fupdateNLPID(&nlpid1);                     // new in 1.05
taskEXIT_CRITICAL();
// #endif
#else
taskENTER_CRITICAL();
#pragma FUNC_ALWAYS_INLINE(DCL_fupdatePID)

                   DCL_fupdatePID(pid1);
taskEXIT_CRITICAL();
//		  ;
#endif
#else
////		if (control_valH < 52 && control_valL >= 47)
///	        if (control_valH < 100 && control_valL >= 47)
		{
#if 0
		    if ( *(p->sp) == 0 )
		    {
		        sprintf((Uint8 *)&debug_string[0], "SET POINT IS ZERO!\r\n" );
		        USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));
///		        error();
///		        *(p->sp) = (float) 48.0f;

		    }
#endif

			*(p->pv) = fminf((float)control_valH,(float)control_valL);
//			*(p->pv) *= 2.0f;
//			if ( *(p->pv) > 48.0f)
//			    *(p->pv) = 48.0f;

//			*(p->pv) = fabsf(*(p->pv) - 24.0f);                      // bias but split measurement down the middle
//		    *(p->pv) = fmaxf((float)control_valH,(float)control_valL) / 2.0f;
//		    *(p->pv) = fmaxf((float)control_valH,(float)control_valL);
		   
//		calcStartHRCAPCount0 = *(p->pv);
		
		// TODO, Calculate ???
		PWM_X = 0.0;
		//	PWM_X = 0;
		PID_Adjust = 1.0;
		PID_Composite = 0.0;
#if 0		
		if (( *(p->pv) >= *(p->sp) - HRCAP_BAND ) && (*(p->pv) <= *(p->sp) + HRCAP_BAND ))
		{
		    pid_tune ((struct _pid *)(p), p->pgain, p->igain, p->dgain, p->deadband );
		    pid_setinteg((struct _pid *)(p),p->integral);
		    pid_bumpless((struct _pid *)(p));
		}
#endif
taskENTER_CRITICAL();
			pid_calc((struct _pid *)(p));
taskEXIT_CRITICAL();
		PID_Composite = (p->uof);

		HRCAP_Delta = *(p->pv) - HRCAP_OldData;
		HRCAP_OldData = *(p->pv);
		PID_Delta = PID_Composite - PID_OldData;
		PID_OldData = PID_Composite;
		
		if ( (*(p->sp)) > 100.0f )			/// xxx::: upper limit guard used during testing
			(*(p->sp)) = 48.0f;

		HRCAP_Diff = (*(p->pv) - (*(p->sp)));			// this is same as error term

		PWM_X = PID_Composite;
		
//		PWM_X /= 2.0f;
#if 0
		if ( HRCAP_Diff < 0 )	// heater on
		{
//			heater_on = 1;
		}
#endif
#if 0
		if ( HRCAP_Diff > 0 )	// cooling cycle
		{
//			heater_on = -1;

#ifdef COOL_CYCLE_DEBUG_CURRENT_CHECK

			if ( HRCAP_Diff > 25. )			// still have to drive hard to make temp in range times
			{ 
#if 1			
				if ( (*(p->sp) == 20.0 && (*(p->pv) <= 20.0) && WE_ARE_HERE++ < 250) && WE_ARE_HERE_FLAG )				// just for this case now for test
				{
					PWM_X = 100;				// artificially bias gain until those functions are ready to go in the GUI; and this is currently for testing
				} else {
					WE_ARE_HERE = 0;					// only saturate for limited time
					WE_ARE_HERE_FLAG = 0;
					PWM_X *= p->PWMCoolingRatio;
				}
			}
#endif
#endif			
//			}
		}
#endif
		if ( HRCAP_Diff == 0 )	// neither
		{
//			heater_on = 0;
			PWM_X = 0.0f;
		}

		if (PWM_X < 0)
		{	//bug if hit					// xxx:::
			PWM_X = -1 * PWM_X;
		}
		if (PWM_X >= 100)					// orig float to int comparison
		{
			PWM_X = 100;
//		    PWM_X = 00.0f;
		}
//		PWM_state = PWM_X;
//		heater_state = heater_on;

		//protection
#if 0		
		if ((*(p->pv)  >= 100.0f ) || //in case over heat											// malfunctioning HRCAP will still display strange readings possibly but PWM will be OFF.
		    (*(p->pv)  <= 0.0f))	  //in case under cool
//		if ((calcTempLocal  >= 65.0) || //in case over heat
//				(calcTempLocal  <= 10.0))	  //in case under cool		
		{
			heater_on = 0;
			PWM_X = 0;
		}
#endif		
//		if ((heater_on == 1) || (heater_on == -1))
		{
//		PWM_SetDC(1,654.0*PWM_X/100.0);
//		PWM_SetDC(2,654.0*PWM_X/100.0);

#ifdef FTX700D
			if (heater_on == 1)
			{
				GPIO_WriteBit(GPIOA, BRIDGE_PWM_CONTROL_1, Bit_SET);			//enable
				GPIO_WriteBit(GPIOA, BRIDGE_PWM_CONTROL_2, Bit_SET);			//heater
				PWM_SetDC(2,(uint16_t)(654.0*PWM_X/100.0));
			}
			else if (heater_on == -1)
			{
				GPIO_WriteBit(GPIOA, BRIDGE_PWM_CONTROL_1, Bit_SET);		    //enable
				GPIO_WriteBit(GPIOA, BRIDGE_PWM_CONTROL_2, Bit_RESET);			//cooler
				PWM_SetDC(2,(uint16_t)(654.0*PWM_X/100.0));
//			PWM_SetDC(2,100.0);
			}
#endif

		}
//	}

//		if (heater_on == 0)
		{
#if 0	//turn on for test 
//		PWM_SetDC(1, 25);
////xxx			PWM_SetDC(2, 75);
#else			
//		PWM_SetDC(1, 0);
////xxx			PWM_SetDC(2, 0);
#endif
#ifdef FTX700D
			GPIO_WriteBit(GPIOA, BRIDGE_PWM_CONTROL_1, Bit_RESET);			//heater/cooler
			GPIO_WriteBit(GPIOA, BRIDGE_PWM_CONTROL_2, Bit_RESET);			//disable
#endif
		}

//
#if 0
		if (phase_lock_flag == pdTRUE)
		{
			if (PWM_X > PWMX_LAST + 5.0f || PWM_X < PWMX_LAST - 5.0f)
				PWM_X = PWMX_LAST;
		}
#endif
		
//		if ( PWM_X != 0.0f )
		if ( *(p->sp) != *(p->pv))          // if error is Zero don't change
		{
		Duty = (PWM_X / 200.0f + 0.5f) * (float) 1440.0f;                    // 1440.
#if TRACK_DIGITAL_PHASE_LOCK
		if (!phase_lock_flag || lock_counter != 1)
#endif
		EPwm7Regs.CMPA.half.CMPA = (Uint16) (Duty);
		}
		}   // end if for valid values for measured process variable
// #endif
///		sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, control_valH, uk:%f, lk:%f, PDC:%f, rk:%f, yk:%f, HR:%d, HF:%d, DC:%d, EF:%d\r\n", control_valH, control_valL, Duty, S_PID_VDATA.uk, S_PID_VDATA.lk, (100 - (Duty/1440.)*100.), S_PID_VDATA.rk, S_PID_VDATA.yk, HRCAP2Data.HRC2_HCCAPDLYRISE0[HRCAP2Data.datacounter2], HRCAP2Data.HRC2_HCCAPDLYFALL0[HRCAP2Data.datacounter2], HRCAP2Data.datacounter2, edge_flag);  /* (100 - (Duty/1440.)*100.)*/
		if ( control_valL >=45 && control_valL <=50 && control_valH >= 45 && control_valH <= 50) {
//			sprintf((Uint8 *)&debug_string[0], "CH:%lu, CL:%lu, D:%f, PWM_X:%f, PV:%f, SP:%f, PDC:%f, Err:%f, DC:%d !DPHASE_LOCK!\r\n", control_valH, control_valL, Duty, PWM_X, *(p->pv), *(p->sp), (100 - (Duty/1440.)*100.), HRCAP_Diff, HRCAP2Data.datacounter2 );  /* (100 - (Duty/1440.)*100.)*/
			phase_lock_flag = pdTRUE;
			DutyData->Duty_data[l] = Duty;              // keep track of the last duty cycle calculations in a lock state
			l++;
	        lock_counter++;
#if 0
			if ( lock_counter >= 6 )
			{
			    lock_counter = 0;
			    vTaskSuspend(NULL);

			}
#endif
			if (l++ > 9)
			    l = 0;

			PWMX_LAST = PWM_X;
		} else {		
		
//			sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu,D:%f,PWM_X:%f,PV:%f,SP:%f,PDC:%f,Err:%f,PD:%f,DC:%d\r\n", control_valH, control_valL, Duty, PWM_X, *(p->pv), *(p->sp), (100 - (Duty/1440.)*100.), HRCAP_Diff, PID_Delta, HRCAP2Data.datacounter2 );  /* (100 - (Duty/1440.)*100.)*/
			phase_lock_flag = pdFALSE;
			lock_counter = 0;
		}			
//		  USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));

		
		  HRCAP2Data.runECTask = pdFALSE;

		  if( xSemaphoreGive( xUSBPrintSemaphore ) != pdTRUE )
		                                                   {
		                                                                      DINT;
		                                                                      while(1);
		                                                   }


#endif
#if 0
	          EALLOW;
	                 if ( PieCtrlRegs.PIEIER4.bit.INTx8 == 0 )
	                 {
	                 PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 8 - HRCAP2
	                 }
	          EDIS;
#endif

	          if( xSemaphoreGive( xControlECTaskSemaphore ) != pdTRUE )
		                 {
		                 /* This call should fail because the semaphore has not yet been
		                 ‘taken’. */
		                  DINT;
		                  while(1);
		                 }



#if 0
		      if( xSemaphoreGive( xControl2Semaphore ) != pdTRUE )
		       {
		           error();

		       } else {
		                                  ;
		         }
#endif
	}
// #ifdef DELAY_MAIN_HRCAP_CONTROL_LOOP
		vTaskDelay(100 / portTICK_PERIOD_MS);              // 1000
// #endif
#if 0
		task_suspend_counter++;

		             if (task_suspend_counter > 20)
		                 vTaskSuspend(NULL);                        // checking MTI-270 OCXO
#endif
	}



}            /// xxx:::
//
// scia_fifo_init -
//
void scia_fifo_init()
{
    //
    // 1 stop bit,  No loopback, No parity,8 char bits, async mode,
    // idle-line protocol
    //
    SciaRegs.SCICCR.all =0x0007;

    //
    // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    //
    SciaRegs.SCICTL1.all =0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA =1;
    SciaRegs.SCICTL2.bit.RXBKINTENA =1;
    SciaRegs.SCIHBAUD = 0x0000;
    SciaRegs.SCILBAUD = SCI_PRD;
    SciaRegs.SCICCR.bit.LOOPBKENA = 0;   // Enable loop back
    SciaRegs.SCIFFTX.all=0xC022;
    SciaRegs.SCIFFRX.all=0x0022;
    SciaRegs.SCIFFCT.all=0x00;

    SciaRegs.SCICTL1.all =0x0023;       // Relinquish SCI from Reset
    SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET=1;
}
//
// sciaTxFifoIsr -
//
__interrupt void
sciaTxFifoIsr(void)
{
    Uint16 i = 0;

//    for(i=0; i< 2; i++)
//    {
        SciaRegs.SCITXBUF=sdataA[i];     // Send data
//        ScibRegs.SCITXBUF=sdataA[i];     // Send data
//    }
#if 0
    for(i=0; i< 2; i++)                 //Increment send data for next cycle
    {
        sdataA[i] = (sdataA[i]+1) & 0x00FF;
    }
#endif
//    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;  // Clear SCI Interrupt flag
//    PieCtrlRegs.PIEACK.all|=0x100;      // Issue PIE ACK
//    ScibRegs.SCIFFTX.bit.TXFFINTCLR=1;  // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;      // Issue PIE ACK
}

//
// sciaRxFifoIsr -
//
__interrupt void sciaRxFifoIsr(void)
{
    Uint16 i;
    for(i=0;i<2;i++)
    {
        rdataA[i]=SciaRegs.SCIRXBUF.all;  // Read data
    }
    for(i=0;i<2;i++)                     // Check received data
    {
       if(rdataA[i] != ( (rdata_pointA+i) & 0x00FF) )
       {
///           error();
       }
    }
    rdata_pointA = (rdata_pointA+1) & 0x00FF;

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

//    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack

//    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
//    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack

}

//
// scib_fifo_init -
//
void scib_fifo_init()
{
    //
    // 1 stop bit,  No loopback, No parity,8 char bits, async mode,
    // idle-line protocol
    //
    ScibRegs.SCICCR.all =0x0007;

    //
    // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    //
    ScibRegs.SCICTL1.all =0x0043;                       // 0x0003
    ScibRegs.SCICTL2.bit.TXINTENA =1;
    ScibRegs.SCICTL2.bit.RXBKINTENA =1;
    ScibRegs.SCIHBAUD = 0x0000;
    ScibRegs.SCILBAUD = SCI_PRD;
    ScibRegs.SCICCR.bit.LOOPBKENA = 0;   // Enable loop back
    ScibRegs.SCIFFTX.all=0xC028;        // 0x22
    ScibRegs.SCIFFRX.all=0x0028;        // 0x22
    ScibRegs.SCIFFCT.all=0x00;

    ScibRegs.SCICTL1.all =0x0063;       // Relinquish SCI from Reset  // 23
    ScibRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    ScibRegs.SCIFFRX.bit.RXFIFORESET=1;
}

//
// scibRxFifoIsr -
//
__interrupt void scibRxFifoIsr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int32_t i32Char, i32Errors;
    uint8_t ui8Char;
    uint32_t ui32Space;
    static uint8_t i = 0;
    static uint8_t j = 0;


//    j = 0;
#if 0
    if ( (xSemaphoreTakeFromISR(xHRCAP1DataDebugSemaphore, &xHigherPriorityTaskWoken)) == pdTRUE )
    {
//        while ( debug_string[j] != NULL)
        while ( j < 20 )
        {
            sdataA[i] = debug_string[j];
            j++;

        }
        j = 0;
    }

#endif
#if 1

#if 0
//    sdataA[i] = (char)(struct pkt *)&SNTPpkt[j++];
      sdataA[i] = (char) ((char * )(ptr[j++]));
//    sdataA[i] = ui8Char;
    if ( j >= 48 )
        j = 0;
#endif

//    while(UARTCharsAvail(UART1_BASE))
    {
    ui8Char = (unsigned char)(ScibRegs.SCIRXBUF.all & 0x7f);
    if(ui8Char >= 0x0a )
    {
//        SciaRegs.SCITXBUF = ui8Char;
        sdataA[i] = ui8Char;



#if COPY_TO_USB_SERIAL_FROM_UBLOX
        USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (uint8_t *)&ui8Char, 1);
#endif
    }
#if 0
    if ( i == 2)
        i = 0;
#endif
    }
#endif
#if 0

//    portENTER_CRITICAL();
//    vTaskSuspendAll();
    //
    // Clear the error indicator.
    //
    i32Errors = 0;
    //
    // How much space is available in the buffer?
    //
#if 1
    ui32Space = USBBufferSpaceAvailable((tUSBBuffer *)&g_sTxBuffer);
#if 0
    if (!(ui32Space))
        {
        USBBufferFlush((tUSBBuffer *)&g_sTxBuffer);
        ui32Space = USBBufferSpaceAvailable((tUSBBuffer *)&g_sTxBuffer);
        }
#endif
    //
    // Read data from the UART FIFO until there is none left or there is no
    // more space in the receive buffer.
    //
//    vTaskSuspendAll();
    while(ui32Space && UARTCharsAvail(UART1_BASE))
#endif
//        while (UARTCharsAvail(UART1_BASE) )
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
//        if(!(i32Char & ~0xFF))
//         if (1)
        {
            ui8Char = (unsigned char)(i32Char & 0xFF);
           if (ui8Char >= 0x10 && ui8Char < 0x7F)
//              if (ui8Char < 0x7F)
            USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (uint8_t *)&ui8Char, 1);

            //
            // Decrement the number of bytes the buffer can accept.
            //
            ui32Space--;
        }
        else
        {
#ifdef DEBUG
            //
            // Increment our receive error counter.
            //
            g_ui32UARTRxErrors++;
#endif
            //
            // Update the error accumulator.
            //
            i32Errors |= i32Char;
        }
        //
                // Update our count of bytes received via the UART.
                //
                g_ui32UARTRxCount++;
    }
//    xTaskResumeAll();
 //   portEXIT_CRITICAL();
    //
    // Pass back the accumulated error indicators.
    //
//    UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, false));
//    USBBufferInit((tUSBBuffer *)&g_sTxBuffer);
//    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
//    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
//    return(i32Errors);
// }

#endif

#if 0

    Uint16 i;
    for(i=0;i<2;i++)
    {
        rdataB[i]=ScibRegs.SCIRXBUF.all;  // Read data
    }

    for(i=0;i<2;i++)                     // Check received data
    {

        USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (uint8_t *)&(rdataB[i]), 1);

    }


#if 0
    for(i=0;i<2;i++)                     // Check received data
    {
       if(rdataA[i] != ( (rdata_pointA+i) & 0x00FF) )
       {
///           error();
       }
    }
    rdata_pointA = (rdata_pointA+1) & 0x00FF;
#endif

    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

//    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack

//    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
//    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
    UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, false));

#endif
          UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, false));
//          USBBufferFlush((tUSBBuffer *)&g_sTxBuffer);
          ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
          ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
          SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;  // Clear SCI Interrupt flag
//          ScibRegs.SCIFFTX.bit.TXFFINTCLR=1;
//    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
         PieCtrlRegs.PIEACK.bit.ACK9 = 1;

}

void InitEPwm7Gpio(void)
{
    EALLOW;

    //
    // Disable internal pull-up for the selected output pins for reduced power
    // consumption. Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 1; // Disable pull-up on GPIO30 (EPWM7A)
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;
//    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 1;   // Disable pull-up on GPIO40 (EPWM7A)
    //GpioCtrlRegs.GPBPUD.bit.GPIO58 = 1; // Disable pull-up on GPIO58 (EPWM7A)

//    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1;   // Disable pull-up on GPIO41 (EPWM7B)
    //GpioCtrlRegs.GPBPUD.bit.GPIO44 = 1; // Disable pull-up on GPIO44 (EPWM7B)

    //
    // Configure EPWM-7 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM7 functional
    // pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 3;   // Configure GPIO30 as EPWM7A
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
//    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 1;   // Configure GPIO40 as EPWM7A
    //GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 3;   // Configure GPIO58 as EPWM7A

//    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 1;   // Configure GPIO41 as EPWM7B
    //GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 3;   // Configure GPIO44 as EPWM7B

    EDIS;
}

//
// FreqCtlISR - interrupts at ePWM1 TBCTR = 0.
// This ISR updates the compare and period registers for ePWM modules within
// the same period.
//
__interrupt void FreqCtlISR(void)
{
    EALLOW;
    EPwm7Regs.TBCTL.bit.PHSEN = 0;
    EDIS;

    isr_cnt++;

    if (update == 1)
    {
        EPwm7Regs.CMPA.half.CMPA = CMP_Reg;
        EPwm7Regs.CMPA.half.CMPAHR = CMP_HR;         // change duty cycle

        EPwm7Regs.CMPA.half.CMPA = CMP_Reg;
        EPwm7Regs.CMPA.half.CMPAHR = CMP_HR;         // change duty cycle

        update = 0;
    }

    //
    // re-initialise for next PWM interrupt
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;  // acknowledge PIE interrupt
    EPwm7Regs.ETCLR.bit.INT = 1;             // clear interrupt bit
}

//
// HRPWM_Config - Configures all ePWM channels and sets up HRPWM on ePWMxA
// channels
//
// PARAMETERS:  period - desired PWM period in TBCLK counts
// RETURN:      N/A
//
void HRPWM_ConfigLocal(period)
{
   Uint16 j;

   //
   // ePWM channel register configuration with HRPWM
   // ePWMxA toggle low/high with MEP control on Rising edge
   //
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;     // Disable TBCLK within the EPWM
   EDIS;

//   for (j=1;j<3;j++)
   j = 7;
   {
#if 0
       (*ePWM[j]).TBCTL.bit.PRDLD = TB_SHADOW;   // set Shadow load
       (*ePWM[j]).TBPRD = period;                // PWM frequency = 1/(2*TBPRD)
       (*ePWM[j]).CMPA.half.CMPA = period / 2;   // set duty 50% initially
       (*ePWM[j]).CMPA.half.CMPAHR = (0 << 8);   // initialize HRPWM extension
       (*ePWM[j]).TBPHS.all = 0;

       (*ePWM[j]).TBCTR = 0;

       //
       // Select up-down count mode
       //
       (*ePWM[j]).TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
       (*ePWM[j]).TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
       (*ePWM[j]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
       (*ePWM[j]).TBCTL.bit.CLKDIV = TB_DIV1;    // TBCLK = SYSCLKOUT
       (*ePWM[j]).TBCTL.bit.FREE_SOFT = 0;

       (*ePWM[j]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;   // LOAD CMPA on CTR = 0
//       (*ePWM[j]).CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
       (*ePWM[j]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
//       (*ePWM[j]).CMPCTL.bit.SHDWBMODE = CC_SHADOW;

       (*ePWM[j]).AQCTLA.bit.CAU = AQ_SET;             // PWM toggle high/low
       (*ePWM[j]).AQCTLA.bit.CAD = AQ_CLEAR;
//       (*ePWM[j]).AQCTLB.bit.ZRO = AQ_SET;             // PWM toggle high/low
//       (*ePWM[j]).AQCTLB.bit.PRD = AQ_CLEAR;
        EALLOW;

       (*ePWM[j]).HRCNFG.all = 0x0;

       //
       // MEP control on both edges
       //
       (*ePWM[j]).HRCNFG.bit.EDGMODE = HR_BEP;

       //
       // CMPAHR and TBPRDHR HR control
       //
       (*ePWM[j]).HRCNFG.bit.CTLMODE = HR_CMP;

       //
       // load on CTR = 0 and CTR = TBPRD
       //
       (*ePWM[j]).HRCNFG.bit.HRLOAD  = HR_CTR_ZERO_PRD;

       //
       // Enable autoconversion for HR period
       //
       (*ePWM[j]).HRCNFG.bit.AUTOCONV = 1;

       //
       // Turn on high-resolution period control.
       //
       (*ePWM[j]).HRPCTL.bit.HRPE = 1;

       (*ePWM[j]).TBCTL.bit.PHSEN = 1;

       //
       // Enable TBPHSHR sync (required for updwn count HR control)
       //
       (*ePWM[j]).HRPCTL.bit.TBPHSHRLOADE = 1;
#endif
        EDIS;
    }

    EALLOW;
    EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // disable ePWM1 SYNC output[3]
    EPwm7Regs.TBCTL.bit.PHSEN = 1;
    EPwm7Regs.TBCTL.bit.PHSDIR = 1;             // count up after SYNC event

    //
    // initial - coarse phase offset relative to ePWM1
    //
    EPwm7Regs.TBPHS.half.TBPHS = 2;

    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;     // Enable TBCLK within the EPWM

    //
    // Synchronize high resolution phase to start HR period
    //
    EPwm7Regs.TBCTL.bit.SWFSYNC = 1;

    EDIS;
}

//
// HRPWM1_Config -
//
void HRPWM_ConfigLocal2(Uint16 period)
{
    //
    // ePWM1 register configuration with HRPWM
    // ePWM1A toggle low/high with MEP control on Rising edge
    //
#if 1
//    period = 360;         //  1MHz - 500kHz  - 360 - 250kHz
//    period = 720;           // 720 -> 125 kHz
    period = 1440;            // 1440 -> 62.5 kHz
//      period = 2880;
    EPwm7Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;    // set Immediate load
    EPwm7Regs.TBPRD = period-1;                  // PWM frequency = 1 / period
//    EPwm7Regs.TBPRD = period;
    EPwm7Regs.CMPA.half.CMPA = period / 2;       // set duty 50% initially
    EPwm7Regs.CMPA.half.CMPAHR = (1 << 8);       // initialize HRPWM extension
//    EPwm7Regs.CMPB = period / 2;                 // set duty 50% initially
    EPwm7Regs.TBPHS.all = 0;
    EPwm7Regs.TBCTR = 0;

    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;      // EPwm1 is the Master
    EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
//    EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
//    EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

    EPwm7Regs.AQCTLA.bit.ZRO = AQ_CLEAR;        // PWM toggle low/high
    EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;
//    EPwm7Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
//    EPwm7Regs.AQCTLB.bit.CBU = AQ_SET;

    EALLOW;
    EPwm7Regs.HRCNFG.all = 0x0;
    EPwm7Regs.HRCNFG.bit.EDGMODE = HR_REP;      // MEP control on Rising edge
    EPwm7Regs.HRCNFG.bit.CTLMODE = HR_CMP;
    EPwm7Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO;
//    EPwm7Regs.HRPCTL.bit.TBPHSHRLOADE = 1;
    EDIS;

//    EPwm7Regs.CMPA.half.CMPAHR = DutyFine << 8;
#endif
#if 1
    period = 9;             // 10MHz

    EPwm2Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;    // set Immediate load
    EPwm2Regs.TBPRD = period-1;                  // PWM frequency = 1 / period
//    EPwm7Regs.TBPRD = period;
    EPwm2Regs.CMPA.half.CMPA = period / 2;       // set duty 50% initially
    EPwm2Regs.CMPA.half.CMPAHR = (1 << 8);       // initialize HRPWM extension
//    EPwm7Regs.CMPB = period / 2;                 // set duty 50% initially
    EPwm2Regs.TBPHS.all = 0;
    EPwm2Regs.TBCTR = 0;

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
//    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;      // EPwm1 is the Master
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;      // EPwm1 is the Master
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm2Regs.TBPHS.half.TBPHSHR = TB_ENABLE;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.SWFSYNC = 1;            // ored with SYNCI once

    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
//    EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
//    EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

    EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;        // PWM toggle low/high
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
//    EPwm7Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
//    EPwm7Regs.AQCTLB.bit.CBU = AQ_SET;

#if 1
    EALLOW;
    EPwm2Regs.HRCNFG.all = 0x0;
    EPwm2Regs.HRCNFG.bit.EDGMODE = HR_REP;      // MEP control on Rising edge
    EPwm2Regs.HRCNFG.bit.CTLMODE = HR_CMP;
    EPwm2Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO;
//    EPwm2Regs.HRPCTL.bit.TBPHSHRLOADE = 1;
//    EPwm2Regs.HRPCTL.bit.HRPE = 1;
//    EPwm2Regs.HRPCTL.bit.PWMSYNCSEL = 1;
    EDIS;
#endif
//    EPwm7Regs.CMPA.half.CMPAHR = DutyFine << 8;

#endif

}


// task for printing debug values.
void USB_Print_Task(void * pvParameters)
{

    static Uint8 task_counter = 0;

    for(EVER)
    {
//      vTaskSuspend(NULL);

        if(xSemaphoreTake( xUSBPrintSemaphore, portMAX_DELAY ) == pdTRUE)
        {

            if ( control_valL >=45 && control_valL <=50 && control_valH >= 45 && control_valH <= 50) {

// #define LAST_STATE_LOCKED (1)
// #define LAST_STATE_UNLOCKED (2)

// static Uint8 digitalLockState = LAST_STATE_UNLOCKED;

                digitalLockState = LAST_STATE_LOCKED;


#ifdef USE_ALTERNATE_PID
                sprintf((Uint8 *)&debug_string[0], "CH:%lu, CL:%lu, D:%.2f, PWM_X:%.2f, PV:%.2f, SP:%.2f, PDC:%.2f, Err:%.2f, DC:%d !DPHASE_LOCK!\r\n", control_valH, control_valL, Duty, PWM_X, *(controlPID->pv), *(controlPID->sp), (100 - (Duty/1440.)*100.), HRCAP_Diff, HRCAP2Data.datacounter2 );  /* (100 - (Duty/1440.)*100.)*/
#else
#ifdef RUN_LINEAR_PID
                sprintf((Uint8 *)&debug_string[0], "P:%f, I:%f, D:%f, D:%.2f, uk:%.2f, PDC:%.2f, !DLOCK!\r\n", pid1->Kp, pid1->Ki, pid1->Kd, Duty, S_PID_VDATA.uk, (100 -(Duty/1440.)*100.), S_PID_VDATA.yk, HRCAP2Data.datacounter2 );  /* (100 - (Duty/1440.)*100.)*/
#else
#ifdef RUN_NON_LINEAR_PID
  //            sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, D%f, uk:%f, lk:%f, PDC:%f, DC:%d, HR:%d, HF:%d <:!:=>DPHASE LOCK<=:!:>\r\n", control_valH, control_valL, Duty, S_PID_VDATA.uk, S_PID_VDATA.lk, (100 -(Duty/1440.)*100.), HRCAP2Data.datacounter2, HRCAP2Data.HRC2_HCCAPDLYRISE0[HRCAP2Data.datacounter2], HRCAP2Data.HRC2_HCCAPDLYFALL0[HRCAP2Data.datacounter2]);  /* (100 - (Duty/1440.)*100.)*/
//                sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, D%f, uk:%f, lk:%f, PDC:%f, yk:%f, DC:%d <:!:=>DPHASE LOCK<=:!:>\r\n", control_valH, control_valL, Duty, S_PID_VDATA.uk, S_PID_VDATA.lk, (100 -(Duty/1440.)*100.), S_PID_VDATA.yk, HRCAP2Data.datacounter2 );  /* (100 - (Duty/1440.)*100.)*/
                sprintf((Uint8 *)&debug_string[0], "P:%f, I:%f, D:%f, D:%.2f, uk:%.2f, PDC:%.2f, !DLOCK!\r\n", nlpid1->Kp, nlpid1->Ki, nlpid1->Kd, Duty, S_PID_VDATA.uk, (100 -(Duty/1440.)*100.), S_PID_VDATA.yk, HRCAP2Data.datacounter2 );  /* (100 - (Duty/1440.)*100.)*/
#if 0
                task_counter++;

                if ( task_counter > 50 )
                vTaskSuspend(ECTaskHandle);
#endif
#endif
#endif
#endif
            } else

            {

                digitalLockState = LAST_STATE_UNLOCKED;

#ifdef USE_ALTERNATE_PID
                sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu,D:%.2f,PWM_X:%.2f,PV:%.2f,SP:%.2f,PDC:%.2f,Err:%.2f,PD:%.2f,DC:%d\r\n", control_valH, control_valL, Duty, PWM_X, *(controlPID->pv), *(controlPID->sp), (100 - (Duty/1440.)*100.), HRCAP_Diff, PID_Delta, HRCAP2Data.datacounter2 );  /* (100 - (Duty/1440.)*100.)*/
#else
#ifdef RUN_LINEAR_PID
                sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, D%.2f, uk:%.2f, lk:%.2f, PDC:%.2f, rk:%.2f, yk:%.2f, DC:%d, EF:%d\r\n", control_valH, control_valL, Duty, S_PID_VDATA.uk, S_PID_VDATA.lk, (100 - (Duty/1440.)*100.), S_PID_VDATA.rk, S_PID_VDATA.yk, HRCAP2Data.datacounter2, edge_flag);  /* (100 - (Duty/1440.)*100.)*/
#else
#ifdef RUN_NON_LINEAR_PID                // (100 -(Duty/1440.)*100.)
  //            sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, D%f, uk:%f, lk:%f, PDC:%f, rk:%f, yk:%f, HR:%d, HF:%d, DC:%d, EF:%d\r\n", control_valH, control_valL, Duty, S_PID_VDATA.uk, S_PID_VDATA.lk, (100 - (Duty/1440.)*100.), S_PID_VDATA.rk, S_PID_VDATA.yk, HRCAP2Data.HRC2_HCCAPDLYRISE0[HRCAP2Data.datacounter2], HRCAP2Data.HRC2_HCCAPDLYFALL0[HRCAP2Data.datacounter2], HRCAP2Data.datacounter2, edge_flag);  /* (100 - (Duty/1440.)*100.)*/
  //            sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, D%f, uk:%f, lk:%f, PDC:%f, rk:%f, yk:%f, DC:%d, EF:%d, P:%lx\r\n", control_valH, control_valL, Duty, S_PID_VDATA.uk, S_PID_VDATA.lk, (100 - (Duty/1440.)*100.), S_PID_VDATA.rk, S_PID_VDATA.yk, HRCAP2Data.datacounter2, edge_flag, PID_ID);  /* (100 - (Duty/1440.)*100.)*/
                sprintf((Uint8 *)&debug_string[0], "CH:%lu,CL:%lu, D%.2f, uk:%.2f, lk:%.2f, PDC:%.2f, rk:%.2f, yk:%.2f, DC:%d, EF:%d\r\n", control_valH, control_valL, Duty, S_PID_VDATA.uk, S_PID_VDATA.lk, (100 - (Duty/1440.)*100.), S_PID_VDATA.rk, S_PID_VDATA.yk, HRCAP2Data.datacounter2, edge_flag);  /* (100 - (Duty/1440.)*100.)*/
#endif
#endif
#endif
            }

            USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));

        }

    }

} // end USB_Print_Task
//

//
// XINT1_ISR - Connected to PIEIER1_4
// (use MINT1 and MG14 masks)
//
#if (G14PL != 0)
__interrupt void XINT1_ISR(void)
{
    static Uint16 xint1_ctr = 0;
//    static Uint8 flipflopctr = 0;
    //
    // Set interrupt priority:
    //
#if 0
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER1.all;
    IER |= M_INT1;
//    IER    &= MINT1;                         // Set "global" priority
    PieCtrlRegs.PIEIER1.all &= MG14;   // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;   // Enable PIE interrupts
    __asm("  NOP");
    EINT;
    /// :::xxx::: was in timer0 - just to try
#endif
                        EALLOW;
#if POLARITY_CHANGE
                        if ( XINT_polarity_state == XINT_RISING )
                         {
                             XINT_polarity_state = XINT_FALLING;
                             XIntruptRegs.XINT1CR.bit.POLARITY = 2;

                         }
                         else
                         {
                             XINT_polarity_state = XINT_RISING;
                             XIntruptRegs.XINT1CR.bit.POLARITY = 1;
                         }
#endif
//                        if ( flipflopctr >= 4 )
//                        {

//                            flipflopctr = 0;
                       if ( PieCtrlRegs.PIEIER4.bit.INTx8 == 0 )
                       {
                       if (HRCAP2Data.runECTask==pdFALSE)
                       PieCtrlRegs.PIEIER4.bit.INTx8=1;     // Enable PIE Group 4, INT 8 - Enable HRCAP2 IRQ
                       }

//                        } else {
//                            flipflopctr++;
//                        }

                       EDIS;

                       xint1_ctr = XIntruptRegs.XINT1CTR;
#if 0
    //
    // Insert ISR Code here
    //
//    __asm("      NOP");

    //
    // Restore registers saved
    //
    DINT;
    PieCtrlRegs.PIEIER1.all = TempPIEIER;
#endif



    //
    //  Add ISR to Trace
    //
///    ISRTrace[ISRTraceIndex] = 0x0014;
///    ISRTraceIndex++;
}
#endif
//
// XINT1_ISR - connected to PIEIER1_4 (use MINT1 and MG14 masks)
//
#if 0
#if (G14PL != 0)
__interrupt void
XINT1_ISR(void)
{
    //
    // Set interrupt priority:
    //
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER1.all;
    IER |= MINT1;                      // Set "global" priority
    PieCtrlRegs.PIEIER1.all &= MG14;   // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;   // Enable PIE interrupts
    EINT;

    //
    // Restore registers saved
    //
    DINT;
    PieCtrlRegs.PIEIER1.all = TempPIEIER;

    //
    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    //
   __asm ("      ESTOP0");
    for(;;);
}
#endif
#endif

// End of File
//

