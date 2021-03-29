/*  control-pid.c */
// Author:  John C. Westmoreland
// Date:    Oct. 12, 2017 - date PID operational - adapted for
// TMS320F28069 for test and comparison purposes.
// This file replaces control.c which has been deprecated.
// I make no original claims regarding the PID algorithm other than
// making it work correctly for the first time. --- jcw
#if 0
#if defined CMSIS_RTOS  
#include "cmsis_os.h"
#endif
#include <rtl.h>
#include "stdio.h"
#include "cmsis_os.h"
#include "stm32f4xx.h"                  /* STM32F4xx Definitions             */
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "common.h"
#include "string.h"
#include "rtos.h"
#include "bsp.h"
// // #include "control.h"
#include "adc.h"
#include "menu.h"
#include "cmd_task.h"
#include "misc_gpio_defs.h"
#endif

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>
#include "math.h"
// #include "arm-math.h"
#include "control-pid.h"
// #include "dma.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/usb.h"
#include "inc/hw_usb.h"
#include "include/usblib.h"
#include "include/usbcdc.h"
#include "driverlib/usb_hal.h"
#include "include/device/usbdevice.h"
#include "include/device/usbdcdc.h"
#include "usb_serial_structs.h"
extern Uint8 * debug_string; 
#define EVER 	;;
// #define DEBUG_TIMING_DEFINED
// #define PWM_DUTY_CHECK
// these constants appear to be largely nonsense.
// the ones that are zeroed basically zeroed the equations they were
// used in.

#define TURN_POINT_PID 			2.5f
//#define CHECK_POINT_RTD 		25.0f
#define HEAT_CONSTANT_FAST 	0.0f	// 1.0
#define HEAT_CONSTANT_SLOW 	0.0f	// 1.0
//#define HEAT_CONSTANT 			0.0f
#define COOL_CONSTANT_FAST 	0.0f	//7.5f
#define COOL_CONSTANT_SLOW 	0.0f	//8.5f
//#define COOL_CONSTANT 			12.5f

#define SIMPLE_PWM_CONTROL				// simple PWM control
#define COOL_CYCLE_DEBUG_CURRENT_CHECK			// drive negative slope hard since TECs not as efficient on cooling cycle
#define DEBUG_TIMING_DEFINED	


/// #define PID_VAR_DBG

#ifdef ENABLE_FTIO_DBG
extern uint8_t str[128];					
#endif
// extern osThreadId idPID_CTRL;
/// extern void Delay(__IO uint32_t nCount);

extern float RTD_UPPER_THRESHOLD;
extern float RTD_LOWER_THRESHOLD;

uint8_t STAGE_COMPLETE_FLAG = 0;
//extern float tInboundTime;
extern uint32_t tInboundTime;
//extern float RTD_Measured;
// extern int8_t Trend;
// extern float pwm_base;

extern uint32_t middleData;

// extern arm_pid_instance_f32 PID_instance;
// extern myPID pidSTRUCT;               //  PID Control Structure
extern pid *tecControlPID;			      //  new PID structure 

/// extern __IO uint32_t nInBound_Counter;

// extern osMutexId idSPI1_Busy_Mutex;
// extern osMutexId  idRTD_DATA_Mutex;
extern uint32_t cbArg1;
//#if defined CMSIS_RTOS
float TMC_Measured 	= 0.0;
extern float RTD_Measured;
extern float PWM_X;
// extern int32_t PWM_X;
extern uint8_t Start_PID;
extern uint8_t Start_Fine_Tuning;
//extern uint8_t Laser1_Triggered;
//extern uint8_t Laser2_Triggered;
extern int8_t heater_on;
extern int8_t heater_state;
//extern volatile uint8_t bADCReady;
extern volatile uint8_t bSystemReady;
// extern osTimerId uId1;
float PID_Composite = 0.0;
//uint8_t Stop_RTD_Sum = 0;
// extern osStatus  status;
extern PIDStage *pidStage0, *pidStage1, *pidStage2, *pidStage3, *pidStage4;
extern float stage_temp_band;							// this may need to be adjusted for going pos vs. neg

uint8_t bSensorOpenSent = 0;
uint8_t bSensorShortSent = 0;
uint8_t bCryOutMessage2Sent = 0;
uint8_t bCryOutMessage3Sent = 0;
extern uint8_t bReachTargetSent;

/// extern float PWM_state;
//float RTD_Base 		= 0.0;
/// float RTD_Delta 	= 0.0;
/// float RTD_OldData = 0.0;
/// float PID_Delta 	= 0.0;
/// float PID_OldData = 0.0;

// static uint16_t nDAC_data = 0;

// #define USE_FIR_FILTER

#if 1
#ifdef USE_FIR_FILTER
// #define			Q				32768	// I1Q15 fraction scaling value
// #define			NLP				5		// low-pass filter tap length
int16_t	aLP[NLP] = {Q*1/16, Q*4/16, Q*6/16,Q*4/16,Q*1/16};	// I1Q15 low-pass FIR filter coefficients
#endif
#endif

// Local Static data
static float calcTempLocal = 0.0;

/* System Explanation:
   The sensed temperature is the process variable (PV).  => RTD measurement from ADS1248.
   The desired temperature is called the setpoint (SP).  => Our desired temperature.
   The difference between the PV and SP is the error (e), which quantifies whether the temperature
   is too low or too high and by how much.
	The input to the process (the PWM of the H-Bridge for our TEC Chuck (power for the TEC's)
	is the output from the PID controller. It is called either the manipulated variable (MV)
	or the control variable (CV).
   By measuring the temperature (PV), and subtracting it from the setpoint (SP), the error (e) is found,
   and from it the controller calculates how much electric current to supply to the TEC's (MV).
   The PWM to the TEC's is derived from the temperature.
   The Laplace Transform of the PID is:
       L(s) = Kp + Ki/(s) +Kd(s)
	      Kp is the proportional gain
	      Ki is the integral gain
	      Kd is the derivative gain
*/
/* Some definitions:
 * Deadband:
 * 
 * Many PID loops control a mechanical device (for example, a valve).
   Mechanical maintenance can be a major cost and wear leads to control degradation in the form
   of either stiction or a deadband in the mechanical response to an input signal. The rate of mechanical
   wear is mainly a function of how often a device is activated to make a change. Where wear is a
   significant concern, the PID loop may have an output deadband to reduce the frequency of
   activation of the output (valve). This is accomplished by modifying the controller to hold its
   output steady if the change would be small (within the defined deadband range).
   The calculated output must leave the deadband before the actual output will change.

   Integral windup:
		 One common problem resulting from the ideal PID implementations is integral windup.
		 Following a large change in setpoint the integral term can accumulate an error larger
		 than the maximal value for the regulation variable (windup), thus the system overshoots
		 and continues to increase until this accumulated error is unwound.
		 This problem can be addressed by:

     Disabling the integration until the PV has entered the controllable region
     Preventing the integral term from accumulating above or below pre-determined bounds
     Back-calculating the integral term to constrain the regulator output within feasible bounds.

     Overshooting from known disturbances:
     
     For example, a PID loop is used to control the temperature of an electric resistance furnace
     where the system has stabilized. Now when the door is opened and something cold is put into
     the furnace the temperature drops below the setpoint. The integral function of the controller
     tends to compensate for error by introducing another error in the positive direction.
     This overshoot can be avoided by freezing of the integral function after the opening of the
     door for the time the control loop typically needs to reheat the furnace.

   Setpoint step change:
   
     The proportional and derivative terms can produce excessive movement in the output when a
     system is subjected to an instantaneous step increase in the error, such as a large setpoint change.
     In the case of the derivative term, this is due to taking the derivative of the error, which is very
     large in the case of an instantaneous step change. As a result, some PID algorithms incorporate some
     of the following modifications:
     
     Setpoint ramping
     In this modification, the setpoint is gradually moved from its old value to a newly specified value
     using a linear or first order differential ramp function. This avoids the discontinuity present in a simple step change.
     
     Derivative of the process variable
     In this case the PID controller measures the derivative of the measured process variable (PV),
     rather than the derivative of the error. This quantity is always continuous (i.e., never has a step change as a
     result of changed setpoint). This modification is a simple case of setpoint weighting.
     
     Setpoint weighting
     Setpoint weighting adds adjustable factors (usually between 0 and 1) to the setpoint in the error
     in the proportional and derivative element of the controller. The error in the integral term must
     be the true control error to avoid steady-state control errors. These two extra parameters do not
     affect the response to load disturbances and measurement noise and can be tuned to improve the controller's setpoint response.

    Feed-forward:
		 The control system performance can be improved by combining the feedback (or closed-loop) control of a
		 PID controller with feed-forward (or open-loop) control. Knowledge about the system (such as the desired
	         acceleration and inertia) can be fed forward and combined with the PID output to improve the overall system
		 performance. The feed-forward value alone can often provide the major portion of the controller output.
		 The PID controller primarily has to compensate whatever difference or error remains between the setpoint
		 (SP) and the system response to the open loop control. Since the feed-forward output is not affected by
		 the process feedback, it can never cause the control system to oscillate, thus improving the system
		 response without affecting stability. Feed forward can be based on the setpoint and on extra measured
		 disturbances. Setpoint weighting is a simple form of feed forward.

		 For example, in most motion control systems, in order to accelerate a mechanical load under control,
		 more force is required from the actuator. If a velocity loop PID controller is being used to control
		 the speed of the load and command the force being applied by the actuator, then it is beneficial to
		 take the desired instantaneous acceleration, scale that value appropriately and add it to the output
		 of the PID velocity loop controller. This means that whenever the load is being accelerated or
		 decelerated, a proportional amount of force is commanded from the actuator regardless of the
		 feedback value. The PID loop in this situation uses the feedback information to change the
		 combined output to reduce the remaining difference between the process setpoint and the feedback
		 value. Working together, the combined open-loop feed-forward controller and closed-loop PID
		 controller can provide a more responsive control system.
		 
 Bumpless operation:
 
 PID controllers are often implemented with a "bumpless" initialization feature that recalculates the integral accumulator
 term to maintain a consistent process output through parameter changes.  A partial implementation is to store the
 integral of the integral gain times the error rather than storing the integral of the error and postmultiplying
 by the integral gain, which prevents discontinuous output when the I gain is changed, but not the P or D gains.

 Other improvements:
 
 In addition to feed-forward, PID controllers are often enhanced through methods such as PID gain scheduling
 (changing parameters in different operating conditions), fuzzy logic, or computational verb logic.
 Further practical application issues can arise from instrumentation connected to the controller.
 A high enough sampling rate, measurement precision, and measurement accuracy are required to achieve
 adequate control performance. Another new method for improvement of PID controller is to increase
 the degree of freedom by using fractional order. The order of the integrator and differentiator add
 increased flexibility to the controller.

 Drive hard to within a band of the setpoint - this can be a form of
 deadband - the PID isn't even on until within 5 degrees C of the
 setpoints - can be worth a try.

  Cascade control
  One distinctive advantage of PID controllers is that two PID controllers can be used together to
  yield better dynamic performance. This is called cascaded PID control. In cascade control there are
  two PIDs arranged with one PID controlling the setpoint of another. A PID controller acts as outer
  loop controller, which controls the primary physical parameter, such as fluid level or velocity.
  The other controller acts as inner loop controller, which reads the output of outer loop controller
  as setpoint, usually controlling a more rapid changing parameter, flowrate or acceleration.
  It can be mathematically proven that the working frequency of the controller
  is increased and the time constant of the object is reduced by using cascaded PID controllers.
 
*/

// our pid structure; add intregral windup protection?
/*
   struct  _pid {
	int *pv;  	// pointer to an integer that contains the process value
	int *sp;  	// pointer to an integer that contains the set point 
	float integral;
	float pgain;
	float igain;
	float dgain;
	int deadband;
	int last_error;
};
*/

/*------------------------------------------------------------------------
   pid_init
   DESCRIPTION   This function initializes the pointers in the _pid structure
   to the process variable and the setpoint.  *pv and *sp are
   (were) integer pointers.  Maybe that should be changed back.
   ------------------------------------------------------------------------*/
   
void pid_init(struct _pid *a, float *pv, float *sp)
// struct _pid *a;
// int *pv, *sp;
{
	a->pv = pv;
	a->sp = sp;
}

/*------------------------------------------------------------------------
   pid_tune
   DESCRIPTION   Sets the proportional gain (p_gain), integral gain (i_gain),
   derivitive gain (d_gain), and the dead band (dead_band) of
   a pid control structure _pid.
   ------------------------------------------------------------------------*/

void pid_tune(struct _pid *a, float p_gain, float i_gain, float d_gain, int dead_band)
// struct _pid *a;
// float p_gain, i_gain, d_gain;
// int dead_band;
{
	a->pgain = p_gain;
	a->igain = i_gain;
	a->dgain = d_gain;
	a->deadband = dead_band;
	a->integral=0.0;
	a->last_error=0;
}

/*------------------------------------------------------------------------
   get_gains
   DESCRIPTION   Returns the gains and dead band in a _pid control structure
   in the locations pointed to by the p_gain, i_gain, d_gain,
   and dead_band pointers.

   ALSO SEE      pid_tune              
   ------------------------------------------------------------------------*/
void get_gains(struct _pid *a, float *p_gain, float *i_gain, float *d_gain, int *dead_band)
// struct _pid *a;
// float *p_gain, *i_gain, *d_gain;
// int *dead_band;
{
	*p_gain = a->pgain;
	*i_gain = a->igain;
	*d_gain = a->dgain;
	*dead_band = a->deadband;
}

/*------------------------------------------------------------------------
   pid_setinteg
   DESCRIPTION   Set a new value for the integral term of the pid equation.
   This is useful for setting the initial output of the
   pid controller at start up.
   ------------------------------------------------------------------------*/

void pid_setinteg(struct _pid *a, float new_integ)
// struct _pid *a;
// float new_integ;
{
	a->integral=new_integ;
	a->last_error=0;
}

/*------------------------------------------------------------------------
   pid_bumpless
   DESCRIPTION   Bumpless transfer algorithim.  When suddenly changing
   setpoints, or when restarting the PID equation after an
   extended pause, the derivative of the equation can cause 
   a bump in the controller output.  This function will help 
   smooth out that bump. The process value in *pv should
   be the updated just before this function is used.
   ------------------------------------------------------------------------*/

void pid_bumpless(struct _pid *a)
// struct _pid *a;
{
	a->last_error = (int) (*(a->sp) - *(a->pv));
}

/*------------------------------------------------------------------------
   pid_calc
   DESCRIPTION   Performs PID calculations for the _pid structure *a.  This
   function uses the positional form of the pid equation, and
   incorporates an integral windup prevention algorithim.
   Rectangular integration is used, so this function must be
   repeated on a consistent time basis for accurate control.
   RETURN VALUE  The new output value for the pid loop.
   USAGE: (example)         
   main() {
	struct _pid PID;
	int process_variable, set_point;
	pid_init(&PID, &process_variable, &set_point);
	pid_tune(&PID, 4.3, 0.2, 0.1, 2);
	set_point = 500;
	pid_setinteg(&PID,30.0);
	process_variabe = read_temp();
	pid_bumpless(&PID);
	for(;;) {
		process_variable = read_temp();
		output( pid_calc(&PID) );
		wait(1.0);
	}
}
------------------------------------------------------------------------*/

// float pid_calc(struct _pid *a)
void pid_calc(struct _pid *a)	
// struct _pid *a;
{
	int err;
//	float err;
	float pterm, dterm, result, ferror;
	
//		__disable_irq();
	
// 'safeties' so to speak
	
	if ( *(a->sp) > 100.0 )				// only 1 set point checked??????
		*(a->sp) = 48.0;

	if ( *(a->sp) < 10.0 )				// only 1 set point checked??????
		*(a->sp) = 48.0;
	
	err = *(a->sp) - *(a->pv);			// 'error' term - difference in calculated temperature and the PID's set-point
#if 0
///	if (a->error_sum == 0)				// only when this wraps around and the first time it runs ('orig' code)
	{
		if (err < 0)
		{
			Trend = -1;
			pwm_base = err * -1.0;
//			pwm_base = err * -4.0;		// don't we have to drive harder this way?
		}
		else if (err > 0)
		{
			Trend = 1;
			pwm_base = err * 1.0;
		}
		else
		{
			Trend = 0;
		}
	}	
#endif
	a->error_sum += err;
	
	if (a->error_sum > 100 )				// integral windup protection
		a->error_sum = 100;
	if ( a->error_sum < -100 )
		a->error_sum = -100;
	
	if (abs(err) > a->deadband) {
		ferror = (float) err;    			/*do integer to float conversion only once*/
//		ferror = err;    						
		pterm = a->pgain * ferror;
		
		if (pterm > 100 || pterm < -100) 
			a->integral = 0.0;
			else {
				a->integral += a->igain * ferror;
				
	if (a->integral > a->integral_error_max) a->integral = 100.0; // integral windup protection
		else if (a->integral < a->integral_error_min) a->integral = -100.0;
			}		
			dterm = (err - a->last_error) * a->dgain;
			result = pterm + a->integral + dterm;
			a->uof = result;
	}
	
	else 
	{
		result = a->integral;
		a->uof = result;												// change this function just to update structure
	}
	a->last_error=err;
	
	if ( result > 100 )
	{
		result = 100;
		a->uof = 100;													// added to structure
	} else {
		if ( result < -100 ) 
		{
				result = -100;
				a->uof = -100;
			}
		}

#ifdef PID_VAR_DBG
//		memset(str, 0, 128);		
		sprintf((Uint8 *)&debug_string[0], "Kp: %f, Ki: %f, Kd: %f\n\r", a->pgain, a->igain, a->dgain);			
//		Serial_PutString(str);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, (Uint8 *)&debug_string[0], strlen(debug_string));		
#endif	

//	__enable_irq();
//	return (result > 100.0 ? 100.0 : (result < -100.0 ? -100.0 : result));			// saturate result if necessary
}

#ifdef USE_OLD_TEMP_CONTROL
// void Temperature_Control((void const *)(struct _pid *p))
void Temperature_Control(void const *argument)
{
	
	static uint8_t WE_ARE_HERE_FLAG, WE_ARE_HERE = 0;
	static uint8_t STAGE_COMPLETE_COUNTER = 0;
//	static uint8_t STAGE_COMPLETE_FLAG = 0;
//	static uint8_t stage_temp_band = 2.5;						// this may need to be adjusted for going pos vs. neg
#if 1

///	 SPI1_Config(RTD_MODE);
	pid *p;																		// our task pointer to the tecControlPID structure
	p = ( struct _pid * )(&argument);		
	p = tecControlPID;
	
	for(EVER)
{
///	  osSignalWait(0x0001, osWaitForever);						// PID mutex
// xxx::: SPI1 Mutex is deprecated per RevD --- jcw
//		osMutexWait(idSPI1_Busy_Mutex,osWaitForever);
//		SPI1_Config(RTD_MODE);					// 6 devices on SPI1 bus - 3 are active and don't use the same SPI mode --- jcw
////		osMutexWait(idRTD_DATA_Mutex, osWaitForever);								// this naturally throttles and forces synchronization
//		pid_setinteg(&tecControlPID,30.0);
//		pid_bumpless(&tecControlPID);
///		Temp_PID_Control();
///  	void Temp_PID_Control(void)	
{		
#ifdef ENABLE_PID_DBG		

	uint8_t Turning = 'R';			// not sure this is useful anymore either...
	float PID_Current = 0;
	float PWM_Current = 0;
#endif
	float RTD_Diff = 0;
	float PID_Adjust = 0;

	/* Duty cycle for PWM */
	float pwm_duty = 0;
//	uint32_t pwm_duty = 0;
//  SerialPutString("C+");

//	RTD_Measured = ADS1248_ReadData(RTD_ADC);

	calcTempLocal = *(p->pv);			// mutex protected by caller; maybe this can be deprecated soon
	
#ifdef USE_FIR_FILTER
//	*(tecControlPID->pv) = tecControlPID->yf;		// use filtered value
	*(p-pv) = p->yf;	
#else	
//	*(tecControlPID->pv) = calcTempLocal;		// our PV is the current calculated temp -> move this to ADC
#if 0
	  *(p->pv) = calcTempLocal;								// there now 
#endif		
#endif	

    calcTempLocal = *(p->pv);								// for ftio.c
							// Note we will
							// pass this
							// through a
							// low-pass FIR
							// filter -
							// like
							// arm_fir_fast_q15.c 
	
	if ( (*(p->pv) == 20.0 ) && ( WE_ARE_HERE < 100 ))
			WE_ARE_HERE_FLAG = 1;
	
//	if (*(tecControlPID->pv)  == NAN)
  if (*(p->pv)  == NAN )		
	{	//RTD sensor open detected
		if (bSensorOpenSent == 0)
		{
			bSensorOpenSent = 1;
//			SendAlertMessage(CRY_OUT_HELP, 3);
			bSensorShortSent = 0;
		}
//		RTD_Measured = 0.0f;
		return;
	}
	else if (*(p->pv)  == INFINITY)
		
	{	//RTD sensor short detected
		if (bSensorShortSent == 0)
		{
			bSensorShortSent = 1;
//			SendAlertMessage(CRY_OUT_HELP, 2);
			bSensorOpenSent = 0;
		}
//		RTD_Measured = 0.0f;
		return;
	}
	else
	{
		//reset flag
		bSensorOpenSent = 0;
		bSensorShortSent = 0;
	}

	//TODO, Calculate ???
	PWM_X = 0.0;
//	PWM_X = 0;
	PID_Adjust = 1.0;
	PID_Composite = 0.0;

	//prepare for system temperature base line
//	if (Stop_RTD_Sum == 0)
	{
//		RTD_Total = RTD_Total + RTD_Measured;
	}
//	RTD_count++;

	heater_on = 0;

// we will fix this with mutex...
	
	if (Start_PID==0x00)	//check PID system start yet?
	{
//		if (RTD_count>100)
		{
//			RTD_Base = RTD_Total/RTD_count;		//TODO, when to apply???
		}
//		Stop_RTD_Sum = 1;
//		RTD_count=0.0;
//		RTD_Total=0.0;

// need to satisfy inbound time...
		
		if (*(p->pv)  < RTD_LOWER_THRESHOLD)
		{
//			nInBound_Counter = 0;
#if 1
			if ((bCryOutMessage2Sent == 0) && (bReachTargetSent == 1) && (bCryOutMessage3Sent == 1))
			{
				bCryOutMessage2Sent = 1;
//				SendAlertMessage(OUTBOUND_TEMPERATURE, 2);
			}
#endif			
		} else if (*(p->pv)  > RTD_UPPER_THRESHOLD)
		{
//			nInBound_Counter = 0;
#if 1
			if ((bCryOutMessage2Sent == 0) && (bReachTargetSent == 1) && (bCryOutMessage3Sent == 1))
			{
				bCryOutMessage2Sent = 1;
//				SendAlertMessage(OUTBOUND_TEMPERATURE, 2);
			}
#endif			
		}
#if 1			
		else
		{
			if (bReachTargetSent == 0)														/// xxx:::xxx FIXME
			{
				bReachTargetSent = 1;
/// xxx::: 11/29/17:  Change per Simon's request:
///
#if 0				
				SendAlertMessage(REACH_PID_TEMPERATURE, 0x01);
#endif				
	///			osTimerStop ( uId1 );
				cbArg1 = 0x02;
	///			status = osTimerStart(uId1, tInboundTime);							// for timeouts  tInboundTime tTimeoutTime
//				if ( status != osOK )  while(1) {;}
				
			}
		}
#endif			

		// Need to determine when to change stage
#ifndef NO_STAGES_IN_TEST_RUN
		if ( p->current_stage < p->numOfStages && !(STAGE_COMPLETE_FLAG) )
		{
		
		switch ( p->current_stage )

		{

			case 1:
			
			if (( *(p->pv) >= pidStage0->stageTemp - stage_temp_band ) && (*(p->pv) <= pidStage0->stageTemp + stage_temp_band ))
			{
				STAGE_COMPLETE_COUNTER++;

				if ( STAGE_COMPLETE_COUNTER >= 5 )				// 5 is arbitrary now
				{
					if ( p->numOfStages > p->current_stage )		// progress to next stage 
					{
						pid_tune ((struct _pid *)(p), pidStage1->kp, pidStage1->ki, pidStage1->kd, p->deadband );
						pid_setinteg((struct _pid *)(p),pidStage1->initIntegral);
						pid_bumpless((struct _pid *)(p));
						p->current_stage++;
						STAGE_COMPLETE_COUNTER = 0;
					}
				}
			}
			break;

			case 2:
				
			if (( *(p->pv) >= pidStage1->stageTemp - stage_temp_band ) && (*(p->pv) <= pidStage1->stageTemp + stage_temp_band ))
			{
				STAGE_COMPLETE_COUNTER++;

				if ( STAGE_COMPLETE_COUNTER >= 5 )				// 5 is arbitrary now
				{
					if ( p->numOfStages > p->current_stage )		// progress to next stage 
					{
						pid_tune ((struct _pid *)(p), pidStage2->kp, pidStage2->ki, pidStage2->kd, p->deadband );
						pid_setinteg((struct _pid *)(p),pidStage2->initIntegral);
						pid_bumpless((struct _pid *)(p));
						p->current_stage++;
						STAGE_COMPLETE_COUNTER = 0;
					}
				}
			}
			break;
			
			case 3:
				if (( *(p->pv) >= pidStage2->stageTemp - stage_temp_band ) && (*(p->pv) <= pidStage2->stageTemp + stage_temp_band ))
				{
					STAGE_COMPLETE_COUNTER++;

					if ( STAGE_COMPLETE_COUNTER >= 5 )				// 5 is arbitrary now
					{
						if ( p->numOfStages > p->current_stage )		// progress to next stage 
						{
							pid_tune ((struct _pid *)(p), pidStage3->kp, pidStage3->ki, pidStage3->kd, p->deadband );
							pid_setinteg((struct _pid *)(p),pidStage3->initIntegral);
							pid_bumpless((struct _pid *)(p));
							p->current_stage++;
							STAGE_COMPLETE_COUNTER = 0;
						}
					}
				}
				break;
				
			case 4:
				if (( *(p->pv) >= pidStage3->stageTemp - stage_temp_band ) && (*(p->pv) <= pidStage3->stageTemp + stage_temp_band ))
				{
					STAGE_COMPLETE_COUNTER++;

					if ( STAGE_COMPLETE_COUNTER >= 5 )				// 5 is arbitrary now
					{
						if ( p->numOfStages > p->current_stage )		// progress to next stage 
						{
							pid_tune ((struct _pid *)(p), pidStage4->kp, pidStage4->ki, pidStage4->kd, p->deadband );
							pid_setinteg((struct _pid *)(p),pidStage4->initIntegral);
							pid_bumpless((struct _pid *)(p));
							p->current_stage++;
							STAGE_COMPLETE_COUNTER = 0;
							STAGE_COMPLETE_FLAG = 1;
						}
					}
				}
				break;
			case 5:
#if 0				
				if (( *(p->pv) >= pidStage4->stageTemp - stage_temp_band ) && (*(p->pv) <= pidStage4->stageTemp + stage_temp_band ))
				{
					STAGE_COMPLETE_COUNTER++;

					if ( STAGE_COMPLETE_COUNTER >= 5 )				// 5 is arbitrary now
					{
						if ( p->numOfStages > p->current_stage )		// progress to next stage 
						{
							pid_tune ((struct _pid *)(p), pidStage5->kp, pidStage5->ki, pidStage5->kd, p->deadband );
							pid_setinteg((struct _pid *)(p),pidStage5->initIntegral);
							pid_bumpless((struct _pid *)(p));
							p->current_stage++;
							STAGE_COMPLETE_COUNTER = 0;
						}
					}
				}
#endif				
				break;

			default:					// err-ctr here?
				STAGE_COMPLETE_COUNTER = 0;
				break;
		}
			
		}
#endif		// stages		
		
//		PID_Composite = PIDCalculated(&pidSTRUCT);  //PID_Composite will determine the DUTY-CYCLE of the PWM Waveform to the H-Bridges
//		PID_Composite = PIDCalculated(&tecControlPID);  //PID_Composite will determine the DUTY-CYCLE of the PWM Waveform to the H-Bridges
		  pid_calc((struct _pid *)(p));
//		  PID_Composite = pid_calc(&tecControlPID);  //PID_Composite will determine the DUTY-CYCLE of the PWM Waveform to the H-Bridges
		  PID_Composite = p->uof;  //PID_Composite will determine the DUTY-CYCLE of the PWM Waveform to the H-Bridges
		
#if 0
		if ( fabs ( PID_Composite ) > (*(tecControlPID->sp)) )
				SendAlertMessage(CRY_OUT_HELP, 2);		/// xxx::: - check this
#endif

		RTD_Delta = *(p->pv) - RTD_OldData;
		RTD_OldData = *(p->pv);
		PID_Delta = PID_Composite - PID_OldData;
		PID_OldData = PID_Composite;

		if ( (*(p->sp)) > 65.0 )			/// xxx::: upper limit guard used during testing
			(*(p->sp)) = 65.0;

//		RTD_Diff = fabs(RTD_Measured - pidSTRUCT.SetPoint);   xxx:::
		RTD_Diff = (*(p->pv) - (*(p->sp)));			// this is same as error term

//		PWM_X = RTD_Diff;			// PWM based on RTD difference


// we are running with this defined - so pretty much all of the PWM
// mumbo-jumbo has been deprecated --- jcw		
#ifndef SIMPLE_PWM_CONTROL
		PWM_X = RTD_Diff;			// PWM based on RTD difference
		
#ifdef	ENABLE_FTIO_DBG		
		memset(str, 0, 128);		
		sprintf((char *)str, "PWM_X: %f, PID_Composite %f\n\r", PWM_X, PID_Composite);			
		Serial_PutString(str);
#endif				
#ifdef ENABLE_PID_DBG		
		PWM_Current = PWM_X;
		PID_Current = PID_Composite;
#endif
		
		if ((fabsf(PID_Delta) > TURN_POINT_PID) /* && (pwm_base > CHECK_POINT_RTD)*/&& (PID_Composite != 0.0f) && (Start_Fine_Tuning == 0))
		{
			PID_Adjust = sqrt(fabsf(PID_Composite*0.123f)); //TODO, speed up falling and rising speed
//			PID_Adjust = sqrt(fabsf(PID_Composite*0.111f)); //TODO, speed up falling and rising speed
			arm_pid_init_f32(&tecControlPID, &PID_instance, 1);
		}
		
		else
		{
//			pwm_duty = arm_pid_f32(&PID_instance, RTD_Diff);
			pwm_duty = pid_calc(&tecControlPID);
			Start_Fine_Tuning = 1;
			PID_Adjust = 1.0;
		}

		if (RTD_Diff > 0)
		{
			heater_on = -1;
			if (Trend == 1)
			{
				heater_on = 1;
				
				if ((fabsf(PID_Delta) > TURN_POINT_PID)/*&& (pwm_base > CHECK_POINT_RTD)*/ && (Start_Fine_Tuning == 0))
				{
#ifdef ENABLE_PID_DBG		
					Turning = '1';
#endif					
					PWM_X = (PWM_X*(-1)+COOL_CONSTANT_FAST)*PID_Adjust;
//					PWM_X = -25;
				}
				
				else
				{
#ifdef ENABLE_PID_DBG		
					Turning = '2';
#endif					
					PWM_X = pwm_duty;
//					PWM_X = sqrt(PWM_X*(-1)+COOL_CONSTANT_SLOW)*10*PID_Adjust;
				}
			}
			else if (Trend == -1)
			{
				if ((fabsf(PID_Delta) > TURN_POINT_PID)/*&& (pwm_base > CHECK_POINT_RTD)*/ && (Start_Fine_Tuning == 0))
				{
#ifdef ENABLE_PID_DBG		
					Turning = '3';
#endif					
					PWM_X =(PWM_X+HEAT_CONSTANT_FAST)*PID_Adjust;		// so, this was always zero....
//					PWM_X = 25;
				}
				else
				{
#ifdef ENABLE_PID_DBG		
					Turning = '4';
#endif					
					PWM_X = pwm_duty;
//					PWM_X = sqrt(PWM_X+HEAT_CONSTANT_SLOW)*10*PID_Adjust;
					if ((PWM_X < 0.) && (Start_Fine_Tuning == 1))
					{
#ifdef ENABLE_PID_DBG		
						Turning = 'A';
#endif						
//						heater_on = -1;
#if 1	//TODO??? disable or not, test it plase, 20170309					// xxx:::xxx:::
						PWM_X = PWM_state * 0.99875f;//4.0 / 5.0;
						heater_on = heater_state;
#endif
					}
				}
			}
			
			else
			{
				//RTD-diff == 0, in some case
				PWM_X = PWM_state;
				heater_on = heater_state;
			}
		}
		
		else if (RTD_Diff < 0)
			
		{
			heater_on = 1;
			if (Trend == 1)
			{
				if ((fabsf(PID_Delta) > TURN_POINT_PID) /*&& (pwm_base > CHECK_POINT_RTD)*/ && (Start_Fine_Tuning == 0))
				{
#ifdef ENABLE_PID_DBG		
					if (Start_Fine_Tuning == 1)
						Turning = 'X';
					else
						Turning = '5';
#endif					
					PWM_X = (PWM_X*(-1)+COOL_CONSTANT_FAST)*PID_Adjust;       // this was zero too?
//					PWM_X = -25;
				}
				
				else
				{
#ifdef ENABLE_PID_DBG		
					Turning = '6';
#endif					
					PWM_X = pwm_duty;
//					PWM_X = sqrt(PWM_X*(-1)+COOL_CONSTANT_SLOW)*10*PID_Adjust;
				}
			}
			
			else if (Trend == -1)
			{
//				heater_on = -1;
				
				if ((fabsf(PID_Delta) > TURN_POINT_PID) /*&& (pwm_base > CHECK_POINT_RTD)*/ && (Start_Fine_Tuning == 0))
				{
#ifdef ENABLE_PID_DBG		
					if (Start_Fine_Tuning == 1)
						Turning = 'Y';
					else
						Turning = '7';
#endif					
					PWM_X = (PWM_X*(-1)+HEAT_CONSTANT_FAST)*(-1)*PID_Adjust;
//					PWM_X = -25;
				}
				else
				{
#ifdef ENABLE_PID_DBG		
					Turning = '8';
#endif					
					PWM_X = pwm_duty;
//					PWM_X = sqrt(PWM_X*(-1)+HEAT_CONSTANT_SLOW)*10*(-1)*PID_Adjust;
					if ((PWM_X > 0) && (Start_Fine_Tuning == 1))
					{
#ifdef ENABLE_PID_DBG		
						Turning = 'B';
#endif						
//						heater_on = -1;
#if 1	//TODO??? disable or not, test it plase, 20170309					// xxx::: tested?
						PWM_X = PWM_state * 0.99875f;// * 3.0 / 4.0;
						heater_on = heater_state;
#endif						
					}
				}
			}
			else
			{
				//RTD-diff == 0, in some case
				PWM_X = PWM_state;
				heater_on = heater_state;
			}
		}
		else
		{
			//RTD-diff == 0, in some case
			PWM_X = PWM_state;
			heater_on = heater_state;
		}
		// here:
#else
// for testing...
		PWM_X = PID_Composite;
		
		if ( RTD_Diff < 0 )	// heater on
		{
			heater_on = 1;
		}
		if ( RTD_Diff > 0 )	// cooling cycle
		{
			heater_on = -1;
			
#ifdef COOL_CYCLE_DEBUG_CURRENT_CHECK

			if ( RTD_Diff > 25. )			// still have to drive hard to make temp in range times
			{ 
#if 0				
			{
				PWM_X = 100;
			}
			else
			{
			PWM_X *= p->PWMCoolingRatio;
			}
#endif			
#if 1			
			if ( (*(p->sp) == 20.0 && (*(p->pv) <= 20.0) && WE_ARE_HERE++ < 250) && WE_ARE_HERE_FLAG )				// just for this case now for test
			{
				PWM_X = 100;				// artificially bias gain until those functions are ready to go in the GUI; and this is currently for testing
			} else {
				WE_ARE_HERE = 0;					// only saturate for limited time
				WE_ARE_HERE_FLAG = 0;
				PWM_X *= p->PWMCoolingRatio;
			}
// #else
//			if ( RTD_Diff > 25. )
//				PWM_X = 100;
//			PWM_X *= 5.;		// 5x pwm drive
#endif
#endif			
		}
	}
		if ( RTD_Diff == 0 )	// neither
		{
			heater_on = 0;
			PWM_X = 0.;
		}
#endif
#if 0		
		if (PWM_X < 100)					// orig float to int comparison
		{
			PWM_X = 100;
		}
#endif
		if (PWM_X < 0)
		{	//bug if hit					// xxx:::
			PWM_X = -1 * PWM_X;
		}
		if (PWM_X > 100)					// orig float to int comparison
		{
			PWM_X = 100;
		}
		PWM_state = PWM_X;
		heater_state = heater_on;
		
		//protection
		if ((*(p->pv)  >= 70.0 ) || //in case over heat											// malfunctioning RTD will still display strange readings possibly but PWM will be OFF.
		    (*(p->pv)  <= 10.0))	  //in case under cool
//		if ((calcTempLocal  >= 65.0) || //in case over heat
//				(calcTempLocal  <= 10.0))	  //in case under cool		
		{
			heater_on = 0;
			PWM_X = 0;
		}
		
		// in case of RTD sensor circuit open/short
		if ((bSensorOpenSent == 1) ||
		    (bSensorShortSent == 1))
		{
			heater_on = 0;
			PWM_X = 0;
		}
	}

	if ((heater_on == 1) || (heater_on == -1))
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

	if (heater_on == 0)
	{
#if 0	//turn on for test 
//		PWM_SetDC(1, 25);
		PWM_SetDC(2, 75);
#else			
//		PWM_SetDC(1, 0);
		PWM_SetDC(2, 0);
#endif
#ifdef FTX700D
		GPIO_WriteBit(GPIOA, BRIDGE_PWM_CONTROL_1, Bit_RESET);			//heater/cooler
		GPIO_WriteBit(GPIOA, BRIDGE_PWM_CONTROL_2, Bit_RESET);			//disable
#endif
	}
#if 1
	if (bSystemReady == 1)
	{
#ifdef ENABLE_PID_DBG		
#ifdef ENABLE_FTIO_DBG			
		memset(str, 0, 128);		
#if 1
		sprintf((char *)str, "Temp:% 10.2f\n\r", *(p->pv));			
//		sprintf((char *)str, "RTDM  % 10.2f, RTData  0x%X\n\r", calcTempLocal, middleData);			
#else

		sprintf((char *)str, "Temp:% 10.2f,H:%d,TR:%d,TN:%c\n\r",*(p->pv),heater_on, Trend, Turning);			
//		sprintf((char *)str, "Temp:%5.2f,PIDC:%7.2f,PWMC:%6.2f,PWMX:%6.2f,Heater:%d,Trend:%d,T:%c\n\r", calcTempLocal, PID_Current, PWM_Current, PWM_X, heater_on, Trend, Turning);			
#endif
		
		if (str[0]==0)
		{
			//
		}
		else
		{
			Serial_PutString(str);
		}
#endif		
#endif
	}
#else
//  SerialPutString("C-");
#endif
}		
		
		
		//nDAC_data=65535;
//		Laser_power_control(nDAC_data, 2);
//		nDAC_data=nDAC_data+4095;  //1024;
//		Laser_comm_sync();		// SMB related???? --- jcw
//		SPI_Cmd(SPI1,DISABLE); // DISable SPI1
//		;

//		osDelay(300);	
//		osDelay(50);											// backed off - for now
///		osDelay(20);											// backed off - for now
//		osDelay(10);											// backed off - for now
//		osDelay(2);											// backed off - for now
//		osDelay(15);
///		osMutexRelease(idRTD_DATA_Mutex);		// remember mutex's are additive
//		osDelay(200);
#ifdef PWM_DUTY_CHECK

		while (1) {
			PWM_SetDC(2, 100);
			osDelay(100);
			PWM_SetDC(2, 50);
			osDelay (100);
		}		

#endif
#ifdef DEBUG_TIMING_DEFINED		
		ping(1);
#endif		
///		osSignalSet(idPID_CTRL, 0x0001);
///			osDelay(1000);											// backed off - for now
	}

#endif
}		// end Temperature Control task


#if 0																			// if thermistor used
#ifdef CMSIS_RTOS
void Temp_REF_Control(void)
{
#if 0					// no thermistor in system
	TMC_Measured = ADS1248_ReadData(Therm_ADC);
#endif	
}
#endif
#endif


/*
* Function Prototype: void FirFilter(PID*, int*, int)                *
   * Useage: PidCtrl1(x, a, n);                                         *
   * Input Parameters: x = structure of type PID                        *
   *                   a = array of coefficients                        *
   *                   n = number of coefficients                       *
   * Return Value: none                                                 *
   * Notes: The filtering is done from last tap to first tap.  This     *
   *   allows more efficient delay chain updating.  The user must make  *
   *   sure the PID structure has at least as many delay chain entries  *
   *   as there are coefficients.                                       *
*********************************************************************/

// FirFilter(&TEC1, aLP, NLP);		// call the FIR filter function

#ifdef USE_FIR_FILTER

void FirFilter(struct _pid *x, int16_t *a, int16_t n)
{
/*** Local variables ***/
	int32_t temp32;						// general purpose unsigned long32
	int16_t *p;							// general purpose pointer to int16
	uint16_t i;							// general purpose unsigned int16

/*** Setup pointers ***/
	p = &x->y + (n-1);					// pointer setup to end of delay chain
	a = a + (n-1);						// point to the last coefficient
	temp32 = 0;							// initialize the sum

/*** Last tap has no delay chain update ***/
	temp32 = temp32 + ( ((int32_t)*a--) * ((int32_t)*p--) );

/*** Do the rest of the taps with delay chain update ***/
	for(i=n-1; i>0; i--)
	{
	temp32 = temp32 + ( ((int32_t)*a--) * ((int32_t)*p--) );
	*(p+1) = *p;					// update delay chain
	}

/*** Convert final value to floating point and put into the PID structure ***/
//	x->yf = (3.0/0x3FFC0000L)*(float)(temp32);
	x->yf = (3.0/0x00003FFCL)*(float)(temp32);
//	x->yf = (3.0/0x0000FFFFL)*(float)(temp32);

} //end FirFilter()
#endif

#endif
// *eof



