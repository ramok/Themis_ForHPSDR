/// Note:  control.h has been deprecated and replaced with
///        this file.
// tec-control-pid.h
// Oct. 8, 2017


#ifndef __PID_CONTROL_PID__
#define __PID_CONTROL_PID__
// #include "arm-math.h"

// current states - for imaging
typedef enum   
{
	StateNotReady = 0,
	StateReady, 
	StateExposure1,
	StateEOFDelay1,
	StateExposure2,
	StateEOFDelay2,
	StateMove,
	StateMoveSent,
	StateMoveComplete,											// for ACS move complete
	StateStaticScan,													// new state for just taking a pic - no stage movement
	StateACS_MC
}ScanState;

// max of 5:
typedef struct _PIDStage { 	// 20 bytes; so this is 100 bytes total
	float stageTemp;
	float kp;
	float ki;
	float kd;
	float initIntegral;
} PIDStage;

typedef struct _pid {
//    uint8_t dummy_pad;
	float *pv;  	/*pointer to an <integer> that contains the process value*/
//	uint8_t dummy_pad1;
	float *sp;  	/*pointer to an <integer> that contains the set point*/
	float integral;
	float pgain;		// Kp: Proportional Gain
	float igain;		// Ki: Integral Gain
	float dgain;		// Kd: Derivative Gain
	int deadband;		// band of which response should be held constant
	int last_error;		// Last calculated error
	float prev_error;		// Last -1 Error (n-1)
	int error_sum;		// Sum Of Errors - note we will have integral windup protection now
	float integral_error_max;	// The max value of Error
	float integral_error_min; // The min value of Error
	float uof;		// unfiltered plant output
	float  yf;	 	// yf(k), filtered plant output
	int16_t y;		// y(k), plant output, I1Q15
	int16_t y1;   	        // y(k-1), previous plant output, I1Q15
	int16_t y2; 		// y(k-2), previous plant output, I1Q15
	int16_t y3;		// y(k-3), previous plant output, I1Q15
	int16_t y4;		// y(k-4), previous plant output, I1Q15
	float TargetTemperature;
	float Temperature_UpperThreshold;
	float Temperature_LowerThreshold;
	float Temperature_TimeInRange;
	float Temperature_TimeoutSetTemp;
	float PWMCoolingRatio;
	uint8_t numOfStages;	// number of stages...
	uint8_t current_stage;
//    float *pv;      /*pointer to an <integer> that contains the process value*/
//  uint8_t dummy_pad1;
//    float *sp;      /*pointer to an <integer> that contains the set point*/
//	uint8_t pad1;
//	uint8_t pad2;
	
} pid;

#define			Q				32768	// I1Q15 fraction scaling value
#define			NLP				5		// low-pass filter tap length






void Temp_REF_Control(void);
void Temp_PID_Control(void);
#ifdef TWO_THREAD_MODE
void Laser1_Control(void const *argument);
void Laser2_Control(void const *argument);
#else
// void Temperature_Control((void const *)(struct _pid *p));
void Temperature_Control(void const *argument);
#endif

void pid_init(struct _pid *a, float *pv, float *sp);
void pid_tune(struct _pid *a, float p_gain, float i_gain, float d_gain, int dead_band);
void get_gains(struct _pid *a, float *p_gain, float *i_gain, float *d_gain, int *dead_band);
void pid_setinteg(struct _pid *a, float new_integ);
void pid_bumpless(struct _pid *a);
// float pid_calc(struct _pid *a);
void pid_calc(struct _pid *a);
void FirFilter(struct _pid *x, int16_t *a, int16_t n);

















#endif

// *eof

