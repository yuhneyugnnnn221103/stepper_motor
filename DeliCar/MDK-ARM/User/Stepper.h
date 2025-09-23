#ifndef __STEPPER_H_
#define __STEPPER_H_

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include <math.h>
#include "timer.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Extern */
extern int time_start;
extern int time_end;
extern int time_process;
/* USER CODE END Extern */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
typedef struct{
	uint16_t ppr,rpm,Jog_rpm,accel,decel;
	uint8_t nEn;     	// Low : Enable , High : Disable
	int StepMode ;   	// Full , Half, 1/4,1/8,1/16,1/32
	uint8_t CtrlMode; // Position : 0 ; Speed :1
  float GearRatio, BaseStepAngle_Degree, RealStepAngle_Rad;
	int8_t move, Direction, state, cmd, PosFlag;
  long Position_count_total,Position_count_signed;
	float maxSpeed_RadpSec, Accel_Radps2, Decel_Radps2, SetAngle_Deg;
	long Total_cnt, Total_cnt_temp, Forward_cnt, Forward_cnt_temp, Curve_cnt, Curve_cnt_temp;
	long run_cnt , run_set_cnt;
	long accel_cnt, accel_set_cnt;
	long decel_set_cnt, decel_cnt, decel_cnt_temp;
	long max_s_lim, accel_lim;
	long iCn_cnt,iCn,Cn, Cn_1, Crun ,rest;
	uint8_t id;
	float target_position_degree;
	long accel_cnt_convert_curve;
	uint8_t direct;
	uint8_t flag_phase_1,flag_phase_2, flag_busy, flag_calib;
	long Co, Co_temp;
	long Cn_max;
	long qr_cnt,qr_set_cnt,calib_cnt, qr_total_cnt, qr_current_cnt;
	uint8_t flag_calib_type1,flag_calib_type2;
	uint8_t flag_res;
	long total_curve_equipvalent;
	uint8_t flag_accel,flag_accel_done, flag_decel, flag_decel_done;
	long accel_set_cnt_temp, accel_set_cnt_temp_speed_high, accel_set_cnt_temp_speed_low;
	uint8_t flag_tracking;
	long tracking_cnt;
	uint8_t flag_calib_accel, flag_calib_decel;
	long accel_cnt_temp;
	uint8_t flag_pause, flag_pause_done, flag_emergency; 
	float speed_ratio_1;
	float speed_ratio_2;
	float speed_ratio;
	uint8_t direction_temp;
	float fForward_cnt, fForward_ratio;
	uint8_t flag_calib_qr_curve;
	long desire_step, temp_step;
	
	long step_feedback;
}Stepper;

#define NumofStepMotor 3

// Stepper motor state
#define stRUN    2			//Acc, Run or Dec
#define stLOCKROTOR 1		//Lock rotor by holding torque
#define stOFF    0			//Free wheeling - Iset = 0

// Stepper motor command
#define Runcmd  1
#define Stopcmd 0

//Direction
#define FWD 0
#define REV 1

//Control mode
#define mPOSITION 0
#define mSPEED    1

//Timer 4 is use for Stepper Motor control
#define f_Timer4 100 //kHz
#define AutReloadReg_Max 10

#define PI  3.141592
#define Accel_min 5 // rad/s^2
#define Accel_max 200
#define Decel_min 5
#define Decel_max 200
#define rpm_min  10  // round per minute
#define rpm_max  300
#define Jog_rpm_max 100
#define Jog_rpm_min 10

//extern TIM_HandleTypeDef htim1;//stepper right
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern Stepper StepVar;
/* USER CODE END PV */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PF */
void Stepper_LockRotor(Stepper *StepVar);
void Stepper_FreeRotor(Stepper *StepVar);
void step(Stepper *StepVar);
void Step_Direction (Stepper *StepVar, uint8_t direct);
void Stepper_Init(Stepper *StepVar,float StepAngle_Deg, uint8_t stpMode, uint8_t CtrlMode, float GearRatio, uint8_t id);
void MotionProfile_Set(Stepper *StepVar, float Accel_Radps2, float Decel_Radps2, float maxRPM, float SetAngle_Deg, float QR_Distance_degree);

//void MotionProfile_Accel_Only_Set(Stepper *StepVar, float Accel_Radps2, float Decel_Radps2, float maxRPM, float SetAngle_Deg);
void MotionProfile_Accel_Only_Set(Stepper *StepVar, float Accel_Radps2, float Decel_Radps2, float Speed_RadpSec);

void MotionVar_Prepare(Stepper *StepVar);
void Stepper_StateRst(Stepper *StepVar);
void Stepper_withCurve_Start(Stepper *StepVar);
void Stepper_Stop(Stepper *StepVar);
void Stepper_Emergency_Stop(Stepper *StepVar);
void Stepper_End_Jogging(Stepper *StepVar);
void Stepper_Control(Stepper *StepVar);
void Stepper_Control_withCurve(Stepper *StepVar, Stepper *StepVar_temp1, Stepper *StepVar_temp2);
void Stepper_Reset(Stepper *StepVar);
/* USER CODE END PF */

/************************************************************************************/
#endif /* _STEPPER_H_*/



