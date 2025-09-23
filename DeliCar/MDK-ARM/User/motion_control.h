#ifndef _MOTION_CONTROL_H_
#define _MOTION_CONTROL_H_

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "Stepper.h"
#include "math.h"
/* USER CODE END Includes */
extern Stepper MotorLeft; //id = 0
extern Stepper MotorRight; //id = 1

// extern volatile uint8_t flag_curve;
typedef struct{
  float Wheel_Radius;
	float Robot_Radius;
	float StepAngle_Deg;
	float StepAngle_dis;
	float Wheel_circuit;//distance in m to rotate 1 round of wheel
	float fX_Global, fY_Global, fAngle_Global;
	float fX_Global_aim, fY_Global_aim, fAngle_Global_aim;
	float fX_Global_real, fY_Global_real, fAngle_Global_real;
	float delta_Angle_Global_rad;
	float fAngle_Global_Rad, fRobot_Global_Position, fRobot_Global_Position_previous;
	float QR_Distance,QR_Distance_degree;
	long QR_Distance_step;
	float fVelo, fVeloHigh, fVeloLow, fVeloRotate, fAccel_Decel;
}Robot;
typedef struct{
  float fR1;
	float fR1_angle_rad;
	float fR1_distance;
	float fR2;
	float fR2_angle_rad;
	float fR2_distance;
	float fL;
	float fDelta_S;
	float fPhi;
	float fC_constant_bigger;
	float fC_constant_smaller;
	float speed_ratio_1;
	float speed_ratio_2;
	float Left_Distance,Right_Distance,Left_Accel_Decel,Right_Accel_Decel, Left_RPM, Right_RPM,fRemain_Distance;
	float x, y, delta_angle, angle;
}Curve_Profile;
void Robot_Init(Robot *RobotVar, float Wheel_Radius, float Robot_Radius, float StepAngle_Deg, int max_ppr, float QR_Distance);
void Motion_Start();
void Rotate(float Accel_Radps2, float Decel_Radps2, float maxRPM,  float Target_Angle_Global);
float SteptoDegree(Robot *RobotVar, float *Step);
float RadtoDegree(float *Rad);
float DegreetoRad(float *Degree);
void Motion_Stop();
void Curve_Profile_Init(Curve_Profile *CurveVar);
void Curve_Profile_Set(Curve_Profile *CurveVar, Robot* RobotVar, float L, float Delta_S, float Phi);
void Curve_Start(Curve_Profile *CurveVar,float L, float Delta_S, float Phi);
void Move_ForwardwithOneStepper(float Accel_Radps2, float Decel_Radps2, float maxRPM, float Target_Distance, Robot *RobotVar, Stepper *StepVar);
void Move_ForwardwithTwoStepper(float Accel_Radps2, float Decel_Radps2, float maxRPM, float Target_Distance, Robot *RobotVar, Stepper *StepVar1, Stepper *StepVar2);
void Move_ForwardwithCurve(float Accel_Radps2, float Decel_Radps2, float maxRPM, float Target_Distance);
void Update_Position(Robot *RobotVar);
long Accel_cnt_convert_with_Speed(float SpeedRadpS, float StepAngleRad, float AccelRadpS2);
//long Accel_cnt_convert_with_Accel_cnt(float SpeedRadpS, float StepAngleRad, float AccelRadpS2);
void Curve_Start_withVeloChange(Curve_Profile *CurveVar,float L, float Delta_S, float Phi);
void QR_Curve_Start(Robot *RobotVar, Curve_Profile *CurveVar);

//void Rotate_Motor(Stepper* pStep,float Accel_Radps2, float Decel_Radps2, float maxRPM, float target_position_deg);
void Rotate_Motor(Stepper* pStep, float Accel_Radps2, float Decel_Radps2, float Speed_RadpSec);

void Motor_Move(Stepper *StepVar, float Speed_RadpSec);
int DistantFeedback(Robot *RobotVar, Stepper *StepVar);

#endif
