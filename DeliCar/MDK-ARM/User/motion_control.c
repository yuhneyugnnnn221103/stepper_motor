#include "motion_control.h"
#include "define.h"

extern float accel_set;
extern float decel_set;

void Robot_Init(Robot *RobotVar, float Wheel_Radius, float Robot_Radius, float StepAngle_Deg, int max_ppr, float QR_Distance)
	{
		RobotVar->Wheel_Radius = Wheel_Radius;
		RobotVar->Robot_Radius = Robot_Radius;
		RobotVar->StepAngle_Deg = StepAngle_Deg;
		RobotVar->Wheel_circuit = 2 * PI * RobotVar->Wheel_Radius;
		RobotVar->StepAngle_dis = RobotVar->Wheel_circuit/max_ppr;
		RobotVar->fX_Global = 0;
		RobotVar->fY_Global = 0;
		RobotVar->fAngle_Global = 0;
		RobotVar->fX_Global_aim = 0;
		RobotVar->fY_Global_aim = 0;
		RobotVar->fAngle_Global_aim = 0;
		RobotVar->fX_Global_real = 0;
		RobotVar->fY_Global_real = 0;
		RobotVar->fAngle_Global_real = 0;
		RobotVar->fAngle_Global_Rad = 0;
		RobotVar->fRobot_Global_Position = 0;
		RobotVar->fRobot_Global_Position_previous = 0;
		RobotVar->QR_Distance = QR_Distance;
		RobotVar->QR_Distance_degree = ((float)QR_Distance / RobotVar->Wheel_circuit)*360;
		RobotVar->QR_Distance_step = ((float)QR_Distance / RobotVar->Wheel_circuit)*1000;
	}
void Motion_Start()
	{
		HAL_GPIO_WritePin(MOTOR_1_ENA_Port, MOTOR_1_ENA_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_2_ENA_Port, MOTOR_2_ENA_PIN, GPIO_PIN_RESET);	
	}
void Rotate(float Accel_Radps2, float Decel_Radps2, float maxRPM, float Target_Angle_Global)
	{

	}
float SteptoDegree(Robot *RobotVar, float *Step)
	{
		return (float)360 *(*Step)/1000;
	}
float RadtoDegree(float *Rad)
	{
		return (float)(*Rad)*180/ PI;
	}
float DegreetoRad(float *Degree)
	{
		return (float)(*Degree)*PI/ 180;
	}
void Motion_Stop()
	{
		HAL_GPIO_WritePin(MOTOR_1_ENA_Port, MOTOR_1_ENA_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_2_ENA_Port, MOTOR_2_ENA_PIN, GPIO_PIN_SET);		
	}
void Curve_Profile_Init(Curve_Profile *CurveVar)
	{
		  CurveVar->fR1 = 0;
			CurveVar->fR1_angle_rad = 0;
			CurveVar->fR1_distance = 0;
			CurveVar->fR2 = 0;
			CurveVar->fR2_angle_rad = 0;
			CurveVar->fR2_distance = 0;
			CurveVar->fL = 0;
			CurveVar->fDelta_S = 0;
			CurveVar->fPhi = 0;
			CurveVar->fC_constant_bigger = 0;
			CurveVar->fC_constant_smaller = 0;
			CurveVar->speed_ratio_1 = 0;
			CurveVar->speed_ratio_2 = 0;
	}
void Curve_Profile_Set(Curve_Profile *CurveVar, Robot* RobotVar, float L, float Delta_S, float Phi)
	{
			CurveVar->fL = L;
			CurveVar->fDelta_S = Delta_S;
			CurveVar->fPhi = Phi;
			float fDelta_S_abs = fabs(CurveVar->fDelta_S);
		  if(CurveVar->fDelta_S < 0)
			{				
				if(CurveVar->fPhi > 0)
				{
					float fPhi_rad = (float)(CurveVar->fPhi * PI)/ 180;
					CurveVar->fR1 = (float)sqrt(fDelta_S_abs * fDelta_S_abs + CurveVar->fL * CurveVar->fL) / (2 *sin(0.5 *fPhi_rad));
					CurveVar->speed_ratio_2 = (float)(CurveVar->fR1 + RobotVar->Robot_Radius) / (CurveVar->fR1 - RobotVar->Robot_Radius);
					CurveVar->fR1_angle_rad = fPhi_rad;
					CurveVar->fR1_distance = CurveVar->fR1_angle_rad * CurveVar->fR1;
				}
				if(CurveVar->fPhi == 0)
				{
					CurveVar->fR1_angle_rad = PI - 2 * atan(CurveVar->fL/fDelta_S_abs);
					float fR1_bottom = 0.5 * sqrt(fDelta_S_abs * fDelta_S_abs + CurveVar->fL * CurveVar->fL);
					CurveVar->fR1 = (float)fR1_bottom / (2*sin(0.5*CurveVar->fR1_angle_rad));
					CurveVar->speed_ratio_1 = (float)(CurveVar->fR1 + RobotVar->Robot_Radius) / (CurveVar->fR1 - RobotVar->Robot_Radius);;
					CurveVar->fR1_distance = CurveVar->fR1_angle_rad * CurveVar->fR1;
					
					CurveVar->fR2 = CurveVar->fR1;
					CurveVar->speed_ratio_2 = CurveVar->speed_ratio_1;
					CurveVar->fR2_angle_rad = CurveVar->fR1_angle_rad;
					CurveVar->fR2_distance = CurveVar->fR1_distance;
				}
				if(CurveVar->fPhi < 0)
				{
					float fPhi_rad_abs = -(float)(CurveVar->fPhi * PI)/ 180;
					CurveVar->fR1 = (float)CurveVar->fL / (4*sin(fPhi_rad_abs));
					CurveVar->speed_ratio_1 = (float)(CurveVar->fR1 + RobotVar->Robot_Radius) / (CurveVar->fR1 - RobotVar->Robot_Radius);
					CurveVar->fR1_angle_rad = 2*fPhi_rad_abs;
					CurveVar->fR1_distance = CurveVar->fR1_angle_rad * CurveVar->fR1;
					float fR2_bottom = sqrt(fDelta_S_abs * fDelta_S_abs + 0.25* CurveVar->fL * CurveVar->fL);
					CurveVar->fR2 = (float) fR2_bottom / (2 *sin(0.5* fPhi_rad_abs));
					CurveVar->speed_ratio_2 = (float)(CurveVar->fR2 + RobotVar->Robot_Radius) / (CurveVar->fR2 - RobotVar->Robot_Radius);
					CurveVar->fR2_angle_rad = fPhi_rad_abs;
					CurveVar->fR2_distance =  CurveVar->fR2_angle_rad * CurveVar->fR2;
				}
			}
			if(CurveVar->fDelta_S > 0)
			{				
				if(CurveVar->fPhi < 0)
				{
					float fPhi_abs = fabs(CurveVar->fPhi);
					float fPhi_rad = (float)(fPhi_abs * PI)/ 180;
					CurveVar->fR1 = (float)sqrt(fDelta_S_abs * fDelta_S_abs + CurveVar->fL * CurveVar->fL) / (2 *sin(0.5 *fPhi_rad));
					CurveVar->speed_ratio_2 = (float)(CurveVar->fR1 + RobotVar->Robot_Radius) / (CurveVar->fR1 - RobotVar->Robot_Radius);
					CurveVar->fR1_angle_rad = fPhi_rad;
					CurveVar->fR1_distance = CurveVar->fR1_angle_rad * CurveVar->fR1;
				}
				if(CurveVar->fPhi == 0)
				{
					CurveVar->fR1_angle_rad = PI - 2 * atan(CurveVar->fL/fDelta_S_abs);
					float fR1_bottom = 0.5 * sqrt(fDelta_S_abs * fDelta_S_abs + CurveVar->fL * CurveVar->fL);
					CurveVar->fR1 = (float)fR1_bottom / (2*sin(0.5*CurveVar->fR1_angle_rad));
					CurveVar->speed_ratio_1 = (float)(CurveVar->fR1 + RobotVar->Robot_Radius) / (CurveVar->fR1 - RobotVar->Robot_Radius);;
					CurveVar->fR1_distance = CurveVar->fR1_angle_rad * CurveVar->fR1;
					
					CurveVar->fR2 = CurveVar->fR1;
					CurveVar->speed_ratio_2 = CurveVar->speed_ratio_1;
					CurveVar->fR2_angle_rad = CurveVar->fR1_angle_rad;
					CurveVar->fR2_distance = CurveVar->fR1_distance;
				}
				if(CurveVar->fPhi > 0)
				{
					float fPhi_rad_abs = (float)(CurveVar->fPhi * PI)/ 180;
					CurveVar->fR1 = (float)CurveVar->fL / (4*sin(fPhi_rad_abs));
					CurveVar->speed_ratio_1 = (float)(CurveVar->fR1 + RobotVar->Robot_Radius) / (CurveVar->fR1 - RobotVar->Robot_Radius);
					CurveVar->fR1_angle_rad = 2*fPhi_rad_abs;
					CurveVar->fR1_distance = CurveVar->fR1_angle_rad * CurveVar->fR1;
					float fR2_bottom = sqrt(fDelta_S_abs * fDelta_S_abs + 0.25* CurveVar->fL * CurveVar->fL);
					CurveVar->fR2 = (float) fR2_bottom / (2 *sin(0.5* fPhi_rad_abs));
					CurveVar->speed_ratio_2 = (float)(CurveVar->fR2 + RobotVar->Robot_Radius) / (CurveVar->fR2 - RobotVar->Robot_Radius);
					CurveVar->fR2_angle_rad = fPhi_rad_abs;
					CurveVar->fR2_distance =  CurveVar->fR2_angle_rad * CurveVar->fR2;
				}
			}
	}
void Curve_Start(Curve_Profile *CurveVar,float L, float Delta_S, float Phi)
	{

	}
void Move_ForwardwithCurve(float Accel_Radps2, float Decel_Radps2, float maxRPM, float Target_Distance)
	{

	}
void Move_ForwardwithOneStepper(float Accel_Radps2, float Decel_Radps2, float maxRPM, float Target_Distance, Robot *RobotVar, Stepper *StepVar)
	{
		float delta_Distance = Target_Distance;
		float target_position_degree = 0;
		if(delta_Distance != 0)// meter unit
		{
			target_position_degree = ((float)delta_Distance / RobotVar->Wheel_circuit)*360;
			//Set motion profile
			MotionProfile_Set(StepVar, Accel_Radps2, Decel_Radps2, maxRPM, target_position_degree, RobotVar->QR_Distance_degree);	
			//Clear state vairable of the stepper driver
			Stepper_StateRst(StepVar);
			//Reset variables used in motion profile before using
			MotionVar_Prepare(StepVar);
			//Start stepper motor control
			Stepper_withCurve_Start(StepVar);
			}
	}	
void Move_ForwardwithTwoStepper(float Accel_Radps2, float Decel_Radps2, float maxRPM, float Target_Distance, Robot *RobotVar, Stepper *StepVar1, Stepper *StepVar2)
	{
		float delta_Distance = Target_Distance;
		float target_position_degree = 0;
		if(delta_Distance != 0)// meter unit
		{
			target_position_degree = ((float)delta_Distance / RobotVar->Wheel_circuit)*360;
			//Set motion profile
			MotionProfile_Set(StepVar1, Accel_Radps2, Decel_Radps2, maxRPM, -target_position_degree, RobotVar->QR_Distance_degree);	
			MotionProfile_Set(StepVar2, Accel_Radps2, Decel_Radps2, maxRPM, target_position_degree, RobotVar->QR_Distance_degree);
			//Clear state vairable of the stepper driver
			Stepper_StateRst(StepVar1);
			Stepper_StateRst(StepVar2);
			//Reset variables used in motion profile before using
			MotionVar_Prepare(StepVar1);
			MotionVar_Prepare(StepVar2);
			//Start stepper motor control
			Stepper_withCurve_Start(StepVar1);
			Stepper_withCurve_Start(StepVar2);
			}
	}
void Update_Position(Robot *RobotVar)
	{

	}
long Accel_cnt_convert_with_Speed(float SpeedRadpS, float StepAngleRad, float AccelRadpS2)
	{
		long accel_cnt_equip = (SpeedRadpS*SpeedRadpS) / (2*StepAngleRad*AccelRadpS2);
		return accel_cnt_equip;
	}

void Curve_Start_withVeloChange(Curve_Profile *CurveVar,float L, float Delta_S, float Phi)
	{

	}
	void QR_Curve_Start(Robot *RobotVar, Curve_Profile *CurveVar)
	{
		if((CurveVar->angle == 90) && (CurveVar->y>0) && (CurveVar->delta_angle >0))
		{
			float delta_angle_rad, phi, KB, xK, xB_dot, xR, xL, yR, yL, x0, y0, rR, rL, ratio = 0;
			delta_angle_rad = (float)(CurveVar->delta_angle * PI)/ 180;
			phi = (PI/2) - delta_angle_rad;
			KB = CurveVar->y/(sin(phi));
			xK = CurveVar->x + CurveVar->y/(tan(phi));
			xB_dot= xK +KB;
			xR = CurveVar->x + RobotVar->Robot_Radius * cos(delta_angle_rad);
			xL = CurveVar->x - RobotVar->Robot_Radius * cos(delta_angle_rad);
			yR = CurveVar->y + RobotVar->Robot_Radius * sin(delta_angle_rad);
			yL = CurveVar->y - RobotVar->Robot_Radius * sin(delta_angle_rad);
			x0 = xB_dot;
			y0 = ((yR-CurveVar->y)/(xR-CurveVar->x))*xB_dot + (CurveVar->y - ((yR - CurveVar->y)/(xR - CurveVar->x))*CurveVar->x);
			rR = sqrt((x0-xR)*(x0-xR)+(y0-yR)*(y0-yR));
			rL = sqrt((x0-xL)*(x0-xL)+(y0-yL)*(y0-yL));
			ratio = rL/rR;
			CurveVar->Left_Distance = -rL * phi;// back move
			CurveVar->Right_Distance = -rR * phi;// back move
			CurveVar->Left_Accel_Decel = ratio * 10;
			CurveVar->Right_Accel_Decel = 10;
			CurveVar->Left_RPM = ratio * 25;
			CurveVar->Right_RPM = 25;
			CurveVar->fRemain_Distance = xB_dot;
		}
		if((CurveVar->angle == 90) && (CurveVar->y>0) && (CurveVar->delta_angle <0))
		{
			float delta_angle_rad, phi, beta, KB, xK, xB_dot, xR, xL, yR, yL, x0, y0, rR, rL, ratio = 0;
			delta_angle_rad = (float)(-CurveVar->delta_angle * PI)/ 180;
			beta = (PI/2) - delta_angle_rad;
			phi = PI - beta;
			KB = CurveVar->y/(sin(beta));
			xK = CurveVar->x - CurveVar->y/(tan(beta));
			xB_dot= xK +KB;
			xR = CurveVar->x + RobotVar->Robot_Radius * cos(delta_angle_rad);
			xL = CurveVar->x - RobotVar->Robot_Radius * cos(delta_angle_rad);
			yR = CurveVar->y - RobotVar->Robot_Radius * sin(delta_angle_rad);
			yL = CurveVar->y + RobotVar->Robot_Radius * sin(delta_angle_rad);
			x0 = xB_dot;
			y0 = ((yR-CurveVar->y)/(xR-CurveVar->x))*xB_dot + (CurveVar->y - ((yR - CurveVar->y)/(xR - CurveVar->x))*CurveVar->x);
			rR = sqrt((x0-xR)*(x0-xR)+(y0-yR)*(y0-yR));
			rL = sqrt((x0-xL)*(x0-xL)+(y0-yL)*(y0-yL));
			ratio = rL/rR;
			CurveVar->Left_Distance = -rL * phi;//back move
			CurveVar->Right_Distance = -rR * phi;//back move
			CurveVar->Left_Accel_Decel = ratio * 10;
			CurveVar->Right_Accel_Decel = 10;
			CurveVar->Left_RPM = ratio * 25;
			CurveVar->Right_RPM = 25;
			CurveVar->fRemain_Distance = xB_dot;
		}
		if((CurveVar->angle == -90) && (CurveVar->y>0) && (CurveVar->delta_angle >0))
		{
			float delta_angle_rad, phi, beta, KB, xK, xB_dot, xR, xL, yR, yL, x0, y0, rR, rL, ratio = 0;
			delta_angle_rad = (float)(CurveVar->delta_angle * PI)/ 180;
			beta = (PI/2) - delta_angle_rad;
			phi = PI - beta;
			KB = CurveVar->y/(sin(beta));
			xK = CurveVar->x + CurveVar->y/(tan(beta));
			xB_dot= xK -KB;
			xR = CurveVar->x + RobotVar->Robot_Radius * cos(delta_angle_rad);
			xL = CurveVar->x - RobotVar->Robot_Radius * cos(delta_angle_rad);
			yR = CurveVar->y + RobotVar->Robot_Radius * sin(delta_angle_rad);
			yL = CurveVar->y - RobotVar->Robot_Radius * sin(delta_angle_rad);
			x0 = xB_dot;
			y0 = ((yR-CurveVar->y)/(xR-CurveVar->x))*xB_dot + (CurveVar->y - ((yR - CurveVar->y)/(xR - CurveVar->x))*CurveVar->x);
			rR = sqrt((x0-xR)*(x0-xR)+(y0-yR)*(y0-yR));
			rL = sqrt((x0-xL)*(x0-xL)+(y0-yL)*(y0-yL));
			ratio = rR/rL;
			CurveVar->Left_Distance = -rL * phi;//back move
			CurveVar->Right_Distance = -rR * phi;//back move
			CurveVar->Right_Accel_Decel = ratio * 10;
			CurveVar->Left_Accel_Decel = 10;
			CurveVar->Right_RPM = ratio * 25;
			CurveVar->Left_RPM = 25;
			CurveVar->fRemain_Distance = -xB_dot;
		}
		if((CurveVar->angle == -90) && (CurveVar->y>0) && (CurveVar->delta_angle <0))
		{
			float delta_angle_rad, phi, KB, xK, xB_dot, xR, xL, yR, yL, x0, y0, rR, rL, ratio = 0;
			delta_angle_rad = (float)(-CurveVar->delta_angle * PI)/ 180;
			phi = (PI/2) - delta_angle_rad;
			KB = CurveVar->y/(sin(phi));
			xK = CurveVar->x - CurveVar->y/(tan(phi));
			xB_dot= xK -KB;
			xR = CurveVar->x + RobotVar->Robot_Radius * cos(delta_angle_rad);
			xL = CurveVar->x - RobotVar->Robot_Radius * cos(delta_angle_rad);
			yR = CurveVar->y - RobotVar->Robot_Radius * sin(delta_angle_rad);
			yL = CurveVar->y + RobotVar->Robot_Radius * sin(delta_angle_rad);
			x0 = xB_dot;
			y0 = ((yR-CurveVar->y)/(xR-CurveVar->x))*xB_dot + (CurveVar->y - ((yR - CurveVar->y)/(xR - CurveVar->x))*CurveVar->x);
			rR = sqrt((x0-xR)*(x0-xR)+(y0-yR)*(y0-yR));
			rL = sqrt((x0-xL)*(x0-xL)+(y0-yL)*(y0-yL));
			ratio = rR/rL;
			CurveVar->Left_Distance = -rL * phi;//back move
			CurveVar->Right_Distance = -rR * phi;//back move
			CurveVar->Right_Accel_Decel = ratio * 10;
			CurveVar->Left_Accel_Decel = 10;
			CurveVar->Right_RPM = ratio * 25;
			CurveVar->Left_RPM = 25;
			CurveVar->fRemain_Distance = -xB_dot;
		}
		if((CurveVar->angle == 90) && (CurveVar->y<0) && (CurveVar->delta_angle >0))
		{
			float delta_angle_rad, phi, KB, xK, xB_dot, xR, xL, yR, yL, x0, y0, rR, rL, ratio = 0;
			delta_angle_rad = (float)(CurveVar->delta_angle * PI)/ 180;
			phi = (PI/2) - delta_angle_rad;
			KB = -CurveVar->y/(sin(phi));
			xK = CurveVar->x - (-CurveVar->y)/(tan(phi));
			xB_dot= xK -KB;
			xR = CurveVar->x + RobotVar->Robot_Radius * cos(delta_angle_rad);
			xL = CurveVar->x - RobotVar->Robot_Radius * cos(delta_angle_rad);
			yR = CurveVar->y + RobotVar->Robot_Radius * sin(delta_angle_rad);
			yL = CurveVar->y - RobotVar->Robot_Radius * sin(delta_angle_rad);
			x0 = xB_dot;
			y0 = ((yR-CurveVar->y)/(xR-CurveVar->x))*xB_dot + (CurveVar->y - ((yR - CurveVar->y)/(xR - CurveVar->x))*CurveVar->x);
			rR = sqrt((x0-xR)*(x0-xR)+(y0-yR)*(y0-yR));
			rL = sqrt((x0-xL)*(x0-xL)+(y0-yL)*(y0-yL));
			ratio = rR/rL;
			CurveVar->Left_Distance = rL * phi;
			CurveVar->Right_Distance = rR * phi;
			CurveVar->Right_Accel_Decel = ratio * 10;
			CurveVar->Left_Accel_Decel = 10;
			CurveVar->Right_RPM = ratio * 25;
			CurveVar->Left_RPM = 25;
			CurveVar->fRemain_Distance = xB_dot;
		}
		if((CurveVar->angle == 90) && (CurveVar->y<0) && (CurveVar->delta_angle <0))
		{
			float delta_angle_rad, phi, beta, KB, xK, xB_dot, xR, xL, yR, yL, x0, y0, rR, rL, ratio = 0;
			delta_angle_rad = (float)(-CurveVar->delta_angle * PI)/ 180;
			beta = (PI/2) - delta_angle_rad;
			phi = PI - beta;
			KB = -CurveVar->y/(sin(beta));
			xK = CurveVar->x + (-CurveVar->y)/(tan(beta));
			xB_dot= xK -KB;
			xR = CurveVar->x + RobotVar->Robot_Radius * cos(delta_angle_rad);
			xL = CurveVar->x - RobotVar->Robot_Radius * cos(delta_angle_rad);
			yR = CurveVar->y - RobotVar->Robot_Radius * sin(delta_angle_rad);
			yL = CurveVar->y + RobotVar->Robot_Radius * sin(delta_angle_rad);
			x0 = xB_dot;
			y0 = ((yR-CurveVar->y)/(xR-CurveVar->x))*xB_dot + (CurveVar->y - ((yR - CurveVar->y)/(xR - CurveVar->x))*CurveVar->x);
			rR = sqrt((x0-xR)*(x0-xR)+(y0-yR)*(y0-yR));
			rL = sqrt((x0-xL)*(x0-xL)+(y0-yL)*(y0-yL));
			ratio = rR/rL;
			CurveVar->Left_Distance = rL * phi;
			CurveVar->Right_Distance = rR * phi;
			CurveVar->Right_Accel_Decel = ratio * 10;
			CurveVar->Left_Accel_Decel = 10;
			CurveVar->Right_RPM = ratio * 25;
			CurveVar->Left_RPM = 25;
			CurveVar->fRemain_Distance = xB_dot;
		}
		if((CurveVar->angle == -90) && (CurveVar->y<0) && (CurveVar->delta_angle >0))
		{
			float delta_angle_rad, phi, beta, KB, xK, xB_dot, xR, xL, yR, yL, x0, y0, rR, rL, ratio = 0;
			delta_angle_rad = (float)(CurveVar->delta_angle * PI)/ 180;
			beta = (PI/2) - delta_angle_rad;
			phi = PI - beta;
			KB = -CurveVar->y/(sin(beta));
			xK = CurveVar->x - (-CurveVar->y)/(tan(beta));
			xB_dot= xK +KB;
			xR = CurveVar->x + RobotVar->Robot_Radius * cos(delta_angle_rad);
			xL = CurveVar->x - RobotVar->Robot_Radius * cos(delta_angle_rad);
			yR = CurveVar->y + RobotVar->Robot_Radius * sin(delta_angle_rad);
			yL = CurveVar->y - RobotVar->Robot_Radius * sin(delta_angle_rad);
			x0 = xB_dot;
			y0 = ((yR-CurveVar->y)/(xR-CurveVar->x))*xB_dot + (CurveVar->y - ((yR - CurveVar->y)/(xR - CurveVar->x))*CurveVar->x);
			rR = sqrt((x0-xR)*(x0-xR)+(y0-yR)*(y0-yR));
			rL = sqrt((x0-xL)*(x0-xL)+(y0-yL)*(y0-yL));
			ratio = rL/rR;
			CurveVar->Left_Distance = rL * phi;
			CurveVar->Right_Distance = rR * phi;
			CurveVar->Left_Accel_Decel = ratio * 10;
			CurveVar->Right_Accel_Decel = 10;
			CurveVar->Left_RPM = ratio * 25;
			CurveVar->Right_RPM = 25;
			CurveVar->fRemain_Distance = -xB_dot;
		}
		if((CurveVar->angle == -90) && (CurveVar->y<0) && (CurveVar->delta_angle <0))
		{
			float delta_angle_rad, phi, KB, xK, xB_dot, xR, xL, yR, yL, x0, y0, rR, rL, ratio = 0;
			delta_angle_rad = (float)(-CurveVar->delta_angle * PI)/ 180;
			phi = (PI/2) - delta_angle_rad;
			KB = -CurveVar->y/(sin(phi));
			xK = CurveVar->x + (-CurveVar->y)/(tan(phi));
			xB_dot= xK +KB;
			xR = CurveVar->x + RobotVar->Robot_Radius * cos(delta_angle_rad);
			xL = CurveVar->x - RobotVar->Robot_Radius * cos(delta_angle_rad);
			yR = CurveVar->y - RobotVar->Robot_Radius * sin(delta_angle_rad);
			yL = CurveVar->y + RobotVar->Robot_Radius * sin(delta_angle_rad);
			x0 = xB_dot;
			y0 = ((yR-CurveVar->y)/(xR-CurveVar->x))*xB_dot + (CurveVar->y - ((yR - CurveVar->y)/(xR - CurveVar->x))*CurveVar->x);
			rR = sqrt((x0-xR)*(x0-xR)+(y0-yR)*(y0-yR));
			rL = sqrt((x0-xL)*(x0-xL)+(y0-yL)*(y0-yL));
			ratio = rL/rR;
			CurveVar->Left_Distance = rL * phi;
			CurveVar->Right_Distance = rR * phi;
			CurveVar->Left_Accel_Decel = ratio * 10;
			CurveVar->Right_Accel_Decel = 10;
			CurveVar->Left_RPM = ratio * 25;
			CurveVar->Right_RPM = 25;
			CurveVar->fRemain_Distance = -xB_dot;
		}
	}

//void Rotate_Motor(Stepper* pStep,float Accel_Radps2, float Decel_Radps2, float maxRPM, float target_position_deg)
//{
//	//Set motion profile	
//	MotionProfile_Set(pStep, Accel_Radps2, Decel_Radps2, maxRPM, target_position_deg, 0);
//	//Clear state vairable of the stepper driver
//	Stepper_StateRst(pStep);
//	//Reset variables used in motion profile before using
//	MotionVar_Prepare(pStep);
//	//Start stepper motor control
//	Stepper_withCurve_Start(pStep);	
//}
	
void Rotate_Motor(Stepper* pStep,float Accel_Radps2, float Decel_Radps2, float Speed_RadpSec)
{
	//Set motion profile	
	MotionProfile_Accel_Only_Set(pStep, Accel_Radps2, Decel_Radps2, Speed_RadpSec);
	//Clear state vairable of the stepper driver
	Stepper_StateRst(pStep);
	//Reset variables used in motion profile before using
	MotionVar_Prepare(pStep);
	//Start stepper motor control
	Stepper_withCurve_Start(pStep);	
}

/**********************************************************************/
void Motor_Move(Stepper *StepVar, float Speed_RadpSec)
{
	if(fabs(Speed_RadpSec) >= 0.5f) {
	
		if(Speed_RadpSec >= 10.0f) StepVar->maxSpeed_RadpSec = 10.0f;
		else if(Speed_RadpSec <= -10.0f) StepVar->maxSpeed_RadpSec = -10.0f;
		else StepVar->maxSpeed_RadpSec = Speed_RadpSec;
		
		if(StepVar->state != stRUN) {
			//Set motion profile
			MotionProfile_Accel_Only_Set(StepVar, accel_set, decel_set, StepVar->maxSpeed_RadpSec);
			//Clear state vairable of the stepper driver
			Stepper_StateRst(StepVar);
			//Reset variables used in motion profile before using
			MotionVar_Prepare(StepVar);
			//Start stepper motor control
			Stepper_withCurve_Start(StepVar);
		}
		else {
			uint8_t newdir = (StepVar->maxSpeed_RadpSec < 0) ? 0 : 1;
			if(StepVar->direction_temp != newdir) {
				StepVar->accel_set_cnt = 0;
				if(StepVar->accel_cnt <= 5) {
					Step_Direction(StepVar, newdir);
					StepVar->direction_temp = newdir;
				}
			}
			else {
				StepVar->accel_set_cnt = (StepVar->maxSpeed_RadpSec)*(StepVar->maxSpeed_RadpSec)/(2 * StepVar->RealStepAngle_Rad * StepVar->Accel_Radps2);
			}
		}
	}
	else {
		Stepper_Stop(StepVar);
		if(StepVar->decel_cnt > -5) {
						StepVar->move = 0;
						StepVar->state = stLOCKROTOR;
						StepVar->cmd = 0;
		}
	}
}
/*********************************************************************/
int DistantFeedback(Robot *RobotVar, Stepper *StepVar)
{
	long step_temp = StepVar->step_feedback;
	StepVar->step_feedback = 0;
	
	float s = (float) step_temp * StepVar->RealStepAngle_Rad * RobotVar->Wheel_Radius * 1000;
	
	return (int)s;
}	
/*********************************************************************/