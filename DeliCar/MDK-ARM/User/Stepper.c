#include "Stepper.h"
extern Stepper StepVar;
/**************************************************/
void step(Stepper *StepVar)
{
	if(StepVar->id == 0) //id =0 <==>  motor 1
		{
			HAL_GPIO_WritePin(PUL1_GPIO_Port,PUL1_Pin,GPIO_PIN_SET);
			delayMicros(3);
			HAL_GPIO_WritePin(PUL1_GPIO_Port,PUL1_Pin,GPIO_PIN_RESET);
		}
	if(StepVar->id == 1) //id =1 <==>  motor 2
		{
			HAL_GPIO_WritePin(PUL2_GPIO_Port,PUL2_Pin,GPIO_PIN_SET);
			delayMicros(3);
			HAL_GPIO_WritePin(PUL2_GPIO_Port,PUL2_Pin,GPIO_PIN_RESET);
		}
}
/*******************************************************/
/*Set initial parameters for each stepper motors*/
void Stepper_Init(Stepper *StepVar,float StepAngle_Deg, uint8_t stpMode, uint8_t CtrlMode, float GearRatio, uint8_t id)
{
	StepVar->GearRatio = GearRatio;     								// Ti so truyen 1/GearRatio
	StepVar->StepMode = stpMode;
	StepVar->BaseStepAngle_Degree = StepAngle_Deg;  //Step angle of the motor, which can be seen in the name plate
	switch (StepVar->StepMode)
	{
			case 0://Full
					StepVar->RealStepAngle_Rad = (StepVar->BaseStepAngle_Degree/StepVar->GearRatio)*(PI/180);  			
					break;
			case 1: //Half_A
					StepVar->RealStepAngle_Rad  = (StepVar->BaseStepAngle_Degree*0.5/StepVar->GearRatio)*(PI/180); 
					break;
			case 2: //Micro_1_4
					StepVar->RealStepAngle_Rad  = (StepVar->BaseStepAngle_Degree*0.25/StepVar->GearRatio)*(PI/180); 
					break;
			case 3: //Micro_1_8
					StepVar->RealStepAngle_Rad  = (StepVar->BaseStepAngle_Degree*0.125/StepVar->GearRatio)*(PI/180); 
					break;
			case 4: //Micro_1_16
					StepVar->RealStepAngle_Rad  = (StepVar->BaseStepAngle_Degree*0.0625/StepVar->GearRatio)*(PI/180); 
					break;
			case 5: //Micro_1_32
					StepVar->RealStepAngle_Rad  = (StepVar->BaseStepAngle_Degree*0.03125/StepVar->GearRatio)*(PI/180); 
					break;
	}
	/*Compute real pulse per round*/
	StepVar->ppr = (2*PI)/StepVar->RealStepAngle_Rad;  
	//Control mode: mPOSITION or mSPEED
	StepVar->CtrlMode = CtrlMode;	
	StepVar->id = id;
	StepVar->iCn_cnt = 0;
	
	#if POSITION_MODE
	StepVar->Position_count_total = 0;
	StepVar->Position_count_signed = 0;
	StepVar->accel_cnt_convert_curve = 0;
	#endif
	
	StepVar->direct = -1;
}
/*******************************************************/
void Stepper_LockRotor(Stepper *StepVar)
{
	
}
/*******************************************************/
void Stepper_FreeRotor(Stepper *StepVar)
{
	
}
/*******************************************************/
void Step_Direction (Stepper *StepVar, uint8_t direct)
	{
		StepVar->direct = direct;
		if(StepVar->id == 0)// motor 1
			{
				if(StepVar->direct == 0) HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);//forward			
				else HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);//back
			}
		if(StepVar->id == 1)//  motor 2
			{
				if(StepVar->direct == 0) HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);//forward		
				else HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);//back
			}
	}
/*******************************************************/
/*Set motion profile for POSITION MODE */
void MotionProfile_Set(Stepper *StepVar, float Accel_Radps2, float Decel_Radps2, float maxRPM, float SetAngle_Deg, float QR_Distance_degree)
{
	if(SetAngle_Deg < 0) 
	{
		SetAngle_Deg = -SetAngle_Deg;
		Step_Direction(StepVar,0);
		StepVar->direction_temp = 0; // back
	}
	else
	{
		Step_Direction(StepVar,1);
		StepVar->direction_temp = 1; // forward
	}
	/*Compute total pulse per set angle*/
	StepVar->Total_cnt=(SetAngle_Deg/360)*StepVar->ppr; 
	StepVar->qr_set_cnt = (long)(SetAngle_Deg / QR_Distance_degree);
	StepVar->qr_total_cnt = (QR_Distance_degree/360)*StepVar->ppr; 
	/*Built the speed profile*/
	StepVar->Accel_Radps2 = Accel_Radps2;
	StepVar->Decel_Radps2 = Decel_Radps2;
	StepVar->SetAngle_Deg = SetAngle_Deg;
	StepVar->maxSpeed_RadpSec = maxRPM/9.549296; 	//rpm to rad/s		
	StepVar->max_s_lim=(StepVar->maxSpeed_RadpSec*StepVar->maxSpeed_RadpSec)/(2*StepVar->RealStepAngle_Rad*Accel_Radps2);
	StepVar->accel_lim=(Decel_Radps2*StepVar->Total_cnt)/(Accel_Radps2+Decel_Radps2);
	if(StepVar->max_s_lim<=StepVar->accel_lim) 																									//accel_run_deccel
	{
				StepVar->decel_set_cnt = -(float)(StepVar->max_s_lim*Accel_Radps2)/Decel_Radps2;
				StepVar->run_set_cnt =(StepVar->Total_cnt-(fabs)((double)(StepVar->decel_set_cnt))-StepVar->max_s_lim);
				StepVar->accel_set_cnt = StepVar->max_s_lim;
	}
	else //accel_deccel
	{
				StepVar->decel_set_cnt=-(StepVar->Total_cnt-StepVar->accel_lim);
				StepVar->run_set_cnt=0;
				StepVar->accel_set_cnt=StepVar->accel_lim;
	}
	StepVar->decel_cnt=StepVar->decel_set_cnt;
}
/*******************************************************/
/*Set motion profile for SPEED MODE */
void MotionProfile_Accel_Only_Set(Stepper *StepVar, float Accel_Radps2, float Decel_Radps2, float Speed_RadpSec)
{
	if(Speed_RadpSec < 0) {
		Step_Direction(StepVar,0);
		StepVar->direction_temp = 0;
	}
	else {
		Step_Direction(StepVar,1);
		StepVar->direction_temp = 1; 
	}
	
	/*Built the speed profile*/
	StepVar->Accel_Radps2 = Accel_Radps2;
	StepVar->Decel_Radps2 = Decel_Radps2;
	StepVar->maxSpeed_RadpSec = Speed_RadpSec;
	
	StepVar->max_s_lim=(StepVar->maxSpeed_RadpSec*StepVar->maxSpeed_RadpSec)/(2*StepVar->RealStepAngle_Rad*Accel_Radps2);
	StepVar->accel_set_cnt = StepVar->max_s_lim;
}
/************************************************************************/
void MotionVar_Prepare(Stepper *StepVar)
{
	/*Compute the first value which is loaded to the Auto Reload Register*/
	StepVar->Co=(f_Timer4*sqrt(2*StepVar->RealStepAngle_Rad/StepVar->Accel_Radps2)*1000);
	StepVar->Co_temp = StepVar->Co;
	StepVar->Cn = StepVar->Co;
	StepVar->iCn = (int)StepVar->Cn;
//	StepVar->run_cnt=0;
	StepVar->accel_cnt=0;
	StepVar->Cn_1=StepVar->Cn;
	StepVar->Cn_max = StepVar->Co * ( sqrt(StepVar->accel_set_cnt +1) - sqrt(StepVar->accel_set_cnt));
}
/************************************************************************/
void Stepper_StateRst(Stepper *StepVar)
{
	StepVar->move=0; 
	StepVar->cmd = 0;
}
/********************************************************************/
/*Run stepper motor with corresponding motion profile - Note: MotionProfile_ReSet() must be call first*/
void Stepper_withCurve_Start(Stepper *StepVar) 
{ 
	/*Generate pulse and switch stepper motor state to STEPPER_Run*/
	step(StepVar);
	StepVar->step_feedback = 0;
	
	#if POSITION_MODE
	StepVar->Forward_cnt = 0;
	StepVar->Position_count_total = 0;
	StepVar->Position_count_total ++;
	#endif
	
	StepVar->state = stRUN;
	StepVar->cmd = 1;
	StepVar->flag_busy = 1;
}
/***********************************************************************/
void Stepper_Stop(Stepper *StepVar)
{	
	if(StepVar->move !=0)       //Stepper motor is running
	{
			StepVar->decel_cnt = - StepVar->accel_cnt * StepVar->Accel_Radps2 / StepVar->Decel_Radps2;
			StepVar->move = -1;     //Decelerate and stop
			StepVar->cmd = 0;
	}
}
/***********************************************************************/
void Stepper_Emergency_Stop(Stepper *StepVar)
{	
	if(StepVar->move !=0)       //Stepper motor is running
	{
		StepVar->move = 0;
		StepVar->state = stLOCKROTOR;		
		StepVar->cmd = 0;
		time_end = micros();
		time_process = time_end - time_start;	
		StepVar->flag_busy = 0;
		StepVar->flag_phase_1 = 0;
		StepVar->flag_phase_2 = 0;
		StepVar->flag_calib = 0;
		StepVar->flag_calib_type1 = 0;
		StepVar->flag_calib_type2 = 0;
		StepVar->flag_accel = 0;
		StepVar->flag_accel_done = 0;
		StepVar->flag_decel = 0;
		StepVar->flag_decel_done = 0;
		StepVar->flag_tracking = 0;
		StepVar->flag_calib_accel = 0;
		StepVar->flag_calib_decel = 0;
		StepVar->flag_pause = 0; 
		StepVar->flag_pause_done = 0;
	}
}
/***********************************************************************/
void Stepper_End_Jogging(Stepper *StepVar)
{	
		StepVar->move = 0;
		StepVar->state = stLOCKROTOR;		
		StepVar->cmd = 0;
		time_end = micros();
		time_process = time_end - time_start;	
		StepVar->flag_busy = 0;
		StepVar->flag_phase_1 = 0;
		StepVar->flag_phase_2 = 0;
		StepVar->flag_calib = 0;
		StepVar->flag_calib_type1 = 0;
		StepVar->flag_calib_type2 = 0;
		StepVar->flag_tracking = 0;
		StepVar->flag_pause = 0; 
		StepVar->flag_calib_qr_curve = 0;
}
/***********************************************************************/
void Stepper_Control(Stepper *StepVar)
{  	
	step(StepVar);
	StepVar->Position_count_total ++; 
	
	if(StepVar->direct == 0) {
		StepVar->step_feedback--;
	}
	else StepVar->step_feedback++;
	
	switch(StepVar->CtrlMode)
	{
		#if POSITION_MODE
			case mPOSITION:
			{
					if(StepVar->cmd == 1)
					{
							StepVar->Forward_cnt ++;
							if(!StepVar->flag_pause)
							{
								if((StepVar->accel_cnt<StepVar->accel_set_cnt) && (StepVar->Forward_cnt<(StepVar->Total_cnt+StepVar->decel_cnt)))      //Keep accelerating
								{
									StepVar->move=2;
									StepVar->accel_cnt++;
								}
								else if((StepVar->accel_cnt > StepVar->accel_set_cnt) && (StepVar->Forward_cnt<(StepVar->Total_cnt+StepVar->decel_cnt)))
								{
									StepVar->move = -1; //Keep decelerating
									StepVar->accel_cnt--;													
								}		
								else                               //2-1-2 with V = const 
								{
									if(!StepVar->flag_tracking)
									{
										 if(StepVar->Forward_cnt<(StepVar->Total_cnt+StepVar->decel_cnt))
										 {
													StepVar->move = 1;                                	//Run with v = constant										
										 }
										 else                                                     //Decelerate
										 {
											 StepVar->move = -1;
											 StepVar->accel_cnt--;	
										 }
									 }
									else
									{
										if(StepVar->Forward_cnt<(StepVar->Total_cnt - StepVar->tracking_cnt - StepVar->tracking_cnt  + StepVar->decel_cnt))
										 {
													StepVar->move = 1;                                	//Run with v = constant										
										 }
										else
										{
											 if(StepVar->Forward_cnt<(StepVar->Total_cnt - StepVar->tracking_cnt - StepVar->tracking_cnt - 5))
											 {
												 StepVar->move = -1;
												 StepVar->accel_cnt--;	
											 }
											 else
											 {
												 if(StepVar->Forward_cnt<(StepVar->Total_cnt - 5))
												 {
													 StepVar->move = 1;                                	//Run with v = constant
												 }
												 else
												 {
													 StepVar->move = -1;
													 StepVar->accel_cnt--;	
												 }
											 }
										}
									}										 
								}									
							}
							else
							{
										StepVar->move = -1;
										//StepVar->accel_cnt--;
										StepVar->decel_cnt = 0;
							}					
					}
					else
							StepVar->move = -1;              
					break;
			}
		#endif /* POSITION MODE */
			
			/* MODE SPEED */
			case mSPEED:
			{
				if(StepVar->cmd == 1) {
					if(!StepVar->flag_pause) {
						if(StepVar->accel_cnt < StepVar->accel_set_cnt) {
							StepVar->move=2;
							StepVar->accel_cnt++;
						}
						else if(StepVar->accel_cnt > StepVar->accel_set_cnt) {
							StepVar->move = 2; 
							StepVar->accel_cnt--;													
						}		
						else {                               //2-1-2 with V = const 
							StepVar->move = 1;
						}									
					}
					else {
						StepVar->move = -1;
						StepVar->decel_cnt = 0;
					}					
				}
				else
					StepVar->move = -1;              
				break;
			}
	}
	switch(StepVar->move)
	{
				case -1: 	//Decel
				{
						 if(++StepVar->decel_cnt < 0) {
							 
									StepVar->accel_cnt--;
							 
									StepVar->Cn=StepVar->Co_temp * (sqrt(-StepVar->decel_cnt+1) -sqrt(-StepVar->decel_cnt));
									StepVar->iCn = (int) StepVar->Cn;							 		
						 }
						 else {    					   
									StepVar->move = 0;
									StepVar->state = stLOCKROTOR;
									StepVar->cmd = 0;
									if(StepVar->CtrlMode == mPOSITION)
											StepVar->PosFlag = 1; 									//Reach the set position
									time_end = micros();
									time_process = time_end - time_start;	
									StepVar->flag_tracking = 0;
									StepVar->flag_busy = 0;
									if(StepVar->flag_pause)
										{
											StepVar->flag_pause = 0;
											StepVar->flag_accel = 0;
											StepVar->flag_decel = 0;
											StepVar->flag_calib_type1 = 0;
											StepVar->flag_calib_type2 = 0;
											StepVar->flag_calib = 0;
											StepVar->flag_phase_1 = 0;
											StepVar->flag_phase_2 = 0;
											StepVar->flag_calib_accel = 0;
											StepVar->flag_calib_decel = 0;
											//StepVar->flag_pause_done = 1;
										}
										
									if(StepVar->direction_temp == 0) StepVar->temp_step = StepVar->temp_step - StepVar->Position_count_total; 
									else StepVar->temp_step = StepVar->temp_step + StepVar->Position_count_total; 
						 }
						 break;
				}
				case 1: 	//Run
				{				
					break;
				}
				case 2: 	//Accel
				{  
					 StepVar->Cn=StepVar->Co_temp * (sqrt(StepVar->accel_cnt+1) - sqrt(StepVar->accel_cnt));
					 StepVar->iCn = (int) StepVar->Cn;				
				}
	 }	 
}
/***********************************************************************/
void Stepper_Control_withCurve(Stepper *StepVar, Stepper *StepVar_temp1, Stepper *StepVar_temp2)
{  	
	step(StepVar);
	StepVar->Position_count_total ++; 
//	if(StepVar->direct == 1) StepVar->Position_count_signed++;//forward
//	else StepVar->Position_count_signed--;//back
	switch(StepVar->CtrlMode)
	{
			case mPOSITION:
			{
					if(StepVar->cmd == 1)
					{
						if(!StepVar->flag_calib)
						{
							StepVar->Forward_cnt ++;
							if(!StepVar->flag_pause)
							{
								if((StepVar->accel_cnt<StepVar->accel_set_cnt) && (StepVar->Forward_cnt<(StepVar->Total_cnt+StepVar->decel_cnt)))      //Keep accelerating
								{
									StepVar->move=2;
									StepVar->accel_cnt++;
								}
								else if((StepVar->accel_cnt > StepVar->accel_set_cnt) && (StepVar->Forward_cnt<(StepVar->Total_cnt+StepVar->decel_cnt)))
								{
									StepVar->move = -1; //Keep decelerating
									StepVar->accel_cnt--;													
								}		
								else                               //2-1-2 with V = const 
								{
									if(!StepVar->flag_tracking)
									{
										 if(StepVar->Forward_cnt<(StepVar->Total_cnt+StepVar->decel_cnt))
										 {
													StepVar->move = 1;                                	//Run with v = constant										
										 }
										 else                                                     //Decelerate
										 {
											 StepVar->move = -1;
											 StepVar->accel_cnt--;	
										 }
									 }
									else
									{
										if(StepVar->Forward_cnt<(StepVar->Total_cnt - StepVar->tracking_cnt - StepVar->tracking_cnt  + StepVar->decel_cnt))
										 {
													StepVar->move = 1;                                	//Run with v = constant										
										 }
										else
										{
											 if(StepVar->Forward_cnt<(StepVar->Total_cnt - StepVar->tracking_cnt - StepVar->tracking_cnt - 5))
											 {
												 StepVar->move = -1;
												 StepVar->accel_cnt--;	
											 }
											 else
											 {
												 if(StepVar->Forward_cnt<(StepVar->Total_cnt - 5))
												 {
													 StepVar->move = 1;                                	//Run with v = constant
												 }
												 else
												 {
													 StepVar->move = -1;
													 StepVar->accel_cnt--;	
												 }
											 }
										}
									}										 
								}	
								if(StepVar->flag_calib_type1)
								{
									StepVar->flag_calib_type1 = 0;
									StepVar->flag_calib = 1;
									StepVar->flag_phase_1 = 0;
									StepVar->flag_phase_2 = 1;
									StepVar->Co_temp = StepVar_temp2->Co;
									StepVar->accel_cnt = StepVar->accel_cnt * StepVar->speed_ratio_2;
									StepVar->decel_cnt = StepVar->decel_cnt * StepVar->speed_ratio_2;
									StepVar->Cn=StepVar->Co_temp * (sqrt(StepVar->accel_cnt+1) -sqrt(StepVar->accel_cnt));
									StepVar->iCn = (int) StepVar->Cn;
									StepVar->move = 1;
									StepVar->decel_cnt_temp = StepVar->decel_cnt;
									StepVar->fForward_cnt = StepVar->Forward_cnt;
								}
								if(StepVar->flag_calib_type2)
								{
									StepVar->flag_calib_type2 = 0;
									StepVar->flag_calib = 1;
									StepVar->flag_phase_1 = 1;
									StepVar->Co_temp = StepVar_temp1->Co;
									StepVar->accel_cnt = StepVar->accel_cnt * StepVar->speed_ratio_1;
									StepVar->decel_cnt = StepVar->decel_cnt * StepVar->speed_ratio_1;
									StepVar->Cn=StepVar->Co_temp * (sqrt(StepVar->accel_cnt+1) -sqrt(StepVar->accel_cnt));
									StepVar->iCn = (int) StepVar->Cn;
									StepVar->move = 1;  
									StepVar->decel_cnt_temp = StepVar->decel_cnt;
									StepVar->fForward_cnt = StepVar->Forward_cnt;
								}
							}
							else
							{
										StepVar->move = -1;
										//StepVar->accel_cnt--;
										StepVar->decel_cnt = 0;
							}
						}
						else
						{
							StepVar->Curve_cnt++;
							StepVar->fForward_cnt += StepVar->fForward_ratio;
								if(StepVar->flag_phase_1)
									{
//										if(StepVar->fForward_cnt<(StepVar->Total_cnt - StepVar->total_curve_equipvalent + StepVar->decel_cnt_temp))
										if(StepVar->fForward_cnt<(StepVar->Total_cnt + StepVar->decel_cnt_temp))
										{
											 if((StepVar->accel_cnt < StepVar_temp1->accel_set_cnt) && (StepVar->Curve_cnt < StepVar_temp1->Total_cnt))
											 {
													StepVar->move = 2; //Keep accelerating
													StepVar->accel_cnt++;
											 }
											 else if ((StepVar->accel_cnt > StepVar_temp1->accel_set_cnt) && (StepVar->Curve_cnt < StepVar_temp1->Total_cnt))
											 {
													StepVar->move = -1; //Keep decelerating
													StepVar->accel_cnt--;
											 }
											 else
											 {
												 if(StepVar->Curve_cnt < StepVar_temp1->Total_cnt)           //Check if finishing curve 1 
												 {
														StepVar->move = 1;                                	//Run with v = constant									
												 }
												 else                                                     //Decelerate
												 {
													 StepVar->flag_phase_1 = 0;
													 StepVar->accel_cnt = StepVar->accel_cnt * StepVar->speed_ratio_2;
													 StepVar->decel_cnt = StepVar->decel_cnt * StepVar->speed_ratio_2;
													 StepVar->Co_temp = StepVar_temp2->Co;
													 StepVar->Cn=StepVar->Co_temp * (sqrt(StepVar->accel_cnt+1) -sqrt(StepVar->accel_cnt));
													 StepVar->iCn = (int) StepVar->Cn;
													 StepVar->flag_phase_2 = 1;
													 StepVar->move = 1;
												}
											 }
										 }
										 else
										 {
											 if(StepVar->Curve_cnt < StepVar_temp1->Total_cnt)           //Check if finishing curve 1 
											 {
													StepVar->move = -1;                                	//Decelerate
													StepVar->accel_cnt--;
											 }
											 else                                                    
											 {
												 StepVar->flag_phase_1 = 0;
												 StepVar->accel_cnt = StepVar->accel_cnt * StepVar->speed_ratio_2;
												 StepVar->decel_cnt = StepVar->decel_cnt * StepVar->speed_ratio_2;
												 StepVar->Co_temp = StepVar_temp2->Co;
												 StepVar->Cn=StepVar->Co_temp * (sqrt(StepVar->accel_cnt+1) -sqrt(StepVar->accel_cnt));
												 StepVar->iCn = (int) StepVar->Cn;
												 StepVar->flag_phase_2 = 1;
												 StepVar->move = 1;
											}
										 }
									}
								if(StepVar->flag_phase_2)
									{
//										if(StepVar->fForward_cnt<(StepVar->Total_cnt - StepVar->total_curve_equipvalent + StepVar->decel_cnt_temp))
										if(StepVar->fForward_cnt<(StepVar->Total_cnt + StepVar->decel_cnt_temp))	
										{
											if((StepVar->accel_cnt < StepVar_temp2->accel_set_cnt) && (StepVar->Curve_cnt < (StepVar_temp1->Total_cnt+StepVar_temp2->Total_cnt)))
											{
												StepVar->move = 2; //Keep accelerating
												StepVar->accel_cnt++;												
											}
											else if((StepVar->accel_cnt > StepVar_temp2->accel_set_cnt) && (StepVar->Curve_cnt < (StepVar_temp1->Total_cnt+StepVar_temp2->Total_cnt)))  
											{
												StepVar->move = -1; //Keep decelerating
												StepVar->accel_cnt--;												
											}
											else 
											{
												if(StepVar->Curve_cnt < (StepVar_temp1->Total_cnt+StepVar_temp2->Total_cnt))
												{
													StepVar->move = 1;                                	//Run with v = constant										
												}
												else
												{
													StepVar->flag_phase_2 = 0;
													StepVar->flag_calib = 0;
													StepVar->Curve_cnt = 0;
													//StepVar->Forward_cnt += StepVar->total_curve_equipvalent;//////////////////////////
													StepVar->Forward_cnt = (long)StepVar->fForward_cnt;
													StepVar->accel_cnt = StepVar->accel_cnt * StepVar->speed_ratio;
													StepVar->decel_cnt = StepVar->decel_cnt * StepVar->speed_ratio;
													StepVar->Co_temp = StepVar->Co;
													StepVar->Cn=StepVar->Co_temp * (sqrt(StepVar->accel_cnt+1) -sqrt(StepVar->accel_cnt));
													StepVar->iCn = (int) StepVar->Cn;
													StepVar->move = 1;
													//StepVar->flag_res = 1;
												}
											}
										}
										else
										{
											if(StepVar->Curve_cnt < (StepVar_temp1->Total_cnt+StepVar_temp2->Total_cnt))           //Check if finishing curve 2 
											 {
													StepVar->move = -1;                                	//Decelerate			
												  StepVar->accel_cnt--;	
											 }
											 else                                                    
											 {
												 StepVar->flag_phase_2 = 0;
													StepVar->flag_calib = 0;
													StepVar->Curve_cnt = 0;
												  StepVar->Forward_cnt = (long)StepVar->fForward_cnt;
													StepVar->accel_cnt = StepVar->accel_cnt * StepVar->speed_ratio;
													StepVar->decel_cnt = StepVar->decel_cnt * StepVar->speed_ratio;
													StepVar->Co_temp = StepVar->Co;
													StepVar->Cn=StepVar->Co_temp * (sqrt(StepVar->accel_cnt+1) -sqrt(StepVar->accel_cnt));
													StepVar->iCn = (int) StepVar->Cn;
													StepVar->move = 1;
													//StepVar->flag_res = 1;
											}
										}
							}
						}
					}
					else
							StepVar->move = -1;              
					break;
			}
	}
	switch(StepVar->move)
	{
				case -1: 	//Decel
				{
						 if(++StepVar->decel_cnt< 0)
						 {
									StepVar->Cn=StepVar->Co_temp * (sqrt(-StepVar->decel_cnt+1) -sqrt(-StepVar->decel_cnt));
									StepVar->iCn = (int) StepVar->Cn;///////////////////////////////							 		
						 }
						 else
						 {    					   
									StepVar->move = 0;
									StepVar->state = stLOCKROTOR;		
									//Stepper_LockRotor(mID);
									StepVar->cmd = 0;
									if(StepVar->CtrlMode == mPOSITION)
											StepVar->PosFlag = 1; 									//Reach the set position
									time_end = micros();
									time_process = time_end - time_start;	
									StepVar->flag_tracking = 0;
									StepVar->flag_busy = 0;
									if(StepVar->flag_pause)
										{
											StepVar->flag_pause = 0;
//											StepVar->Forward_cnt_temp = StepVar->Forward_cnt;
//											StepVar->Total_cnt_temp = StepVar->Total_cnt;
											StepVar->flag_accel = 0;
											StepVar->flag_decel = 0;
											StepVar->flag_calib_type1 = 0;
											StepVar->flag_calib_type2 = 0;
											StepVar->flag_calib = 0;
											StepVar->flag_phase_1 = 0;
											StepVar->flag_phase_2 = 0;
											StepVar->flag_calib_accel = 0;
											StepVar->flag_calib_decel = 0;
											//StepVar->flag_pause_done = 1;
										}
									if(StepVar->flag_calib_qr_curve == 1) StepVar->flag_calib_qr_curve = 2;
									else StepVar->flag_calib_qr_curve = 0;
										
									if(StepVar->direction_temp == 0) StepVar->temp_step = StepVar->temp_step - StepVar->Position_count_total; 
									else StepVar->temp_step = StepVar->temp_step + StepVar->Position_count_total; 
						 }
						 break;
				}
				case 1: 	//Run
				{				
					break;
				}
				case 2: 	//Accel
				{  
					 StepVar->Cn=StepVar->Co_temp * (sqrt(StepVar->accel_cnt+1) -sqrt(StepVar->accel_cnt));
					 StepVar->iCn = (int) StepVar->Cn;				
				}
	 }	 
}
/***********************************************************************/
	void Stepper_Reset(Stepper *StepVar)
	{
		StepVar->move = 0;
		StepVar->state = stLOCKROTOR;		
		StepVar->cmd = 0;
		StepVar->flag_busy = 0;
		StepVar->flag_phase_1 = 0;
		StepVar->flag_phase_2 = 0;
		StepVar->flag_calib = 0;
		StepVar->flag_calib_type1 = 0;
		StepVar->flag_calib_type2 = 0;
		StepVar->flag_accel = 0;
		StepVar->flag_accel_done = 0;
		StepVar->flag_decel = 0;
		StepVar->flag_decel_done = 0;
		StepVar->flag_tracking = 0;
		StepVar->flag_calib_accel = 0;
		StepVar->flag_calib_decel = 0;
		StepVar->flag_pause = 0; 
		StepVar->flag_pause_done = 0;
		StepVar->Forward_cnt_temp = 0;
		StepVar->Total_cnt_temp = 0;
	}
/***********************************************************************/	