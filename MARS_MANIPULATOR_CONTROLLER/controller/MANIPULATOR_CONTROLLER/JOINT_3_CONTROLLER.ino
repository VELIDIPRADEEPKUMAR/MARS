void JOINT_3_CONTROLLER(){


// ******************************************8 JOINT_3_POSITION_PID *************************************

JOINT_3_T_ERROR = (JOINT_3_SET_T  - JOINT_3_FEEDBACK_T)*0.1 - JOINT_3_FEEDBACK_I;

JOINT_3_PID = PID( JOINT_3_T_ERROR, JOINT_3_T_PREV_ERROR, &JOINT_3_T_I, JOINT_3_T_Kp, JOINT_3_T_Ki, JOINT_3_T_Kd,JOINT_3_SAT_PID,JOINT_3_PID,TIMER_3);

JOINT_3_T_PREV_ERROR = JOINT_3_T_ERROR;

JOINT_3_SAT_PID = SATURATE(JOINT_3_PID,-SAT_PID_VALUE,SAT_PID_VALUE);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

TIMER_3 = micros();
