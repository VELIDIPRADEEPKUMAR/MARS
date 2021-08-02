void JOINT_5_CONTROLLER(){


// ******************************************8 JOINT_5_POSITION_PID *************************************

JOINT_5_T_ERROR = (JOINT_5_SET_T  - JOINT_5_FEEDBACK_T)*0.1 - JOINT_5_FEEDBACK_I;

JOINT_5_PID = PID( JOINT_5_T_ERROR, JOINT_5_T_PREV_ERROR, &JOINT_5_T_I, JOINT_5_T_Kp, JOINT_5_T_Ki, JOINT_5_T_Kd,JOINT_5_SAT_PID,JOINT_5_PID,TIMER_5);

JOINT_5_T_PREV_ERROR = JOINT_5_T_ERROR;

JOINT_5_SAT_PID = SATURATE(JOINT_5_PID,-SAT_PID_VALUE,SAT_PID_VALUE);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

TIMER_5 = micros();


}
