void JOINT_4_CONTROLLER(){

// ******************************************8 JOINT_4_CURRENT-BASED_POSITION_PID *************************************

JOINT_4_T_ERROR = (JOINT_4_SET_T  - JOINT_4_FEEDBACK_T)*0.1 - JOINT_4_FEEDBACK_I;      // DETERMING THE ERROR IN CURRENT 

JOINT_4_PID = PID(JOINT_4_T_ERROR,JOINT_4_T_PREV_ERROR,&JOINT_4_T_I,JOINT_4_T_Kp,JOINT_4_T_Ki,JOINT_4_T_Kd,JOINT_4_SAT_PID,JOINT_4_PID,TIMER_4);

JOINT_4_T_PREV_ERROR = JOINT_4_T_ERROR;

JOINT_4_SAT_PID = SATURATE(JOINT_4_PID,-SAT_PID_VALUE,SAT_PID_VALUE);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

TIMER_4 = micros();


}
