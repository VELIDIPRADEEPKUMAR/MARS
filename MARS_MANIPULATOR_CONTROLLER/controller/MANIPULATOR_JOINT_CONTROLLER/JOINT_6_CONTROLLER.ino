void JOINT_6_CONTROLLER(){

// ******************************************8 JOINT_6_ANGULAR_SPEED_PID *************************************

JOINT_6_W_ERROR = JOINT_6_SET_W  - JOINT_6_FEEDBACK_W;

JOINT_6_PID_W = PID( JOINT_6_W_ERROR, JOINT_6_W_PREV_ERROR, &JOINT_6_W_I, JOINT_6_W_Kp, JOINT_6_W_Ki, JOINT_6_W_Kd,JOINT_6_SAT_PID_W,JOINT_6_PID_W,TIMER_6);

JOINT_6_W_PREV_ERROR = JOINT_6_W_ERROR;

JOINT_6_SAT_PID_W = SATURATE(JOINT_6_PID_W,-SAT_PID_VALUE,SAT_PID_VALUE);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// ******************************************8 JOINT_6_POSITION_PID *************************************

JOINT_6_T_ERROR = (JOINT_6_SET_T  - JOINT_6_FEEDBACK_T)*0.1 - JOINT_6_FEEDBACK_I;

JOINT_6_PID_T = PID( JOINT_6_T_ERROR, JOINT_6_T_PREV_ERROR, &JOINT_6_T_I, JOINT_6_T_Kp, JOINT_6_T_Ki, JOINT_6_T_Kd,JOINT_6_SAT_PID_T,JOINT_6_PID_T,TIMER_6);

JOINT_6_T_PREV_ERROR = JOINT_6_T_ERROR;

JOINT_6_SAT_PID_T = SATURATE(JOINT_6_PID_T,-SAT_PID_VALUE,SAT_PID_VALUE);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

TIMER_6 = micros();

if(SET_PID) JOINT_6_SAT_PID = JOINT_6_SAT_PID_W*0.6  + JOINT_6_SAT_PID_T*0.4 ;    //  tune the gains to the pid_w & pid_t to get best output  
else        JOINT_6_SAT_PID = JOINT_6_SAT_PID_T ;

}
