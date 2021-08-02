void YAW_CONTROLLER(){

// ******************************************8 YAW_ANGULAR_SPEED_PID *************************************

YAW_W_ERROR = YAW_SET_W  - YAW_FEEDBACK_W;

YAW_PID_W = PID( YAW_W_ERROR, YAW_W_PREV_ERROR, &YAW_W_I, YAW_W_Kp, YAW_W_Ki, YAW_W_Kd,YAW_SAT_PID_W,YAW_PID_W,TIMER_YAW);

YAW_W_PREV_ERROR = YAW_W_ERROR;

YAW_SAT_PID_W = SATURATE(YAW_PID_W,-SAT_PID_VALUE,SAT_PID_VALUE);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// ******************************************8 YAW_POSITION_PID *************************************

YAW_T_ERROR = (YAW_SET_T  - YAW_FEEDBACK_T)*0.1 - YAW_FEEDBACK_I;

YAW_PID_T = PID( YAW_T_ERROR,YAW_T_PREV_ERROR, &YAW_T_I, YAW_T_Kp, YAW_T_Ki, YAW_T_Kd,YAW_SAT_PID_T,YAW_PID_T,TIMER_YAW);

YAW_T_PREV_ERROR = YAW_T_ERROR;

YAW_SAT_PID_T = SATURATE(YAW_PID_T,-SAT_PID_VALUE,SAT_PID_VALUE);
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

TIMER_YAW = micros();

if(SET_PID) YAW_SAT_PID = YAW_SAT_PID_W*0.6  + YAW_SAT_PID_T*0.4 ;
else        YAW_SAT_PID = YAW_SAT_PID_T ;

}
