
void GRIPPER_CONTROLLER(){


// ---------------------------------------- FINDING GRIPPER MOTOR TORQUE ---------------------------------------
GRIPPER_TORQUE = GRIPPER_FEEDBACK_I*Kt;       // MOTOR TORQUE 
//---------------------------------------------------------------------------------------------------------------


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRIPPER TORQUE CONTROLLER  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

GRIPPER_ERROR = GRIPPER_SET  - GRIPPER_TORQUE;    // FINDING ERROR 

GRIPPER_PID = PID(GRIPPER_ERROR, GRIPPER_PREV_ERROR, &GRIPPER_I, GRIPPER_Kp, GRIPPER_Ki, GRIPPER_Kd, GRIPPER_SAT_PID, GRIPPER_PID, G_T);  // CALICULATING PID 

GRIPPER_PREV_ERROR = GRIPPER_ERROR;     

GRIPPER_SAT_PID  = SATURATE(GRIPPER_PID,-SAT_PID_VALUE,SAT_PID_VALUE);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

G_T = micros();


}
