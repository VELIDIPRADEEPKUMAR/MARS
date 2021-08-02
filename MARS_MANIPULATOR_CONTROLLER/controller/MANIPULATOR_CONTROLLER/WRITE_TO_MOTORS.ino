

void WRITE_TO_MOTORS(){

/////////////////////////////////////////// JOINT_3_MOTOR  ////////////////////////////////////////////////////////

if(JOINT_3_SAT_PID>0) {
  digitalWrite(J_3_INT1,HIGH);      //  if the speed is positive we have to run in clock wise direction ...
  digitalWrite(J_3_INT2,LOW);
}
else if(JOINT_3_SAT_PID<0) {
  digitalWrite(J_3_INT1,LOW);      //  if the speed is negative we have to run in anticlock wise direction ...
  digitalWrite(J_3_INT2,HIGH);
}

analogWrite(M1,map(abs(JOINT_3_SAT_PID),0,SAT_PID_VALUE,0,65535));   //  writing th pwm to the motor1 from the absolute value of the pid output ...

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////// YAW_MOTOR ////////////////////////////

if(YAW_SAT_PID>0) {                              
  digitalWrite(YAW_INT1,HIGH);
  digitalWrite(YAW_INT2,LOW);
}
else if(YAW_SAT_PID<0) {
  digitalWrite(YAW_INT1,LOW);
  digitalWrite(YAW_INT2,HIGH);
}

analogWrite(M2,map(abs(YAW_SAT_PID),0,SAT_PID_VALUE,0,65535));

///////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////// GRIPPER_MOTOR /////////////////////////////////////

if(GRIPPER_SAT_PID>0) {                              
  digitalWrite(GRIPPER_INT1,HIGH);
  digitalWrite(GRIPPER_INT2,LOW);
}
else if(GRIPPER_SAT_PID<0) {
  digitalWrite(GRIPPER_INT1,LOW);
  digitalWrite(GRIPPER_INT2,HIGH);
}

analogWrite(M3,map(abs(GRIPPER_SAT_PID),0,SAT_PID_VALUE,0,65535));

/////////////////////////////////////////////////////////////////////////////////////

}
