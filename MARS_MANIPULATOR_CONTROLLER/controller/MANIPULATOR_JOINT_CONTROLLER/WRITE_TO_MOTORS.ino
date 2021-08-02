

void WRITE_TO_MOTORS(){

///////// ********** JOINT_4_MOTOR ************//////////////

if(JOINT_4_SAT_PID>0) {
  digitalWrite(J_4_INT1,HIGH);      //  if the speed is positive we have to run in clock wise direction ...
  digitalWrite(J_4_INT2,LOW);
}
else if(JOINT_4_SAT_PID<0) {
  digitalWrite(J_4_INT1,LOW);      //  if the speed is negative we have to run in anticlock wise direction ...
  digitalWrite(J_4_INT2,HIGH);
}

analogWrite(M1,map(abs(JOINT_4_SAT_PID),0,SAT_PID_VALUE,0,65535));   //     writing th pwm to the motor1 from the absolute value of the pid output ...

///////// ********** JOINT_5_MOTOR ************//////////////

if(JOINT_5_SAT_PID>0) {                              
  digitalWrite(J_5_INT1,HIGH);
  digitalWrite(J_5_INT2,LOW);
}
else if(JOINT_5_SAT_PID<0) {
  digitalWrite(J_5_INT1,LOW);
  digitalWrite(J_5_INT2,HIGH);
}

analogWrite(M2,map(abs(JOINT_5_SAT_PID),0,SAT_PID_VALUE,0,65535));

///////// ********** JOINT_6_MOTOR ************//////////////

if(JOINT_6_SAT_PID>0) {                              
  digitalWrite(J_6_INT1,HIGH);
  digitalWrite(J_6_INT2,LOW);
}
else if(JOINT_6_SAT_PID<0) {
  digitalWrite(J_6_INT1,LOW);
  digitalWrite(J_6_INT2,HIGH);
}

analogWrite(M3,map(abs(JOINT_6_SAT_PID),0,SAT_PID_VALUE,0,65535));


}


////////////////////////////////////////////
//
//void PRINTDATA(void){
//#if 1
//
////////////// I2C DATA ////////////////////
//
//Serial.println(" I2C DATA FROM MASTER");
//Serial.print(" VEL = ");
//Serial.println(R_M_VEL);
//Serial.print(" STEER = ");
//Serial.println(L_M_VEL);
//Serial.print("FEEDBACK_STEER = ");
//Serial.println(FEEDBACK_STEER);
//
//#endif
//  
//}
