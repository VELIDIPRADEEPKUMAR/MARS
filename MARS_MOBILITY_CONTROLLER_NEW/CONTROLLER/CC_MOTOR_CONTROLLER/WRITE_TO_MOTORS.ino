

void WRITE_TO_MOTORS(){

///////// ********** LEFT MOTOR WRITE ************//////////////

if(SAT_L_PID>0) {
  digitalWrite(LEFT_MOTOR_INT1,HIGH);      //  if the speed is positive we have to run in clock wise direction ...
  digitalWrite(LEFT_MOTOR_INT2,LOW);
}
else if(SAT_L_PID<0) {
  digitalWrite(LEFT_MOTOR_INT1,LOW);      //  if the speed is negative we have to run in anticlock wise direction ...
  digitalWrite(LEFT_MOTOR_INT2,HIGH);
}

analogWrite(LEFT_MOTOR,map(abs(SAT_L_PID),0,SAT_PID_VALUE,0,65535));   //     writing th pwm to the motor1 from the absolute value of the pid output ...

///////// ********** RIGHT MOTOR WRITE ************//////////////

if(SAT_R_PID>0) {                              
  digitalWrite(RIGHT_MOTOR_INT1,HIGH);
  digitalWrite(RIGHT_MOTOR_INT2,LOW);
}
else if(SAT_R_PID<0) {
  digitalWrite(RIGHT_MOTOR_INT1,LOW);
  digitalWrite(RIGHT_MOTOR_INT2,HIGH);
}

analogWrite(RIGHT_MOTOR,map(abs(SAT_R_PID),0,SAT_PID_VALUE,0,65535));


}


////////////////////////////////////////////

void PRINTDATA(void){
#if 1

//////////// I2C DATA ////////////////////

Serial.println(" I2C DATA FROM MASTER");
Serial.print(" VEL = ");
Serial.println(R_M_VEL);
Serial.print(" STEER = ");
Serial.println(L_M_VEL);
Serial.print("FEEDBACK_STEER = ");
Serial.println(FEEDBACK_STEER);

#endif
  
}
