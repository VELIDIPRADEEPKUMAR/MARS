

//////////////  I2C REQUIST FORM ////////////////

void RequestEvent(void){

////// writing data to master ///////////////
byte OUTPUTDATA[6];

OUTPUTDATA[0] =  uint16_t(JOINT_3_FEEDBACK_T*100) & 0xFF;            // WRITING JOINT 4 ANGLE 
OUTPUTDATA[1] = (uint16_t(JOINT_3_FEEDBACK_T*100) >> 8) & 0xFF;

OUTPUTDATA[2] =  uint16_t(YAW_FEEDBACK_T*100) & 0xFF;            // WRITING JOINT 5 ANGLE 
OUTPUTDATA[3] = (uint16_t(YAW_FEEDBACK_T*100) >> 8) & 0xFF;

OUTPUTDATA[4] =  uint16_t(GRIPPER_FEEDBACK_T*100) & 0xFF;            // WRITING JOINT 5 ANGLE 
OUTPUTDATA[5] = (uint16_t(GRIPPER_FEEDBACK_T*100) >> 8) & 0xFF;


Wire.write(OUTPUTDATA,6);       // WRITE DATA TO MASTER 
  
}

////// //////// I2C RECIEVE FORM //////////////////// 

void RECIEVESPEED(int MARS) {  // to work I2C RECIEVESPEED WE HAVE TO PROVIDE A INPUT , BUT THE INPUT IS NOT USEFUL ;

for(uint8_t i = 0; i<6 ;i++){

  if(Wire.available()){
    INPUTDATA[i] = Wire.read();
  }
  else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < 1000) && !Wire.available());
      if (Wire.available())
        INPUTDATA[i] = Wire.read();
      else {
        Serial.println("i2cRead timeout");
      }  
    }
}
  
}
