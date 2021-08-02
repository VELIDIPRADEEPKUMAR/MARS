

//////////////  I2C REQUIST FORM ////////////////

void requestEvent(void){

////// writing data to master ///////////////
byte I2C_DATA_TO_MASTER[16];


I2C_DATA_TO_MASTER[0]  =  int32_t(F1) & 0xFF;       //  WRITING F1 DATA 
I2C_DATA_TO_MASTER[1]  = (int32_t(F1)>>8) & 0xFF;
I2C_DATA_TO_MASTER[2]  = (int32_t(F1)>>16) & 0xFF;
I2C_DATA_TO_MASTER[3]  = (int32_t(F1)>>24) & 0xFF;

I2C_DATA_TO_MASTER[4]  =  int32_t(F2) & 0xFF;       //  WRITING F2 DATA 
I2C_DATA_TO_MASTER[5]  = (int32_t(F2)>>8) & 0xFF;
I2C_DATA_TO_MASTER[6]  = (int32_t(F2)>>16) & 0xFF;
I2C_DATA_TO_MASTER[7]  = (int32_t(F2)>>24) & 0xFF;

I2C_DATA_TO_MASTER[8]  =  E1 & 0xFF;       //  WRITING E1 DATA 
I2C_DATA_TO_MASTER[9]  = (E1>>8) & 0xFF;
I2C_DATA_TO_MASTER[10] = (E1>>16) & 0xFF;
I2C_DATA_TO_MASTER[11] = (E1>>24) & 0xFF;

I2C_DATA_TO_MASTER[12] =  E2 & 0xFF;       //  WRITING E2 DATA 
I2C_DATA_TO_MASTER[13] = (E2>>8) & 0xFF;
I2C_DATA_TO_MASTER[14] = (E2>>16) & 0xFF;
I2C_DATA_TO_MASTER[15] = (E2>>24) & 0xFF;

Wire.write(I2C_DATA_TO_MASTER,16);       // WRITE DATA TO MASTER 
  
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
