//////////// I2C FUNCTIONS /////////////////////////

//////////// I2C DATA WRITE FUNCTIONS /////////////

void I2C_WRITE_FRONT_CCC(){

byte I2C_DATA_TO_STM[6];

I2C_DATA_TO_STM[0] =  F_L_M & 0xFF;
I2C_DATA_TO_STM[1] = (F_L_M >> 8) & 0xFF;
I2C_DATA_TO_STM[2] = F_R_M & 0xFF;
I2C_DATA_TO_STM[3] = (F_R_M >> 8) & 0xFF;
I2C_DATA_TO_STM[4] = YAW_RATE & 0xFF;
I2C_DATA_TO_STM[5] = (YAW_RATE >> 8) & 0xFF;

  Wire.beginTransmission(101);
  Wire.write(I2C_DATA_TO_STM,6);  
  Wire.endTransmission();   
}

void I2C_WRITE_MIDLE_CCC(){

byte I2C_DATA_TO_STM[6];

I2C_DATA_TO_STM[0] = M_L_M & 0xFF;
I2C_DATA_TO_STM[1] = (M_L_M >> 8) & 0xFF;
I2C_DATA_TO_STM[2] = M_R_M & 0xFF;
I2C_DATA_TO_STM[3] = (M_R_M >> 8) & 0xFF;
I2C_DATA_TO_STM[4] = YAW_RATE & 0xFF;
I2C_DATA_TO_STM[5] = (YAW_RATE >> 8) & 0xFF;

  Wire.beginTransmission(102);
  Wire.write(I2C_DATA_TO_STM,6);  
  Wire.endTransmission();     
}

void I2C_WRITE_BACK_CCC(){

byte I2C_DATA_TO_STM[6];

I2C_DATA_TO_STM[0] = B_L_M & 0xFF;
I2C_DATA_TO_STM[1] = (B_L_M >> 8) & 0xFF;
I2C_DATA_TO_STM[2] = B_R_M & 0xFF;
I2C_DATA_TO_STM[3] = (B_R_M >> 8) & 0xFF;
I2C_DATA_TO_STM[4] = YAW_RATE & 0xFF;
I2C_DATA_TO_STM[5] = (YAW_RATE >> 8) & 0xFF;

  Wire.beginTransmission(103);
  Wire.write(I2C_DATA_TO_STM,6);  
  Wire.endTransmission();     
}

///////////////////////////////////////////////////////////

///////////////// I2C DATA READ FUNCTIONS /////////////////

void I2C_READ_FRONT_CCC(){

Wire.requestFrom(101,16);

for(uint8_t i = 0; i<16 ;i++){

  if(Wire.available()){
    I2C_DATA_FCCC[i] = Wire.read();
  }
  else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < 1000) && !Wire.available());
      if (Wire.available())
        I2C_DATA_FCCC[i] = Wire.read();
      else {
        Serial.println("i2cRead timeout");
      }
    }
}
  
}
  


void I2C_READ_MIDLE_CCC(){

Wire.requestFrom(102,16);

for(uint8_t i = 0; i<16 ;i++){

  if(Wire.available()){
    I2C_DATA_MCCC[i] = Wire.read();
  }
  else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < 1000) && !Wire.available());
      if (Wire.available())
        I2C_DATA_MCCC[i] = Wire.read();
      else {
        Serial.println("i2cRead timeout");
      }
    }
}
  
}
  


void I2C_READ_BACK_CCC(){

Wire.requestFrom(103,16);

for(uint8_t i = 0; i<16 ;i++){

  if(Wire.available()){
    I2C_DATA_BCCC[i] = Wire.read();
  }
  else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < 1000) && !Wire.available());
      if (Wire.available())
        I2C_DATA_BCCC[i] = Wire.read();
      else {
        Serial.println("i2cRead timeout");
      }
    }
}
  
}
  
