//////////// I2C FUNCTIONS /////////////////////////

//////////// I2C DATA WRITE FUNCTIONS /////////////

void I2C_WRITE(byte ADDR, int16_t D1,int16_t D2,int16_t D3){

byte I2C_DATA_TO_STM[6];

I2C_DATA_TO_STM[0] =  D1 & 0xFF;
I2C_DATA_TO_STM[1] = (D1 >> 8) & 0xFF;
I2C_DATA_TO_STM[2] =  D2 & 0xFF;
I2C_DATA_TO_STM[3] = (D2 >> 8) & 0xFF;
I2C_DATA_TO_STM[4] =  D3 & 0xFF;
I2C_DATA_TO_STM[5] = (D3 >> 8) & 0xFF;

  Wire.beginTransmission(ADDR);
  Wire.write(I2C_DATA_TO_STM,6);  
  Wire.endTransmission();   
}

///////////////////////////////////////////////////////////

///////////////// I2C DATA READ FUNCTIONS /////////////////

void I2C_READ(byte ADDR,byte DATA){

Wire.requestFrom(ADDR);

for(uint8_t i = 0; i<6 ;i++){

  if(Wire.available()){
    DATA[i] = Wire.read();
  }
  else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < 1000) && !Wire.available());
      if (Wire.available())
        DATA[i] = Wire.read();
      else {
        Serial.println("i2cRead timeout");
      }
    }
}
  
}
  
  
