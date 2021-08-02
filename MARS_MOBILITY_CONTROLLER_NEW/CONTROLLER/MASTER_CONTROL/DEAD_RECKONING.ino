void DEAD_RECKONING(){
  
//////////// READING THE DATA FROM SLAVES VIA I2C ////////////////////////////////

/////////////////// CALLED THE I2C FUNCTION ///////////////////////////////////////
I2C_READ_FRONT_CCC();
I2C_READ_MIDLE_CCC();
I2C_READ_BACK_CCC();

////////////////// PREVIOUS ENCODER DATA ////////////////////////////////////////
P_E1 = E1;
P_E2 = E2;
P_E3 = E3;
P_E4 = E4;
P_E5 = E5;
P_E6 = E6;


//////////////// READ ENCODER DATA FROM I2C  ///////////////////////////////////////////////
//----------------------------------------------------------------------------------------------------------------
E1 = ((I2C_DATA_FCCC[11]<<24)|(I2C_DATA_FCCC[10]<<16)|(I2C_DATA_FCCC[9]<<8) |(I2C_DATA_FCCC[8])); // READING ENCODER DATA OF THE 
E2 = ((I2C_DATA_FCCC[15]<<24)|(I2C_DATA_FCCC[14]<<16)|(I2C_DATA_FCCC[13]<<8)|(I2C_DATA_FCCC[12]));               // FRONT WHEELS
//----------------------------------------------------------------------------------------------------------------
E3 = ((I2C_DATA_MCCC[11]<<24)|(I2C_DATA_MCCC[10]<<16)|(I2C_DATA_MCCC[9]<<8) |(I2C_DATA_MCCC[8])); // READING ENCODER DATA OF THE
E4 = ((I2C_DATA_MCCC[15]<<24)|(I2C_DATA_MCCC[14]<<16)|(I2C_DATA_MCCC[13]<<8)|(I2C_DATA_MCCC[12]));              // MIDDLE WHEELS
//----------------------------------------------------------------------------------------------------------------
E5 = ((I2C_DATA_BCCC[11]<<24)|(I2C_DATA_BCCC[10]<<16)|(I2C_DATA_BCCC[9]<<8) |(I2C_DATA_BCCC[8])); // READING ENCODER DATA OF THE
E6 = ((I2C_DATA_BCCC[15]<<24)|(I2C_DATA_BCCC[14]<<16)|(I2C_DATA_BCCC[13]<<8)|(I2C_DATA_BCCC[12]));                // BACK WHEELS
//----------------------------------------------------------------------------------------------------------------


///////////////// DETERMINING CHANGE IN ENCODER READING ///////////////////////
if(E1 - P_E1 < -5000) DELTA1 = (4294967295 - P_E1 + E1);      // when owerflow occure  ((2^32) - 1) = 4294967295 ; 
else DELTA1 = E1 - P_E1;
if(E2 - P_E2 < -5000) DELTA2 = (4294967295 - P_E2 + E2);      // when owerflow occure 
else DELTA2 = E2 - P_E2;
if(E3 - P_E3 < -5000) DELTA3 = (4294967295 - P_E3 + E3);      // when owerflow occure 
else DELTA3 = E3 - P_E3;
if(E4 - P_E4 < -5000) DELTA4 = (4294967295 - P_E4 + E4);      // when owerflow occure 
else DELTA4 = E4 - P_E4;
if(E5 - P_E5 < -5000) DELTA5 = (4294967295 - P_E5 + E5);      // when owerflow occure 
else DELTA5 = E5 - P_E5;
if(E6 - P_E6 < -5000) DELTA6 = (4294967295 - P_E6 + E6);      // when owerflow occure 
else DELTA6 = E6 - P_E6;


/////////////// DETERMINING THE AMOUNT OF DISTANCE THE ROVER HAD MOVED ///////////////
DELTA_D = (DELTA1 + DELTA2 + DELTA3 + DELTA4 + DELTA5 + DELTA6)/(6*N*G);         // DETERMINING THE AVERAGE ENCODER COUNT ...
DELTA_D = (DELTA_D + (RPM1 + RPM2 + RPM3 + RPM4 + RPM5 + RPM6)/(F*60*6))/2;     // APPROXIMATING ROVERS MOTION THROUGH VELOCITY DATA ...
DELTA_D =  DELTA_D*(2*3.1415926*WHEEL_RADIUS);                                 // UNITS IN "mm" 


///////////////// DETERMINING THE COORDINATES ///////////////////////////////////////
X1 = X1 + DELTA_D*sin(YAW*DEG_RAD)*cos(PITCH*DEG_RAD);          // units in mm
Y1 = Y1 + DELTA_D*cos(YAW*DEG_RAD)*cos(PITCH*DEG_RAD);          // units in mm


/////////////// GETTING RPM FROM I2C DATA ///////////////////////////////////////////
//----------------------------------------------------------------------------------------------------------------
RPM1 = ((I2C_DATA_FCCC[3]<<24)|(I2C_DATA_FCCC[2]<<16)|(I2C_DATA_FCCC[1]<<8)|(I2C_DATA_FCCC[1]));  // READING VELOCITY DATA OF THE 
RPM2 = ((I2C_DATA_FCCC[7]<<24)|(I2C_DATA_FCCC[6]<<16)|(I2C_DATA_FCCC[5]<<8)|(I2C_DATA_FCCC[4]));                  // FRONT WHEELS
//----------------------------------------------------------------------------------------------------------------
RPM3 = ((I2C_DATA_MCCC[3]<<24)|(I2C_DATA_MCCC[2]<<16)|(I2C_DATA_MCCC[1]<<8)|(I2C_DATA_MCCC[1]));  // READING VELOCITY DATA OF THE 
RPM4 = ((I2C_DATA_MCCC[7]<<24)|(I2C_DATA_MCCC[6]<<16)|(I2C_DATA_MCCC[5]<<8)|(I2C_DATA_MCCC[4]));                  // MIDDLE WHEELS
//----------------------------------------------------------------------------------------------------------------
RPM5 = ((I2C_DATA_BCCC[3]<<24)|(I2C_DATA_BCCC[2]<<16)|(I2C_DATA_BCCC[1]<<8)|(I2C_DATA_BCCC[1]));  // READING VELOCITY DATA OF THE 
RPM6 = ((I2C_DATA_BCCC[7]<<24)|(I2C_DATA_BCCC[6]<<16)|(I2C_DATA_BCCC[5]<<8)|(I2C_DATA_BCCC[4]));                  // BACK WHEELS
//----------------------------------------------------------------------------------------------------------------


}
