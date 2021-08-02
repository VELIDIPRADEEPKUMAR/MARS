
 /* Mobility controller code for ARDUINO MEGA ,
   this code is developed by MARS rover team iiitdm kanchipuram */


#include <Wire.h>


uint16_t F = 2500;//frequency we are constraining the loop to

 #define DEG_RAD 0.0174533     // DEG TO RAD CONVERSION FACTOR.
 #define XF 10                // distance from front wheels to middle wheels axis.
 #define XB 10               // distance from back wheels to middle wheels axis.
 #define G 10               // GEAR RATIO.
 #define N 120             // ENCODER COUNTS.
 #define WHEEL_RADIUS 100 // RADIUS IN "mm".
 
///////////////// MOTOR SPEEDS ////////////////////////
int F_L_M,F_R_M;       // FRONT LEFT & RIGHT MOTOR 
int M_L_M,M_R_M;      // MIDDLE FRONT & LEFT MOTOR
int B_L_M,B_R_M;     // BACK MOTORS 


//////////////// I2C DATA ///////////////////////
byte I2C_DATA_FCCC[16];
byte I2C_DATA_MCCC[16];
byte I2C_DATA_BCCC[16];


/////////////// LOOP TIMER VARIABLE ////////////////////////
uint32_t timeOutTimer;
uint32_t T;

//////////// DEAD RECKONING  VARIABLES //////////////////
int16_t X1=0,Y1= 0;      // coordinates of the rover 
uint32_t E1,E2,E3,E4,E5,E6; 
uint32_t P_E1,P_E2,P_E3,P_E4,P_E5,P_E6;
int32_t RPM1,RPM2,RPM3,RPM4,RPM5,RPM6;
int8_t DELTA1,DELTA2,DELTA3,DELTA4,DELTA5,DELTA6;
float DELTA_D;


////////////// IMU VARIABLES ////////////////////////////
double   YAW ,PITCH,ROLL;
int16_t  YAW_RATE;


///////////// CAN BUS VARIABLES ///////////////////////////
int16_t VEL,STEER;
//////////////////////////////////////////////////////////

void setup() {

X1 =0;
Y1 =0;
 
  Wire.begin();
  Serial.begin(115200);
  
}

void loop() {

///////////////// RECIEVE DATA USING CAN BUS /////////////////////////
UPDATE_CAN_DATA();

////////////////// GET IMU DATA FROM SENSOR //////////////////////////
UPDATE_IMU();

//////////// updating the dead reckoning algrothum ///////////////////
DEAD_RECKONING();

//////////////// comanding the slaves ///////////////////////////////////
CONTROLLER_BRAIN();

///////////////////////////////////////////////////////////////////////////
PRINTDATA();

 while(micros()-T < 10000000/F);  // RESTRICT THE LOOP SPEED TO A FREQUIENCY F ; 
  T = micros();

}
