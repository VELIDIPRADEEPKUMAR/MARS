#include<libmaple/scb.h>
#include <Wire_slave.h>

uint16_t F = 2500;
int t=0;

#define    SAT_PID_VALUE    1000       // this is the max val pid can achieve 
#define    M1               PB4        // joint 4 pwm motor pin
#define    J_4_INT1         PB7        // INT 1 OF MOTOR DRIVER 
#define    J_4_INT2         PB8        // INT 2 OF MOTOR DRIVER 
#define    M2               PB5        // joint 5 pwm motor pin
#define    J_5_INT1         PB9        // INT 1 OF MOTOR DRIVER 
#define    J_5_INT2         PB13        // INT 1 OF MOTOR DRIVER 
#define    M3               PA6        // joint 6 pwm motor pin
#define    J_6_INT1         PA7        // INT 1 OF MOTOR DRIVER 
#define    J_6_INT2         PB7        // INT 1 OF MOTOR DRIVER 
#define    E11              PB10       // joint 4  motor encoder pin 1
#define    E12              PB1        // joint 4  motor encoder pin 2
#define    E21              PB11       // joint 5  motor encoder pin 1
#define    E22              PB2        // joint 5  motor encoder pin 2
#define    E31              PB12       // joint 6  motor encoder pin 1
#define    E32              PB3        // joint 6  motor encoder pin 2
#define    PID_SET_PIN      PB0        // pin for setting CONTROLLERS MODE ...
#define    J_4_I            PA1        // CURRENT SENSOR DATA PIN FOR JOINT 4
#define    J_5_I            PA2        // CURRENT SENSOR DATA PIN FOR JOINT 5 
#define    J_6_I            PA3        // CURRENT SENSOR DATA PIN FOR JOINT 6


int N=10,G=100;     // encoder counts & gear ratio 

/////////   ENCODER DATA   ///////////////
uint32_t T1=0,t1=0,tp1=0;
uint32_t T2=0,t2=0,tp2=0;
uint32_t T3=0,t3=0,tp3=0;
int32_t  E1,E2,E3;
double F1,F2,F3;
int8_t DI1 = 1,DI2 = 1,DI3 = 1;

///////////////// I2C DATA VARIABLES ///////////////////////

bool SET_PID;
byte INPUTDATA[6];
uint32_t timeOutTimer;

//********************** JOINT_4_CONTROLLER  PARAMETERS  ****************************************

double JOINT_4_T_ERROR,   JOINT_4_T_PREV_ERROR,     JOINT_4_T_I;
                                                                                 // TUNE THE PID GAINS HERE ......
           byte   JOINT_4_T_Kp = 0,  JOINT_4_T_Kd = 0,  JOINT_4_T_Ki = 0;          // GAINS FOR  POSITION CONTROLLER
           
double JOINT_4_PID;
double JOINT_4_SAT_PID;
double JOINT_4_FEEDBACK_T;
double INITIAL_T_4;
double JOINT_4_FEEDBACK_I;
double JOINT_4_PT;
double  JOINT_4_SET_T;
uint32_t TIMER_4;

///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/

//********************** JOINT_5_CONTROLLER  PARAMETERS  ****************************************

double JOINT_5_T_ERROR,   JOINT_5_T_PREV_ERROR,     JOINT_5_T_I;

           byte JOINT_5_T_Kp = 0,JOINT_5_T_Kd = 0,JOINT_5_T_Ki = 0;    // GAINS FOR  POSITION CONTROLLER

double JOINT_5_PID;
double JOINT_5_SAT_PID;
double JOINT_5_FEEDBACK_T;
double INITIAL_T_5;
double JOINT_5_FEEDBACK_I;
double JOINT_5_PT;
double  JOINT_5_SET_T;
uint32_t TIMER_5;

///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/

//********************** JOINT_6_CONTROLLER  PARAMETERS  ****************************************

double JOINT_6_W_ERROR,   JOINT_6_W_PREV_ERROR,     JOINT_6_W_I;
double JOINT_6_T_ERROR,   JOINT_6_T_PREV_ERROR,     JOINT_6_T_I;

         byte JOINT_6_W_Kp = 0,JOINT_6_W_Kd = 0,JOINT_6_W_Ki = 0;
                                                                      // pid gains .....
         byte JOINT_6_T_Kp = 0,JOINT_6_T_Kd = 0,JOINT_6_T_Ki = 0;
         
double JOINT_6_PID_W,JOINT_6_PID_T;
double JOINT_6_SAT_PID,JOINT_6_SAT_PID_W,JOINT_6_SAT_PID_T;
double JOINT_6_FEEDBACK_W;
double JOINT_6_FEEDBACK_T;
double INITIAL_T_6;
double JOINT_6_FEEDBACK_I;
double JOINT_6_PT;
int16_t JOINT_6_SET_W ;
double  JOINT_6_SET_T,J6_SET_T;
uint32_t TIMER_6;

///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/




void setup() {
  // put your setup code here, to run once:
  
  Wire.begin(101);
  Wire.onRequest(RequestEvent); // register event
  Wire.onReceive(RECIEVESPEED);

pinMode(PID_SET_PIN,INPUT_PULLUP);
pinMode(E11,INPUT_PULLUP);
pinMode(E21,INPUT_PULLUP);
pinMode(E31,INPUT_PULLUP);
pinMode(M1,OUTPUT);
pinMode(M2,OUTPUT);
pinMode(M3,OUTPUT);


Serial.begin(115200);

attachInterrupt(E11,ENCODER1,RISING);
attachInterrupt(E21,ENCODER2,RISING);
attachInterrupt(E31,ENCODER3,RISING);

INITIAL_T_4 = map(analogRead(PA0),0,4096,0,90);
INITIAL_T_5 = map(analogRead(PA1),0,4096,0,90);
INITIAL_T_6 = map(analogRead(PA2),0,4096,0,90);     // inisilizing to homing position .... 

E1 = 0;
E2 = 0;
E3 = 0;

}




void loop() {
  
// READ POTENTOMETERS 

JOINT_4_PT  = map(analogRead(PA0),0,4096,0,90);
JOINT_5_PT  = map(analogRead(PA1),0,4096,0,90);
JOINT_6_PT  = map(analogRead(PA2),0,4096,0,90);


/////////////////// ESTIMATING THE ANGLE OF EACH JOINT ////////////////////////////
///////////////////////// COMPLIMENTARY FILTER ////////////////////////////////////

JOINT_4_FEEDBACK_T = ((INITIAL_T_4  + ((E1*360)/double(G*N)))*0.85 + JOINT_4_PT*0.15) ;  // USING A COMPLIMENTARY FILTER TO 
JOINT_5_FEEDBACK_T = ((INITIAL_T_5  + ((E2*360)/double(G*N)))*0.85 + JOINT_5_PT*0.15) ;     // ESTIMATE THE JOINT ANGLES ...
JOINT_6_FEEDBACK_T = ((INITIAL_T_6  + ((E3*360)/double(G*N)))*0.85 + JOINT_6_PT*0.15) ;

//////////////////////////  FINDING FEEDBACK SPEED ///////////////////////////////

JOINT_4_FEEDBACK_W =  F1*60;               // SPEED OF MOTOR  IN "rpm"
JOINT_5_FEEDBACK_W =  F2*60;
JOINT_6_FEEDBACK_W =  F3*60;

///////////////////// DETERMINING SET POINTS /////////////////////////////////////

SET_PID = digitalRead(PID_SET_PIN );     // reading the pid status set  pin

JOINT_4_SET_T = double((INPUTDATA[1]<<8) | INPUTDATA[0]);
JOINT_5_SET_T = double((INPUTDATA[3]<<8) | INPUTDATA[2]);

if(SET_PID) {

JOINT_6_SET_W = (INPUTDATA[5]<<8) | INPUTDATA[4];

J6_SET_T      =  JOINT_6_FEEDBACK_T*1000.0 + (JOINT_6_SET_W*60000.0)/F ; // DESIRED SET ANGLE IN "milli deg"
JOINT_6_SET_T = J6_SET_T/1000.0;

}
else {
  
JOINT_6_SET_T = double((INPUTDATA[5]<<8) | INPUTDATA[4]);
  
}

JOINT_4_FEEDBACK_I    =   analogRead(J_4_I);  //   reading  current sensor data
JOINT_5_FEEDBACK_I    =   analogRead(J_5_I);
JOINT_6_FEEDBACK_I    =   analogRead(J_6_I);


////////////////////////// calling the functions //////////////////////////////

JOINT_4_CONTROLLER();

JOINT_5_CONTROLLER();

JOINT_6_CONTROLLER();

//////////////////////////////////////////////////////////////////////////////

WRITE_TO_MOTORS();      // WRITE TO MOTORS 

  
while((micros() - t ) < (1000000/F));   // running the loop with const frequiency 
t = micros();


}
