#include<libmaple/scb.h>
#include<Wire_slave.h>

int F = 250;
int t = 0;
int N=10,G=100;

#define    SAT_PID_VALUE    1000       // this is the max val pid can achieve 
#define    M1               PB4        // joint 4 pwm motor pin
#define    J_3_INT1         PB7        // INT 1 OF MOTOR DRIVER 
#define    J_3_INT2         PB8        // INT 2 OF MOTOR DRIVER 
#define    M2               PB5        // joint 5 pwm motor pin
#define    YAW_INT1         PB9        // INT 1 OF MOTOR DRIVER 
#define    YAW_INT2         PB13       // INT 1 OF MOTOR DRIVER 
#define    M3               PA6        // joint 6 pwm motor pin
#define    GRIPPER_INT1     PA7        // INT 1 OF MOTOR DRIVER 
#define    GRIPPER_INT2     PB7        // INT 1 OF MOTOR DRIVER 
#define    E11              PB10       // joint 4  motor encoder pin 1
#define    E12              PB1        // joint 4  motor encoder pin 2
#define    E21              PB11       // joint 5  motor encoder pin 1
#define    E22              PB2        // joint 5  motor encoder pin 2
#define    E31              PB12       // joint 6  motor encoder pin 1
#define    E32              PB3        // joint 6  motor encoder pin 2
#define    PID_SET_PIN      PB0        // pin for setting CONTROLLERS MODE ...
#define    J_4_I            PA1        // CURRENT SENSOR DATA PIN FOR JOINT 3
#define    J_5_I            PA2        // CURRENT SENSOR DATA PIN FOR  yaw 
#define    J_6_I            PA3        // CURRENT SENSOR DATA PIN FOR gripper


/////////   ENCODER DATA   ///////////////
uint32_t T1=0,t1=0,tp1=0;
uint32_t T2=0,t2=0,tp2=0;
uint32_t T3=0,t3=0,tp3=0;
int32_t  E1,E2,E3;
double F1,F2,F3;
int8_t DI1 = 1,DI2 = 1,DI3 = 1;
//////////////////////////////////////////


///////////////// I2C DATA VARIABLES ///////////////////////

bool SET_PID;
byte INPUTDATA[6];
uint32_t timeOutTimer;


//********************** JOINT_3_CONTROLLER  PARAMETERS  ****************************************

double JOINT_3_T_ERROR,   JOINT_3_T_PREV_ERROR,     JOINT_3_T_I;

           byte   JOINT_3_W_Kp = 0,  JOINT_3_W_Kd = 0,  JOINT_3_W_Ki = 0;          // GAINS FOR SPEED CONTROLLER 
                                                                                 // TUNE THE PID GAINS HERE ......
           byte   JOINT_3_T_Kp = 0,  JOINT_3_T_Kd = 0,  JOINT_3_T_Ki = 0;          // GAINS FOR  POSITION CONTROLLER
           
double JOINT_3_PID;
double JOINT_3_SAT_PID;
double JOINT_3_FEEDBACK_T;
double INITIAL_T_3;
double JOINT_3_FEEDBACK_I;
double JOINT_3_PT;
double  JOINT_3_SET_T;
uint32_t TIMER_3;
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/



//******************** YAW CONTROLLER PARAMETERS ************************************************

double YAW_W_ERROR,   YAW_W_PREV_ERROR,     YAW_W_I;
double YAW_T_ERROR,   YAW_T_PREV_ERROR,     YAW_T_I;

           byte   YAW_W_Kp = 0,  YAW_W_Kd = 0,  YAW_W_Ki = 0;          // GAINS FOR SPEED CONTROLLER 
                                                                                 // TUNE THE PID GAINS HERE ......
           byte   YAW_T_Kp = 0,  YAW_T_Kd = 0,  YAW_T_Ki = 0;          // GAINS FOR  POSITION CONTROLLER
           
double YAW_PID_W,YAW_PID_T;
double YAW_SAT_PID,YAW_SAT_PID_W,YAW_SAT_PID_T;
double YAW_FEEDBACK_W;
double YAW_FEEDBACK_T;
double INITIAL_T_Y;
double YAW_FEEDBACK_I;
double YAW_PT;
int16_t YAW_SET_W;
double  YAW_SET_T,Y_SET_T;
uint32_t TIMER_YAW;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/



//*****************  GRIPPER CONTROLLER PARAMETERS ***********************************************

double GRIPPER_ERROR,    GRIPPER_PREV_ERROR,      GRIPPER_I;

         byte   GRIPPER_Kp = 0;
         byte   GRIPPER_Kd = 0;
         byte   GRIPPER_Ki = 0;
         
double GRIPPER_PID ,GRIPPER_SAT_PID;
double GRIPPER_FEEDBACK_T;
double INITIAL_T_G;
double GRIPPER_FEEDBACK_I;
double GRIPPER_TORQUE ;
double GRIPPER_W;
int16_t GRIPPER_SET;
uint32_t G_T;
 
double Kt = .5;        // torque const of gripper motor 

double GRIPPER_PT;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/





void setup() {

pinMode(PA0,INPUT);  // ANALOG PINS 
pinMode(PA1,INPUT);
pinMode(PA2,INPUT);


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


INITIAL_T_3  = map(analogRead(PA0),0,4096,0,90);
INITIAL_T_Y  = map(analogRead(PA1),0,4096,0,90);
INITIAL_T_G  = map(analogRead(PA2),0,4096,0,90);

E1 = 0;
E2 = 0;
E3 = 0;

}




void loop() {


//////////////////// READ POTENTOMETERS /////////////////////////////

JOINT_3_PT = map(analogRead(PA0),0,4095,0,90);
YAW_PT     = map(analogRead(PA1),0,4095,0,90);
GRIPPER_PT = map(analogRead(PA2),0,4095,0,90);

///////////////// estimating the joint angles by complimentary filter //////////////////////////

JOINT_3_FEEDBACK_T = ((INITIAL_T_3  + ((E1*360)/double(G*N)))*0.85 + JOINT_3_PT*0.15) ;
YAW_FEEDBACK_T     = ((INITIAL_T_Y  + ((E1*360)/double(G*N)))*0.85 + YAW_PT*0.15);
GRIPPER_FEEDBACK_T = (INITIAL_T_G   + ((E1*360)/double(G*N)));

///////////////////// finding  feedback //////////////////////////

JOINT_3_FEEDBACK_W =  F1*60;
YAW_FEEDBACK_W     =  F2*60;
GRIPPER_W          =  F3*60;

//////////////////////// INPUT DATA TO CONTROLLRS //////////////////////////

SET_PID = digitalRead(PID_SET_PIN );     // reading the pid status set  pin 

GRIPPER_SET   = (INPUTDATA[5]<<8) | INPUTDATA[4];
JOINT_3_SET_T = (INPUTDATA[1]<<8) | INPUTDATA[0];

if(SET_PID) {
  
YAW_SET_W     = (INPUTDATA[3]<<8) | INPUTDATA[2];
Y_SET_T       =  YAW_FEEDBACK_T*1000.0     + (YAW_SET_W*60000.0)/F ;  // DESIRED SET ANGLE IN "milli deg"
YAW_SET_T     = Y_SET_T/1000.0;
}
else {
  
YAW_SET_T     = double((INPUTDATA[3]<<8) | INPUTDATA[2]);
}

////////////////// READING CURRENT SENSORS ///////////////////////////////////

JOINT_3_FEEDBACK_I    =   analogRead(J_4_I);  //   reading  current sensor data
YAW_FEEDBACK_I        =   analogRead(J_5_I);
GRIPPER_FEEDBACK_I    =   analogRead(J_6_I);

///////////////////////// calling functions //////////////////////////////////

GRIPPER_CONTROLLER();

JOINT_3_CONTROLLER();

YAW_CONTROLLER();

/////////////////////////////////////////////////////////////////////////

  
while((micros() - t ) < (1000000/F));   // running the loop with const frequiency 
t = micros();

} // end of loop 
