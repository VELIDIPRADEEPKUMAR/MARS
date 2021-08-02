 
 /*   Mobility controller code for stm32f103c8t6 ,
   ** this code is developed by MARS rover team iiitdm kanchipuram */


#include <libmaple/scb.h>
#include <libmaple/timer.h>
#include <Wire_slave.h> 

uint16_t F = 2500;//frequency we are constraining the loop to
uint32_t T=0;//T is for keeping track of the frequency

/////////////////////// I2C data variables /////////////////////////
byte INPUTDATA[6];
uint32_t timeOutTimer;


//////***************PIN DESCRIPTION AND DECLERATION ********************//

 #define LEFT_MOTOR            PA9            // left motor 
 #define LEFT_MOTOR_INT1       PA11
 #define LEFT_MOTOR_INT2       PA1
 #define RIGHT_MOTOR           PA10           // right motor 
 #define RIGHT_MOTOR_INT1      PA3
 #define RIGHT_MOTOR_INT2      PA5
 #define ENCODER1_PIN          PB2           // encoder1 
 #define ENCODER1_DIR_PIN      PB4
 #define ENCODER2_PIN          PB3           // encoder2 
 #define ENCODER2_DIR_PIN      PB6

       /////////////////////////////////////////////////////////////////////////////////

        ///////////////////********* PID GAINS ***********//////////////////////////// tune the controller by adjusting the gains 
        //////
               uint16_t     L_Kp=0,  L_Kd=0,  L_Ki=0;     //////// left motor pid ;
        //////
               uint16_t     R_Kp=0,  R_Kd=0,  R_Ki=0;     //////// RIGHT MOTOR PID ;
        //////
               uint16_t     S_Kp=0,  S_Kd=0,  S_Ki=0;     //////// CCC PID ;
        //////
       ////////////////// *******************************/////////////////////////////

      ///////////////////////////////////////////////////////////////////////////////////



//******** pid variables ***************//////////////
double  L_PID,SAT_L_PID,L_I=0;      ////////front left motor pid ;
double  R_PID,SAT_R_PID,R_I=0;      ////////FRONT RIGHT MOTOR PID ;
double  S_PID,SAT_S_PID,S_I;
double  ERROR_L,ERROR_R,ERROR_S;   // errors 
double  PREV_ERROR_L,PREV_ERROR_R,PREV_ERROR_S;   // previous error

#define SAT_PID_VALUE 1000               // this will saturate the pid value to SAT_PID

//*****************************************************************
int16_t  R_M_VEL,L_M_VEL;            ////Change this according to user input. MAX VEL is 1m/s
double   FEEDBACK_L,FEEDBACK_R;
int16_t  FEEDBACK_STEER;
uint32_t TIMER1,TIMER2,TIMER3;



/////********** ENCODER VARIABLES **************
uint32_t previous_state1,current_state1,pulse_time1;  // Encoder_read function varaibles
uint32_t previous_state2,current_state2,pulse_time2;

double   F1,Fp1=0;
double   F2,Fp2=0;

uint32_t E1,EP1 = 0;
uint32_t E2,EP2 = 0;

int8_t   DIR1 = 1,DIR2 = 1;
uint32_t TI;

uint16_t N = 100, G = 40;         // encoder turns and gear ratio 
///////////////////////////////////////////////////////////////////////////////




// ******** FUNCTION DECLERATOIN *************
void   PID_CCC(void);
float  PID(float ERROR_, float PREV_ERROR_, float *I,byte Kp,byte Ki,byte Kd,float SAT_PID,float PID);
void   ENCODER(void);
double SATURATE(float INPUT_ , float L , float U );
void   WRITE_TO_MOTORS();

/////////////////////////////////////////////////////////////////////////////



void setup() {
  
  //****************** pin setup ***************
  pinMode(LEFT_MOTOR,OUTPUT);                    // SETUP MOTOR PINS 
  pinMode(LEFT_MOTOR_INT1,OUTPUT);
  pinMode(LEFT_MOTOR_INT2,OUTPUT);
  pinMode(RIGHT_MOTOR,OUTPUT);
  pinMode(RIGHT_MOTOR_INT1,OUTPUT);
  pinMode(RIGHT_MOTOR_INT2,OUTPUT);

  attachInterrupt(ENCODER1_PIN,ENCODER1,RISING);      // SETUP INTERUPTS FOR ENCODER 
  pinMode(ENCODER1_DIR_PIN,INPUT_PULLUP);             // PINMODE FOR SECOND ENCODER PIN FOR DIRECTION 
  attachInterrupt(ENCODER2_PIN,ENCODER2,RISING);
  pinMode(RIGHT_MOTOR_INT2,INPUT_PULLUP);

  //**********************************************

  
  Serial.begin(115200);
  
 

  // I2C SETUP 

  Wire.begin(101);
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(RECIEVESPEED);

  TI = micros();
}


void loop() {

//////////////// I2C SPEED ///////////////////////////////////////////

L_M_VEL             = (INPUTDATA[1]<<8)|INPUTDATA[0];
R_M_VEL             = (INPUTDATA[3]<<8)|INPUTDATA[2];
FEEDBACK_STEER      = (INPUTDATA[5]<<8)|INPUTDATA[4];

//////////////// GETTING INPUTS FOR CONTROL USING ENCODER DATA ////////////////////////////
 
  FEEDBACK_L = (E1 - EP1 < 1)?0:F1*60;    // if no change in encoder reading set speed to "0"
  FEEDBACK_R = (E2 - EP2 < 1)?0:F2*60;

//////////// CALLING THE CONTROLLER /////////////////////////
  
    PID_CCC();

/////////////  WRITE DATA TO MOTORS ////////////////////////////

     WRITE_TO_MOTORS();

//////////////////////////////////////////////////////////////////

      PRINTDATA();
  
  while(micros()-T < 10000000/F);  // RESTRICT THE LOOP SPEED TO A FREQUIENCY F ; 
  T = micros();
  
} // end loop 
