#include<libmaple/scb.h>

//********************* ENCODER **************************************************
int N=7,G=80;
double F1;
uint32_t T1=0,t1=0,tp1=0,E1,EP1;
uint32_t Ti;
int DI1 = 1;
float RPM , RPM_EST , PRPM;
uint8_t int_flag1;
uint32_t systick_counter,To;

//***********************************************************************************

int F = 2500;
int t,T,tp;
//************************* pid gains ********************************////////// 

float Kp=2,Kd=0,Ki=2000;      ////////motor pid 

//***************************************************************************
double ERROR_,PREV_ERROR_;
double P,I,D;
double PID,SAT_PID;
double VEL;
double FEEDBACK_SPEED;
int int_flag = 0;

void setup() {

  
pinMode(PB7,INPUT_PULLUP);
pinMode(PB8,INPUT_PULLUP);
pinMode(PB2,OUTPUT);
pinMode(PB3,OUTPUT);
pinMode(PA8,OUTPUT);
pinMode(PB6,OUTPUT);

Serial.begin(115200);

attachInterrupt(PB7,ENCODER1,RISING);
 

 

t = micros();

}

void loop() {
  // put your main code here, to run repeatedly:
  
VEL = 50 ;
  
//***************PID CODE ******************************

RPM_EST = (1000000*60*(E1-EP1))/(N*G*(micros()-Ti));
EP1 = E1 ;
RPM = F1;
FEEDBACK_SPEED  = RPM*60;

ERROR_ = (VEL - FEEDBACK_SPEED);

///////////// pid caliculation ///////////////////
P = ERROR_*Kp;
I = I + (ERROR_*float(Ki)*(micros()-Ti))/1000000;
I = I + 3*(SAT_PID - PID);   
D = ((PREV_ERROR_ - ERROR_)*Kd)/(micros()-Ti);
/////////////////////////////////////////////////
Ti = micros();
PREV_ERROR_  = ERROR_;

PID = P + I + D;

SAT_PID = SATURATE(PID,-60000,60000);  

if(SAT_PID>=0) {
//  digitalWrite(PB2,HIGH);      //  if the speed is positive we have to run in clock wise direction ...
//  digitalWrite(PB3,LOW);
analogWrite(PB6,0);
analogWrite(PA8,map(abs(SAT_PID),0,60000,0,65535));
}
else if(SAT_PID<0) {
//  digitalWrite(PB2,LOW);      //  if the speed is negative we have to run in anticlock wise direction ...
//  digitalWrite(PB3,HIGH);
analogWrite(PA8,0);
analogWrite(PB6,map(abs(SAT_PID),0,60000,0,65535));
}


Serial.print("SET SPEED ");
Serial.print(VEL);
Serial.print(" , RPM = ");
Serial.println(RPM*60);
//Serial.print(" , P = ERROR_*Kp ");
//Serial.println(ERROR_);
//Serial.print(" , I ");
//Serial.println(I);
//Serial.print(" , D = ");
//Serial.println(D);
//Serial.print(" , SAT_PID = ");
//Serial.println(SAT_PID);

//******--------------************----------**********------

while(micros() - t < 1000000/F );
   t = micros();
  
}



void ENCODER1(){
  
  // micros function 
  
  systick_counter = SYSTICK_BASE->CNT;
  
  if(0b1 & SCB_BASE->ICSR >> 26){
    int_flag1 = 1;
    systick_counter = SYSTICK_BASE->CNT;
  }
  else int_flag1 = 0;

  t1 = (systick_uptime_millis * 1000) + (SYSTICK_RELOAD_VAL + 1 - systick_counter) / CYCLES_PER_MICROSECOND ;
  //t1 = micros();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 if(int_flag1) t1 = t1 + 1000;
  T1 = (t1 - tp1);

//  if(digitalRead(PB8)) DI1 = 1;
//  else DI1 = -1;
//  if((GPIOB_BASE->IDR >> (uint32_t)8) & 0x1)  DI1 = 1; // reading PB8 pin for direction 
//  else DI1 = -1;


  F1 = (DI1*(float(1000000/float(T1*N*G)))) ;  // frequiency of revolution  of the motor 
  
  E1 = E1 + DI1;
  tp1 = t1;
  
}


float SATURATE(float INPUT_ , float L , float U ){

  if(INPUT_ > U) return U;                                // saturation  the pid 
  else if(INPUT_ < L) return L;
  else return INPUT_ ;
  
}
