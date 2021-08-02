#include<Wire.h>
#include<SPI.h>

#define   L1              100    //length of joint 1 in "mm"   
#define   L2              100    //length of joint 2 in "mm" 


/////////////// I2C DATA ///////////////////////////////

uint32_t timeOutTimer ;
byte I2C_DATA_EF[6];
byte I2C_DATA_JC[6];
bool TELE;                               
//////////////////////////////////////////////////////////

////////////////// INVERSE KINAMATICS /////////////////////

double SET_YAW,X,Y;
double T1,T2;
double K1,K2;
int16_t J3_SET,J4_SET,J5_SET;  // SETTING THE JOINT 3 ANGLE FOR ORIENTATION OG THE ENDEFFECTOR 
int16_t THETA1,THETA2,THETA3,THETA4,THETA5,THRTA6;


///////////// CAN_BUS ////////////////////////////////////

int16_t CAN_J_1,CAN_J_2,CAN_J_3,CAN_J4,CAN_J5,CAN_J6;
//////////////////////////////////////////////////////////



void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  pinMode(PB1,OUTPUT);
  
  

}

void loop() {
  // put your main code here, to run repeatedly:



//////////// WRITING DATA TO MANIPULATOR CONTROLLERS ///////////////////////

if(TELE){
  
  digitalWrite(PB1,HIGH);                // IF TELEOPERATION IS ACTIVATED 
  I2C_WRITE(101,CAN_J1,CAN_J2,J3_SET);         // WRITE DATA TO ENDEFFECTOR CONTROLLER
  I2C_WRITE(102,SET_J4,SET_J5,CAN_J6);         // WRITE DATA TO JOINT CONTROLLER 
  
}
else  {
  
  digitalWrite(PB1, LOW);      
  I2C_WRITE(101,CAN_J1,CAN_J2,CAN_J3);         // WRITE DATA TO ENDEFFECTOR CONTROLLER    
  I2C_WRITE(102,CAN_J4,CAN_J5,CAN_J6);         // WRITE DATA TO JOINT CONTROLLER       
}

//////////////////////////////////////////////////////////////////////////////


}
