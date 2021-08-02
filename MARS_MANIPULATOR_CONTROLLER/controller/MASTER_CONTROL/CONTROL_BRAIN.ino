
void CONTROLLER_BRAIN(){

I2C_READ(101,I2C_DATA_EF);
I2C_READ(102,I2C_DATA_JC);

THETA1   = (I2C_DATA_EF[1]<<8) | I2C_DATA_EF[0];
THETA2   = (I2C_DATA_EF[3]<<8) | I2C_DATA_EF[2];
THETA3   = (I2C_DATA_EF[5]<<8) | I2C_DATA_EF[4];
THETA4   = (I2C_DATA_JC[1]<<8) | I2C_DATA_JC[0];
THETA5   = (I2C_DATA_JC[3]<<8) | I2C_DATA_JC[2];
THETA6   = (I2C_DATA_JC[5]<<8) | I2C_DATA_JC[4];

////////////////// DETERMINING THE ORIENTATION OF ENDEFFECTOR /////////////////

SET_YAW = (90 - THETA4 - THETA5)*100 + double(CAN_J_3*100)/F;
J3_SET  = SET_YAW/100;

////////////////////// FORWARD  KINEMATICS  /////////////////////////////

X = L1*sin(THETA4) + L2*sin(THETA5) + double(CAN_J_4)/F;
Y = L1*cos(THETA4) + L2*sin(THETA5) + double(CAN_J_5)/F;

///////////////////// INVERSE KINAMATICS ///////////////////////////////

T2     = acos((X^2 + Y^2 - L1^2 - L2^2)/(2*L1*L2));
J4_SET = T2;
K1     = L1 + L2*cos(T2);
K2     = L2*sin(T2);
T1     = atan2(X/Y) - atan2(K1/K2); 
J5_SET = T1;




} 
