
void CONTROLLER_BRAIN(){

////////////////// FRONT CROSS COUPPLED CONTROLLER DATA ///////////////////////////
F_L_M = VEL - (STEER*XB/XF); //(STEER*XB/XF) THIS IS TO MAKESURE THAT THE AXIS OF ROTATION 
F_R_M = VEL + (STEER*XB/XF);                           // SHOULD BE IN THE CENTER OF ROVER .
I2C_WRITE_FRONT_CCC();


////////////////// MIDLE CROSS COUPPLED CONTROLLER DATA ///////////////////////////
M_L_M  = VEL - STEER;
M_R_M  = VEL + STEER; 
I2C_WRITE_MIDLE_CCC();


////////////////// BACK CROSS COUPPLED CONTROLLER DATA ///////////////////////////
B_L_M = VEL - STEER;
B_R_M = VEL + STEER;
I2C_WRITE_BACK_CCC();


} 
