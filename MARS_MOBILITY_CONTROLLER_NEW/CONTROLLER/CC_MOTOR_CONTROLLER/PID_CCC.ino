
void PID_CCC()
{
  
                    //////////////////////////////////////////////////////////////////////////////////
                    //--------------------- CROSS COUPLED CONTROLLER ------------------------------//
                    /////////////////////////////////////////////////////////////////////////////////
        

  // STEERING PID FOR FRONT MOTORS

  ERROR_S      = (L_M_VEL - R_M_VEL) - (FEEDBACK_L - FEEDBACK_R) - FEEDBACK_STEER;
//----------------------------------------------------------------------------------------------------------------
  S_PID        = PID(ERROR_S, PREV_ERROR_S, &S_I,S_Kp, S_Ki,S_Kd, SAT_S_PID, S_PID,TIMER1); // PID FUNCTION USAGE 
//----------------------------------------------------------------------------------------------------------------
  TIMER1       = micros();
  SAT_S_PID    = SATURATE(S_PID,-SAT_PID_VALUE,SAT_PID_VALUE);          // SATURATE THE PID 
  
  PREV_ERROR_S = ERROR_S;

  // *********************************************************************

  
  // PID CONTROLLER FOR LEFT FRONT MOTOR
  
  ERROR_L      = L_M_VEL - FEEDBACK_L - (S_PID * 0.3);
//-------------------------------------------------------------------------------------------------------------  
  L_PID        = PID(ERROR_L,PREV_ERROR_L,&L_I,L_Kp,L_Ki,L_Kd,SAT_L_PID,L_PID,TIMER2);  // PID FUNCTION USAGE 
//-------------------------------------------------------------------------------------------------------------
  TIMER2       = micros();
  SAT_L_PID    = SATURATE(L_PID,-SAT_PID_VALUE,SAT_PID_VALUE);          // SATURATE THE PID 
  
  PREV_ERROR_L = ERROR_L;

  // *******************************************************************
 


  // PID CONTROLLER FOR FRONT RIGHT MOTOR
  
  ERROR_R      = R_M_VEL - FEEDBACK_R + (S_PID * 0.3);
//-------------------------------------------------------------------------------------------------------------
  R_PID        = PID(ERROR_R,PREV_ERROR_R,&R_I,R_Kp,R_Ki,R_Kd,SAT_R_PID,R_PID,TIMER3);  // PID FUNCTION USAGE
//-------------------------------------------------------------------------------------------------------------
  TIMER3       = micros();
  SAT_R_PID    = SATURATE(R_PID,-SAT_PID_VALUE,SAT_PID_VALUE);          // SATURATE THE PID 
  
  PREV_ERROR_R = ERROR_R;
 
  
}



double SATURATE(double INPUT_ , uint32_t L , uint32_t U ){

  if(INPUT_ > U) return U;                                // saturation  the pid 
  else if(INPUT_ < L) return L;
  else return INPUT_ ;
  
}




  
