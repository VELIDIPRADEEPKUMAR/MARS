
double PID(double ERROR_, double PREV_ERROR_, double *I,byte Kp,byte Ki,byte Kd,double SAT_PID,double PID,uint32_t TIMER){

// caliculate the pid according to gains . 

double P,D;
double FREQ;

FREQ = micros() - TIMER;

  P = ERROR_ * (double)Kp;               //    finding  propotianal 
  *I = *I + ERROR_ * (Ki / FREQ);     // finding integral 

  *I = *I + 2*(SAT_PID - PID);          // eleminating the integral windup . 
  
  D = (PREV_ERROR_ - ERROR_) * (double)Kd * FREQ;        // finding derivative term 

  return (P + *I + D);

}

double SATURATE(double INPUT_ , uint32_t L , uint32_t U ){

  if(INPUT_ > U) return U;                                // saturation  the pid 
  else if(INPUT_ < L) return L;
  else return INPUT_ ;
  
}
