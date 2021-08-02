void ENCODER1(){
  
  t1 = MICROS();
  T1 = (t1 - tp1);
  
  if((GPIOB_BASE->IDR >> (uint32_t)12) & 0B1)  DI1 = 1; // reading PB12 pin for direction 
  else DI1 = -1;
  
  F1 = DI1*(float(1000000/float(T1*N*G)));
 
  E1 = E1 + DI1;
  tp1 = t1;
  
}

void ENCODER2(){
  
  t2 = MICROS();
  T2 = (t2 - tp2);

  if((GPIOB_BASE->IDR >> (uint32_t)13) & 0x1)  DI2 = 1; // reading PB12 pin for direction 
  else DI2 = -1;
  
  F2 = DI2*(float(1000000/float(T2*N*G)));       
   
  E2 = E2 + DI2;
  tp2 = t2;
  
}

void ENCODER3(){
  
  t3 = MICROS();
  T3 = (t3 - tp3);

  if((GPIOB_BASE->IDR >> (uint32_t)5) & 0x1)  DI3 = 1; // reading PB12 pin for direction 
  else DI3 = -1;
  
  F3 = DI3*(float(1000000/float(T2*N*G)));       
   
  E3 = E3 + DI3;
  tp3 = t3;
  
}

/////////////////// MICROS FUNCTION WHICH WORKS INSIDE INTERUPTS ////////////////////

uint32_t MICROS(void) {

  uint8_t  int_flag;
  int32_t  systick_counter,current_state;

     systick_counter = SYSTICK_BASE->CNT;               // System timer(Systick) base  
  if(0b1 & SCB_BASE->ICSR >>26){  //Check if Systick interrupt pending flag is set
     int_flag = 1;
     systick_counter = SYSTICK_BASE->CNT;  //Re-read Systick Timer
  }
  else int_flag = 0;    // Systick interrupt pending flag is not set
       current_state = (systick_uptime_millis * 1000) + (SYSTICK_RELOAD_VAL + 1 - systick_counter) / CYCLES_PER_MICROSECOND;
       
  if(int_flag) current_state += 1000;

  return current_state;
}
