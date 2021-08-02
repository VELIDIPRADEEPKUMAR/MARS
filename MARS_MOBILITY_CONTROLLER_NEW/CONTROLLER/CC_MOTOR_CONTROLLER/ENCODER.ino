

////////////////////// READING ENCODER1 DATA USING THE SYSTICK TIMER(MILLIS) ///////////////////////////////

void ENCODER1() {  /// determining value of micros in the time of an interupt, where micros won't update
   
  current_state1  = MICROS();                        
  pulse_time1     = current_state1 - previous_state1;
  previous_state1 = current_state1;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if((GPIOB_BASE->IDR >> (uint32_t)13) & 0x1)  DIR1 = 1; // reading PB13 pin  for second encoder signal for direction
  else DIR1 = -1;
  
  F1 = DIR1*(double(1000000/double(pulse_time1*N*G)));   
  E1 = E1 + DIR1;
 
}


////////////////////// READING ENCODER2 DATA USING THE SYSTICK TIMER(MILLIS) ///////////////////////////////

void ENCODER2() {
  
  current_state2  = MICROS(); 
  pulse_time2     = current_state2 - previous_state2;
  previous_state2 = current_state2;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if((GPIOB_BASE->IDR >> (uint32_t)12) & 0x1)  DIR2 = 1; // reading PB12 pin  for second encoder signal for direction 
  else DIR2 = -1;
  
  F2 = DIR2*(double(1000000/double(pulse_time2*N*G)));          
  E2 = E2 + DIR2;

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


////////////////////// READING ENCODER DATA USING ENCODER MODE ///////////////////////////////////////////////////////////

//??????????????????????????????  UNDER DEVELOPMENT ???????????????????????????????????????????????????????

/*void set_encoder_mode1()
{
  TIMER4_BASE->ARR = 0xFFFF;    // It takes one count to overflow
  TIMER4_BASE->CCMR1 |= (TIMER_CCMR1_CC1S_INPUT_TI1 | TIMER_CCMR1_CC2S_INPUT_TI2);    //Mapping channel 1 and 2 as inputs to Timer 2
  TIMER4_BASE->CCER  |= (TIMER_CCER_CC1P | TIMER_CCER_CC2P);      //Sets channel to trigger on rising edge
  TIMER4_BASE->SMCR |= (TIMER_SMCR_SMS_ENCODER3 | TIMER_SMCR_SMS_ENCODER3);   //This is for counting the pulse on both TI1 and TI2 channels
  TIMER4_BASE->CR1 |= TIMER_CR1_CEN;  // enable the counter.
 }
 void set_encoder_mode2()
 {
  TIMER4_BASE->ARR = 0xFFFF;    // It takes one count to overflow
  TIMER4_BASE->CCMR1 |= (TIMER_CCMR1_CC1S_INPUT_TI1 | TIMER_CCMR1_CC2S_INPUT_TI2);    //Mapping channel 1 and 2 as inputs to Timer 2
  TIMER4_BASE->CCER  |= (TIMER_CCER_CC1P | TIMER_CCER_CC2P);      //Sets channel to trigger on rising edge
  TIMER4_BASE->SMCR |= (TIMER_SMCR_SMS_ENCODER3 | TIMER_SMCR_SMS_ENCODER3);   //This is for counting the pulse on both TI1 and TI2 channels
  TIMER4_BASE->CR1 |= TIMER_CR1_CEN;  // enable the counter.
 }

 int get_encoder_pulses()
 {
    return TIMER2_BASE->CNT;
 }
 */
  
///////////////////////////////////////////////////////////////////////////////////
