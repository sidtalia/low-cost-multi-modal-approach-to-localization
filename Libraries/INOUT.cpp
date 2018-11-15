#include"INOUT.h"

volatile int32_t input_start,input[8],delT,reset_timer,tick=0;
bool servoWrite = false;


void set_Outputs(float throttle, float steering) //throttle is between -100,100, steering -100,100.
{
  int motor, servo;
  motor = int(throttle)*5 + 1500;
  servo = int(steering)*5 + 1500;
  TIMER1_BASE->CCR1 = motor; //A8
  TIMER1_BASE->CCR4 = servo; //A11
}

void get_Inputs(float I[8])
{
  for(int i = 0;i<8;i++)
  {
    I[i] = float(input[i]) - 1000.0f;
  }
}

void setup_esc_control()
{  
  pinMode(PA8, PWM); //using only PA8 and PA11
  pinMode(PA11, PWM);
  
  // if(howmany == 4)//for dronewa
  // {
  //   pinMode(PB8, PWM);
  //   pinMode(PB9, PWM);
    
  //   TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  //   TIMER4_BASE->CR2 = 0;
  //   TIMER4_BASE->SMCR = 0;
  //   TIMER4_BASE->DIER = 0;
  //   TIMER4_BASE->EGR = 0;
  //   TIMER4_BASE->CCMR1 = 0;
  //   TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE |(0b110 << 12) | TIMER_CCMR2_OC4PE;
  //   TIMER4_BASE->CCER = TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  //   TIMER4_BASE->PSC = CLOCK_SPEED;
  //   TIMER4_BASE->ARR = 2500;
  //   TIMER4_BASE->DCR = 0;
  //   TIMER4_BASE->CCR3 = 1000;
  //   TIMER4_BASE->CCR4 = 1000;
  // }

  TIMER1_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  TIMER1_BASE->CR2 = 0;
  TIMER1_BASE->SMCR = 0;
  TIMER1_BASE->DIER = 0;
  TIMER1_BASE->EGR = 0;
  TIMER1_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE;
  TIMER1_BASE->CCMR2 = (0b110 << 12) | TIMER_CCMR2_OC4PE;
  TIMER1_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC4E;
  TIMER1_BASE->PSC = CLOCK_SPEED;
  TIMER1_BASE->ARR = 2500;
  TIMER1_BASE->DCR = 0;
  TIMER1_BASE->CCR1 = 1500;
  TIMER1_BASE->CCR4 = 1500;
}

void handler_channel_1(void)
{
  if (0b1 & GPIOA_BASE->IDR  >> 0) //check if pin PA0 is high, the GPIOA_BASE refers to port A, IDR refers to interrupt detect register which contains the pin number
  {                                // of the pin that just went high and >> 0 is us shifting the IDR by 0 bits. if the IDR's first bit is 1, 0b1&1 would give 1, this means
                                   //that there was a rising edge (the CCER is set to detect the rising edge at first)
    delT = TIMER2_BASE->CCR1 - input_start;
    if (delT < 0)delT+= 0xFFFF;
    if(reset_timer > 2000)
    {
      tick = 0;
      servoWrite = true;
    }
    else
    {
      input[tick++] = delT;
    }
    input_start = TIMER2_BASE->CCR1;//the time of the rising edge is stored in input_start[0] variable
    TIMER2_BASE->CCER |= TIMER_CCER_CC1P;// the Compare capture enable register (CCER) is set to detect the falling edge on pin PA0 (CC1P-> a binary value 
  }                                       // in which the value corresponding to pin 1(PA0) is set to high (how do i know its set to high? look at the |= ).
  else    //when the interrupt is generated BUT the edge is not the rising edge but rather the falling edge
  {
    reset_timer = TIMER2_BASE->CCR1 - input_start;//the pulse time is the falling edge time - rising edge time.
    if (reset_timer< 0)reset_timer += 0xFFFF; //if for some reason the value of the channel is less than 0(due to timer restart) then just take the complement.
    TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P; //reset the CCER to detect the rising edge by taking bitwise AND with the compliment of CC1P(doesn't touch anything else but resests
  }                                         //the bit we care about.
}

void setup_receiver_channels()
{
  Timer2.attachCompare1Interrupt(handler_channel_1);//the timer 2 has it's first pin connected to PA0
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE ;

  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER2_BASE->CCMR2 = 0b100000001; //Register is set like this due to a bug in the define table (30-09-2017)
  TIMER2_BASE->CCER = TIMER_CCER_CC1E ;

  TIMER2_BASE->PSC = CLOCK_SPEED;
  TIMER2_BASE->ARR = 0xFFFF;
  TIMER2_BASE->DCR = 0;
}

void IO_init()
{
  setup_esc_control();
  setup_receiver_channels();
}