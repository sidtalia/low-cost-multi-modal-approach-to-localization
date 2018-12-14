#ifndef _INOUT_H_
#define _INOUT_H_

#include"Arduino.h"

#define CLOCK_SPEED 127

// volatile int32_t input_start,input[8],delT,reset_timer,tick=0;
// bool servoWrite = false;
void setup_esc_control();

void setup_receiver_channels();

void IO_init();
void set_Outputs(float throttle, float steering);
void set_Outputs_Raw(int throttle,int steering);
void get_Inputs(float I[8]);

void handler_channel_1(); //PA0

#endif
