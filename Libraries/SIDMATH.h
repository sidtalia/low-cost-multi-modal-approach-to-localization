#ifndef _SIDMATH_H_
#define _SIDMATH_H_

#include"Arduino.h"
#include"math.h"

#define DEG2RAD (float)0.0174533
#define RAD2DEG (float)57.2958
#define DEG2METER (double)111392.84
#define METER2DEG (double)1/DEG2METER
#define GRAVITY (float)9.3
#define G_INVERSE (float)1/GRAVITY
#define G_SQUARED (float)GRAVITY*GRAVITY

float anglecalcy(float x1,float x2,float y1,float y2);  

float distancecalcy(float y1,float y2,float x1,float x2,int i);

float mod(float a);  //taking mod of a number
//this is a psuedo kalman filter. its a quick and dirty method of getting the position estimates.
float gpsOpFlowKalman(float gpscord,float gpsError,float estimate,uint8_t trustInEstimate);

void Fuse(float &A, float &A_Error, float &B, float &B_Error);

float depress(float a,float k);

float my_asin(float a);

float my_cos(float a);

float my_sin(float a);

float spike(float mean, float x);

float exp_spike(float mean, float x);

#endif 
