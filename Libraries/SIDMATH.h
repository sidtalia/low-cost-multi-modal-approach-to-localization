#ifndef _SIDMATH_H_
#define _SIDMATH_H_

#include"Arduino.h"
#include"math.h"

#define DEG2RAD (float) 0.0174533
#define RAD2DEG (float) 57.2958
#define DEG2METER (double) 111392.84
#define METER2DEG (double) 1/DEG2METER
#define GRAVITY (float) 9.8
#define G_INVERSE (float) (1/GRAVITY)
#define G_SQUARED (float) (GRAVITY*GRAVITY)
#define M_2PI (float) 6.283185
#define M_3PIB2 (float) 4.712388
#define M_PIB2 (float) 1.570796
#define M_2PI_DEG (float) 360.0
#define M_PIB2_DEG (float) 90.0
#define M_PI_DEG (float) 180.0
#define M_2PI_INV (float) 0.1591549
#define M_PI_INV (float) 0.3183098

#define exp_spike_k (float) 0.2
#define spike_k (float) 0.2
#define spike_d (float) 1
#define exp_spike_c (float)400.0

static inline __always_inline float fast_sqrt(float x)//inversion of fast inverse square root. :P
{
  x = fabs(x); //avoid naans.
  long i;
  float x2, y;
  const float threehalfs = 1.5f;

  x2 = x*0.5f;
  y  = x;
  i  = * ( long * ) &y;                       // evil floating point bit level hacking
  i  = 0x5f3759df - ( i >> 1 );               // what the fuck? 
  y  = * ( float * ) &i;
  y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//  y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

  return float(1/y);
}
 
inline __always_inline float distancecalcy(float y1,float y2,float x1,float x2,int i)
{
  float delX = (x2-x1);
  float delY = (y2-y1);
  delX *= delX;
  delY *= delY;
  if(i==1)
  {
    return  DEG2METER*fast_sqrt(delX + delY);   //distance between 2 points
  }
  else
  {
    return fast_sqrt(delX + delY);    //distance directly in meters when input is in meters
  }
}//70us for calculating in meters.

float anglecalcy(float x1,float x2,float y1,float y2); 
//this is a psuedo kalman filter. its a quick and dirty method of getting the position estimates.
float gpsOpFlowKalman(float gpscord,float gpsError,float estimate,uint8_t trustInEstimate);

void Fuse(float &A, float &A_Error, float &B, float &B_Error);

float depress(float a,float k);

inline __always_inline float my_asin(float a)
{
  return a*(1+(float(0.5)*a*a*a*a)); //55us still thrice as fast.
}

inline __always_inline float my_cos(float a)
{
  int factor;
  if(a>M_2PI) //in case the values are outside [0,2pi]
  {
    factor = int(a*M_2PI);
    a -= float(factor*M_2PI); //rudimentary implementation of a "modulus"(%) operator for floating points.
  }
  if(a<0)
  {
    factor = 1-int(a*M_2PI_INV);
    a += float(factor*M_2PI);
  }
  if(a>M_PIB2 && a<M_3PIB2)
  {
    a = M_PI-a;
    return ((float(0.42)*a*a)-1);
  }
  if(a>=M_3PIB2)
  {
    a -=M_2PI;
  }
  return (1-0.42f*a*a); //25us
} // 50us


inline __always_inline float my_sin(float a)
{
  return my_cos(a- M_PIB2); //I are smart.
}//57us



//float my_tan(float x)
//{
//  float x2,ans; 
//  if(x>1.57&&x<4.71)
//  {
//    x -= 3.14;
//  }
//  if(x>=4.71)
//  {
//    x -= 6.28;
//  }
//  x2 = x*x;
//  ans = x*(1 + 0.333*x2 + 0.1333*x2*x2);
//  if(x>1.45||x<-1.45)
//  {
//    ans *= ans;
//  }
//  if(x>1.55||x<-1.55)
//  {
//    ans *= ans;
//  }
//  if(x>1.56||x<-1.56)
//  {
//    ans *= ans;
//  }
//  return ans;
//} //88us 


inline __always_inline float spike(float center, float x)
{
  float i = fabs(center - x);
  return spike_k/(spike_d + i);
}

inline __always_inline float exp_spike(float center, float x)
{
  float i = fabs(center - x);
  return exp_spike_k/(spike_d + exp_spike_c*i);
}

inline __always_inline void Sanity_Check(float limit, float &input)
{
 if(input>limit)
 {
  input = limit;
 }
 if(input< -limit)
 {
  input = -limit;
 }
 return;
}

#endif 
