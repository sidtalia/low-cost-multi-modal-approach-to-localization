//probably half decent efficiency math code. better than standard but probably not the best.

#include"SIDMATH.h"


float anglecalcy(float x1,float x2,float y1,float y2)  //everything is inline because fuck you thats why
{
  float angle = RAD2DEG*atan2((y2-y1),(x2-x1));
  if(angle<0)
  {
    angle += 360;
  }
  return angle;
}//215us

float distancecalcy(float y1,float y2,float x1,float x2,int i)
{
  float delX = (x2-x1);
  float delY = (y2-y1);
  delX *= delX;
  delY *= delY;
  if(i==1)
  {
    return  111692.84*sqrt(delX + delY);   //distance between 2 points
  }
  else
  {
    return sqrt(delX + delY);    //distance directly in meters when input is in meters
  }
}//70us for calculating in meters.

float mod(float a)  //taking mod of a number
{
  if(a<0)
  {
    return -a;
  }
  return a;
}

//this is a psuedo kalman filter. its a quick and dirty method of getting the position estimates.
float Kalman(float gpscord,float gpsError,float estimate,uint8_t trustInEstimate)   //used in localization tab
{
  float estimateError=(50.0/float(trustInEstimate));     //biasing was changed to 30 on 1/4/17 (not the american date standard.)
  //estimateError*=estimateError; //squaring the estimate error because i want the trust to rise very quickly if surface quality is good and fall very quickly if it is lower than 20)
  float KG=(estimateError/(estimateError+gpsError));
  return (KG*gpscord+(1-KG)*estimate);  
}//95us


void Fuse(float &A, float &A_Error, float &B, float &B_Error)
{
  float KG = A_Error/(A_Error + B_Error);
  A_Error *= (1-KG);
  B_Error *= (1-KG); //reduce the error.
  A = (1-KG)*B + KG*A;
  B = A;
}

float depress(float a,float k)               //used to depress accelgyro values
{
  return (a*a*a*a)/(k+a*a*a*a);
}//74.5us 

float my_asin(float a)
{
  return a*(1+(0.5*a*a)); //35us
}

float my_cos(float a)
{
  int factor;
  if(a>6.28) //in case the values are outside [0,2pi]
  {
    factor = int(a*0.15923);
    a -= float(factor*6.28); //rudimentary implementation of a "modulus"(%) operator for floating points.
  }
  if(a<0)
  {
    factor = 1-int(a*0.15923);
    a += float(factor*6.28);
  }
  if(a>1.57&&a<4.71)
  {
    a = 3.14-a;
    return ((0.41*a*a)-1);
  }
  if(a>=4.17)
  {
    a -=6.28;
  }
  return (1-(0.42*a*a)); //25us
} // 50us


float my_sin(float a)
{
  return my_cos(a-1.57); //I are smart.
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


float spike(float mean, float x)
{
  float i = mod(mean - x);
  return 0.05/(1+i);
}
