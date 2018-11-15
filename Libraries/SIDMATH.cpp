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

float get_T(float V,float X1,float Y1,float X2,float Y2,float X3,float Y3,float X4,float Y4,float d,float dt)
{
  float L1,L2,L3,L;
  L1 = distancecalcy(Y1,Y2,X1,X2,0);
  L2 = distancecalcy(Y2,Y3,X2,X3,0);
  L3 = distancecalcy(Y3,Y4,X3,X4,0);
  L = L1+L2+L3;
  L = 0.5*(L+d);
  float parameter = 16*dt*V/L; //approximate value of T. Yes I know that the distances are not uniform,
                              // however, when it comes to the constrained case of a car, the ratio of 
                              //distance covered in 2 cycles to the approximate total arc length gives 
                              // a *fairly accurate*(for our use case) approximation of the parameter t
                              //which can then be used to estimate the required steering angle and speed.
  if(parameter>1)
  {
    return 1;
  }
  return parameter;
}//255us 
 
float ROC(float X1,float Y1,float X2,float Y2,float X3,float Y3,float X4,float Y4,float t,float max_Acceleration)
{
  float KX1,KX2,KX3,KY1,KY2,KY3;
  float C;
  float delX,delY,del2X,del2Y;
  float denominator;
  KX1 = 9*X2 + 3*X4 - 3*X1 - 9*X3; //54
  KY1 = 9*Y2 + 3*Y4 - 3*Y1 - 9*Y3; //54
  KX2 = 6*X1 - 12*X2 + 6*X3; //39
  KY2 = 6*Y1 - 12*Y2 + 6*Y3; //39
  KX3 = 3*(X2 - X1); //15
  KY3 = 3*(Y2 - Y1); //15
  delX = t*t*KX1 + t*KX2 + KX3; //39
  delY = t*t*KY1 + t*KY2 + KY3; //39
  del2X = 2*t*KX1 + KX2; //24
  del2Y = 2*t*KY1 + KY2; //24
  denominator = delX*delX + delY*delY; //24
  denominator *= denominator*denominator; //17
  denominator = sqrt(denominator); //35
  C = ((delX*del2Y) - (delY*del2X)); //24
  C /= denominator; //30
  
  // correction = 14.6115*C; //8.5 . atan of small values is almost the same as the value.(tan(x) = x for small x) 
  // Expected_YR = 57.3*V*C; // V is velocity, C is 1/ROC, 57.3->takes radians/s to deg/s 
  // yaw_Compensation = Expected_YR - yawRate;//6.5us
  if(mod(C)<0.001)
  {
    if(C>0)
    {
      C=0.001;
    }
    else
    {
      C = -0.001;
    }
  }
  return C; 
} //540us.

void get_Intermediate_Points(float slope1,float slope2,float X1,float X2,float Y1,float Y2,float d)
{
  float int1[2],int2[2];//intermediate points
  int1[0] = X1 + 0.4*my_cos(slope1*DEG2RAD)*d; //57us
  int1[1] = Y1 + 0.4*my_sin(slope1*DEG2RAD)*d; //67
  int2[0] = X2 - 0.4*my_cos(slope2*DEG2RAD)*d; //57us
  int2[1] = Y2 - 0.4*my_sin(slope2*DEG2RAD)*d; //67
}//248us

void generate_Slopes(coordinates c[],int n) 
{
  for(int i = 1;i<n-1;i++)
  {
    c[i].slope = (anglecalcy( c[i-1].retXO(),c[i].retXO(),c[i-1].retYO(),c[i].retYO() )+anglecalcy( c[i].retXO(),c[i+1].retXO(),c[i].retYO(),c[i+1].retYO() ))*0.5;
  }
}

float spike(float mean, float x)
{
  float i = mod(mean - x);
  return 0.05/(1+i);
}
