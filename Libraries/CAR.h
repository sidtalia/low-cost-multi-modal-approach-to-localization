#ifndef _CAR_H_
#define _CAR_H_

#include"Arduino.h"
#include"INOUT.h"
#include"SIDMATH.h" //will need some math functions here yo.


#define WHEELBASE (float)0.254 //wheelbase in meters
#define STEERING_MAX 30 //30 degrees max steering 
#define STEERING_PULSE_LOCK2CENTER 372 //pulse difference between center steering and steering at full lock. they might be different for your car.
#define ABSOLUTE_MAX_ACCELERATION 10 //10m/s*s. This is the absolute maximum acceleration the car can handle
#define MAX_ACCELERATION (float)0.7*ABSOLUTE_MAX_ACCELERATION
//maximum safe acceleration (omnidirectional) that the car can handle without any problems whatsoever 
//for example braking and turning at the same time. keep this as ~70% of maximum friction force that the tires can provide

#define maxValue (int)2000
#define minValue (int)1000
#define STEERINGNULL 1500
#define THROTTLENULL 1500
#define VMAX (float)17.0 //maximum speed that my car can hit.
#define SAFE_SPEED (float)2.5 //safe cruise speed defined as 2.5 m/s. Basically this is how fast I can run after the car if something went wrong.
#define OPEN_GAIN (float)20.0 //open loop throttle gain. I am assuming a linear relationship between throttle input and speed
#define CLOSED_GAIN (float)10 //closed loop gain. This helps me deal with the fact that the relation between throttle and speed is not exactly linear
#define BRAKE_GAIN (float)250/MAX_ACCELERATION //brake gain. Explained later.
#define STEERING_OPEN_GAIN (float)STEERING_PULSE_LOCK2CENTER/STEERING_MAX //open loop gain between steering angle and pwm value. The relationship is assumed to be linear but is not and 
#define STEERING_CLOSED_GAIN (float)0.7 //hence I use a closed loop gain as well to control the steering. This gain is based on "how fast can your steering turn lock to lock?"
			//I will make it a little more generic so that ya'll can just put in your servo 
			// (slightly inaccurate open + closed) loop control > mathematical model of the system. Why? because life is full of uncertainties.

#define LUDICROUS 0x04
#define CRUISE 0x03
#define MANUAL_P 0x02
#define MANUAL 0x01
#define STOP 0x00


float Curvature_To_Angle(float C);
{
	if(mod(C)<1)
	{
		return RAD2DEG*WHEELBASE*C; //tan(x) = x for small values of x. bite me.
	}
	else
	{
		return RAD2DEG*atan(WHEELBASE*C);
	}
}


int limiter(float input) // prevent the values from going outside the 2000-1000us range
{
	if(input>maxValue)
	{
		return maxValue;
	}
	if(input<minValue)
	{
		return minValue;
	}
	return int(input);
}


//the following function takes the required Curvature, the speed of the car, the measured yaw Rate, measured horizontal accelerations and car's MODE
void driver(float C, float V, float yawRate, float Ax, float Ay, uint8_t MODE) // function to operate the servo and esc.
{
	float deceleration, backoff, resultant, correction, yaw_Compensation, Vmax;
	
	resultant = sqrt(Ax*Ax + Ay*Ay); //resultant horizontal acceleration
	correction = Curvature_To_Angle(C); //convert radius of curvature to steering angle. this actually depends on wheelbase.
	yaw_Compensation = RAD2DEG*V*C; - yawRate;//Expected Yaw rate - measured Yaw rate. This is basically the error fed to the closed loop steering control
	Vmax = sqrt(MAX_ACCELERATION/mod(C)); //max speed that we can hit. TODO : incorporate some way to slow down for a sharp turn beforehand.


	if(MODE == LUDICROUS)
	{
		if(Vmax>VMAX)//if maximum possible speed is more than the speed limit. This will happen on straights
		{
			Vmax = VMAX; //speed setpoint is the speed limit.
		}
	} 
	else if(MODE == CRUISE)
	{
		if(Vmax>SAFE_SPEED)
		{
			Vmax = SAFE_SPEED;
		}
	}
	else if(MODE == STOP)
	{
		Vmax = 0;
	}
	error = Vmax - V; //speed setpoint - current speed
	if(error>=0)//Required velocity is greater than the current velocity
	{
		if(Ha>0)//if we are already speeding up
		{
			backoff = 20*(resultant - MAX_ACCELERATION);//if the resultant is more than the max acceleration, back the fuck off.
			if(backoff<0)				//as you may have noted, MAX_ACC is the safe maximum g force the car can handle. it is not the absolute maximum
			{							//therefore the resultant can be higher. If the resultant is higher the max acceleration, the car starts backing
				backoff = 0;			//off from the throttle or the breaks to keep the car within it's limit of grip
			}
		}
		else
		{
			backoff = 0;
		}
		throttle = limiter(THROTTLENULL + OPEN_GAIN*Vmax + CLOSED_GAIN*error - backoff); //open loop + closed loop control for reducing error at a faster rate.
	}
	if(error<0)//required velocity is less than current velocity.
	{
		if(Ha<0) //if we are already slowing down
		{
			backoff = 20*(resultant - MAX_ACCELERATION);//same logic as before
			if(backoff<0)
			{
				backoff = 0;
			}
		}
		else
		{
			backoff = 0;
		}
		//TODO : improve this on the basis of how much braking distance we got left
		deceleration = error*50 - Ha;//required deceleration - measured deceleration 
		if(deceleration<-10) //prevent reset windup
		{
			deceleration = -10;
		}
		throttle = limiter(THROTTLENULL + BRAKE_GAIN*deceleration + backoff);
	}

	steer = limiter(STEERINGNULL + STEERING_OPEN_GAIN*correction + STEERING_CLOSED_GAIN*yaw_Compensation); //open loop + closed loop control 

	set_Outputs_Raw(throttle,steer); //defined in INOUT
}//140us


