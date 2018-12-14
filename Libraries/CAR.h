#ifndef _CAR_H_
#define _CAR_H_

#include"Arduino.h"
#include"INOUT.h"
#include"SIDMATH.h" //will need some math functions here yo.
#include"PARAMS.h"


#define WHEELBASE (float)0.254 //wheelbase in meters
#define STEERING_MAX 30 //30 degrees max steering 
#define STEERING_PULSE_LOCK2CENTER 372 //pulse difference between center steering and steering at full lock. they might be different for your car.
#define ABSOLUTE_MAX_ACCELERATION 10 //10m/s*s. This is the absolute maximum acceleration the car can handle
#define MAX_ACCELERATION (float)0.7*ABSOLUTE_MAX_ACCELERATION
//maximum safe acceleration (omnidirectional) that the car can handle without any problems whatsoever 
//for example braking and turning at the same time. keep this as ~70% of maximum friction force that the tires can provide
#define SAFE_DECELERATION -0.4*MAX_ACCELERATION //the threshold deceleration for us to start braking. This is ~1/2 of the MAX_ACCELERATION 
		//because weight shifting under braking is an absolute bitch when it comes to mid-CG rear wheel braking cars (mine for example)

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
			//I will make it a little more generic so that ya'll can just put in your servo specs
			// (slightly inaccurate open + closed) loop control > mathematical model of the system. Why? because life is full of uncertainties.


float Curvature_To_Angle(float C)
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
void driver(float C[2], float braking_distance, float V, float yawRate, float Ax, float Ay, uint8_t MODE, float inputs[8]) // function to operate the servo and esc.
{
	float deceleration, backoff, resultant, correction, yaw_Compensation, V_target, V_error;
	int throttle = THROTTLENULL,steer = STEERINGNULL; //default values.
	
	resultant = sqrt(Ax*Ax + Ay*Ay); //resultant horizontal acceleration
	correction = Curvature_To_Angle(C[0]); //convert radius of curvature to steering angle. this actually depends on wheelbase.
	yaw_Compensation = RAD2DEG*V*C[0] - yawRate;//Expected Yaw rate - measured Yaw rate. This is basically the V_error fed to the closed loop steering control
	
	V_target = sqrt(MAX_ACCELERATION/mod(C[0])); //target maximum velocity

	if(C[1] < C[0]) //if the turning gets sharper, we have to brake a little early. if the turning doesn't get any sharper then no premature braking needed
	{
		//braking distance is how much distance we have in front of us to slow down.
		float V1, deceleration_required;
		V1 = sqrt(MAX_ACCELERATION/mod(C[1]));//max speed at the sharpest portion of the turn
		
		deceleration_required = V*(V1-V)/braking_distance; //dv/dt = (dv/dx)*(dx/dt) :P. faster than v^2 = u^2 + 2.a.S. note that this value will be -ve
		//TODO : make deceleration required as positive. 
		if(deceleration_required < SAFE_DECELERATION)//retardation required is more than the safe braking limit.
		{
			V_target = V1; //set that velocity as the setpoint. This keeps happening until the deceleration_required is less than safety limit.
		}
	}

	if(MODE == MODE_PARTIAL)
	{
		backoff = 20*(resultant - MAX_ACCELERATION);
		if(backoff<0)
			backoff = 0;
		if(Ay>0) //Ay lmao we speeding up homie.
			throttle = inputs[0] - backoff;
		else
			throttle = inputs[0] + backoff;

		steer = inputs[1] + STEERING_CLOSED_GAIN*yaw_Compensation;
		
		set_Outputs_Raw(throttle,steer); //defined in INOUT

		return; //return from here
	}
	else if(MODE == MODE_MANUAL)
	{
		set_Outputs_Raw(int(inputs[0]),int(inputs[1])); //pass through
		return ; //GTFO 
	}

	else if(MODE == LUDICROUS)// should I rename this mode to be "LUCIFEROUS"?
	{
		if(V_target>VMAX)//if maximum possible speed is more than the speed limit. This will happen on straights
		{
			V_target = VMAX; //speed setpoint is the speed limit.
		}
	} 
	else if(MODE == CRUISE)
	{
		if(V_target>SAFE_SPEED)
		{
			V_target = SAFE_SPEED;
		}
	}

	else if(MODE == MODE_STOP)
	{
		V_target = 0;
	}
	V_error = V_target - V; //speed setpoint - current speed
	if(V_error>=0)//Required velocity is greater than the current velocity
	{
		if(Ay>0)//if we are already speeding up
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
		throttle = limiter(THROTTLENULL + OPEN_GAIN*V_target + CLOSED_GAIN*V_error - backoff); //open loop + closed loop control for reducing V_error at a faster rate.
	}
	if(V_error<0)//required velocity is less than current velocity.
	{
		if(Ay<0) //if we are already slowing down
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
		deceleration = V_error*50 - Ay;//required deceleration - measured deceleration 
		if(deceleration<-10) //prevent reset windup
		{
			deceleration = -10;
		}
		throttle = limiter(THROTTLENULL + BRAKE_GAIN*deceleration + backoff);
	}

	steer = limiter(STEERINGNULL + STEERING_OPEN_GAIN*correction + STEERING_CLOSED_GAIN*yaw_Compensation); //open loop + closed loop control 

	set_Outputs_Raw(throttle,steer); //defined in INOUT

	return; //good practice

}//140us


#endif
