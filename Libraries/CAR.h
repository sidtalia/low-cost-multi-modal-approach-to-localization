#ifndef _CAR_H_
#define _CAR_H_

#include"Arduino.h"
#include"INOUT.h"
#include"SIDMATH.h" //will need some math functions here yo.
#include"PARAMS.h"


#define WHEELBASE (float)0.254 //wheelbase in meters
#define STEERING_MAX 25 //25 degrees max steering 
#define STEERING_PULSE_LOCK2CENTER 372 //pulse difference between center steering and steering at full lock. they might be different for your car.
#define ABSOLUTE_MAX_ACCELERATION 10 //10m/s*s. This is the absolute maximum acceleration the car can handle
#define MAX_ACCELERATION (float) (0.7*ABSOLUTE_MAX_ACCELERATION)
//maximum safe acceleration (omnidirectional) that the car can handle without any problems whatsoever 
//for example braking and turning at the same time. keep this as ~70% of maximum friction force that the tires can provide
#define SAFE_DECELERATION -0.4*MAX_ACCELERATION //the threshold deceleration for us to start braking. This is ~1/2 of the MAX_ACCELERATION 
		//because weight shifting under braking is an absolute bitch when it comes to mid-CG rear wheel braking cars (mine for example)

#define maxValue (int) 2000
#define minValue (int) 1000
#define STEERINGNULL 1500
#define THROTTLENULL 1500
#define VMAX (float) 17.0 //maximum speed that my car can hit.
#define SAFE_SPEED (float) 2.5 //safe cruise speed defined as 2.5 m/s. Basically this is how fast I can run after the car if something went wrong.
#define OPEN_GAIN (float) 25.0 //open loop throttle gain. I am assuming a linear relationship between throttle input and speed
#define CLOSED_GAIN (float) 10 //closed loop gain. This helps me deal with the fact that the relation between throttle and speed is not exactly linear
#define BRAKE_GAIN (float) (250/MAX_ACCELERATION) //brake gain. Explained later.
#define STEERING_OPEN_GAIN (float)(STEERING_PULSE_LOCK2CENTER/STEERING_MAX) //open loop gain between steering angle and pwm value. The relationship is assumed to be linear but is not and 
#define STEERING_CLOSED_GAIN (float)0.7 //hence I use a closed loop gain as well to control the steering. This gain is based on "how fast can your steering turn lock to lock?"
			//I will make it a little more generic so that ya'll can just put in your servo specs
			// (slightly inaccurate open + closed) loop control > mathematical model of the system. Why? because life is full of uncertainties.
#define STEERING_TRUST (float) 0.8
#define STEERING_TRUST_1 (float) 0.2

#define CRITICAL_YAW (float) 90 //at 1 g, given a 1 m turning radius, yaw rate is roughly 180 degrees
#define VARIABLE_GAIN (float) (1/CRITICAL_YAW)
#define LPF_GAIN_THROTTLE (float) 1.0f/128.321336 //this is for 200Hz sample rate.
#define C1_THROTTLE (float) 0.984414
#define OPEN_GAIN_INVERSE (float) (1/OPEN_GAIN)

#define LEARNING_RATE (float) dt

float yaw_correction(float input)
{
	return STEERING_CLOSED_GAIN*(1+VARIABLE_GAIN*fabs(input))*input;
}

float check(float input,float threshold)
{
	if(input < threshold && input>=0)
	{
		input = threshold;
	}
	if(input > -threshold && input<0)
	{
		input = -threshold;
	}
	return input;
}

class controller
{
	long stamp,showboat_timer;
	int throttle,steer;
	float speed,speed_Error,roc,La,yR,last_Speed,last_Throttle,Ha,Process_noise,ground_Speed;
	float steering_bias;
	float xA[2][2],yA[2][2];//for low pass filter
	bool IsLearning,IsOversteering;
	public:
	float feedback_factor;
	bool control_check;
	controller()
	{
		stamp = millis(); //get time stamp
		speed = 0;
		speed_Error = 0;
		Process_noise = INITIAL_NOISE;
		last_Throttle = THROTTLENULL;
		last_Speed = 0;
		feedback_factor = 1;
		IsLearning = true;
		IsOversteering = false;
		ground_Speed = 0;
		roc = 1e6;
		steering_bias=0;
		control_check = false;
	}

	float LPF(int i,float x)
	{
	  xA[i][0] = xA[i][1]; 
	  xA[i][1] = x*LPF_GAIN_THROTTLE;
	  yA[i][0] = yA[i][1]; 
	  yA[i][1] =   (xA[i][0] + xA[i][1]) + ( C1_THROTTLE* yA[i][0]); // first order LPF to predict new speed.
	  return yA[i][1];
	}

	float Curvature_To_Angle(float C)
	{
		if(fabs(C)<=1.1)
		{
			return RAD2DEG*WHEELBASE*C; //tan(x) = x for small values of x. bite me.
		}
		else
		{
			return RAD2DEG*atanf(WHEELBASE*C);
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

	void get_model(float array[3])
	{
		array[0] = speed;
		array[1] = speed_Error;
		array[2] = roc;
	}

	void feedback(float est_speed,float est_error)
	{
		ground_Speed = est_speed; //get external speed : is injected into the LPF model when the car is under retardation
		Process_noise += DECAY_RATE;
		if(est_speed>MAX_LEARNING_SPEED or est_speed < MIN_LEARNING_SPEED or throttle<THROTTLE_OFFSET or fabs(est_speed-speed)>speed)
		{
			IsLearning = false;
			return;
		}
		IsLearning = true;

		float gain = Process_noise/(Process_noise + est_error);
		gain = check(gain,MAX_GAIN); // prevent jerk in the gain when learning is restarted.
		Process_noise *= (1-gain);
		feedback_factor += (speed - est_speed)*gain*LEARNING_RATE; //note that load has an inverse relationship with the speed.
		feedback_factor = check(fabs(feedback_factor),MIN_FEEDBACK_FACTOR); //prevent values smaller than 0.7
		return;
	}

	void calc_speed()
	{
		roc = DEG2RAD*(steer - STEERINGNULL)/STEERING_OPEN_GAIN;
		roc = check(roc,1e-3);//the steering angle must not be less than 1e-3 radians or 0.0573 degrees effectively a radius of more than 225 meters will not be accepted. This prevents the possibility of running into NaNs 
		roc = WHEELBASE/tanf(roc);//this is how you reuse variables kids. roc does not have to be very accurate but it does have to be stable.
		// if(throttle - last_Throttle > THROTTLE_DELTA)
		// {
		// 	throttle = last_Throttle + THROTTLE_DELTA;
		// }
		// last_Throttle = throttle;
		
		if(fabs(yR) > 0.17)
		{
			float dummy = roc;
			float meas_roc = -La/(yR*yR);
			roc = STEERING_TRUST*(roc+steering_bias) + STEERING_TRUST_1*meas_roc;
			steering_bias = roc - dummy; //this fixes the problem of understeer/oversteer throwing off roc estimates.
			if(dummy*meas_roc<0)
			{
				roc = meas_roc;//TODO : CHECK IF THIS WAS A CAUSE.
				IsOversteering = true;
			}
			else
			{
				IsOversteering = false;
			}
		}
		else
		{
			steering_bias=0; //reset bias to 0 to prevent problems.
		}
		if(throttle<THROTTLE_OFFSET)
		{
			speed = LPF(0,ground_Speed);//inject the estimated speed into the LPF to prime it until the car is on throttle again.
			last_Speed = speed;
			speed_Error = 1e6;

		}	
		else
		{
			if(throttle>THROTTLE_MAX)
			{
				throttle = THROTTLE_MAX;
			}
			float dummy = (throttle - THROTTLE_OFFSET)*THROTTLE_RANGE_INV;
			speed = A0*pow(dummy,3) + A1*pow(dummy,2) + A2*dummy + A3;
			float load = fabs(La*COG/roc);
			speed /= (feedback_factor + load/ROLL_RES);
			speed = LPF(0,speed);
			if(speed>MIN_LEARNING_SPEED)
			{
				Sanity_Check(MAX_ACCELERATION*CONTROL_TIME+last_Speed, speed);
			}
			last_Speed = speed;

			IsLearning|IsOversteering ? speed_Error = 1e6 : speed_Error = fabs(max(speed,3.0f));// max(1e2*fabs(speed - dummy),1.0f);//error is proportional to the target - estimated speed by Low pass filter model.
		}//changes made on 5/5/19
	}

	//the following function takes the required Curvature, the speed of the car, the measured yaw Rate, measured horizontal accelerations and car's MODE
	void driver(float C[2], float braking_distance, float V, float yawRate, float Ax, float Ay, uint8_t MODE, float inputs[8]) // function to operate the servo and esc.
	{
		La = Ax;//transfer information
		yR = yawRate*DEG2RAD;
		Ha = LPF(1,Ay);
		if( millis() - stamp < CONTROL_TIME)
		{
			return; //maintain a defined control frequency separate from observation frequency. Only used here because synchronising the time stamps across 2 objects would be difficult(sort of. could fix. create a pull request if you want it fixed)
		}

		float deceleration, backoff, resultant, correction, yaw_Compensation, V_target, V_error;
		throttle = THROTTLENULL;
		steer = STEERINGNULL; //default values.
		
		resultant = fast_sqrt(Ax*Ax + Ay*Ay); //resultant horizontal acceleration
		correction = Curvature_To_Angle(C[0]); //convert radius of curvature to steering angle. this actually depends on wheelbase.
		yaw_Compensation = RAD2DEG*V*C[0] - yawRate;//Expected Yaw rate - measured Yaw rate. This is basically the V_error fed to the closed loop steering control
		
		V_target = fast_sqrt(MAX_ACCELERATION/fabs(C[0])); //target maximum velocity

		if(C[1] < C[0]) //if the turn gets sharper, we have to brake a little early. if the turning doesn't get any sharper then no premature braking needed
		{
			//braking distance is how much distance we have in front of us to slow down.
			float V1, deceleration_required;
			V1 = fast_sqrt(MAX_ACCELERATION/fabs(C[1]));//max speed at the sharpest portion of the turn TODO : OPTIMIZE
			
			deceleration_required = V*(V1-V)/braking_distance; //dv/dt = (dv/dx)*(dx/dt) :P. faster than v^2 = u^2 + 2.a.S. note that this value will be -ve
			//TODO : make deceleration required as positive. 
			if(deceleration_required < SAFE_DECELERATION)//retardation required is more than the safe braking limit.
			{
				V_target = V1; //set that velocity as the setpoint. This keeps happening until the deceleration_required is less than safety limit.
			}
		}

		if(MODE == MODE_PARTIAL)
		{
			backoff = 0;//20*(resultant - MAX_ACCELERATION);
			if(backoff<0)
				backoff = 0;
			if(Ay>0) //Ay lmao we speeding up homie.
				throttle = inputs[2] - backoff;
			else
				throttle = inputs[2] + backoff;

			steer = inputs[0] + yaw_correction(yaw_Compensation) ;
			
			calc_speed();
			set_Outputs_Raw(throttle,steer); //defined in INOUT

			return; //return from here
		}
		else if(MODE == MODE_MANUAL)
		{
			throttle = inputs[2];
			steer = inputs[0];
			calc_speed();
			set_Outputs_Raw(throttle,steer); //pass through
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
			if(V<0.5)
			{
				throttle = THROTTLENULL;
				steer = inputs[0];
				calc_speed();
				set_Outputs_Raw(throttle,steer);
				return; //gtfo;
			}
		}

		else if(MODE == MODE_STANDBY||MODE == MODE_CONTROL_CHECK)
		{
			throttle = THROTTLENULL;
			steer = inputs[0];
			if(MODE==MODE_CONTROL_CHECK||control_check)
			{
				if(!control_check)
				{
					showboat_timer = millis();
					control_check = true;
				}
				else if(control_check && millis()-showboat_timer>1000)
				{
					control_check = false;
				}
				float t = (millis()-showboat_timer)*1e-3;
				steer = STEERINGNULL + STEERING_PULSE_LOCK2CENTER*sinf(M_2PI*t);
			}
			calc_speed();
			set_Outputs_Raw(throttle,steer); //pass through but only for steering
			return ; //GTFO 
		}

		V_error = V_target - V; //speed setpoint - current speed
		if(V_error>=0)//Required velocity is greater than the current velocity
		{
			if(Ay>0)//if we are already speeding up
			{
				backoff = 20*(resultant - MAX_ACCELERATION);//if the resultant is more than the max acceleration, back the fuck off.
				if(backoff<0)				//as you may have noted, MAX_ACC is the safe maximum g force the car can handle. it is not the absolute maximum
				{							//therefore the resultant can be higher. If the resultant is higher the max acceleration, the car starts backing
					backoff = 0;			//off from the throttle or the brakes to keep the car within it's limit of grip
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

		steer = limiter(STEERINGNULL + STEERING_OPEN_GAIN*correction + yaw_correction(yaw_Compensation) ); //open loop + closed loop control 

		calc_speed();
		set_Outputs_Raw(throttle,steer); //defined in INOUT

		return; //good practice

	}//140us
};
#endif
