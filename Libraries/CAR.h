#ifndef _CAR_H_
#define _CAR_H_

#include"Arduino.h"
#include"INOUT.h"
#include"SIDMATH.h" //will need some math functions here yo.
#include"PARAMS.h"


#define WHEELBASE (float) 0.254f //wheelbase in meters
#define CG_HEIGHT (float) 0.03f //assumed cg height. should be about the same as this.
#define FR_ratio (float) 0.52f //assumed ratio of CG distance from front to the wheelbase
#define H_WB_ratio (float) CG_HEIGHT/(WHEELBASE*FR_ratio)
#define H_GBR_INV (float) CG_HEIGHT/(GRAVITY*WHEELBASE*(1 - FR_ratio))

#define STEERING_MAX 25 //25 degrees max steering 
#define STEERING_PULSE_LOCK2CENTER 372 //pulse difference between center steering and steering at full lock. they might be different for your car.
// #define ABSOLUTE_MAX_ACCELERATION 7 //10m/s*s. This is the absolute maximum acceleration the car can handle
// #define MAX_ACCELERATION (float) (0.7*ABSOLUTE_MAX_ACCELERATION)
// #define MAX_ACCELERATION_SQ (float) MAX_ACCELERATION*MAX_ACCELERATION
// //maximum safe acceleration (omnidirectional) that the car can handle without any problems whatsoever 
// //for example braking and turning at the same time. keep this as ~70% of maximum friction force that the tires can provide
// #define SAFE_DECELERATION -0.3*MAX_ACCELERATION //the threshold deceleration for us to start braking. This is ~1/2 of the MAX_ACCELERATION 
// 		//because weight shifting under braking is an absolute bitch when it comes to mid-CG rear wheel braking cars (mine for example)
#define DEFAULT_ABS_MAX_ACC (float) 10.0f
#define DEFAULT_ABS_MIN_ACC (float) 5.0f
#define DRIFT_RATIO_CUTOFF (float) 0.1f
#define DEFAULT_MU (float) 1.0f
#define DONT_CARE_RADIUS (float) WP_CIRCLE + 0.5f

#define maxValue (int) 2000
#define minValue (int) 1000
#define STEERINGNULL 1500
#define THROTTLENULL 1500
#define VMAX (float) 10.0 //maximum speed that the car is allowed to hit.
#define SAFE_SPEED (float) 4.0 //safe cruise speed defined as 2.5 m/s. Basically this is how fast I can run after the car if something went wrong.
#define OPEN_GAIN (float) 25.0 //open loop throttle gain. I am assuming a linear relationship between throttle input and speed
#define CLOSED_GAIN (float) 10.0f //closed loop gain. This helps me deal with the fact that the relation between throttle and speed is not exactly linear
#define DEFAULT_BRAKE_GAIN (float) (500/DEFAULT_ABS_MAX_ACC) //brake gain. Explained later.
#define STEERING_OPEN_GAIN (float)(STEERING_PULSE_LOCK2CENTER/STEERING_MAX) //open loop gain between steering angle and pwm value. The relationship is assumed to be linear but is not and 
#define STEERING_OPEN_GAIN_INV (float) 1.0f/STEERING_OPEN_GAIN
#define STEERING_CLOSED_GAIN (float)0.5 //hence I use a closed loop gain as well to control the steering. This gain is based on "how fast can your steering turn lock to lock?"
			//I will make it a little more generic so that ya'll can just put in your servo specs
#define STEERING_TRUST (float) 0.8
#define STEERING_TRUST_1 (float) 0.2

#define CRITICAL_YAW (float) 90 //at 1 g, given a 1 m turning radius, yaw rate is roughly 180 degrees
#define VARIABLE_GAIN (float) (1/CRITICAL_YAW)
#define LPF_THROTTLE_FREQ (float) 1.0f
#define LPF_GAIN_THROTTLE (float) 1.0f/64.6567 //1 Hz LPF for 200Hz sample rate.
#define C1_THROTTLE (float) 0.969067f
#define LPF_GAIN_10 (float) 1.0f/7.31375
#define C1_10 (float) 0.726542
#define OPEN_GAIN_INVERSE (float) (1/OPEN_GAIN) //open loop throttle gain
#define THROTTLE_TIME_CONSTANT (float) 1/(M_2PI*LPF_THROTTLE_FREQ)

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
	float speed,speed_Error,roc,La,yR,last_Speed,last_Throttle,Process_noise,ground_Speed,load;
	float steering_bias;
	float xA[4][2],yA[4][2];//for low pass filter
	bool IsLearning,IsOversteering;
	float S_V_Error;
	float SAFE_DECELERATION,ABSOLUTE_MAX_ACCELERATION,MAX_ACCELERATION,MAX_ACCELERATION_SQ,BRAKE_GAIN;
	float C_Critical, mu, C_Gain, Bruh_G, Bruh;
	bool preemptive;
	public:
	float feedback_factor,Ha;
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
		load = 0;
		S_V_Error = 0;

		ABSOLUTE_MAX_ACCELERATION = DEFAULT_ABS_MAX_ACC;
		MAX_ACCELERATION = 0.6*ABSOLUTE_MAX_ACCELERATION;
		MAX_ACCELERATION_SQ = MAX_ACCELERATION*MAX_ACCELERATION;
		SAFE_DECELERATION = -0.4*MAX_ACCELERATION;
		BRAKE_GAIN = 500.0f/fabs(SAFE_DECELERATION);
		// C_Critical = 0.33f;
		Bruh = DEFAULT_MU*H_WB_ratio;
		Bruh_G = Bruh/(1 + Bruh*Bruh);
		C_Gain = fast_sqrt(Bruh_G/(WHEELBASE*DEFAULT_MU*GRAVITY));
	}

	void adjust_g_force_limits(float drift_ratio)
	{
		ABSOLUTE_MAX_ACCELERATION -= drift_ratio*dt; //reduce absolute limits
		if(ABSOLUTE_MAX_ACCELERATION < DEFAULT_ABS_MIN_ACC)
		{
			ABSOLUTE_MAX_ACCELERATION = DEFAULT_ABS_MIN_ACC;
		}
		//the rest are adjusted accordingly
		MAX_ACCELERATION = ABSOLUTE_MAX_ACCELERATION;
		MAX_ACCELERATION_SQ = MAX_ACCELERATION*MAX_ACCELERATION;
		SAFE_DECELERATION = -0.4*MAX_ACCELERATION;
		BRAKE_GAIN = 500.0f/fabs(SAFE_DECELERATION);
		mu = (ABSOLUTE_MAX_ACCELERATION/DEFAULT_ABS_MAX_ACC)*DEFAULT_MU;
		Bruh = 0.19f*mu*H_WB_ratio;
		Bruh_G = Bruh/(1+Bruh*Bruh);
		C_Gain = fast_sqrt(Bruh_G/(WHEELBASE*mu*GRAVITY));
		// float theta_critical = asinf(1/fast_sqrt(1+k*k)) - atanf(k);
		// C_Critical = tanf(theta_critical)/WHEELBASE;
		return;
	}

	float LPF(int i,float x)
	{
	  xA[i][0] = xA[i][1]; 
	  xA[i][1] = x*LPF_GAIN_THROTTLE;
	  yA[i][0] = yA[i][1]; 
	  yA[i][1] =   (xA[i][0] + xA[i][1]) + ( C1_THROTTLE* yA[i][0]); // first order LPF to predict new speed.
	  return yA[i][1];
	}

	float LPF_10(int i,float x)//10 hz low pass filter
	{
	  xA[i][0] = xA[i][1]; 
	  xA[i][1] = x*LPF_GAIN_10;
	  yA[i][0] = yA[i][1]; 
	  yA[i][1] =   (xA[i][0] + xA[i][1]) + ( C1_10* yA[i][0]); // first order LPF to predict new speed.
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

	void feedback(float est_speed,float est_error,float OF_V_Error)
	{
		ground_Speed = est_speed; //get external speed : is injected into the LPF model when the car is under retardation
		Process_noise += DECAY_RATE;
		if(est_speed < MIN_LEARNING_SPEED or est_speed > MAX_LEARNING_SPEED or throttle<THROTTLE_OFFSET or fabs(est_speed-speed)>speed or OF_V_Error>OP_FLOW_MAX_V_ERROR)//making sure the car doesn't learn when i need the model to provide an estimate
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

	float throttle_to_speed(float x)
	{
		float x2 = x*x;
		float x3 = x2*x;
		return A0*x3 + A1*x2 + A2*x + A3;
	}

	float speed_to_throttle(float x)
	{
		x *= (feedback_factor + load/ROLL_RES);
		float x2 = x*x;
		float x3 = x2*x;
		float output = AI0*x3 + AI1*x2 + AI2*x + AI3;
		output = THROTTLE_RANGE*output + THROTTLE_OFFSET;
		return output;
	}

	void calc_speed()
	{
		roc = DEG2RAD*(steer - STEERINGNULL)*STEERING_OPEN_GAIN_INV;
		roc = check(roc,1e-3);//the steering angle must not be less than 1e-3 radians or 0.0573 degrees effectively a radius of more than 225 meters will not be accepted. This prevents the possibility of running into NaNs 
		roc = WHEELBASE/tanf(roc);//this is how you reuse variables kids. roc does not have to be very accurate but it does have to be stable.
		// roc = tanf(roc)/WHEELBASE; //curvature.
		

		last_Throttle = throttle; //this actually has it's use elsewhere.
		
		// if(fabs(La)>0.1)
		if(fabs(yR) > 0.17)
		{
			float dummy = roc;
			float meas_roc = -La/(yR*yR);
			// float meas_roc = -(yR*yR)/La; //this is also curvature.
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

		// if(roc<0.01 and roc>0)
		// {
		// 	roc = 0.01;
		// }
		// if(roc>(-0.01) and roc<0)
		// {
		// 	roc = -0.01;
		// }
		// roc = 1/roc; //convert curvature to radius of curvature.
		
		if(throttle<THROTTLE_OFFSET)
		{
			speed = LPF(0,ground_Speed);//inject the estimated speed into the LPF to prime it until the car is on throttle again.
			speed = ground_Speed;
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
			speed = throttle_to_speed(dummy);
			load = fabs(La*COG/roc);
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

	float input_to_speed(float x)
	{
		float y = (x - THROTTLE_OFFSET)*THROTTLE_RANGE_INV;
		if(y>=0)
			return 6*y;
		else
			return 0;
	}

	float smoother(float input,float last_input,float delta)
	{
		if(input<THROTTLE_OFFSET)
		{
			delta = 10.0f;
		}
		if(input - last_input > delta)
		{
			input = last_input + delta;
		}
		else if(input - last_input < -delta)
		{
			input = last_input - delta;
		}
		return input;
	}

	float critical_braking_distance(float D, float Cf, float Ci, float Vc)
	{
		if(fabs(Ci)>=fabs(Cf))
		{
			return D; //return the same distance if the next curvature is less than current because then slope will just come out 0
		}//this condition ensures fabs(m)>0

		float k = Vc*C_Gain;
		C_Critical = signum(Cf)*min((Bruh_G/(k*WHEELBASE)),0.5);

		float m,d;
		m = (Cf - Ci)/D;
		if(fabs(m)<0.01f)
		{
			return D; //because m is ridiculously small.
		}

		d = (C_Critical - Ci)/m;

		if(d>D)
		{
			return D; //if the critical braking distance comes out more than the maximum braking distance, braking distance is the maximum distance
		}
		if(d<0)
		{
			return WHEELBASE;
		}
		return d;
	}

	//the following function takes the required Curvature, the speed of the car, the measured yaw Rate, measured horizontal accelerations and car's MODE
	void driver(float C[2], float braking_distance, float V, float drift_ratio, float yawRate, float Ax, float Ay, uint8_t MODE, float inputs[8]) // function to operate the servo and esc.
	{
		La = LPF_10(3,Ax);//10 Hz low pass filter.
		yR = yawRate*DEG2RAD;
		Ha = LPF_10(1,Ay);//10 Hz low pass filter
		if( millis() - stamp < CONTROL_TIME)
		{
			return; //maintain a defined control frequency separate from observation frequency. Only used here because synchronising the time stamps across 2 objects would be difficult(sort of. could fix. create a pull request if you want it fixed)
		}

		float LIDAR_RANGE = (inputs[7] - 1000)*0.0045;
		LIDAR_RANGE = LIDAR_RANGE <= 4.0f ? LIDAR_RANGE : 1e2;
		LIDAR_RANGE = max(LIDAR_RANGE,0.1);
		float obstacle_avoidance_decel = -V*V/LIDAR_RANGE;

		float deceleration, backoff, resultant, correction, yaw_Compensation, V_target, V_error;
		float V1,deceleration_required;
		throttle = THROTTLENULL;
		steer = STEERINGNULL; //default values.
		
		resultant = fast_sqrt(Ax*Ax + Ay*Ay); //resultant horizontal acceleration
		correction = Curvature_To_Angle(C[0]); //convert radius of curvature to steering angle. this actually depends on wheelbase.
		yaw_Compensation = (RAD2DEG*V*C[0] - yawRate) - drift_ratio;//Expected Yaw rate - measured Yaw rate and compensating for drift
		
		V_target = fast_sqrt(ABSOLUTE_MAX_ACCELERATION/fabs(C[0])); //target maximum velocity

		//braking distance is how much distance we have in front of us to slow down.
		V1 = fast_sqrt(ABSOLUTE_MAX_ACCELERATION/fabs(C[1]));//max speed at the sharpest portion of the turn TODO : OPTIMIZE
		
		// braking_distance *= exp(-fabs(C[1])/C_Critical);
		//TODO: a don't care condition where it doesn't care about braking distance if it is less than certain distance
		
		braking_distance = critical_braking_distance(braking_distance,C[1],C[0],V1); //braking distance before critical curvature
		deceleration_required = V*(V1-V)/braking_distance; //dv/dt = (dv/dx)*(dx/dt) :P. faster than v^2 = u^2 + 2.a.S. note that this value will be -ve
		if(deceleration_required < SAFE_DECELERATION)//retardation required is more than the safe braking limit.
		{
			V_target = V1; //set that velocity as the setpoint. This keeps happening until the deceleration_required is less than safety limit.
			preemptive = true;
		}
		else if(obstacle_avoidance_decel < SAFE_DECELERATION)
		{
			V_target = 0;
			preemptive = true;
			deceleration_required = obstacle_avoidance_decel;
		}
		else
		{
			preemptive = false;
		}

		if(MODE == MODE_PARTIAL)
		{
			V_target = 0;
			if(obstacle_avoidance_decel > SAFE_DECELERATION)
			{
				V_target = input_to_speed(inputs[2]);
			}
			correction = (inputs[0] - STEERINGNULL)*STEERING_OPEN_GAIN_INV;
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

		float acceleration_buffer_total = fast_sqrt(MAX_ACCELERATION_SQ - (V*yR)*(V*yR)); //total acceleration on all 4 wheels
		float acceleration_buffer = acceleration_buffer_total*(FR_ratio + Ha*H_GBR_INV); //total grip on rear wheels (in RWD RWB, the rear wheels provide all the acceleration/deceleration)
		V_error = V_target - V; //speed setpoint - current speed
		if(fabs(V_error)>fabs(acceleration_buffer*THROTTLE_TIME_CONSTANT))
		{
			V_target = V + signum(V_error)*acceleration_buffer*THROTTLE_TIME_CONSTANT;
			V_error = signum(V_error)*acceleration_buffer*THROTTLE_TIME_CONSTANT;
		}
		if(V_error>= MIN_SPEED_ERROR)//Required velocity is greater than the current velocity
		{
			if(Ha>=0)//if we are already speeding up
			{
				backoff = RAD2DEG*fabs(drift_ratio);//if the resultant is more than the max acceleration, back the fuck off.
				if(backoff<0)				//as you may have noted, MAX_ACC is the safe maximum g force the car can handle. it is not the absolute maximum
				{							//therefore the resultant can be higher. If the resultant is higher the max acceleration, the car starts backing
					backoff = 0;			//off from the throttle or the brakes to keep the car within it's limit of grip
				}
			}
			else
			{
				backoff = 0;
			}
			backoff = 0;
			throttle = limiter(speed_to_throttle(V_target) + CLOSED_GAIN*V_error - backoff );
			throttle = smoother(throttle,last_Throttle,5.0f);

		}
		if(V_error< MIN_SPEED_ERROR)//required velocity is less than current velocity.
		{
			if(Ha<=0) //if we are already slowing down
			{
				backoff = RAD2DEG*fabs(drift_ratio);//same logic as before
				if(backoff<0)
				{
					backoff = 0;
				}
			}
			else
			{
				backoff = 0;
			}
			backoff = 0;
			preemptive? deceleration = deceleration_required : deceleration = V_error*10.0f;
			if(deceleration > -acceleration_buffer)
			{
				deceleration = -acceleration_buffer; //prevent wheels from locking up.
			}
			throttle = limiter(THROTTLE_OFFSET + BRAKE_GAIN*deceleration + backoff  );
			throttle = smoother(throttle,last_Throttle,100.0f);
		}

		steer = limiter(STEERINGNULL + STEERING_OPEN_GAIN*correction + yaw_correction(yaw_Compensation) ); //open loop + closed loop control 
		
		if(fabs(drift_ratio)>DRIFT_RATIO_CUTOFF)
		{
			adjust_g_force_limits(fabs(drift_ratio));
		}
		
		calc_speed();
		set_Outputs_Raw(throttle,steer); //defined in INOUT

		return; //good practice

	}//140us
};
#endif
