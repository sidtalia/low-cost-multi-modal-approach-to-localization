#ifndef _STATE_H_
#define _STATE_H_
#include"SIDMATH.h"
#include"PARAMS.h"
#include"Arduino.h"

#define DEFAULT_DT dt
#define GPS_UPDATE_RATE 10 //gps update rate in Hz

class STATE
{
public :
	double iLat,iLon,lastLat,lastLon, latitude, longitude; 
	float heading, Velocity, past_Velocity, last_Velocity,past_VelError, Acceleration;
	float X, last_X, past_X, past_PosError_X, Y, last_Y, past_Y, past_PosError_Y, PosError_X, PosError_Y, VelError, PosError_tot;
	float AccBias;
	bool position_reset;
	float drift_Angle;

	void initialize(double lon,double lat,double Hdop, float head, float Vel, float acc)
	{
		iLat = lastLat = latitude = lat;
		iLon = lastLon = longitude = lon;
		X = last_X = past_X = 0;
		Y = last_Y = past_Y = 0;
		PosError_X = PosError_Y = float(Hdop);
		if(Hdop>100)
		{
			position_reset = true;//iLat and iLon will need to be reset later if gps becomes available mid-mission
		}
		else
		{
			position_reset = false;
		}
		past_PosError_X = PosError_X;
		past_PosError_Y = PosError_Y;
		VelError = past_VelError = 0;
		heading = head;
		Velocity = past_Velocity = last_Velocity = Vel;
		Acceleration = acc; //initially acc, vel should be close to 0
	}

	//fuse GPS, magnetometer, Acclereometer, Optical Flow
	void state_update(double lon, double lat, bool tick,double Hdop, float mh, float mh_Error, float Acceleration,float Vacc, float VError,
						  float OF_X, float OF_Y, float OF_V_X, float OF_V_Y, float OF_P_Error, float OF_V_Error)
	{
		float cosmh = cos(mh*DEG2RAD);
		float sinmh = sin(mh*DEG2RAD);
		float Xacc,Yacc,dS,dSError,dTheta;
		float PosGain_Y, PosGain_X, VelGain;
		heading = mh;
		VelError = VError; //VelError is the velocity erro in the "state." I'm transferring the velocity error from outside to the object member
		float GPS_Velocity,GPS_SAcc; //velocity from gps
		/*
		OVERVIEW:
		OVERVIEW OF COMMON SENSE FILTER : (aka kalman filter)
		imagine a 1-D case. You have 1 position measurement device and 1 speed measurement device. say the error in speed is 0.1 m/s 
		(constant for the device)
		Assume the position measurement error is variable(which it usually is). Assume that the velocity is also more or less constant 
		(For the sake of simplicity)
		lets say that the initial position was X = 0. Assume that initially, position error was = 1
		At t = 0 the speed measurement device gave speed = 5m/s
		at t = 1 second, the estimated position = speed*time_elapsed = 5*1 = 5 meters.
		the error in the estimated position can be upto Previous_Error + (error_in_speed*time_elapsed) = 1 + 0.1 = 1.1 meters
		the position measurement device gives the position as 7 meters with an error of 2.2 meters. 
		the gain for the measurement is calculated as 
							error in estimate 					1.1
		gain = --------------------------------------- = 	----------- = 1/3 = 0.33
				error in estiate + error in measurement		 2.2 + 1.1
		
		corrected position = meas*gain + (1-gain)*estimate (1)
		the corrected position is = 7*0.33 + 5(1-0.33) = 5.66 meters. 
		now that the position has been corrected, the error in the position must have also been corrected and so the new position error is
		positionError = previous position Error * (1 - 0.33) ;

		These formulas can be derived very easily. The kalman filter assumes that the distribution of the variables 
		(estimated position and measured position)is gaussian (which is a fair assumption for most quantities). 
		When 2 gaussian distributions are multiplied, the mean of the new gaussian distribution is calculated by formula (1) 
		(you can try this yourself. Multiply 2 gaussian distributions and see what the final one's mean looks like)
		and the new "variance" (which we call error here) is the estimate's error multiplied by the complement of the gain.
		check out this website for more math http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
		
		Is the implementation here a kalman filter (or EKF)? Well, yes and no. Fundamentally, it follows the steps laid out above
		(which btw are followed by all variations of KF. The only things that change are how you get the error in the estimates). 
		However, the implementation avoids the use of matrices for the sake of understandability for the layman who doesn't have
		a clue what a jacobian matrix is. You see, in almost all implementations of the KF, the logic appears to be more complicated 
		than it actually is. This particular implementation isn't short but it is a lot easier to understand in my opinion because
		you can see what is really happening with the errors and the estimates. In other implementations, you look at a matrix 'Q' 
		and say oh its the covariance matrix and god only knows what's really going on with the terms inside it. Here, there is no 
		covariance matrix. The covariance matrix essentially represents the inter dependence between the variance of the variables 
		being tracked simultaneously (Say the inter dependence of speed and position). Here, I work with the terms that would be inside
		that matrix without ever invoking matrix into the code (error in position is dependent on error in speed which is dependent on
		error in acceleration.). Some part of the actual filter isn't even in this header file,
		it is in the MPU9150 header file(for a good reason which I will not elaborate here), 
		which is why I would suggest against copy pasting this implementation(also, why would you. 
		The code should be sufficiently easy to understand in order for you to write
		your own implementation of it for whatever purpose you may have).

		Extended kalman filter brief : 
		In a linear kalman filter, it is assumed that the estimate varies linearly and so does the error in the estimate
		(the 1-D case is a perfect example). However, if you consider a case where the object is moving in 2 dimensions,
		say moving on a curved path, then the error in the estimated position is not only due to the error in speed but also due to
		the error in the estimated bearing. When we deal with a 1-D case, the formula for error in the estimated position is obtained 
		from the derivative of the position, i.e., is position = V*dt, then position_Error = dV *dt, where dV is the error in speed.
		here, the position is 2 dimensional, position_X = speed*cos(bearing)*dt, position_Y = speed*sin(bearing)*dt,
		therefore error_X = (error_in_speed*cos(bearing) - speed*sin(bearing)*error_in_bearing)*dt and similarly you can find the formula for error_Y

		Now in a standard implementation of EKF, the above thing is actually represented by a jacobian matrix 
		(the determinant of the jacobian should result in the above equations). Here, I lay out the inner workings in plain view for all to understand.
		(this is pretty much all the difference between a linear KF and an EKF. Let me know if I missed something!)

		Also one more thing, This isn't a mathematically perfect implementation, and by perfect I mean absolutely perfect, like no discrepancies 
		between this and theory sort of perfect. When estimating position from the optical Flow, I don't consider the error in the heading 
		(it seems a bit dubious because the error isn't going to be integrated and the error in the optical flow readings is so small (at most 5 mm or so)

		This function takes data from position, Hdop and validity from gps, takes heading, heading error, acceleration, estimated Velocity,
		error in estimated velocity from IMU, change in position along car's X, Y axis, speed along car's X, Y axis, position Error and 
		Velocity Error from optical flow
		
		WORKING: (the comments alonside the code are also a part of this section)
		the velocity is estimated and error is integrated within the IMU code (MPU9150.cpp) itself but the corrections and bias calculations 
		are performed here
		The next order of business is to find the distance moved since the last cycle(2.5ms) (dS) based on the velocity estimate(V) 
		Then we compute the estimated position based on the distance moved(but this isn't the final estimate) from the IMU (Xacc->X accelerometer)
		then we compute the error in distance moved (dSError) and the error in the heading(well it's given already we just change the units from degrees to radians)
		the error in the position estimate is not just the error in velocity*dt. As explained in the Extended kalman briefing, the formula here is different
		*/
		//POSITION ESTIMATE USING THE ACCELEROMETER (WORKING CONTINUED)
		//Acceleration bias is removed in the MPU9250 code itself(the name of the library is 9150 but it can be used with 9250 as well).
		//CORRECTING VELOCITY FIRST
		//The optical Flow's error skyrockets(goes from a few millimeters (normal) to 1000 meters) when the surface quality is bad or if the sensor is defunct
		VelGain = VelError/(VelError + OF_V_Error);//the reason why velocity has only one dimension is because the car's motion is constrained. While you could compute the 
								//Velocity in the NED fashion, it would be equivalent to going on a fools errand here. If there is a constraint, exploit it.

		Velocity = OF_V_Y*VelGain + (1-VelGain)*Vacc;//correcting the velocity estimate
		VelError *= (1-VelGain);//reduce the error in the estimate.
		//find the difference between prediction and measurement.
		//this bias is for "tuning" the accelerometer for times when the optical flow isn't reliable
		if(OF_P_Error<1) //don't touch this bias if you have an error more than 1 
		{
			AccBias += (Vacc - Velocity)*VelGain;//keep adjusting bias while optical flow is trustworthy. dt is just there to make the adjustments smaller
		}
		//this is the covariance stuff(using the corrected estimates to correct errors in states other than the one being corrected)
		//distance moved in last cycle
		dS = Velocity*dt + 0.5*Acceleration*dt*dt;//is this formula correct? hmm..(does it matter? seeing that the first term is 2 orders of magnitude larger than the second one under most circumstances?)
		Xacc = X + dS*cosmh + sinmh*OF_X; //estimated X position. OF_X is the sideways movement measured by the optical flow senosr(can't do that with a wheel encoder can you now?)
		Yacc = Y + dS*sinmh + cosmh*OF_X; //estimated Y position

		dSError = VelError*dt;//error in instantaneous distance travelled
		dTheta = mh_Error*DEG2RAD;//error in heading

		PosError_Y += dSError*cosmh - dS*sinmh*dTheta;//remember that thing called the "Jacobian matrix" in EKF? Yeah. These are the terms from that matrix.
		PosError_X += dSError*sinmh + dS*cosmh*dTheta;//the jacobian is simply a matrix that contains the partial derivatives. See how much simpler it is to
														//understand when you DON'T use matrices(looking at you Ardupilot, px4, etc)??

		//POSITION ESTIMATE USING BOTH THE ACCELEROMETER AND THE OPTICAL FLOW
		//note that the optical flow error will skyrocket if it the sensor is defunct or if the surface quality is poor.
		X += cosmh*OF_Y + sinmh*OF_X; // the optical flow can measure movement along the car's X and Y directions.
		Y += sinmh*OF_Y + cosmh*OF_X; //

		PosGain_X = PosError_X/(PosError_X + OF_P_Error); //optical flow error is assumed to be circular
		PosGain_Y = PosError_Y/(PosError_Y + OF_P_Error);

		X = X*PosGain_X + (1-PosGain_X)*Xacc;//in most implementations, you would see this happening through matrix multiplication. I hate that.
		Y = Y*PosGain_Y + (1-PosGain_Y)*Yacc;//Yes matrix multiplication makes it easier for programmers, but it makes it impossible to understand for 
											 //just about every body else who doesn't already have a degree in thermonuclear astrophysics(sarcasm).
											 //basically if someone needs background knowledge in some subject just to understand what the code is doing,
											 // you need to work on the understandability of your code 
											 //(and if you think its not important then you might as well straight up work in hex code)
		PosError_X *= (1-PosGain_X);
		PosError_Y *= (1-PosGain_Y);
			                         

		//POSITION ESTIMATION USING GPS + ESTIMATED POSITION FROM PREVIOUS METHODS
		if(tick && (Hdop<2.5) && !position_reset )//if new GPS data was received and the data is useful, fuse it with the estimates(because why would you want to fuse garbage into garbage)
		{	/*
			We have the current estimate for the position, but the gps data corresponds to the position 100ms ago (the gps update rate is 10Hz).
			So we have to do the data fusion in the past, find the corrected position in the past and then shift the current position estimate by
			the difference in the corrected past position and the estimated past position.
			so lets take a simple example to understand the code. lets say during runtime, the last position known from GPS is X=1
			this would have been my position 200ms ago. The position I get now is say X = 3. This would have been my position 100 ms ago 
			(the gps data lags by 100ms).The last_X = (3-1) + 1 = 3.(its possible that the 1 wasn't 1 but like 1.25 after filtering, 
			making the actual last_X = 3.25 but we'll assume perfect case for now for the sake of simplicity).
			lets say my current estimate for position is 5 meters and my past_X was 3.4 meters. 
			lets say my past Error estimate is equal to the gps's Hdop(meaning both have equal error). So the filtered past_X = 3.2 meters
			now my current position of 5 meters is predicated on the assumption that I was at 3.4 meters 100ms ago. So, in order to correct that
			I will shift my current position by the "difference between past_X estimate and past_X filtered value", which in this case would be 
			3.2 - 3.4 = -0.2 
			meaning that my current position estimate is shifted to 5 + (-0.2) = 4.8 meters.
			*/
			float temp_X = last_X; //last corrected position estimates 
			float temp_Y = last_Y; 

			last_X = float((lon - lastLon)*DEG2METER) + last_X;//getting the last gps position 
			last_Y = float((lat - lastLat)*DEG2METER) + last_Y; 

			lastLon = lon;//setting lastLat, lastLon for next iteration
			lastLat = lat; 

			//the gps is assumed to have a circular error, meaing it's error in X direction is equal to it's error in Y direction = Hdop
			PosGain_X = (past_PosError_X / (past_PosError_X + float(Hdop) )); //new position gain for X (East-West)
			PosGain_Y = (past_PosError_Y / (past_PosError_Y + float(Hdop) )); //new position gain for Y (North-South)

			last_X = PosGain_X*last_X + (1-PosGain_X)*past_X; // past_X,past_Y are the past Estimates for position 
			last_Y = PosGain_Y*last_Y + (1-PosGain_Y)*past_Y; // last_X,last_Y are the past corrected position 
															  //(I didn't create separate variables for measurement because it seemed like a waste of memory)
			
			if(Velocity > 10)//explanation given in the codeblock itself.
			{
				GPS_Velocity = distancecalcy(last_X,temp_X,last_Y,temp_Y,0)*GPS_UPDATE_RATE;//calculating velocity from corrected estimates
				GPS_SAcc = distancecalcy(PosError_X*(1-PosGain_X*0.5), 0, PosError_Y*(1-PosGain_Y*0.5), 0, 0)*GPS_UPDATE_RATE; //Velocity error = position error/dt . this is according to the correlation between speed and distance
				//the velocity error is the average of the old position estimate error and the new position estimate error multiplied by the update rate. 
				VelGain = (past_VelError/(past_VelError + GPS_SAcc)); //find the gain to correct the past estimate
				last_Velocity = VelGain*GPS_Velocity + (1-VelGain)*past_Velocity;//correct the last velocity.
				Velocity += (last_Velocity - past_Velocity);//shift the new velocity by the "innovation"
				VelError *= (1-VelGain); //reduce the velocity error
				//this is the "magical thing" about kfs that the kf boys(including myself) nut to before we sleep. 
				//Not only is it correcting the position estimate(which is what you initially wanted), it is also correcting the velocity estimate by
				//exploiting the relation between speed and position. Now the problem here is that if you're travelling at really slow speeds 
				//by gps standards, ie, less than 10m/s, this correction might actually be counter-productive.
				//consider the case of the car making a U turn of radius 1m at a speed of 2 m/s. this is well within the car's capabilities.
				// However, the above calculations for speed use a straight line approximation which fails quite rapidly as you start considering low speed 
				//cases. To solve this, you need to consider the change in the heading of the car, which further relies on the assumption that the 
				//turning radius doesn't change which also fails quite rapidly at slow speeds. So you can see why there is an if condition to prevent us from 
				//incorrectly exploiting the relationship between speed and position. The relations only hold when you're travelling fairly fast.
			}

			PosError_X *= (1 - PosGain_X); //reduce the position error after each correction from gps.
			PosError_Y *= (1 - PosGain_Y); //this isn't exactly right(mathematically speaking) but then it would take too much memory 
			past_PosError_X = PosError_X;  //to remember the position error at every instant of time and re-calculate it recursively upto the current
			past_PosError_Y = PosError_X;  //time every correction step. (there are 40 estimate steps between each correction step)
										   //In my estimation, this should produce decent results too.
			past_VelError = VelError; //past Velocity error is the current velocity error now

			X += (last_X - past_X);
			Y += (last_Y - past_Y); //shift by the difference between last corrected position and last estimated position.

			past_X = X; //the value of X,Y,V become the "past estimates"(=last predicted velocities) for the next iteration
			past_Y = Y;
			past_Velocity = Velocity;
		}
		if(position_reset && Hdop < 5.0 && tick)
		{
			lastLat = lat;
			lastLon = lon;
			last_X = past_X = X - Velocity*0.1*cosmh; //too dumb to solve this problem in a sophisticated manner 
			last_Y = past_Y = Y - Velocity*0.1*sinmh;
			past_Velocity = last_Velocity = Velocity; //initialize the "past_velocity"
			past_VelError = VelError;
			past_PosError_X = PosError_X;
			past_PosError_Y = PosError_Y;

			iLat = lat - double(Y*METER2DEG);
			iLon = lon - double(X*METER2DEG);

			position_reset = false;//prevent this code block from being re-executed
		}
		latitude = iLat + double(Y*METER2DEG);
		longitude = iLon + double(X*METER2DEG);

		PosError_tot = distancecalcy(0,PosError_Y,0,PosError_Y,0);
		drift_Angle = (OF_V_X/Velocity); //uncomment when you have a quick atan function
		return ;
		//----------LOCALIZATION ENDS-------------------------------------
	}//on an STM32F103C8T6 running at 128MHz clock speed, this function takes 60.61 us to execute and 44 bytes of extra memory for local variables.

};
//this class takes 80 bytes in variables




#endif
