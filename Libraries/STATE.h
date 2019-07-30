#ifndef _STATE_H_
#define _STATE_H_
#include"SIDMATH.h"
#include"PARAMS.h"
#include"Arduino.h"

#define GPS_UPDATE_RATE (float) 10 //gps update rate in Hz
#define GPS_UPDATE_TIME (float) 0.1
#define MIN_GPS_SPEED (float) 3.0 //min speed till which gps is not used for velocity correction
#define GPS_HDOP_LIM (float)2.5
#define GPS_GLITCH_RADIUS (float) 3.0 
#define C1_STATE (float) 0.984414
#define LPF_GAIN_STATE (float) 1/128.321336

class STATE
{
public :
	double iLat,iLon,lastLat,lastLon, latitude, longitude; 
	float heading, Velocity, past_Velocity, last_Velocity,past_VelError, Acceleration;
	float X, last_X, past_X, past_PosError_X, Y, last_Y, past_Y, past_PosError_Y, PosError_X, PosError_Y, VelError, PosError_tot;
	float gps_X,gps_Y;
	float AccBias;
	bool position_reset;
	float drift_Angle;
	float xA[2][4],yA[2][4];
	float GPS_Velocity,GPS_SAcc; //velocity from gps
	float last_cosmh,last_sinmh;
	float declination;

    float LPF(int i,float x)
	{
	  xA[i][0] = xA[i][1]; 
	  xA[i][1] = x*LPF_GAIN_STATE;
	  yA[i][0] = yA[i][1]; 
	  yA[i][1] =   (xA[i][0] + xA[i][1]) + ( C1_STATE* yA[i][0]); // first order LPF to predict new speed.
	  return yA[i][1];
	}

	void initialize(double lon,double lat,double Hdop, float head, float Vel, float acc)
	{
		iLat = lastLat = latitude = lat;
		iLon = lastLon = longitude = lon;
		X = last_X = gps_X = past_X = 0;
		Y = last_Y = gps_Y = past_Y = 0;
		PosError_X = PosError_Y = float(Hdop);
		if(Hdop>2.5)
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
		last_cosmh = cosf(head*DEG2RAD);
		last_sinmh = sinf(head*DEG2RAD);
		Velocity = past_Velocity = last_Velocity = Vel;
		Acceleration = acc; //initially acc, vel should be close to 0
		declination = 0;
	}

	//fuse GPS, magnetometer, Acclereometer, Optical Flow
	void state_update(double lon, double lat, bool tick,double Hdop, float GPS_Velocity, float GPS_SAcc, float gHead, float headAcc, float mh, float mh_Error, float Acceleration,float Vacc, float VError,
						  float OF_X, float OF_Y, float OF_V_X, float OF_V_Y, float OF_P_Error, float OF_V_Error,float model[3])
	{
		// mh += declination;// COMMENT
		// if(mh >= M_2PI_DEG) // the mh must be within [0.0,360.0]
		// {
		// 	mh -= M_2PI_DEG;
		// }
		// if(mh < 0.0f)
		// {
		// 	mh += M_2PI_DEG;
		// }
		float cosmh = cosf(mh*DEG2RAD); //OPTIMIZE
		float sinmh = sinf(mh*DEG2RAD);
		float Xacc,Yacc,dS_y,dS_x,dSError,dTheta,tan_phi;
		float PosGain_Y, PosGain_X, VelGain;
		float separation;
		float shift_X=0,shift_Y=0,innovation=0; // added on 5/5/19
		heading = mh;
		float head_Innovation = 0;
		float head_gain = 0;
		VelError = VError; //VelError is the velocity erro in the "state." I'm transferring the velocity error from outside to the object member
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
		here, the position is 2 dimensional, position_X = speed*cosf(bearing)*dt, position_Y = speed*sinf(bearing)*dt,
		therefore error_X = (error_in_speed*cosf(bearing) - speed*sinf(bearing)*error_in_bearing)*dt and similarly you can find the formula for error_Y

		Now in a standard implementation of EKF, the above thing is actually represented by a jacobian matrix 
		(the determinant of the jacobian should result in the above equations). Here, I lay out the inner workings in plain view for all to understand.
		(Also, note that our state transition is not linear. Let me know if I missed something!)

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
		if(OF_V_Error>OP_FLOW_MAX_V_ERROR) //if optical flow sensor is out, use the model velocity regardless of speed. 
		{
			model[1] = max(model[0],3.0f);
		}
		VelGain = VelError/(model[1] + VelError); //encoder_velocity[0] is speed, [1] is error
		Vacc = (1.0f - VelGain)*Vacc + VelGain*model[0]; //correction step correcting the velocity from the accelerometer section
		VelError *= (1.0f - VelGain); 

		//The optical Flow's error skyrockets(goes from a few millimeters (normal) to 1000 meters) when the surface quality is bad or if the sensor is defunct
		if(Velocity>OP_FLOW_MAX_SPEED) // if velocity is more than 3 m/s, accelerometer becomes reliable. In case that Optical flow error is greater than 1, accelerometer alone is used.
		{							   //while this does mean that velocity is not corrected for these situations, it is important as during such situations the optical flow is not reliable, at least not ADNS3080
			OF_V_Error *= 1e3;
			OF_P_Error = OF_V_Error;
		}	
		VelGain = VelError/(VelError + OF_V_Error);//the reason why velocity has only one dimension is because the car's motion is constrained. While you could compute the 
								//Velocity in the NED fashion, it would be equivalent to going on a fools errand here. If there is a constraint, exploit it.

		Velocity = OF_V_Y*VelGain + (1-VelGain)*Vacc;//correcting the velocity estimate
		VelError *= (1-VelGain);//reduce the error in the estimate.
		//find the difference between prediction and measurement.
		//this bias is for "tuning" the accelerometer for times when the optical flow isn't reliable
		if(OF_V_Error<10)
		{
			AccBias += (Vacc - OF_V_Y)*VelGain*dt*dt;//keep adjusting bias while optical flow is trustworthy. dt is just there to make the adjustments smaller
		}
		//use the optical flow speed to prevent the bias from skyrocketting the fuck outta dodge.
		//this is the covariance stuff(using the corrected estimates to correct errors in states other than the one being corrected)
		//distance moved in last cycle
		
		dS_y = Velocity*dt;// + 0.5*Acceleration*dt*dt;//is this formula correct? hmm..(does it matter? seeing that the first term is 2 orders of magnitude larger than the second one under most circumstances?)
		Xacc = X + dS_y*cosmh;//estimated X position.
		Yacc = Y + dS_y*sinmh;//estimated Y position
		dSError = VelError*dt;//error in instantaneous distance travelled
		dTheta = mh_Error*DEG2RAD;//error in heading

		PosError_X += fabs(dSError*cosmh - dS_y*sinmh*dTheta);//remember that thing called the "Jacobian matrix" in EKF? Yeah. These are the terms from that matrix.
		PosError_Y += fabs(dSError*sinmh + dS_y*cosmh*dTheta);//the jacobian is simply a matrix that contains the partial derivatives.

		//POSITION ESTIMATE USING BOTH THE ACCELEROMETER AND THE OPTICAL FLOW
		//note that the optical flow error will skyrocket if it the sensor is defunct or if the surface quality is poor.
		X += cosmh*OF_Y; // the optical flow can measure movement along the car's X and Y directions.
		Y += sinmh*OF_Y; //

		PosGain_X = PosError_X/(PosError_X + OF_P_Error); //optical flow error is assumed to be circular
		PosGain_Y = PosError_Y/(PosError_Y + OF_P_Error);
		X = X*PosGain_X + (1-PosGain_X)*Xacc;//in most implementations, you would see this happening through matrix multiplication. 
		Y = Y*PosGain_Y + (1-PosGain_Y)*Yacc;//This is partly because the problem is actually a 3 dimensional position problem for drones whereas it is a 2 dimensional problem for cars											 
		PosError_X *= (1-PosGain_X);//and as you can see, the number of lines I would have to write for 3 dimensional fusion would be even greater than this,
		PosError_Y *= (1-PosGain_Y);// which makes matrix multiplication methods look more attractive.
		
		X += sinmh*OF_X; // the body frame X axis movement has no other source of information, so therefore there can be no filtering for it.
		Y -= cosmh*OF_X;

		//POSITION ESTIMATION USING GPS + ESTIMATED POSITION FROM PREVIOUS METHODS
		if(tick && !position_reset )//if new GPS data was received and the data is useful, fuse it with the estimates(because why would you want to fuse garbage into garbage)
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
			float temp_X = gps_X; //last gps coordinates. edited on 5/5/19. gps_X
			float temp_Y = gps_Y; 
			// Hdop *= Hdop*Hdop;
			gps_X = last_X = float((lon - iLon)*DEG2METER);// + last_X;//getting the last gps position 
			gps_Y = last_Y = float((lat - iLat)*DEG2METER);// + last_Y; 

			lastLon = lon;//setting lastLat, lastLon for next iteration
			lastLat = lat; 
			
			separation = distancecalcy(gps_X,past_X,gps_Y,past_Y,0);
			if(separation> GPS_GLITCH_RADIUS || (Hdop > GPS_HDOP_LIM)) 
			{
				position_reset = true;
				Hdop = 1e7;
			}
			//the gps is assumed to have a circular error, meaing it's error in X direction is equal to it's error in Y direction = Hdop
			PosGain_X = (past_PosError_X / (past_PosError_X + float(Hdop) )); //new position gain for X (East-West)
			PosGain_Y = (past_PosError_Y / (past_PosError_Y + float(Hdop) )); //new position gain for Y (North-South)

			last_X = PosGain_X*last_X + (1-PosGain_X)*past_X; // past_X,past_Y are the past Estimates for position 
			last_Y = PosGain_Y*last_Y + (1-PosGain_Y)*past_Y; // last_X,last_Y are the past corrected position 
															  //(I didn't create separate variables for measurement because it seemed like a waste of memory)
			if(GPS_Velocity > MIN_GPS_SPEED && !position_reset)//explanation given in the codeblock itself.
			{
				//the velocity error is the average of the old position estimate error and the new position estimate error multiplied by the update rate. 
				VelGain = (past_VelError/(past_VelError + GPS_SAcc)); //find the gain to correct the past estimate
				last_Velocity = VelGain*GPS_Velocity + (1-VelGain)*past_Velocity;//correct the last velocity.
				
				innovation = (last_Velocity - past_Velocity);
				
				Velocity += innovation;//shift the new velocity by the "innovation" 5/5/19
				VelError *= (1-VelGain); //reduce the velocity error
				
				shift_X = innovation*GPS_UPDATE_TIME*(cosmh+last_cosmh)*0.5f;// I need you to get wayy off my back on the logistics of this.5/5/19
				shift_Y = innovation*GPS_UPDATE_TIME*(sinmh+last_sinmh)*0.5f;//5/5/19
				last_cosmh = cosmh;//5/5/19
				last_sinmh = sinmh;
				
				//this is the "magical thing" about kfs that the kf boys(including myself) nut to before we sleep. 
				//Not only is it correcting the position estimate(which is what you initially wanted), it is also correcting the velocity estimate by
				//exploiting the relation between speed and position. Now the problem here is that if you're travelling at really slow speeds 
				//by gps standards, i.e., less than 3m/s, this correction might actually be counter-productive.
				
				// head_Innovation = heading - gHead;
				// if(head_Innovation >= M_PI_DEG) // the head_Innovation must be within [0.0,360.0]
				// {
				// 	head_Innovation -= M_2PI_DEG;
				// }
				// if(head_Innovation <= -M_PI_DEG)
				// {
				// 	head_Innovation += M_2PI_DEG;
				// }
				// head_gain = mh_Error/(mh_Error + headAcc);
				// declination -= head_Innovation*head_gain;
			}

			PosError_X *= (1 - PosGain_X); //reduce the position error after each correction from gps.
			PosError_Y *= (1 - PosGain_Y); //this isn't exactly right(mathematically speaking) but then it would take too much memory 
			past_PosError_X = PosError_X;  //to remember the position error at every instant of time and re-calculate it recursively upto the current
			past_PosError_Y = PosError_X;  //time every correction step. (there are 40 estimate steps between each correction step)
										   //In my estimation, this should produce decent results too.
			past_VelError = VelError; //past Velocity error is the current velocity error now

			X += (last_X - past_X) + shift_X; //some part of the shift is due to the velocity correction correcting the position. 5/5/19
			Y += (last_Y - past_Y) + shift_Y; //shift by the difference between last corrected position and last estimated position.

			past_X = X; //the value of X,Y,V become the "past estimates"(=last predicted velocities) for the next iteration
			past_Y = Y;
			past_Velocity = Velocity;
		}
		if(position_reset && Hdop < GPS_HDOP_LIM && tick)//position reset condition is checked before using gps data to prevent jumps in position when gps error drops below 2.5m
		{
			lastLat = lat;
			lastLon = lon;
			last_X = past_X = X;
			last_Y = past_Y = Y;
			past_Velocity = last_Velocity = Velocity; //initialize the "past_velocity"
			past_VelError = VelError;
			past_PosError_X = PosError_X;
			past_PosError_Y = PosError_Y;

			iLat = lat - double(past_Y*METER2DEG);//lat lon reported by gps matches with the past value of position.
			iLon = lon - double(past_X*METER2DEG);

			position_reset = false;//prevent this code block from being re-executed
		}

		latitude = iLat + double(Y*METER2DEG); //change lat and long accordingly. 
		longitude = iLon + double(X*METER2DEG);
		//note that if the location was initially wrong, resetting the iLat resets the lon/lat estimates without disturbing the relative position estimates 
		PosError_tot = distancecalcy(0,PosError_X,0,PosError_Y,0);//OPTIMIZE
		drift_Angle = (OF_V_X/Velocity); //uncomment when you have a quick atan function
		// LPF(0,Velocity); //use this if you have a really noisy accelerometer but avoid at all costs.
		return ;
		//----------LOCALIZATION ENDS-------------------------------------
	}//on an STM32F103C8T6 running at 128MHz clock speed, this function takes 60.61 us to execute and 44 bytes of extra memory for local variables.

};
//this class takes 80 bytes in variables




#endif
