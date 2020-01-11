#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#include"Arduino.h"
#include"SIDMATH.h"
#include"PARAMS.h"

class coordinates //for storing multiple coordinates as we further progress to multiple waypoint trajectories instead of just 1 waypoint
{                               //this will become the main point of focus once the basic tasks are complete
public:
	double longitude,latitude;
	float X,Y;
	float slope;
	float next_gap;
	float next_Kappa;
	float next_X_max;
	float next_Y_max;
	void calcXY(double iLon, double iLat)
	{
		X = float(DEG2METER*(longitude - iLon));
		Y = float(DEG2METER*(latitude - iLat));
	}
	void calcLatLon(double iLon, double iLat)
	{
		longitude = iLon + double(METER2DEG*X);
		latitude  = iLat + double(METER2DEG*Y);
	}
	void copy(coordinates c)
	{
		X = c.X;
		Y = c.Y;
		longitude = c.longitude;
		latitude = c.latitude;
	}
};

bool check_loop(coordinates c1, coordinates c2)
{
	if(distancecalcy(c1.Y, c2.Y, c1.X, c2.X, 0) < WP_CIRCLE)
	{
		return true;
	}
	return false;
}

class trajectory
{
public:
	float int1[2],int2[2],t,T[3];
	float C[2], X_max, Y_max, braking_distance;
	void calculate_Curvatures(float V, float X, float Y, float slope1, float destX, float destY, float slope2)
	{
    	get_Intermediate_Points( slope1, slope2, X, destX, Y, destY); //get the intermediate points
    	get_T( V, X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY, FUTURE_TIME); //t parameter for getting the curvature
    	//the t parameter is evaluated for the point where the car would be 2x(control cycle time) from now. This is because the position I have
    	//corresponds to the position of the car at the beginning of the first control cycle. If I calculate the steering control signal for the point 
    	//where the car will be by the end of the first control cycle, the signals sent out will be executed by the end of the 2nd control cycle, meaning
    	//the car would always be lagging behind the ideal line. Now this wouldn't be noticible at slow speeds but it becomes a problem at high speeds
    	//and fast direction changes
    	get_Curvature(X, Y, int1[0], int1[1], int2[0], int2[1], destX, destY, V);//get the curvature
    	//if C[1] is less than C[0] then the curve is getting sharper and pre-mature braking is required, if not, no need to have pre-mature braking.
    	braking_distance = distancecalcy( X, X_max, Y, Y_max,0); //braking distance.
    	braking_distance = max(braking_distance,0.1);
    }	

    void get_T(float V,float X1,float Y1,float X2,float Y2,float X3,float Y3,float X4,float Y4,float DT)
	{
		float L1,L2,L3,L4,L;
		L1 = distancecalcy(Y1,Y2,X1,X2,0);
		L2 = distancecalcy(Y2,Y3,X2,X3,0);
		L3 = distancecalcy(Y3,Y4,X3,X4,0);
		L4 = distancecalcy(Y4,Y1,X4,X1,0);
		L = L1+L2+L3;
		L = 0.5f*(L+L4);
		T[2] = DT*V/L; //approximate value of T. Yes I know that the distances are not uniform,
		                          // however, when it comes to the constrained case of a car, the ratio of 
		                          //distance covered in 2 cycles to the approximate total arc length gives 
		                          // a *fairly accurate*(for our use case) approximation of the parameter t
		                          //which can then be used to estimate the required steering angle and speed.
		if(T[2]>1)
		{
			T[2] = 1.0f; //exceeding the limits
		}
		T[0] = 0.5f*(L1/(L1+L2));
		T[1] = 1.0f - 0.5f*(L3/(L3+L2));
	}//255us 
	  
	float C_from_k_t(float KX1,float KX2,float KX3,float KY1,float KY2,float KY3,float t)
	{
		float delX,delY,del2X,del2Y,denominator,Curvature;
		delX = t*t*KX1 + t*KX2 + KX3; //39
		delY = t*t*KY1 + t*KY2 + KY3; //39
		del2X = 2.0f*t*KX1 + KX2; //24
		del2Y = 2.0f*t*KY1 + KY2; //24
		denominator = delX*delX + delY*delY; //24
		denominator *= denominator*denominator; //17
		denominator = fast_sqrt(denominator); //35
		Curvature = ((delX*del2Y) - (delY*del2X)); //24
		Curvature /= denominator; //30

		//prevent infinities
		if(fabs(Curvature) - 0.0001f < 0)
		{
			if(Curvature>0)
			{
				Curvature = 0.0001f;
			}
			else
			{
				Curvature = -0.0001f;
			}
		}
		return Curvature;
	}

	float Curv(float KX1,float KX2,float KX3,float KY1,float KY2,float KY3,float t)
	{
		float delX,delY,del2X,del2Y,del3X,del3Y;
		float denominator,second_denominator,third_denominator;
		float sub_term_1,sub_term_2,sub_term_3,sub_term_4,sub_term_5;
		float term_1,term_2,term_3,term_4;
		float dummy,dK,d2K;
		delX = t*t*KX1 + t*KX2 + KX3;
		delY = t*t*KY1 + t*KY2 + KY3;
		del2X = 2.0f*t*KX1 + KX2;
		del2Y = 2.0f*t*KY1 + KY2;
		denominator = delX*delX + delY*delY;
		dummy = denominator;
		denominator *= denominator*denominator;
		denominator = fast_sqrt(denominator);
		del3Y = 2.0f*KY1;
		del3X = 2.0f*KX1;
		second_denominator = denominator*dummy ;
		dK = ((del3Y*delX - del3X*delY)/denominator) - (3.0f*(delX*del2Y - del2X*delY)*(delX*del2X + delY*del2Y)/second_denominator);
		sub_term_1 = (delX*del2Y - del2X*delY);
		sub_term_2 = 2.0f*(delX*del2X + delY*del2Y);
		third_denominator = second_denominator*dummy;
		sub_term_3 = (del3Y*delX - del3X*delY);
		sub_term_4 = 2.0f*(del2X*del2X + del2Y*del2Y + del3X*delX+del3Y*delY);
		sub_term_5 =  -del3X*del2Y + del3Y*del2X;
		term_1 = 3.75f*(sub_term_1*(sub_term_2*sub_term_2))/third_denominator;
		term_2 = -3.0f*(sub_term_3*sub_term_2)/second_denominator;
		term_3 = -1.5f*(sub_term_1*sub_term_4)/second_denominator;
		term_4 = sub_term_5/denominator;
		d2K = term_1 + term_2 + term_3 + term_4;
		if(d2K==0)
		{
			return 10*dK;
		}
		return dK/d2K;
	}

	float check_range(float x,uint8_t i)
	{
		if(i)
		{
			if(x>1)
			{
				return 1;
			}
			if(x<0.5)
			{
				return 0.5;
			}
			return x;
		}
		if(x<0)
		{
			return 0;
		}
		if(x>0.5)
		{
			return 0.5;
		}
		return x;
	}

	void X_Y_from_t(float X1,float Y1,float X2,float Y2,float X3,float Y3,float X4,float Y4,float t,float &Xret,float &Yret)
	{
		float term_1 = (1.0f-t);
		float t_4 = t*t*t; //24
		float t_3 = 3.0f*t*t*term_1; //30
		float t_2 = 3.0f*t*term_1*term_1; //30
		float t_1 = term_1*term_1*term_1; //30
		Xret = t_1*X1 + t_2*X2 + t_3*X3 + t_4*X4 ; //50
		Yret = t_1*Y1 + t_2*Y2 + t_3*Y3 + t_4*Y4 ; //50
	}

	void get_Curvature(float X1,float Y1,float X2,float Y2,float X3,float Y3,float X4,float Y4,float Velocity)
	{
		float KX1,KX2,KX3,KY1,KY2,KY3;
		float h,last_h;//for newton-rhaphson
		float kappa[2];
		float X[2],Y[2];
		float gap,gap_inverse;
		float K1_inv,sqrt_K1_K2_inv;
		float condition;
		//calculate the parameters that will be used over and over and over again, over and over and over again.
		KX1 = 9.0f*X2 + 3.0f*X4 - 3.0f*X1 - 9.0f*X3; //54
		KY1 = 9.0f*Y2 + 3.0f*Y4 - 3.0f*Y1 - 9.0f*Y3; //54
		KX2 = 6.0f*X1 - 12.0f*X2 + 6.0f*X3; //39
		KY2 = 6.0f*Y1 - 12.0f*Y2 + 6.0f*Y3; //39
		KX3 = 3.0f*(X2 - X1); //15
		KY3 = 3.0f*(Y2 - Y1); //15

		C[0] = C_from_k_t(KX1,KX2,KX3,KY1,KY2,KY3,T[2]); //curvature at next point
		//finding best estimates for maximum curvature T param.
		for(uint8_t i=0;i<2;i++)
		{
			h = 1;
			for(uint8_t j=0;j<2;j++)
			{
				float dummy = Curv(KX1,KX2,KX3,KY1,KY2,KY3,T[i]);
				if(j>=1)
				{
					last_h = h;
					h = dummy;
					if(h*last_h<0)
					{
						h = signum(h)*min(fabs(last_h/2),fabs(h));
					}
				}
				else
				{
					h = dummy;
				}
				T[i] -= h;
				T[i] = check_range(T[i],i);
			}
			kappa[i] = C_from_k_t(KX1,KX2,KX3,KY1,KY2,KY3,T[i]);
			X_Y_from_t(X1,Y1,X2,Y2,X3,Y3,X4,Y4,T[i],X[i],Y[i]);
		}
		C[1] = kappa[0];
		X_max = X[0];
		Y_max = Y[0];
		gap = max(distancecalcy(X[0],X[1],Y[0],Y[1],0),0.1);
		if(gap>0.1f && fabs(kappa[0])>0.001f && fabs(kappa[1])>0.001f)
		{
			gap_inverse = 1.0f/gap;
			K1_inv = 1.0f/fabs(kappa[0]);
			sqrt_K1_K2_inv = 1/(fast_sqrt(fabs(kappa[0]*kappa[1])));
			condition = gap_inverse*(K1_inv - sqrt_K1_K2_inv);
			if(condition>1)
			{
				C[1] = kappa[1];
				X_max = X[1];
				Y_max = Y[1];
			}
		}
		//now to determine the more important maxima
	} //3411us on pro mini, 246.3us on STM32F103C8T6 @ 128MHz

	void get_Intermediate_Points(float slope1, float slope2, float X1, float X2, float Y1, float Y2)
	{
		float d = distancecalcy(Y2,Y1,X2,X1,0);
		if(d>CONSTRUCT_LENGTH)
		{
			d = CONSTRUCT_LENGTH;
		}
		int1[0] = X1 + THE_RATIO*my_cos(slope1*DEG2RAD)*d; //57us
		int1[1] = Y1 + THE_RATIO*my_sin(slope1*DEG2RAD)*d; //67
		int2[0] = X2 - THE_RATIO*my_cos(slope2*DEG2RAD)*d; //57us
		int2[1] = Y2 - THE_RATIO*my_sin(slope2*DEG2RAD)*d; //67
	}//248us

	void generate_Slopes(coordinates c[],int16_t n, bool circuit) //MAKE THIS A REAL TIME THING. EACH ANGLECALCY CALL TAKES 40us!!
	{
		float angle1,angle2;
		for(uint8_t i = 1;i<n-1;i++) //don't tell me you're gonna give me more than 255 waypoints
		{
			angle1 = anglecalcy( c[i-1].X, c[i].X, c[i-1].Y, c[i].Y );
			angle2 = anglecalcy( c[i].X, c[i+1].X, c[i].Y, c[i+1].Y );
			if(fabs(angle1 - angle2) > M_PI_DEG)//this happens when the angles are above and below east (350 and 10 degrees will give an average of 180 not 0)
			{
				angle1 -= M_2PI_DEG;
			}
			c[i].slope = ( angle1 + angle2 )*0.5;
		}
		if(circuit)
		{
			angle1 = anglecalcy( c[n-2].X, c[n-1].X, c[n-2].Y, c[n-1].Y );
			angle2 = anglecalcy( c[0].X, c[1].X, c[0].Y, c[1].Y );
			if(fabs(angle1 - angle2) > M_PI_DEG)//this happens when the angles are above and below east (350 and 10 degrees will give an average of 180 not 0)
			{
				angle1 -= M_2PI_DEG;
			}
			c[0].slope = c[n-1].slope =  ( angle1 + angle2 )*0.5;
		}
		else
		{
			c[0].slope = anglecalcy( c[0].X, c[1].X, c[0].Y, c[1].Y ); // if its not a circuit, then the first point's slope is the same as the 
																		//slope of the line joining the first and second point
			c[n-1].slope = anglecalcy( c[n-2].X, c[n-1].X, c[n-2].Y, c[n-1].Y );//and for the last point 
																				// the slope is the same as the line joining the last 2 points
		}
	}

	void get_fixed_maximas(coordinates c[], int16_t n, bool circuit)
	{
		uint8_t s,f;
		for(uint8_t i = 0; i < n-1; i++)
		{
			s = i%n; //start point 
			f = (i+1)%n; //finish point
			calculate_Curvatures(0, c[s].X, c[s].Y, c[s].slope, c[f].X, c[f].Y, c[f].slope);
			c[i].next_gap = braking_distance;
			c[i].next_Kappa = C[1];
			c[i].next_X_max = X_max;
			c[i].next_Y_max = Y_max;
		}
		if(circuit)
		{
			calculate_Curvatures(0, c[0].X, c[0].Y, c[0].slope, c[1].X, c[1].Y, c[1].slope);
			c[n-1].next_gap = braking_distance;
			c[n-1].next_Kappa = C[1];
			c[n-1].next_X_max = X_max;
			c[n-1].next_Y_max = Y_max;	
		}
		else
		{
			c[n-1].next_gap = 100;
			c[n-1].next_Kappa = 0;
			c[n-1].next_X_max = c[n-1].X;
			c[n-1].next_Y_max = c[n-1].Y;		
		}
	}

	void confirm_maxima_priority(coordinates c, float &cur_X_max, float &cur_Y_max, float &cur_Kappa, float &cur_braking_distance)
	{
		float condition, sqrt_K1_K2_inv, K1_inv, gap_inverse;
		float gap = max(distancecalcy(cur_X_max, c.next_X_max, cur_Y_max, c.next_Y_max, 0), 0.1);
		if(gap>0.1f && fabs(c.next_Kappa)>0.001f && fabs(cur_Kappa)>0.001f)
		{
			gap_inverse = 1.0f/gap;
			K1_inv = 1.0f/fabs(cur_Kappa);
			sqrt_K1_K2_inv = 1/(fast_sqrt(fabs(cur_Kappa*c.next_Kappa)));
			condition = gap_inverse*(K1_inv - sqrt_K1_K2_inv);
			if(condition>1)
			{
				cur_Kappa = c.next_Kappa;
				cur_X_max = c.next_X_max;
				cur_Y_max = c.next_Y_max;
				cur_braking_distance += gap;

			}
		}
	}	
};
#endif
