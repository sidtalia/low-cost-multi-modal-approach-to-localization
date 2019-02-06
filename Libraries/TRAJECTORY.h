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
	void calcXY(double iLon, double iLat)
	{
		X = float(DEG2METER*(longitude - iLon));
		Y = float(DEG2METER*(latitude - iLat));
	}
	void calcLatLon(double iLon, double iLat)
	{
		longitude = iLon + double(METER2DEG*X);#ifndef _TRAJECTORY_H_
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
	float int1[2],int2[2],t;
	float C[2], X_max, Y_max, braking_distance;
	void calculate_Curvatures(float V, float X, float Y, float slope1, float destX, float destY, float slope2)
	{
    	get_Intermediate_Points( slope1, slope2, X, destX, Y, destY); //get the intermediate points
    	t = get_T( V, X, Y, int1[0], int1[0], int2[0], int2[0], destX, destY, FUTURE_TIME); //t parameter for getting the curvature
    	//the t parameter is evaluated for the point where the car would be 2x(control cycle time) from now. This is because the position I have
    	//corresponds to the position of the car at the beginning of the first control cycle. If I calculate the steering control signal for the point 
    	//where the car will be by the end of the first control cycle, the signals sent out will be executed by the end of the 2nd control cycle, meaning
    	//the car would always be lagging behind the ideal line. Now this wouldn't be noticible at slow speeds but it becomes a problem at high speeds
    	//and fast direction changes
    	get_Curvature(X, Y, int1[0], int1[0], int2[0], int2[0], destX, destY, t);//get the curvature
    	//if C[1] is less than C[0] then the curve is getting sharper and pre-mature braking is required, if not, no need to have pre-mature braking.
    	braking_distance = distancecalcy( Y, Y_max, X, X_max,0); //braking distance. 
    }	

    float get_T(float V,float X1,float Y1,float X2,float Y2,float X3,float Y3,float X4,float Y4,float DT)
	{
		float L1,L2,L3,L4,L;
		L1 = distancecalcy(Y1,Y2,X1,X2,0);
		L2 = distancecalcy(Y2,Y3,X2,X3,0);
		L3 = distancecalcy(Y3,Y4,X3,X4,0);
		L4 = distancecalcy(Y4,Y1,X4,X1,0);
		L = L1+L2+L3;
		L = 0.5*(L+L4);
		float parameter = DT*V/L; //approximate value of T. Yes I know that the distances are not uniform,
		                          // however, when it comes to the constrained case of a car, the ratio of 
		                          //distance covered in 2 cycles to the approximate total arc length gives 
		                          // a *fairly accurate*(for our use case) approximation of the parameter t
		                          //which can then be used to estimate the required steering angle and speed.
		if(parameter>1)
		{
			return 1; //exceeding the limits
		}
		return parameter;
	}//255us 
	  
	void get_Curvature(float X1,float Y1,float X2,float Y2,float X3,float Y3,float X4,float Y4,float t)
	{
		float Curvature[11];//curvatures at equi-distant points (okay roughly equi-distant)
		float C_max=0, t_max=0;
		float KX1,KX2,KX3,KY1,KY2,KY3;
		float delX,delY,del2X,del2Y;
		float denominator;

		//calculate the parameters that will be used over and over and over again, over and over and over again.
		KX1 = 9*X2 + 3*X4 - 3*X1 - 9*X3; //54
		KY1 = 9*Y2 + 3*Y4 - 3*Y1 - 9*Y3; //54
		KX2 = 6*X1 - 12*X2 + 6*X3; //39
		KY2 = 6*Y1 - 12*Y2 + 6*Y3; //39
		KX3 = 3*(X2 - X1); //15
		KY3 = 3*(Y2 - Y1); //15

		for(uint8_t i = 0;i<11;i++) //going through a lot of points on the path and calculating the curvature at each including the FUTURE point.
		{
			if(i!=0)//ignore the first one.
			{
				t = float(i)*0.08 + 0.1; //generate new t parameters between 10-90% of the trajectory. 20% because the car can handle upto 20%. 
			}							  
			delX = t*t*KX1 + t*KX2 + KX3; //39
			delY = t*t*KY1 + t*KY2 + KY3; //39
			del2X = 2*t*KX1 + KX2; //24
			del2Y = 2*t*KY1 + KY2; //24
			denominator = delX*delX + delY*delY; //24
			denominator *= denominator*denominator; //17
			denominator = sqrt(denominator); //35
			Curvature[i] = ((delX*del2Y) - (delY*del2X)); //24
			Curvature[i] /= denominator; //30

			//prevent infinities
			if(mod(Curvature[i])<0.001)
			{
				if(Curvature[i]>0)
				{
					Curvature[i] = 0.0001;
				}
				else
				{
					Curvature[i] = -0.0001;
				}
			}

			if( i == 0) // the first point
			{
				C_max = Curvature[i]; //initialize C_max with the future point.
				t_max = t; //initialize t_max with the t of future point.
			}

			if(mod(Curvature[i]) > mod(C_max) && i!=0 )//if the point has a curvature less than the least one found so far.
			{
				C_max = Curvature[i];
				t_max = t;
			}
		}
		//now that we have found the minimum curvature
		C[0] = Curvature[0]; //next point's curvature
		C[1] = C_max;  //next 
		t = t_max;
		float t_4 = pow(t,3); //24
		float t_3 = 3*t*t*(1-t); //30
		float t_2 = 3*t*pow((1-t),2); //30
		float t_1 = pow((1-t),3); //30
		X_max = t_1*X1 + t_2*X2 + t_3*X3 + t_4*X4 ; //50
		Y_max = t_1*Y1 + t_2*Y2 + t_3*Y3 + t_4*Y4 ; //50
	} //3411us on pro mini, 246.3us on STM32F103C8T6 @ 128MHz

	void get_Intermediate_Points(float slope1, float slope2, float X1, float X2, float Y1, float Y2)
	{
		float d = distancecalcy(Y2,Y1,X2,X1,0);
		int1[0] = X1 + 0.4*my_cos(slope1*DEG2RAD)*d; //57us
		int1[1] = Y1 + 0.4*my_sin(slope1*DEG2RAD)*d; //67
		int2[0] = X2 - 0.4*my_cos(slope2*DEG2RAD)*d; //57us
		int2[1] = Y2 - 0.4*my_sin(slope2*DEG2RAD)*d; //67
	}//248us

	void generate_Slopes(coordinates c[],int16_t n, bool circuit) 
	{
		for(uint8_t i = 1;i<n-1;i++) //don't tell me you're gonna give me more than 255 waypoints
		{
			c[i].slope = ( anglecalcy( c[i-1].X, c[i].X, c[i-1].Y, c[i].Y ) + anglecalcy( c[i].X, c[i+1].X, c[i].Y, c[i+1].Y ) )*0.5;
		}
		if(circuit)
		{
			c[0].slope = c[n-1].slope =  ( anglecalcy( c[n-2].X, c[n-1].X, c[n-2].Y, c[n-1].Y ) + anglecalcy( c[0].X, c[1].X, c[0].Y, c[1].Y ) )*0.5;
		}
		else
		{
			c[0].slope = anglecalcy( c[0].X, c[1].X, c[0].Y, c[1].Y ); // if its not a circuit, then the first point's slope is the same as the 
																		//slope of the line joining the first and second point
			c[n-1].slope = anglecalcy( c[n-2].X, c[n-1].X, c[n-2].Y, c[n-1].Y );//and for the last point 
																				// the slope is the same as the line joining the last 2 points
		}
	}
};
#endif
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
	float int1[2],int2[2],t;
	float C[2], X_max, Y_max, braking_distance;
	void calculate_Curvatures(float V, float X, float Y, float slope1, float destX, float destY, float slope2)
	{
    	get_Intermediate_Points( slope1, slope2, X, destX, Y, destY); //get the intermediate points
    	t = get_T( V, X, Y, int1[0], int1[0], int2[0], int2[0], destX, destY, FUTURE_TIME); //t parameter for getting the curvature
    	//the t parameter is evaluated for the point where the car would be 2x(control cycle time) from now. This is because the position I have
    	//corresponds to the position of the car at the beginning of the first control cycle. If I calculate the steering control signal for the point 
    	//where the car will be by the end of the first control cycle, the signals sent out will be executed by the end of the 2nd control cycle, meaning
    	//the car would always be lagging behind the ideal line. Now this wouldn't be noticible at slow speeds but it becomes a problem at high speeds
    	//and fast direction changes
    	get_Curvature(X, Y, int1[0], int1[0], int2[0], int2[0], destX, destY, t);//get the curvature
    	//if C[1] is less than C[0] then the curve is getting sharper and pre-mature braking is required, if not, no need to have pre-mature braking.
    	braking_distance = distancecalcy( Y, Y_max, X, X_max,0); //braking distance. 
    }	

    float get_T(float V,float X1,float Y1,float X2,float Y2,float X3,float Y3,float X4,float Y4,float DT)
	{
		float L1,L2,L3,L4,L;
		L1 = distancecalcy(Y1,Y2,X1,X2,0);
		L2 = distancecalcy(Y2,Y3,X2,X3,0);
		L3 = distancecalcy(Y3,Y4,X3,X4,0);
		L4 = distancecalcy(Y4,Y1,X4,X1,0);
		L = L1+L2+L3;
		L = 0.5*(L+L4);
		float parameter = DT*V/L; //approximate value of T. Yes I know that the distances are not uniform,
		                          // however, when it comes to the constrained case of a car, the ratio of 
		                          //distance covered in 2 cycles to the approximate total arc length gives 
		                          // a *fairly accurate*(for our use case) approximation of the parameter t
		                          //which can then be used to estimate the required steering angle and speed.
		if(parameter>1)
		{
			return 1; //exceeding the limits
		}
		return parameter;
	}//255us 
	  
	void get_Curvature(float X1,float Y1,float X2,float Y2,float X3,float Y3,float X4,float Y4,float t)
	{
		float Curvature[11];//curvatures at equi-distant points (okay roughly equi-distant)
		float C_max=0, t_max=0;
		float KX1,KX2,KX3,KY1,KY2,KY3;
		float delX,delY,del2X,del2Y;
		float denominator;

		//calculate the parameters that will be used over and over and over again, over and over and over again.
		KX1 = 9*X2 + 3*X4 - 3*X1 - 9*X3; //54
		KY1 = 9*Y2 + 3*Y4 - 3*Y1 - 9*Y3; //54
		KX2 = 6*X1 - 12*X2 + 6*X3; //39
		KY2 = 6*Y1 - 12*Y2 + 6*Y3; //39
		KX3 = 3*(X2 - X1); //15
		KY3 = 3*(Y2 - Y1); //15

		for(uint8_t i = 0;i<11;i++) //going through a lot of points on the path and calculating the curvature at each including the FUTURE point.
		{
			if(i!=0)//ignore the first one.
			{
				t = float(i)*0.08 + 0.1; //generate new t parameters between 10-90% of the trajectory. 20% because the car can handle upto 20%. 
			}							  
			delX = t*t*KX1 + t*KX2 + KX3; //39
			delY = t*t*KY1 + t*KY2 + KY3; //39
			del2X = 2*t*KX1 + KX2; //24
			del2Y = 2*t*KY1 + KY2; //24
			denominator = delX*delX + delY*delY; //24
			denominator *= denominator*denominator; //17
			denominator = sqrt(denominator); //35
			Curvature[i] = ((delX*del2Y) - (delY*del2X)); //24
			Curvature[i] /= denominator; //30

			//prevent infinities
			if(mod(Curvature[i])<0.001)
			{
				if(Curvature[i]>0)
				{
					Curvature[i] = 0.0001;
				}
				else
				{
					Curvature[i] = -0.0001;
				}
			}

			if( i == 0) // the first point
			{
				C_max = Curvature[i]; //initialize C_max with the future point.
				t_max = t; //initialize t_max with the t of future point.
			}

			if(mod(Curvature[i]) > mod(C_max) && i!=0 )//if the point has a curvature less than the least one found so far.
			{
				C_max = Curvature[i];
				t_max = t;
			}
		}
		//now that we have found the minimum curvature
		C[0] = Curvature[0]; //next point's curvature
		C[1] = C_max;  //next 
		t = t_max;
		float t_4 = pow(t,3); //24
		float t_3 = 3*t*t*(1-t); //30
		float t_2 = 3*t*pow((1-t),2); //30
		float t_1 = pow((1-t),3); //30
		X_max = t_1*X1 + t_2*X2 + t_3*X3 + t_4*X4 ; //50
		Y_max = t_1*Y1 + t_2*Y2 + t_3*Y3 + t_4*Y4 ; //50
	} //3411us on pro mini, 246.3us on STM32F103C8T6 @ 128MHz

	void get_Intermediate_Points(float slope1, float slope2, float X1, float X2, float Y1, float Y2)
	{
		float d = distancecalcy(Y2,Y1,X2,X1,0);
		int1[0] = X1 + 0.4*my_cos(slope1*DEG2RAD)*d; //57us
		int1[1] = Y1 + 0.4*my_sin(slope1*DEG2RAD)*d; //67
		int2[0] = X2 - 0.4*my_cos(slope2*DEG2RAD)*d; //57us
		int2[1] = Y2 - 0.4*my_sin(slope2*DEG2RAD)*d; //67
	}//248us

	void generate_Slopes(coordinates c[],int16_t n, bool circuit) 
	{
		for(uint8_t i = 1;i<n-1;i++) //don't tell me you're gonna give me more than 255 waypoints
		{
			c[i].slope = ( anglecalcy( c[i-1].X, c[i].X, c[i-1].Y, c[i].Y ) + anglecalcy( c[i].X, c[i+1].X, c[i].Y, c[i+1].Y ) )*0.5;
		}
		if(circuit)
		{
			c[0].slope = c[n-1].slope =  ( anglecalcy( c[n-2].X, c[n-1].X, c[n-2].Y, c[n-1].Y ) + anglecalcy( c[0].X, c[1].X, c[0].Y, c[1].Y ) )*0.5;
		}
		else
		{
			c[0].slope = anglecalcy( c[0].X, c[1].X, c[0].Y, c[1].Y ); // if its not a circuit, then the first point's slope is the same as the 
																		//slope of the line joining the first and second point
			c[n-1].slope = anglecalcy( c[n-2].X, c[n-1].X, c[n-2].Y, c[n-1].Y );//and for the last point 
																				// the slope is the same as the line joining the last 2 points
		}
	}
};
#endif
