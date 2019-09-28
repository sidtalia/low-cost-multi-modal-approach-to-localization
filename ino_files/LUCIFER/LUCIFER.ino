//You might observe some inconsistency between tabs and spaces between arduino code and the header file code.
//If that is a problem for you then you are free to edit the code yourself. I don't have the time to be concerned 
//with formatting as long as the code remains understandable.

//choose the 128MHz overclocked setting for compilation.

//If you want to cross check the time taken by any function, here is some help:
//in floating point, each multiplication takes 0.61us, each addition/subtraction takes 0.481 us, each division 2.14us
//in integer, divide the above numbers by 2 (roughly speaking). This gives a decent estimate of execution time. 
//The cycle time is constrained to 2500us. While its possible to run it down to 1250us without breaking a sweat, it would sacrifice system safety checks
//and re-initialization processes (say for example the MPU is somehow shut-off by chance. it takes about 470 us to re-initialize it).
#include"MPU9150.h"
#include"OPFLOW.h"
#include"GPS_NAV_PVT.h"
#include"MEMORY.h"
#include"INOUT.h"
#include"SIDMATH.h"
#include"COMS.h"
#include"STATE.h"
#include"CAR.h"
#include"PARAMS.h"
#include"TRAJECTORY.h"

MPU9150 marg;
OPFLOW opticalFlow;
GPS gps;
STATE car;
GCS gcs;
trajectory track;
controller control;

bool GPS_FIX;
byte MODE = MODE_STANDBY;
byte message;
float inputs[8];
int16_t num_waypoints=0;
int16_t point = 0;
int16_t sentinel = 0;
bool circuit = false; //
bool car_ready = false;
float dest_X,dest_Y,slope;

coordinates *c;

void setup() 
{
//  initialize all coms
  Serial.begin(COM_BAUD);
  Serial1.begin(GPS_BAUD);
  Serial2.begin(JEVOIS_BAUD);
  SPI.begin();
  Wire.begin();
  Wire.setClock(400000);  //start initializing driver code
  delay(1000);
  IO_init();
  set_Outputs(0,0);
  
  marg.initialize();
  opticalFlow.initialize();
  opticalFlow.caliberation(ride_height,0.0f ); //ride_height is stored in the param's header
  gps.initialize();

  int16_t A[3],G[3],M[3],T,gain[3];
  if(!check_memory()) //if there are no offsets in the memory
  {
    bool avail = gcs.Get_Offsets(marg.offsetA, marg.offsetG, marg.offsetM, marg.offsetT,marg.axis_gain);
    if(avail)//if GCS already has offsets
    {
      marg.getOffset(A,G,M,T,gain);
      store_memory(0, A,G,M,T,gain);
    }
    //if GCS has no offsets and we don't have the offsets
    else
    {
      gcs.Send_Calib_Command(1); //let GCS know we are doing calib
      delay(2000);
      marg.gyro_caliberation();
      //keep the car still, rotate it 180, keep the car still again, rotate 180.
      gcs.Send_Calib_Command(2);
      delay(2000);
      marg.accel_caliberation();

      gcs.Send_Calib_Command(3);
      delay(2000);
      marg.mag_caliberation();

      gcs.Send_Calib_Command(4);
      
      marg.getOffset(A,G,M,T,gain);
      store_memory(0, A,G,M,T,gain);
      
      gcs.Send_Offsets(marg.offsetA, marg.offsetG, marg.offsetM, marg.offsetT, marg.axis_gain); //send new found offsets to GCS
    }
  }
  else
  {
    read_memory(0, A,G,M,T,gain);
    marg.setOffset(A,G,M,T,gain);
    gcs.Send_Offsets(marg.offsetA, marg.offsetG, marg.offsetM, marg.offsetT, marg.axis_gain); //send new found offsets to GCS
  }
  marg.Setup();
  
  gps.localizer();//get initial location
  long timeout = millis();
  do //wait till we get a GPS fix
  {
    gcs.Send_State(MODE, gps.latitude, gps.longitude,gps.latitude,gps.longitude, marg.V, marg.mh, marg.pitch, marg.roll, marg.Ha, opticalFlow.P_Error, 0, marg.mh_Error, marg.V_Error, 0,gps.Hdop);
    if(gps.Hdop>1000)//if gps is unavailable, skip.
    {
      delay(100);
      break;
    }
  }
  while(gps.fix_type()<2 && millis() - timeout < FIX_TIMEOUT );
//  if(gps.fix_initial_position()&&0) //get initial coordinates
  if(0)
  {
    GPS_FIX = 1;
  }
  else
  {
    GPS_FIX = 0;
    delay(100);
    gps.localizer(); //use latest position from gps as a starting point.
  }

  car.initialize(gps.longitude, gps.latitude, gps.Hdop, marg.mh, 0, marg.Ha);
  
}

unsigned long timer,time_it;
unsigned long T,benchmark;
bool reflect_WP = false;
float dummy;

void clear_wp()
{
  num_waypoints = 0;
  point = 0;
  sentinel = 0;
  circuit = 0;
  car_ready = false;
  delete[] c; //clear all waypoints
}

void loop() 
{
  timer = micros();//this is to ensure that the cycle time remains constant at 2500us. How do I know it's not exceeding that limit? 
                    //I unit test each of the functions to check how much time they take to execute.
  //================GET SENSOR DATA================
  control.get_model(marg.encoder_velocity); //comment out if not using output throttle signal as a rough speed estimate
  marg.compute_All(); //get AHRS (and Velocity as well) from IMU. 980us, has failsafe in case sensor is reset somehow
  
  marg.get_Rotations(opticalFlow.omega); //transfer rates of rotation
  opticalFlow.updateOpticalFlow(); //update optical flow 150us
  if(opticalFlow.failure)
  {
    timer = micros();
  }
  gps.localizer(); //update gps. 12us
  //till here it takes 180us, total at 1170us
  //================SENSOR FUSION===================
  car.state_update(gps.longitude, gps.latitude, gps.tick, gps.Hdop, gps.gSpeed, gps.Sdop, gps.headMot, gps.headAcc, marg.mh, marg.mh_Error, marg.yawRate, marg.Ha, marg.V, marg.V_Error,
             opticalFlow.X, opticalFlow.Y, opticalFlow.V_x, opticalFlow.V_y, opticalFlow.P_Error, opticalFlow.V_Error,marg.encoder_velocity); //I know i could've just passed the gps, marg and optical
                              //flow objects but then the state library would become dependent on these libraries and for some unkown reason I want to keep it a bit more generic
  marg.Velocity_Update(car.Velocity,car.VelError,car.AccBias);//pass the corrected velocity back to marg where it gets low pass filtered too.
  
  control.feedback(car.Velocity,car.VelError,opticalFlow.V_Error);//giving feedback to the car's model for making the machine learn the parameter(s) of the model
//transfer the bias. this is pretty much the reason why the update function does not take arguments by reference
  //till here it takes 120us, total at 1330us
  //================HANDLE COMMUNICATIONS================
  message = gcs.check();//automatically regulates itself at 10Hz, don't worry about it
  if(reflect_WP)//if waypoints are to be sent back, this remains true
  {
    reflect_WP = !(gcs.Send_WP(c[point].X,c[point].Y,point)); //this function will return true when waypoints have been sent back
  }
  else //this is for the general case
  {
    gcs.Send_State(MODE, double(car.X), double(car.Y),gps.longitude, gps.latitude, car.Velocity, marg.mh, marg.pitch, marg.roll, 
                  marg.encoder_velocity[0], control.feedback_factor, car.PosError_tot , marg.mh_Error, car.VelError, inputs[2],gps.Hdop); //also regulated at 10Hz
//      gcs.Send_State(MODE, double(car.X), double(car.Y),double(dest_X),double(dest_Y),int_1_x,int_1_y,int_2_x,int_2_y,
//                    dummy, opticalFlow.SQ, car.PosError_tot , marg.mh_Error, 3.15, benchmark,gps.Hdop);
//    gcs.Send_State(MODE, double(gps.VelNED[1]),double(gps.VelNED[0]) ,gps.longitude, gps.latitude, gps.gSpeed, marg.mh, marg.pitch, marg.roll, 
//                  gps.headVeh, gps.headMot, car.PosError_tot , marg.mh_Error, 3.16, T,gps.Hdop); //also regulated at 10Hz
  }
  if(gcs.get_Mode()!=255)//255 is condition for no message received yet.
  {
    MODE = gcs.get_Mode();
  }
  
  get_Inputs(inputs); //get inputs from r/c receiver
  if(gcs.failsafe)//if gcs has shutdown for some reason, fallback on the transmitter.
  {
    if(inputs[5] <= 1100)
    {
      MODE = MODE_STANDBY;
    }
    if(inputs[5] > 1100 && inputs[5] < 1600)
    {
      MODE = MODE_MANUAL;
    }
    if(inputs[5] > 1600)
    {
      MODE = CRUISE;
    }
    if(inputs[2]<1000)//if even the transmitter is off, then inputs[2] will have a value of less than 1000.
    {
      MODE = MODE_STANDBY;
    }
  }

  if(message == SET_ORIGIN_ID)//this is for resetting the position
  {
    car.initialize(gps.longitude, gps.latitude, gps.Hdop, marg.mh, 0, marg.Ha);
  }
  
  if(message == CALIB_ID)//recalculate offsets
  {
    int16_t A[3],G[3],M[3],gain[3],T;
    gcs.Send_Calib_Command(1); //let GCS know we are doing calib
    delay(2000);
    marg.gyro_caliberation();//keep the car still, rotate it 180, keep the car still again, rotate 180.
    gcs.Send_Calib_Command(2);
    delay(2000);
    marg.accel_caliberation();
    
    marg.getOffset(A,G,M,T,gain);
    store_memory(0, A,G,M,T,gain);
    
    gcs.Send_Offsets(marg.offsetA, marg.offsetG, marg.offsetM, marg.offsetT,marg.axis_gain); //send new found offsets to GCS
    timer = micros(); //reset timer  
  }

  if(message == WP_ID)//if waypoint message is received
  {
    reflect_WP = true; //we'll have to reflect the waypoints
    if(num_waypoints==0)//if we have not initialized the waypoints yet
    {
      num_waypoints = gcs.msg_len;//for WP, the msg_len is not the length of the received packet, its the number of waypoints that will be given to the car in totality.
      c = new coordinates[num_waypoints];
      point = 0;//initialize point.
    }
    if(point < num_waypoints && num_waypoints !=0)
    {
      float dummy_X,dummy_Y;
      gcs.Get_WP(dummy_X, dummy_Y,point); //get the coordinates
      c[point].X = dummy_X;
      c[point].Y = dummy_Y;
      c[point].calcLatLon(car.iLon, car.iLat); // calculate lat lon just in case
      if(point == num_waypoints-1)
      {
        if( check_loop(c[0],c[point]) ) //check if first and last points are within 1/2 a meter range
        {
          circuit = true;
          c[point].copy(c[0]);
        }
        track.generate_Slopes(c,num_waypoints, circuit); // generate the slopes! happens only once so I reset the timer 
        dest_X = c[0].X;
        dest_Y = c[0].Y;
        slope  = c[0].slope;
        car_ready = true;
        sentinel = 0;
        timer = micros();
      }
    }
    else
    {
      gcs.Send_Calib_Command(5);
    }
  }
  if(message == CLEAR_ID && num_waypoints!=0)
  {
    clear_wp();
  }

  if( distancecalcy(car.Y, dest_Y, car.X, dest_X,0) <= WP_CIRCLE && num_waypoints!=0 && car_ready)//checking if waypoint has been reached
  {
    sentinel++;
    if(circuit)
    {
      sentinel = sentinel%num_waypoints;
      dest_X = c[sentinel].X; //TODO : maybe just pass the object of the coordinate instead of transfering all the values manually.
      dest_Y = c[sentinel].Y;
      slope = c[sentinel].slope; 
    }
    else
    {
      sentinel = min(sentinel,num_waypoints-1);
      if(sentinel == num_waypoints-1)
      {
        clear_wp();
      }
      else
      {
        dest_X = c[sentinel].X; //TODO : maybe just pass the object of the coordinate instead of transfering all the values manually.
        dest_Y = c[sentinel].Y;
        slope = c[sentinel].slope; 
      }
    }

  }
  
  /*
   * ADD CODE FOR JEVOIS/COMPANION COMPUTER HERE 
   */
  
  if( (MODE == CRUISE || MODE == LUDICROUS) && point == num_waypoints-1 && num_waypoints!=0 )//autonomous modes. The paranthesis are important! the conditions need to be clubbed together
  {
    time_it = micros();
    track.calculate_Curvatures(car.Velocity, car.X, car.Y, car.heading, dest_X, dest_Y, slope ); 
    benchmark = micros()-time_it;
    control.driver(track.C, track.braking_distance, car.Velocity,car.drift_Angle, marg.yawRate, marg.La, marg.Ha, MODE, inputs); //send data to driver code. automatically maintains a separate control frequency.
    dummy = track.C[0];
  }
  else if(MODE == MODE_PARTIAL || MODE == MODE_MANUAL || MODE == MODE_STOP || MODE == MODE_STANDBY || MODE==MODE_CONTROL_CHECK)//manual modes
  {
    float dum[] = {0.0,0.0};//dummy
    control.driver(dum, 1000, car.Velocity,car.drift_Angle, marg.yawRate, marg.La, marg.Ha, MODE, inputs);
  }
  else
  {
    float dum[] = {0.0,0.0};//dummy
    MODE = MODE_STANDBY;
    control.driver(dum, 1000, car.Velocity,car.drift_Angle, marg.yawRate, marg.La, marg.Ha, MODE, inputs);
  }
  if(T > dt_micros)//in case the execution time exceeds loop time limit
  {
    gcs.Send_Calib_Command(5);
  }
  
  T = max(micros()-timer,T);
  while(micros()-timer < dt_micros ); //dt_micros is defined in PARAMS.h
}
