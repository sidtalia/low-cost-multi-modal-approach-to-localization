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
#include"GPS.h"
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
    if(gps.Hdop>100000)//if gps is unavailable, skip.
    {
      delay(100);
      break;
    }
  }
  while(gps.fix_type()<2 && millis() - timeout < FIX_TIMEOUT );
  if(gps.fix_initial_position()) //get initial coordinates
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

void loop() 
{
  timer = micros();//this is to ensure that the cycle time remains constant at 2500us. How do I know it's not exceeding that limit? 
                    //I unit test each of the functions to check how much time they take to execute.
  //get sensor data
  
  control.get_model(marg.encoder_velocity); //comment out if not using output throttle signal as a rough speed estimate
  marg.compute_All(); //get AHRS (and Velocity as well) from IMU. 980us, has failsafe in case sensor is reset somehow
  control.feedback(marg.encoder_feedback);//giving feedback to the car's model.
  
  marg.get_Rotations(opticalFlow.omega); //transfer rates of rotation
  opticalFlow.updateOpticalFlow(); //update optical flow 150us
  gps.localizer(); //update gps. 12us
  //till here it takes 180us, total at 1170us
  car.state_update(gps.longitude, gps.latitude, gps.tick, gps.Hdop, marg.mh, marg.mh_Error, marg.Ha, marg.V, marg.V_Error,
             opticalFlow.X, opticalFlow.Y, opticalFlow.V_x, opticalFlow.V_y, opticalFlow.P_Error, opticalFlow.V_Error); //I know i could've just passed the gps, marg and optical
                              //flow objects but then the state library would become dependent on these libraries and for some unkown reason I want to keep it a bit more generic
  marg.Velocity_Update(car.Velocity,car.VelError,car.AccBias);//pass the corrected velocity back to marg where it gets low pass filtered too.
//transfer the bias. this is pretty much the reason why the update function does not take arguments by reference
  //till here it takes 120us, total at 1330us
  message = gcs.check();//automatically regulates itself at 10Hz, don't worry about it
//  time_it = micros();
  gcs.Send_State(MODE, car.longitude, car.latitude,gps.longitude, gps.latitude, car.Velocity, marg.mh, marg.pitch, marg.roll, 
                  opticalFlow.V_y, opticalFlow.SQ, car.PosError_tot , marg.mh_Error, marg.V_Error, T,gps.Hdop); //also regulated at 10Hz
//  benchmark = max(micros()-time_it,benchmark);
  if(gcs.get_Mode()!=255)
  {
    MODE = gcs.get_Mode();
  }
  
  get_Inputs(inputs); //get inputs from r/c receiver
  if(gcs.failsafe)
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
    if(inputs[2]<1000)
    {
      MODE = MODE_STANDBY;
    }
  }
  
  if(message == CALIB_ID)//recalculate offsets
  {
    int16_t A[3],G[3],M[3],gain[3],T;
    gcs.Send_Calib_Command(1); //let GCS know we are doing calib
    delay(2000);
    marg.gyro_caliberation();
    //keep the car still, rotate it 180, keep the car still again, rotate 180.
    gcs.Send_Calib_Command(2);
    delay(2000);
    marg.accel_caliberation();
    
    marg.getOffset(A,G,M,T,gain);
    store_memory(0, A,G,M,T,gain);
    
    gcs.Send_Offsets(marg.offsetA, marg.offsetG, marg.offsetM, marg.offsetT,marg.axis_gain); //send new found offsets to GCS
    timer = micros(); //reset timer  
  }
  
  if(message == WP_ID)
  {
    if(num_waypoints==0)
    {
      num_waypoints = gcs.msg_len;//for WP, the msg_len is not the length of the received packet, its the number of waypoints that will be given to the car in totality.
      c = new coordinates[num_waypoints]; //the first message will never contain the coordinates. this is because the waypoints may be marked or sent
      point = 0;
    }
    else if(point < num_waypoints && num_waypoints !=0)//TODO : prevent people from sending more waypoints than num_waypoints
    {
      gcs.Get_WP(c[point].longitude, c[point].latitude);
      c[point].calcXY(car.iLon, car.iLat);
      point++;
      if(point == num_waypoints)
      {
        if( check_loop(c[0],c[point]) ) //check if first and last points are within 1/2 a meter range
        {
          circuit = true;
          c[point].copy(c[0]);
        }
        track.generate_Slopes(c,num_waypoints, circuit); // generate the slopes!TODO : prevent going out of track

        dest_X = c[0].X;
        dest_Y = c[0].Y;
        slope  = c[0].slope;
      }
    }
    else
    {
      //fucc
    }
  }
  if(message == CLEAR_ID)
  {
    num_waypoints = 0;
    point = 0;
    sentinel = 0;
    delete[] c; //clear all waypoints
  }

  if(message == MARK_ID)//TODO: if the waypoints are being marked, shouldn't we still check the looping condition?
  {
    c[point].X = car.X;
    c[point].Y = car.Y;
    c[point].calcLatLon(car.iLon, car.iLat);
    point++;
  }

  if( distancecalcy(car.Y, dest_Y, car.X, dest_X,0) < WP_CIRCLE)
  {
    sentinel++;
    dest_X = c[sentinel].X; //TODO : maybe just pass the object of the coordinate instead of transfering all the values manually.
    dest_Y = c[sentinel].Y;
    slope = c[sentinel].slope; 
  }
  


  /*
   * ADD CODE FOR JEVOIS/COMPANION COMPUTER HERE 
   */
  
  if(MODE == CRUISE || MODE == LUDICROUS || num_waypoints !=0 )//autonomous modes
  {
    track.calculate_Curvatures(car.Velocity, car.X, car.Y, car.heading, dest_X, dest_Y, slope ); //TODO : find only local maxima not global maxima.
    control.driver(track.C, track.braking_distance, car.Velocity, marg.yawRate, marg.La, marg.Ha, MODE, inputs); //send data to driver code. automatically maintains a separate control frequency.
  }
  if(MODE == MODE_PARTIAL || MODE == MODE_MANUAL || MODE == MODE_STOP || MODE == MODE_STANDBY)//manual modes
  {
    float dum[] = {0.0,0.0};//dummy
    control.driver(dum, 1000, car.Velocity, marg.yawRate, marg.La, marg.Ha, MODE, inputs);
  }
  if(T > dt_micros)//in case the execution time exceeds loop time limit
  {
    gcs.Send_Calib_Command(5);
  }
  
  T = max(micros()-timer,T);
  while(micros()-timer < dt_micros ); //dt_micros is defined in PARAMS.h
}
