//-----------------------------------HARDWARE RELATED STUFF-------------------------
void readMPU(uint8_t address)   //function for reading MPU values. its about 80us faster than getMotion6() and hey every us counts!
{
  Wire.beginTransmission(address);  //begin transmission with the gyro
  Wire.write(0x3B); //start reading from high byte register for accel
  Wire.endTransmission();
  Wire.requestFrom(address,14); //request 14 bytes from mpu
  //300us for all data to be received. 
  //each value in the mpu is stored in a "broken" form in 2 consecutive registers.(for example, acceleration along X axis has a high byte at 0x3B and low byte at 0x3C 
  //to get the actual value, all you have to do is shift the highbyte by 8 bits and bitwise add it to the low byte and you have your original value/. 
  a[0]=Wire.read()<<8|Wire.read();  
  a[1]=Wire.read()<<8|Wire.read(); 
  a[2]=Wire.read()<<8|Wire.read(); 
  g[0]=Wire.read()<<8|Wire.read();  //this one is actually temperature but i dont need temp so why waste memory.
  g[0]=Wire.read()<<8|Wire.read();  
  g[1]=Wire.read()<<8|Wire.read();
  g[2]=Wire.read()<<8|Wire.read();
}

void readMag(uint8_t address,int k) // a separate function is used during setup() because in the looping mode 
{                         // there is no 10ms delay after enabling the magnetometer, that 10ms of free time is used to read other sensors.
  I2Cdev::writeByte(address,0x37,0x02);
  I2Cdev::writeByte(0x0C, 0x0A, 0x01); //enable the magnetometer
  I2Cdev::readBytes(0x0C, 0x03, 6, buf); // get 6 bytes of data
  m[1] = (((int16_t)buf[1]) << 8) | buf[0]; // the mag has the X axis where the accelero has it's Y and vice-versa
  m[0] = (((int16_t)buf[3]) << 8) | buf[2]; // so I just do this switch over so that the math appears easier to me. 
  m[2] = (((int16_t)buf[5]) << 8) | buf[4]; // I prefer to have a standardized sense of X and Y instead of each sensor having it's own separate X and Y.
  
  for(i=0;i<3;i++)
  {
    M[i] = m[i];//copy into a floating point number
    M[i] -= offsetM[k][i];//remove offset.
  }
}

void readMagDuringSetup(uint8_t address,int k) // a separate function is used during setup() because in the looping mode 
{                         // there is no 10ms delay after enabling the magnetometer, that 10ms of free time is used to read other sensors.
  I2Cdev::writeByte(address,0x37,0x02);
  I2Cdev::writeByte(0x0C, 0x0A, 0x01); //enable the magnetometer
  delay(10);
  I2Cdev::readBytes(0x0C, 0x03, 6, buf); // get 6 bytes of data
  m[1] = (((int16_t)buf[1]) << 8) | buf[0]; // the mag has the X axis where the accelero has it's Y and vice-versa
  m[0] = (((int16_t)buf[3]) << 8) | buf[2]; // so I just do this switch over so that the math appears easier to me. 
  m[2] = (((int16_t)buf[5]) << 8) | buf[4]; // I prefer to have a standardized sense of X and Y instead of each sensor having it's own separate X and Y.
  
  for(i=0;i<3;i++)
  {
    M[i] = m[i];
    M[i] -= offsetM[k][i];
  }
}

//===================================================================================

//-------------------------------INITIALIZATION RELATED STUFF------------------------
void IMU_Setup()
{
  int i,j,k;
  float accel_mean[2][3];
  accelgyro_1.initialize();
  while(!accelgyro_1.testConnection())
  {  
    accelgyro_1.initialize();
    delay(1);
  }
  accelgyro_2.initialize();
  while(!accelgyro_2.testConnection())
  {  
    accelgyro_2.initialize();
    delay(1);
  }
  //--------------------mean----------------------------------------
  for(j=0;j<5000;j++) 
  {
    for(k=0;k<2;k++)
    {
      readMPU(address[k]);//read 1st or 2nd IMU
      for(i=0;i<3;i++)
      {
        offsetG[k][i] += g[i]; 
        accel_mean[k][i] += a[i];//accel offsets may not be the same as the mean value at start!
      }
    }
  }
  for(k=0;k<2;k++)
  {
    for(i=0;i<3;i++)
    {
      offsetG[k][i] *= 0.0002;
      accel_mean[k][i] *= 0.0002;
    }
  }
  //----------------------variance-----------------------------------

  for(j=0;j<5000;j++) 
  {
    for(k=0;k<2;k++)
    {
      readMPU(address[k]);
      for(i=0;i<3;i++)
      {
        variance[0][k][i] += pow((offsetG[k][i]-g[i]),2);
        variance[1][k][i] += pow((accel_mean[k][i]-a[i]),2);
      }
    }
  }
  for(j=0;j<2;j++)
  {
    for(i=0;i<3;i++)
    {
      for(k=0;k<2;k++)
      {
        variance[j][k][i] = sqrt(variance[j][k][i]*0.0002);
      }
      gain[j][i] = variance[j][0][i]/( variance[j][0][i] + variance[j][1][i] );//bite me. I wrote it at 2 AM in the night. If you don't get whats happenning, neither do I.
    }// the basic idea is to get an idea of the variance of each sensor and accordingly set the gains on each field (ax,ay,az,gx,gy,gz)in order to aid in fusing data later.
  }
}
===================================================================================

//-------------------------------DATA CONDITIONING/ANALYSIS------------------------
void readAll()
{
  int i,k;
  for(k=0;k<2;k++)//2 IMUs
  {
    readMPU(address[k]); //200us
    if(g[0]==lastg[k][0]&&g[1]==lastg[k][1]&&g[2]==lastg[k][2])
    {
      failure[k] = 1;
    }
    else
    {
      for(i=0;i<3;i++)
      {
        A[k][i] = a[i];
        A[k][i] -= offsetA[k][i];
        A[k][i] *= 0.0006103;
        A[k][i] = 0.8*A[k][i] + 0.2*lastA[k][i];
        lastA[k][i] = A[k][i];
    
        G[k][i] = g[i];
        G[k][i] -= offsetG[k][i];
        G[k][i] *= 0.030516;
        G[k][i] = 0.8*G[k][i] + 0.2*lastG[k][i];
        lastG[k][i] = G[k][i];
      }//40us
    }
  }
  if(failure[0]&&failure[1])
  {
    //do something
  }
  if(!failure[0]&&!failure[1])
  {
    for(i=0;i<3;i++)
    {
      G[2][i] = (1-gain[0][i])*G[0][i] + gain[0][i]*G[1][i];
      A[2][i] = (1-gain[1][i])*A[0][i] + gain[1][i]*A[1][i]; //weighted average 
    }
  }
  else
  {
    for(i=0;i<3;i++)
    {
      G[2][i] = (!failure[0])*G[0][i] + (!failure[1])*G[1][i];
      A[2][i] = (!failure[0])*A[0][i] + (!failure[1])*A[1][i]; //let the code figure it out in real time. 
    }
  }
  G[2][2] *= depress(G[2][2],1); //suppress noise due to coriolis force or anything weird like that. math functions tab
  
  updateOpticalFlow(); // 160us
  localizer(); // go into the GPS tab to see the inner workings. 
}//724us


float tilt_Compensate(float roll,float pitch) //function to compensate the magnetometer readings for the pitch and the roll.
{
  float heading;
  float cosRoll = my_cos(roll); //putting the cos(roll) etc values into variables as these values are used over and 
  float sinRoll = my_sin(roll); //over, it would simply be a waste of time to keep on calculating them over and over again 
  float cosPitch = my_cos(pitch);//hence it is a better idea to just calculate them once and use the stored values.
  float sinPitch = my_sin(pitch);
  //the following formula is compensates for the pitch and roll of the object when using magnetometer reading. 
  float Xh = -M[0]*cosRoll + M[2]*sinRoll;
  float Yh = M[1]*cosPitch - M[0]*sinRoll*sinPitch + M[2]*cosRoll*sinPitch;
  
  Xh = Xh*0.2 + 0.8*magbuf[0]; //smoothing out the X readings
  magbuf[0] = Xh;

  Yh = Yh*0.2 + 0.8*magbuf[1]; //smoothing out the Y readings
  magbuf[1] = Yh;
  heading = 57.3*atan2(Yh,Xh);
  if(heading<0) //atan2 goes from -pi to pi 
  {
    return 360 + heading; //2pi - theta
  }
  return heading;
}//443us worst case 

void compute_All()
{ 
  float roll_Radians;
  float diff;
  readAll();//724us
  pitch += G[2][0]*dt;
  roll  += G[2][1]*dt; 
  roll_Radians = roll*0.01745;
  //if the car is like going around a banked turn, then the change in heading is not the same as yawRate*dt. cos is an even function.
  diff = dt*(G[2][2]*my_cos(roll_Radians)+G[2][0]*my_sin(-roll_Radians)); //compensates for pitch and roll of gyro itself (roll pitch compensation to the yaw).  
  //mh += G[2][2]*dt;
  mh += diff;
  del += diff;
  if( mod(A[2][1])<2 )
  {
    pitch = 0.99*pitch + 0.573*my_asin(A[2][1]*0.102); //0.102 = 1/9.8
  }
  if( mod(A[2][0])<2 )
  {
    roll  = 0.99*roll  - 0.573*my_asin(A[2][0]*0.102); //using the accelerometer to correct the roll and pitch.
  }
  if(mh >= 360.0) // the mh must be within [0.0,360.0]
  {
    mh -= 360.0;
  }
  if(mh < 0)
  {
    mh += 360;
  }
  yawRate = G[2][2]; // yaw rate is probably used for steering PID.
}//1124us worst case 

================================================================================
