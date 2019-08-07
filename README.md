## Self-driving-car-STM-32

DISCLAIMER : This project is not as generic as the arduRover project and is not trying to achieve the same goals, although it was inspired by it.

### About the project
This project is supposed to act as a lower level controller for higher level agents. The car can be given a point (X,Y,theta) relative to it's current location. The car can also reactively avoid obstacles if ultrasonic sensors are present in the build. Ideally, the higher level agent should give waypoints that do not force the car to go through an obstacle (that's kind of the point). The lower level controller takes care of figuring out the throttle and steering control for getting to a particular point.

### Odometry
The odometry is obtained by fusing data from GPS, optical flow, IMU+compass and also from a simplistic model of the car's propulsion system. The state estimator also exploits the non-holonomic constraints. Using multiple sources of information allows the car to operate with a high degree of accuracy (long term) when all the sensors are operating in peak condition and with an acceptable level of accuracy when one or two of the sensors are not in peak condition. The odometry assumes that the car is moving on a flat, horizontal plane (fair assumption for a car that has a ride height of 15 mm)

### Control
The control is based on bezier curve(3rd order) based trajectory generation. The reason for selecting 3rd degree polynomial instead of 7 is simplicity. At the moment, the generated trajectory is not "optimal". There is however, support for pre-emptive braking. The speed control uses an asymmetric controller as the car does not speed up and slow down in the same fashion, i.e., when the throttle value is above neutral, the ESC controls the steady state speed (in an open-loop fashion) but when the throttle value is below neutral, the ESC makes the motor apply a braking torque (a steady force proportional to the control signal). The speed control takes as feedback the error in speed, the current acceleration and the current yaw rate of the car. The steering control is an open + closed loop progressive P controller (progressive as in the gain increases with the error, like in a progressive spring). The combination of these 2 controllers allows the car to remain under control for the most part even without preemptive braking.

# Note for potential users : 
The car will not operate without a GCS by default. This is for safety purposes and not for the sake of functionality. If the car were to operate without a GCS connection and only be in control of the user via the on board Radio control, there would be a single point of failure in communications. Adding the GCS-compulsion gives the system 2 independent points of failure. This also compulsorily limits the range of operation (which depends on the kind of transceiver used. For xbee pros, this range is ~80 meters, which is more than enough for doing experiments with a 1/10 scale car.)


## Photos of this project : 
![img_20181223_232752](https://user-images.githubusercontent.com/24889667/51115201-90983580-182d-11e9-9a05-9175e0551990.jpg)

![img_20190114_182337](https://user-images.githubusercontent.com/24889667/51115209-968e1680-182d-11e9-9db0-57a545443a52.jpg)


## Building the project : 

(will update this as I build this project)

For the hardware specifications and 3d print files as well as 2 d drawings, go into the "hardware folder". 

For the back end libraries, go into the "libraries folder".(surprise surprise).

1) You will need : Arduino IDE (1.6.13 or better)

2) You will have to download the hardware files from : https://drive.google.com/file/d/1j8or7khmo2Z-QlrW-FHwhAybPbwVp9Ex/view?usp=sharing (its a slightly modified version of the original fork, I don't actually remember what changes I made but somehow the code after compilation takes 5 kB less memory) and unzip/extract it inside the "Arduino/hardware folder. You will also need to install the cortex M3 SAMD board package in arduino, which can be installed easily through the boards manager in arduino IDE. Use this video for reference, the step of cloning the STM32 repository is replaced by downloading the zip and the rest remains the same : https://www.youtube.com/watch?v=MLEQk73zJoU&t=295s

3) You will need Python 3 (it may or may not work on python 2, I have only tried it with python 3.5 and 3.6.5).
Python dependencies : 
Matplotlib,numpy, pyserial,scikit-learn :
```
pip3 install matplotlib
pip3 install numpy 
pip3 install pyserial
pip3 install scikit-learn
```
