# Self-driving-car-STM-32
do not clone it yet! its a work in progress. Its an improved version of the self driving car. It runs on the stm32 bluepill (STM32F103C8T6) BUT it is not yet even in it's testing phase. I am still working on it.

Why improve the hardware?
Problems with Arduino Pro Mini:
1)Not enough UART ports (required for communication with GCS or a vision system)
2)Not enough processing speed to maintain the same 400Hz state-estimation frequency. I was shaving off like 10-15 us per function minimum
just to meet the requirements (although this did force me to come up with rather ingenious approximations).

Problems with previous layout of hardware:
1)The optical flow sensor was outside of the boundary of the car(behind the car) this caused there to be an unwanted cg offset + ugliness.
The optical flow sensor now sits at the center of the car (at the cost of front wheel power. The car is now RWD).
2)Everything was connected by jumper wires. Yes. Really. In the new version, everything is soldered directly. 

Problem with previous hardware in general (this doesn't come under the above 2 categories) : 
1)The last and final crash of my previous iteration happened on July 22nd 8:30 AM. It happened because the MPU got disconnected mid-communications, which locked up the I2C line, producing a race-around condition. Due to this, the car could no longer be sent into a failsafe as it had gone into a lockout state. The car went at 7 m/s into a wall at an oblique angle, taking out the right front suspension.
Which is why I decided to put 10k resistors between SCL,SDA and VCC to prevent the race-around condition and instead of having just the main controller, have a slave controller that acts as a gateway between the car's hardware and the main controller, so that in case the main controller goes haywire, the slave controller can still shut the car down. How do I know this slave controller won't go into some lockout mode of it's own? I have tested the back end for it over 2 years and it has never failed (since it only uses interrupts and servo library and nothing else). It's sole job is to make sure that the car never goes completely out of control. 

Why improve the code? 
The right question would be, why not? The previous version had certain flaws like lack of modularity (each sub-module should be able to function somewhat on it's own and not be dependent on some other module entirely) as well as misplaced functionalities (like incrementing waypoint index in the code block that is supposed to control the steering and throttle. Yes. Really.) 
If you know how to write code for a GCS in python, please create an issue and let me know, I need some help there. I can't seem to integrate google maps into this. Till then, Waypoints will be marked by driving the car to those waypoints in manual mode.

Apart from the libraries in the "libraries" folder, you will have to clone the following repository : https://github.com/naughtyStark/Arduino_STM32 and follow the instructions in this video for setting up your arduino IDE :
https://www.youtube.com/watch?v=MLEQk73zJoU&t=295s (its pretty simple as arduino already supports it and does not affect your other stuff in arduino so chill)
