# Why I moved on from the Arduino Pro Mini:

## Why improve the hardware?

Problems with Arduino Pro Mini:

1) Not enough UART ports (required for communication with GCS and a higher level system like idk a raspi or a jevois camera)

2) Not enough processing speed to maintain the same 400Hz state-estimation frequency with any added features (like sensor health checks). I was shaving off like 10-15 us per function minimum just to meet the requirements (although this did force me to come up with rather ingenious approximations which I have carried over to this project, so if you use my "math" functions (in the SIDMATH.h header) you can forget worrying about execution time of your algorithms- Its not a full blown math library because it isn't meant to be. It just includes some functions that I used very often like sin and cos my approximations are pretty close and have a worst case execution time which is 1/3rd (if i remember correctly) of the time taken by arduino's sin and cos. ).


Problems with previous layout of hardware:

1) The optical flow sensor was outside of the boundary of the car(behind the car) this caused there to be an unwanted cg offset + ugliness. The optical flow sensor now sits at the center of the car (at the cost of front wheel power. The car is now RWD).

2) Everything was connected by jumper wires. Yes. Really. In the new version, everything is soldered directly. 


Problem with previous hardware in general (this doesn't come under the above 2 categories) : 

1) The last and final crash of my previous iteration happened on July 22nd 8:30 AM. It happened because the MPU got disconnected mid-communications, which locked up the I2C line, producing a race-around condition. Due to this, the car could no longer be sent into a failsafe as it had gone into a lockout state. The car went at 7 m/s into a wall at an oblique angle, taking out the right front suspension.
For this reason I decided to put 10k resistors between SCL,SDA and VCC to prevent the race-around condition and instead of having just the main controller, have a slave controller that acts as a gateway between the car's hardware and the main controller, so that in case the main controller goes haywire, the slave controller can still shut the car down. How do I know this slave controller won't go into some lockout mode of it's own? I have tested the back end for it over 2 years and it has never failed (since it only uses interrupts and servo library and nothing else). It's sole job is to make sure that the car never goes completely out of control. (I am still on the fence about this though).

So whats the alternative? the STM32F103C8T6 (surprise surprise!) (a.k.a. the blue pill). The blue pill has 3 UARTs, 2 SPIs and 2 I2Cs (although I only use 1 I2C and 1 SPI port in this project, its nice to have options), 20kb of sram, 64 kb of flash memory (compared to 2 kb of sram and 32 kb of flash memory on mini pro), and a clock speed of 128MHz (after overclocking, which can be done from within the arduino IDE itself). Going from a Pro mini to the blue pill feels like going from a handgun to a bazooka.

Why improve the code? 
The right question would be, why not? The previous version had certain (kidding, a lot of) flaws like lack of modularity (each sub-module should be able to function somewhat on it's own and not be dependent on some other module entirely. If you look at my previous code, just about all variables are global. It gets hard even on the person who created it after a while (this is why object oriented programming is used kids) ) as well as misplaced functionalities (like incrementing waypoint index in the code block that is supposed to control the steering and throttle. Yes. Really. (cut me some slack I m still learning) ). 

In this version, I wrote back end libraries for just about everything (FYI this is my first time writing back end libraries) from IMU to the communication systems, so that I could divide things into sub-modules. This greatly cleans up the main code (though it's still not that clean). However, everything related to AHRS, from getting sensor data to calculating AHRS happens in the same library so that all the helper functions that could be used are in the same file.

If you know how to write code for a GCS in python, please create an issue and let me know, I need some help there. I can't seem to integrate google maps into this. Till then, Waypoints will be marked by driving the car to those waypoints in manual mode.

## Have I forsaken Arduino? 
No. The code is still compiled by the Arduino IDE. The main code that runs the car is still written in Arduino IDE (bite me). Why not just work with pixhawk (Ardupilot)? 

1) I am too dumb to understand what they're doing there. I am just not up there yet. So its better that I do not fiddle with things I do not understand.

2) I am trying to learn these things on my own. If I use the Ardupilot backend I will never really understand how these things work and what you need to take care of (corner cases, exceptions etc). 

3) If I have my own system, its easier for me to integrate new things into it since I don't have to comply to standards set by someone else. While standards are very useful at the professional level, they can slow down the process of testing and between my undergrad studies, academic projects, research papers and this, I do not have the time to go through these things like a professional.

4) If you wanna create a project using arduino supported mcus, you install the Arduino IDE and maybe some back end support (like back end support for STM32 controllers) which is also downloaded from within the Arduino IDE. Its super simple and you can get started with doing actual work. If you wanna create a project using pixhawk and you're on Windows, may god help you. 

Basically we need something like Multi-Wii but for r/c race cars instead of drones and we need it to be somewhat competitive against the apm rover. We also need it to be a bit more focused on racing, which means a bit more focus on the driving dynamics of the car (which AFAIK the apm does not really have since they simply minimize the angle difference between the direction where they wanna go and where they are going right now, which is fine and even required for WP missions but for racing you need the car to follow a smooth trajectory).
