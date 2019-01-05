# Hardware Specifications for this project

## The car : 
1) HPI sprint 2 (why? its a good balance between price and quality). I would suggest the "All aluminium upgrade".

2) ESC and motor : you're free to chose any ESC and motor combination but I like to run a 120 A ESC (hobbywing) and a (I use speedpassion but you can use any) 3500 kv sensored motor. (if you run the exact same hardware then you don't have to change a thing in the code).
3) Steering servo : SAVOX SC-1251MG . (any servo with equivalent or better torque and speed will work just fine. Any slower and you might face problems with oversteer).
4) Spur and pinion : 31 teeth pinion and 80-81 teeth spur (48 pitch both). if you use a different gearing ratio you'll have to calculate the maximum speed for your setup and put that number in the "CAR.h" file inside "libraries".
5) Drivetrain layout : RWD (this is to make space for the autopilot)
6) Suspension :
    1) Springs and dampers : standard HPI sprint 2 springs with 300 cst silicon oil all around.
    2) Steering geometry : 1 degree TOE-IN. (for stability at speeds). mount the steering links on the outer holes of the steering arms.
    3) Camber : Front = 0.5 or 0 degrees (Front end doesn't need that much camber). Rear = 2 degrees.
    4) Ride height : 25 mm all around. Think of this car as more of a road-rally sort of thing than a pure-bred racer.
7) Battery : 5000-6000 mAh 2S lipo. C rating may or may not matter depending upon your use case. I use 70C batteries from turnigy trackstar.

## The autopilot : 
1) The designs (obviously)
2) 1 x STM32F103C8T6 (if you get the CBT6 that works too) and 1 x Pro mini.
3) 1 x MPU9250s.
4) 1 x ADNS3080 optical flow sensor.
5) 1 x Ublox M8N GPS (no config required, the code will do the config automatically for you).
6) 1 x Jevois A33 (optional) and a usb mini-b male adapter.
7) 2 x Xbee Pro with Xbee pro adapter (one for the car and one for your computer).
8) 1 x pwm to ppm converter (like the one used for Ardupilot).
9) 1 x any radio control transmitter-receiver (for driving the car manually as well as failsafe).
10) 1 x 5V/5A BEC (the one that you get from bang-good is just fine. It can actually only handle 3A without a voltage sag, but you can push it to 5. We won't actually need 5 A at all. We'll need ~1-1.2 A (this includes the current consumed by the jevois camera. If you're not using the jevois camera, the current consumption will be much lower, however I would still recommend using a separate BEC). The BEC
11) 1 x LED taken out from a computer mouse (please do not use a standard red LED. Just take the LED out from an old mouse. 
12) 1 x dot board/Zero pcb, single core wires, solder, male headers.
13) I might add more things for obstacle avoidance later. Right now I m considering the use of the camera itself to detect obstacles or run image segmentation

# Putting it together : (images will be up soon)
1) The chassis and chassis brace drawings are .dwg files because it has no 3d features and you can do a cnc(1.5 mm offset) cut. If someone asks for it I will upload an stl file if you want to 3d print it
2) The LED holder sits on top of the chassis above the slot. The holes have been cut into the chassis.
3) the chassis brace has holes for mounting the container base. mount the container on the chassis brace before adding any components to the autopilot.
4) the MPUs are to be situated at the converging end of the container. The optical flow is placed in the raised slot. The GPS is placed on the container top behind the jevois camera.
5) the xbee is situated near the motor, so is the BEC and the pro mini.
6) The MPU is attached to pins B6 (SCL), B7 (SDA). One of the MPU's AD0 pin has to be connected to Vcc (5 V)
7) The Optical flow is attached to pins PA4 (NSS) PA5 (SCK) PA6 (MISO) PA7 (MOSI) PB0 (RESET)
8) The GPS is attached to PA2 (Tx, Rx of gps connected here) PA3 (Rx, Tx of gps connected here)
9) The Xbee is connected to PA9, PA10. Create a 6 pin male header that can accept an FTDI converter and connect the Xbee (and only the xbee) with jumper wires that can be removed later).
10) The Jevois is connected to pins PB11 (white wire), PB10 (yellow wire).
11) The extend a wire from pin PA8, PA11. These are servo and ESC wires.
12) Connect the PPM encoder to PA0.
