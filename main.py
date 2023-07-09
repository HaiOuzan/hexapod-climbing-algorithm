#!/usr/bin/python3
from dynamixel_sdk import *
from leg_rp import LEG2
from hexapod_rp import HEXAPOD
import sys

print(sys.version)

# Protocol version
BAUDRATE = 57600  # Dynamixel default baudrate : 57600
DEVICENAME1 = '/dev/ttyUSB0'  # Check which port is being used on your controller
DEVICENAME2 = '/dev/ttyUSB1'  # Check which port is being used on your controller

portHandler1 = PortHandler(DEVICENAME1)
portHandler2 = PortHandler(DEVICENAME2)

# Open port num1
if portHandler1.openPort():
    print("Succeeded to open the first port")
else:
    print("Failed to open the first port")
    print("Press any key to terminate...")
    quit()

# Open port num2
if portHandler2.openPort():
    print("Succeeded to open the second port")
else:
    print("Failed to open the second port")
    print("Press any key to terminate...")
    quit()

# Set ports baudrate
if portHandler1.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate #1")
else:
    print("Failed to change the baudrate #1")
    print("Press any key to terminate...")
    quit()

if portHandler2.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate #2")
else:
    print("Failed to change the baudrate #2")
    print("Press any key to terminate...")
    quit()

# leg objects initializations
# NOTE: leg side should be defined as 'left' or 'right' only. position should be 'forward', 'middle' or 'back' only.

leg1 = LEG2(portHandler1, [11, 12, 13], 'left', 'front')
leg2 = LEG2(portHandler1, [21, 22, 23], 'left', 'middle')
leg3 = LEG2(portHandler1, [31, 32, 33], 'left', 'back')
leg4 = LEG2(portHandler2, [41, 42, 43], 'right', 'front')
leg5 = LEG2(portHandler2, [51, 52, 53], 'right', 'middle')
leg6 = LEG2(portHandler2, [61, 62, 63], 'right', 'back')

legs = [leg1, leg2, leg3, leg4, leg5, leg6]

hexapod = HEXAPOD(legs)

### INPUTS GUIDE ###
# walk inputs: steps_num, direction (= 'forward'), y (= 200), z (= 250), step_size (= 150), resolution (= 6), step_height (= 100), step_dist (= 10)
# spin inputs: direction, resolution (= 6), step_size (= 100), y (= 175), z (= 200), step_height (= 50), step_dist (= 10), steps_num (= 4)
# position inputs: y, z, resolution, parts
# roll inputs: angle, resolution, go_back (= 0), y_const (= 200)
# pitch inputs: angle, resolution, go_back (= 0), y_const (= 200), wait_time (= 0)
# yaw inputs: angle, resolution, y_const (= 200), z_const (= 225), go_back (= 0)


#hexapod.walk(9,y=100,z=200,step_height=75,step_dist=-50,step_size=150,resolution=4)
#hexapod.roll(45,5)
#hexapod.position(250,250)
#hexapod.position(450,250)
hexapod.position(300,250)
#hexapod.roll(45,5)
#hexapod.walk(2)
#hexapod.spin('cw',steps_num=3)
#hexapod.position(200,250)
#hexapod.workmode() 
#hexapod.pitch(-40,3)
#hexapod.yaw(30,5,y_const=350)


# Close ports
portHandler1.closePort()
portHandler2.closePort()


### Demonstration script ###
#hexapod.position(y=200,z=250,resolution=5)
#hexapod.walk(3,resolution=10)
#hexapod.walk(2,resolution=5,y=275,z=250,step_size=100)
#hexapod.position(200,250,5)
#hexapod.spin('cw',steps_num=8)
#hexapod.roll(45,5)
#hexapod.roll(-45,5)
#hexapod.pitch(40,3)
#hexapod.pitch(-40,3)
#hexapod.position(500,200,8,1)
#hexapod.position(350,250,8,1)
#hexapod.yaw(30,5,y_const=350)
#hexapod.yaw(-30,5,y_const=350)
#hexapod.position(150,150,4)