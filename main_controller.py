#!/usr/bin/python3
from dynamixel_sdk import *
from leg_rp import LEG2
from hexapod_rp import HEXAPOD
import pygame
pygame.init()

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


pygame.joystick.init()
J = pygame.joystick.Joystick(0)
J.init()

if pygame.joystick.get_init() == True:
    print('Found joystick!')
    print(pygame.joystick.Joystick(0).get_name())
else:
    print('cant find joystick')
    quit()


### INPUTS GUIDE ###
# walk inputs: steps_num, direction (= 'forward'), y (= 200), z (= 250), step_size (= 150), resolution (= 6), step_height (= 100), step_dist (= 10)
# spin inputs: direction, resolution (= 6), step_size (= 100), y (= 175), z (= 200), step_height (= 50), step_dist (= 10), steps_num (= 4)
# position inputs: y, z, resolution, parts
# roll inputs: angle, resolution, go_back (= 0), y_const (= 200)
# pitch inputs: angle, resolution, go_back (= 0), y_const (= 200), wait_time (= 0)
# yaw inputs: angle, resolution, y_const (= 200), z_const (= 225), go_back (= 0)

h_stair = 50
z = hexapod.position(250,200,3,1)
a=1
while True:
    events = pygame.event.get()
    for event in events:
        if J.get_hat(0) == (0,1):
            #print('forward')
            #hexapod.walk(3,resolution=8,step_size=175)
            #hexapod.reach_stair()
            #hexapod.position(250,200,3,1)
            hexapod.COM(pair="f_legs")
            #time.sleep(5)
            #hexapod.climb_gait(1, pair="f_legs")
            #time.sleep(10)
            #hexapod.walk_gait(1, step_size=200, resolution=4, pair="m_legs")
            #time.sleep(10)
            #hexapod.COM(pair="r_legs")
            #time.sleep(10)
            #hexapod.climb_gait(1, step_size=150, resolution=4, pair="m_legs")
            #time.sleep(10)
            #hexapod.COM(pair="r_legs", count=2)
            #time.sleep(10)
            #hexapod.walk_gait(1, step_size=200, resolution=4, pair="r_legs")
            #time.sleep(10)
            #hexapod.rise_position()
            #time.sleep(10)
            #hexapod.COM(pair="r_legs", count=3)
            #time.sleep(10)
            #hexapod.climb_gait(1, step_size=150, resolution=4, pair="r_legs")
            #time.sleep(10)
            #hexapod.position(250,200,3,1)
            #time.sleep(10)
        elif J.get_hat(0) == (0,-1):
            print('backwards')
            hexapod.walk(2,'backwards')
        elif J.get_hat(0) == (1,0):
            print('right')
# spin inputs: direction, resolution (= 6), step_size (= 100), y (= 175), z (= 200), step_height (= 50), step_dist (= 10), steps_num (= 4)
            hexapod.spin('cw',steps_num=2, y=200, z=250)
        elif J.get_hat(0) == (-1,0):
            print('left')
            hexapod.spin('ccw',steps_num=2, y=200, z=250)
        elif J.get_button(0) == 1:
            hexapod.roll(45,3,1)
            hexapod.roll(-45,3)
        elif J.get_button(1) == 1:
            hexapod.pitch(40,2,1)
            hexapod.pitch(-40,2)
        elif J.get_button(6) == 1:
            if z >= 150:
                z = hexapod.position(z-10,250)
            else:
                print('Cant Go any Lower')
        elif J.get_button(7) == 1:
            if z <= 540:
                z = hexapod.position(z+10,250)
            else:
                print('Cant Go any Higher')
        elif J.get_button(3) == 1:
            if a == 1:
                hexapod.workmode()
                a =0
            elif a == 0:
                hexapod.position(250,250,2,8)
                a=1
        elif J.get_button(4) == 1:
            hexapod.yaw(20,2)
            print('Y')




# Close ports
portHandler1.closePort()
portHandler2.closePort()

