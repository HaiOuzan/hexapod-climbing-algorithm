#!/usr/bin/python3
import numpy as np
from dynamixel_sdk import *  # Uses Dynamixel SDK library
from leg_rp import LEG2
import math
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class HEXAPOD:
    def __init__(self, legs):
        self.legs = legs

    def pos_fix(self, pos, resolution, parts=2):
        pos_fix = []
        max_samples = 0

        for leg_num in range(6):
            present_pos = self.legs[leg_num].motors_angles()
            des_pos = self.legs[leg_num].loc2deg(pos[0][leg_num][0], pos[1][leg_num][0], pos[2][leg_num][0])
            max_values = max(abs(present_pos[0] - des_pos[0]), abs(present_pos[1] - des_pos[1]),
                              abs(present_pos[2] - des_pos[2]))
            pos_fix.append(np.ndarray.tolist(np.linspace(present_pos, des_pos, int(max_values / resolution)).astype(int)))
            if max_values > max_samples:
                max_samples = max_values

        if parts == 1:
            for it in range(int(max_samples/resolution)):
                for leg_num in range(6):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
        elif parts == 2:
            for it in range(int(max_samples / resolution)):
                for leg_num in (0, 2, 4):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
            for it in range(int(max_samples / resolution)):
                for leg_num in (1, 3, 5):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
        elif parts == 3:
            for it in range(int(max_samples / resolution)):
                for leg_num in (0, 5):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
            for it in range(int(max_samples / resolution)):
                for leg_num in (1, 4):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
            for it in range(int(max_samples / resolution)):
                for leg_num in (2, 3):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
        elif parts == 4:
            for it in range(int(max_samples / resolution)):
                for leg_num in (2, 5):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
            for it in range(int(max_samples / resolution)):
                for leg_num in (1, 4):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
            for it in range(int(max_samples / resolution)):
                for leg_num in (0, 3):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
        elif parts == 8:
            for it in range(int(max_samples / resolution)):
                for leg_num in (0, 3):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
            for it in range(int(max_samples / resolution)):
                for leg_num in (1, 4):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
            for it in range(int(max_samples / resolution)):
                for leg_num in (2, 5):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                    else:
                        pass
        else:
            for leg_num in (0,3,5,2,1,4):
                for it in range(int(max_samples / resolution)):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])

    def walk(self,steps_num, direction='forward', y=250, z=200, step_size=150, resolution=6, step_height=100, step_dist=10):
        # Moves the robot forward according to received parameters.

        samples = int(step_size/resolution)

        x_f = []                                            # array of x value sets for six legs. 'f' stands for 'forward motion'. leg0 = 0
        y_f = []                                            # array of y value sets for six legs (etc.)
        z_f = []                                            # array of z value sets for six legs (etc.)

        x_b = []                                            # array of x value sets for six legs. 'b' stands for 'backwards motion'. leg0 = 0
        y_b = []                                            # etc..
        z_b = []

        for num, leg in zip(range(6), self.legs):  # creating value sets
            if direction == 'forward':
                x_f.append(leg.x_forward(step_size, resolution))  # calculating x values for forward movement
            elif direction == 'backwards':
                x_f.append(leg.x_backwards(step_size, resolution))    # calculating x values for backwards movement
            else:
                print('walking direction is defined wrong (should be "forward"/"backwards" only). Please re-define')
                quit()

            y_f.append([y] * samples)
            z_f.append([z] * samples)

            x_f_f, y_f_f = leg.pos_fix(x_f[num], y_f[num])  # rotating x and z values to leg axis system
            x_f[num] = x_f_f
            y_f[num] = y_f_f

            x_b.append(np.flip(x_f[num]))  # creating values set for x backwards movement (opposite of x_f)

            z_b1 = np.linspace(z, z - step_height, int(samples / 2))
            z_b2 = np.linspace(z - step_height, z, int(samples / 2))
            z_b.append(np.ndarray.tolist(np.append(z_b1, z_b2)))

            y_b1 = np.linspace(y_f[num][-1], y_f[num][-1] - step_dist, int(samples / 2))
            y_b2 = np.linspace(y_f[num][-1] - step_dist, y_f[num][0], int(samples / 2))
            y_b.append(np.ndarray.tolist(np.append(y_b1, y_b2)))

            if num % 2 != 0:                                # switching double legs (0,2,4) values
                x_f[num], x_b[num] = x_b[num], x_f[num]
                y_f[num], y_b[num] = y_b[num], y_f[num]
                z_f[num], z_b[num] = z_b[num], z_f[num]

            if direction == 'backwards':
                x_f[num], x_b[num] = x_b[num], x_f[num]
                y_f[num], y_b[num] = y_b[num], y_f[num]
                z_f[num], z_b[num] = z_b[num], z_f[num]

        self.pos_fix([x_f, y_f, z_f], 8, 6)

        for step in range(steps_num):
            print('\nFirst half cycle')
            for it in range(samples-1):
                for leg_num, i, j, k in zip(range(6), x_f, y_f, z_f):
                    # print(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                    print('{} {} {}'.format(round(i[it]), round(j[it]), round(k[it])))
                    self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                time.sleep(0.01)

            print('\nSecond half cycle')
            for it in range(samples-1):
                for leg_num, i, j, k in zip(range(6), x_b, y_b, z_b):
                    # print(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                    print('{} {} {}'.format(round(i[it]),round(j[it]),round(k[it])))
                    self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                time.sleep(0.001)

    def spin(self, direction, resolution = 6, step_size = 100, y = 200, z = 175, step_height = 50, step_dist = 10, steps_num = 7):
        x_f = [[], [], [], [], [], []]

        x_f[0] = range(step_size, 0, -resolution)
        x_f[1] = range(round(step_size / 2), -round(step_size / 2), -resolution)
        x_f[2] = range(0, -step_size, -resolution)
        x_f[3] = range(0, step_size, resolution)
        x_f[4] = range(-round(step_size / 2), round(step_size / 2), resolution)
        x_f[5] = range(-step_size, 0, resolution)
        if direction == 'cw':
            pass
        elif direction == 'ccw':
            x_f[0] = np.flip(x_f[0])
            x_f[1] = np.flip(x_f[1])
            x_f[2] = np.flip(x_f[2])
            x_f[3] = np.flip(x_f[3])
            x_f[4] = np.flip(x_f[4])
            x_f[5] = np.flip(x_f[5])
        else:
            print('Invalid direction. Please choose cw or ccw only')
            quit()

        y_f = []  # array of y value sets for six legs (etc.)
        z_f = []  # array of z value sets for six legs (etc.)

        x_b = []  # array of x value sets for six legs. 'b' stands for 'backwards motion'
        y_b = []  # etc..
        z_b = []

        samples = int(step_size/resolution)

        for num, leg in zip(range(6), self.legs):  # creating value sets

            y_f.append([y] * samples)
            z_f.append([z] * samples)

            x_b.append(np.flip(x_f[num]))  # creating values set fot x backwards movement (opposite of x_f)

            z_b1 = np.linspace(z, z - step_height, int(samples / 2))
            z_b2 = np.linspace(z - step_height, z, int(samples / 2))
            z_b.append(np.ndarray.tolist(np.append(z_b1, z_b2)))

            y_b1 = np.linspace(y_f[num][-1], y_f[num][-1] - step_dist, int(samples / 2))
            y_b2 = np.linspace(y_f[num][-1] - step_dist, y_f[num][0], int(samples / 2))
            y_b.append(np.ndarray.tolist(np.append(y_b1, y_b2)))

            if num % 2 != 0:
                x_temp = x_f[num]
                y_temp = y_f[num]
                z_temp = z_f[num]

                x_f[num] = x_b[num]
                y_f[num] = y_b[num]
                z_f[num] = z_b[num]

                x_b[num] = x_temp
                y_b[num] = y_temp
                z_b[num] = z_temp

            if direction == 'ccw':
                x_f[num], x_b[num] = x_b[num], x_f[num]
                y_f[num], y_b[num] = y_b[num], y_f[num]
                z_f[num], z_b[num] = z_b[num], z_f[num]

        self.pos_fix([x_f, y_f, z_f], 8, 3)

        for step in range(steps_num):
            print('\nFirst half cycle')
            for it in range(samples - 1):
                for leg_num, i, j, k in zip(range(6), x_f, y_f, z_f):
                    # print(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                    print('{} {} {}'.format(round(i[it]), round(j[it]), round(k[it])))
                    self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                time.sleep(0.001)

            print('\nSecond half cycle')
            for it in range(samples - 1):
                for leg_num, i, j, k in zip(range(6), x_b, y_b, z_b):
                    # print(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                    print('{} {} {}'.format(round(i[it]), round(j[it]), round(k[it])))
                    self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                time.sleep(0.001)

    def pitch(self, angle, resolution, go_back = 0, z_const = 200, wait_time = 0):
        dist_mid = 0
        dist_non_mid = 127.3
        angles = range(0, abs(angle) + 1, resolution)
        front_offset = 200
        back_offset = front_offset * -1
        y_const = 250

        x_a = [[],[],[],[],[],[]]
        y_a = [[],[],[],[],[],[]]
        z_a = [[],[],[],[],[],[]]

        for num in range(6):
            if num == 1 or num == 4:
                dist = dist_mid
            else:
                dist = dist_non_mid

            if num == 2 or num == 5:
                for ang in angles:
                    z_a[num].append(z_const+(dist*np.sin(np.deg2rad(ang))))
                    y_a[num].append(y_const-(dist-dist*np.cos(np.deg2rad(ang))))
            elif num == 0 or num == 3:
                for ang in angles:
                    z_a[num].append(z_const-(dist*np.sin(np.deg2rad(ang))))
                    y_a[num].append(y_const+(dist-dist*np.cos(np.deg2rad(ang))))
            else:
                for ang in angles:
                    z_a[num].append(z_const + 15)
                    y_a[num].append(y_const + 25)

        for num in range(6):
            for i in range(len(z_a[num])):
                if self.legs[num].position == 'front':
                    x_a[num].append(front_offset)
                elif self.legs[num].position == 'back':
                    x_a[num].append(back_offset)
                else:
                    x_a[num].append(0)

            x_a_f,y_a_f = self.legs[num].pos_fix(x_a[num], y_a[num])
            x_a[num] = x_a_f
            y_a[num] = y_a_f

        if angle < 0:
            for i in [0,3]:
                x_a[i], x_a[2 + i] = x_a[2 + 1], x_a[i]
                y_a[i], y_a[2 + i] = y_a[2 + i], y_a[i]
                z_a[i], z_a[2 + i] = z_a[2 + i], z_a[i]

        self.pos_fix([x_a, y_a, z_a], resolution*2, 1)

        for it in range(len(angles)):
            for num in range(6):
                self.legs[num].set_leg_pos(self.legs[num].loc2deg(round(x_a[num][it]),round(y_a[num][it]),round(z_a[num][it])))
                print(round(x_a[num][it]),round(y_a[num][it]),round(z_a[num][it]))
                time.sleep(wait_time)
        if go_back == 0:
            self.pos_fix([x_a, y_a, z_a], 8, 1)
            time.sleep(wait_time)

    def yaw(self, angle, resolution, go_back = 0, z_const = 200, y_const = 225): # angle < 40!
        step_size = angle*5
        corr = 1
        if angle<0:
            step_size = abs(step_size)
            corr = -1
        # d_z = 174.25

        x_f = [[], [], [], [], [], []]

        for i in range(3):
            x_f[i] = range(0, -step_size*corr, -resolution*corr)
        for i in  range(3,6):
            x_f[i] = range(0, step_size*corr, resolution*corr)

        y_f = []  # array of y value sets for six legs (etc.)
        z_f = []  # array of z value sets for six legs (etc.)

        samples = int(step_size / resolution)

        for num in range(6):  # creating value sets
            z_f.append([z_const] * samples)
            y_f_f  = []
            for x in x_f[num]:
                y_f_f.append(np.sqrt(y_const**2 - x**2))
            y_f.append(y_f_f)

        self.pos_fix([x_f, y_f, z_f], 5, 1)

        for it in range(samples - 1):
            for leg_num, i, j, k in zip(range(6), x_f, y_f, z_f):
                # print(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                print('{} {} {}'.format(round(i[it]), round(j[it]), round(k[it])))
                self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
            time.sleep(0.001)
        time.sleep(0.5)
        if go_back == 0:
            self.pos_fix([x_f, y_f, z_f], 8, 1)

    def position(self, z, y, resolution=2, parts=1):
        # Changes the robot's height

        x_f = []                                            # array of x value sets for six legs. 'f' stands for 'forward motion'. leg0 = 0
        y_f = []                                            # array of y value sets for six legs (etc.)
        z_f = []                                            # array of z value sets for six legs (etc.)

        x_b = []                                            # array of x value sets for six legs. 'b' stands for 'backwards motion'. leg0 = 0
        y_b = []                                            # etc..
        z_b = []

        samples = int(100/resolution)

        for num, leg in zip(range(6), self.legs):  # creating value sets
            x_f.append([0] * samples)  # calculating x values for forward movement
            y_f.append([y] * samples)
            z_f.append([z] * samples)

        self.pos_fix([x_f, y_f, z_f], resolution*2, parts)
        return z

    def workmode(self, z=250, h=-250, resolution =4, parts =4):
        x_t = []
        y_t = []
        z_t = []

        samples = int(100 / resolution)

        for num, leg in zip(range(6), self.legs):  # creating value sets
            if self.legs[num].position == 'front':
                x_t.append(np.linspace(80, 380, samples))
                y_t.append([200] * samples)
                z_t.append(np.linspace(z, h, 30))
            elif self.legs[num].position == 'middle':
                x_t.append([120]*len(x_t[0]))
                y_t.append([350] * len(x_t[0]))
                z_t.append([z] * len(x_t[0]))
            elif self.legs[num].position == 'back':
                x_t.append([-250]*len(x_t[0]))
                y_t.append([250] * len(x_t[0]))
                z_t.append([z-100] * len(x_t[0]))
            else:
                quit()

            x_f_f, y_f_f = leg.pos_fix(x_t[num], y_t[num])  # rotating x and z values to leg axis system
            x_t[num] = x_f_f
            y_t[num] = y_f_f

        self.pos_fix([x_t, y_t, z_t], resolution , parts)

        for it in range(samples):
            for num in range(6):
                self.legs[num].set_leg_pos(self.legs[num].loc2deg(round(x_t[num][it]),round(y_t[num][it]),round(z_t[num][it])))
                print(round(x_t[num][it]),round(y_t[num][it]),round(z_t[num][it]))

    def roll(self, angle, resolution, go_back = 0, z_const = 200):
        dist_mid = 174.25
        dist_non_mid = 139.26
        angles = range(0, abs(angle) + 1, resolution)
        front_offset = 120
        back_offset = front_offset * -1
        y_const = 250

        x_a = [[],[],[],[],[],[]]
        y_a = [[],[],[],[],[],[]]
        z_a = [[],[],[],[],[],[]]

        for num in range(6):
            if num == 1 or num == 4:
                dist = dist_mid
            else:
                dist = dist_non_mid

            if num < 3:
                for ang in angles:
                    z_a[num].append(z_const+(dist*np.sin(np.deg2rad(ang))))
                    y_a[num].append(y_const-(dist-dist*np.cos(np.deg2rad(ang))))
            else:
                for ang in angles:
                    z_a[num].append(z_const-(dist*np.sin(np.deg2rad(ang))))
                    y_a[num].append(y_const+(dist-dist*np.cos(np.deg2rad(ang))))

        for num in range(6):
            for i in range(len(z_a[num])):
                if self.legs[num].position == 'front':
                    x_a[num].append(front_offset)
                elif self.legs[num].position == 'back':
                    x_a[num].append(back_offset)
                else:
                    x_a[num].append(0)
            x_a_f,y_a_f = self.legs[num].pos_fix(x_a[num],y_a[num])
            x_a[num] = x_a_f
            y_a[num] = y_a_f

        if angle < 0:
            for i in range(3):
                x_a[i],x_a[3+i] = x_a[3+i],x_a[i]
                y_a[i],y_a[3+i] = y_a[3+i],y_a[i]
                z_a[i],z_a[3+i] = z_a[3+i],z_a[i]

        self.pos_fix([x_a, y_a, z_a], resolution*3, 1)

        for it in range(len(angles)):
            for num in range(6):
                self.legs[num].set_leg_pos(self.legs[num].loc2deg(round(x_a[num][it]), round(y_a[num][it]), round(z_a[num][it])))
                print(round(x_a[num][it]), round(y_a[num][it]), round(z_a[num][it]))

        if go_back == 0:
            self.pos_fix([x_a, y_a, z_a], 8, 1)

    def height_control(self, z, y, resolution=2, parts=1):
        # Changes the robot's height

        x_f = []  # array of x value sets for six legs. 'f' stands for 'forward motion'. leg0 = 0
        y_f = []  # array of y value sets for six legs (etc.)
        z_f = []  # array of z value sets for six legs (etc.)

        x_b = []  # array of x value sets for six legs. 'b' stands for 'backwards motion'. leg0 = 0
        y_b = []  # etc..
        z_b = []

        samples = int(100 / resolution)

        for num, leg in zip(range(6), self.legs):  # creating value sets
            x_f.append([0] * samples)  # calculating x values for forward movement
            y_f.append([y] * samples)
            z_f.append([z] * samples)

    def find_tips(self):

        angles_to_motors = [math.radians(45), math.radians(90), math.radians(135),
                            math.radians(-45), math.radians(-90), math.radians(-135)]
        distance_to_motor = [118.5, 112.75, 118.5, 118.5, 112.75, 118.5]  # mm

        left = [0, 1, 2]

        l1 = 51.5
        l2 = 210.54
        l3 = 372.4

        tip = [[l3], [0], [0], [1]]

        legs_tip = []

        for leg_num in range(6):

            A = [[math.cos(angles_to_motors[leg_num]), -math.sin(angles_to_motors[leg_num]), 0,
                  distance_to_motor[leg_num] * math.cos(angles_to_motors[leg_num])],
                 [math.sin(angles_to_motors[leg_num]), math.cos(angles_to_motors[leg_num]), 0,
                  distance_to_motor[leg_num] * math.sin(angles_to_motors[leg_num])],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]

            R = [[1, 0, 0, 0],
                 [0, math.cos(math.radians(180)), -math.sin(math.radians(180)), 0],
                 [0, math.sin(math.radians(180)), math.cos(math.radians(180)), 0],
                 [0, 0, 0, 1]]

            present_pos = self.legs[leg_num].motors_angles()
            present_pos = [pos * 0.088 for pos in present_pos]

            B = [[math.cos(math.radians(present_pos[0] - 180)), -math.sin(math.radians(present_pos[0] - 180)), 0,
                  l1 * math.cos(math.radians(present_pos[0] - 180))],
                 [math.sin(math.radians(present_pos[0] - 180)), math.cos(math.radians(present_pos[0] - 180)), 0,
                  l1 * math.sin(math.radians(present_pos[0] - 180))],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]

            if leg_num in left:

                C = [[math.cos(math.radians(present_pos[1] - 140)), 0, math.sin(math.radians(present_pos[1] - 140)),
                      l2 * math.cos(math.radians(present_pos[1] - 140))],
                     [0, 1, 0, 0],
                     [-math.sin(math.radians(present_pos[1] - 140)), 0, math.cos(math.radians(present_pos[1] - 140)),
                      -l2 * math.sin(math.radians(present_pos[1] - 140))],
                     [0, 0, 0, 1]]

                D = [[math.cos(math.radians(present_pos[2] - 132)), 0, math.sin(math.radians(present_pos[2] - 132)), 0],
                     [0, 1, 0, 0],
                     [-math.sin(math.radians(present_pos[2] - 132)), 0, math.cos(math.radians(present_pos[2] - 132)),
                      0],
                     [0, 0, 0, 1]]

                T = np.matmul(A, np.matmul(R, np.matmul(B, np.matmul(C, np.matmul(R, D)))))

                # print("the matrix for leg number", leg_num+1, "are:\n0A1=\n", A, "\n1A2=\n", B, "\n2A3=\n", C, "\n3A4=\n", D, "\n0A4=\n", T, "\n")
            else:

                C = [[math.cos(math.radians(present_pos[1] - 222)), 0, math.sin(math.radians(present_pos[1] - 222)),
                      l2 * math.cos(math.radians(present_pos[1] - 222))],
                     [0, 1, 0, 0],
                     [-math.sin(math.radians(present_pos[1] - 222)), 0, math.cos(math.radians(present_pos[1] - 222)),
                      -l2 * math.sin(math.radians(present_pos[1] - 222))],
                     [0, 0, 0, 1]]

                D = [[math.cos(math.radians(present_pos[2] - 227)), 0, math.sin(math.radians(present_pos[2] - 227)), 0],
                     [0, 1, 0, 0],
                     [-math.sin(math.radians(present_pos[2] - 227)), 0, math.cos(math.radians(present_pos[2] - 227)),
                      0],
                     [0, 0, 0, 1]]

                T = np.matmul(A, np.matmul(R, np.matmul(B, np.matmul(R, np.matmul(C, np.matmul(R, D))))))

            legs_tip.append(T.dot(tip))
        
        return legs_tip

    def walk_gait(self, steps_num, direction='forward', y=250, z=200, step_size=150, resolution=6, step_height=100,
                  step_dist=10, pair=""):

        if pair == "f_legs":
            walk_legs = [0, 3]
        elif pair == "m_legs":
            walk_legs = [1, 4]
        else:
            walk_legs = [2, 5]

        legs_tip = self.find_tips()

        x_f = []                                            # array of x value sets for six legs. 'f' stands for 'forward motion'. leg0 = 0
        y_f = []                                            # array of y value sets for six legs (etc.)
        z_f = []                                            # array of z value sets for six legs (etc.)

        x_b = []                                            # array of x value sets for six legs. 'b' stands for 'backwards motion'. leg0 = 0
        y_b = []                                            # etc..
        z_b = []

        psi_a = []

        samples = int(step_size/resolution)

        for num, leg in zip(range(6), self.legs):  # creating value sets

            if pair == "m_legs":
                x_f.append(np.ndarray.tolist(np.linspace(legs_tip[num][0][0], legs_tip[num][0][0] + step_size, samples)))
            else:
                x_f.append(leg.x_forward(step_size, resolution))                

            y_f.append([y] * samples)

            x_f_f, y_f_f = leg.pos_fix(x_f[num], y_f[num])  # rotating x and y values to leg axis system
            x_f[num] = x_f_f
            y_f[num] = y_f_f

            z_b1 = np.linspace(z, z - step_height, int(samples / 2))
            z_b2 = np.linspace(z - step_height, z, int(samples / 2))
            z_b.append(np.ndarray.tolist(np.append(z_b1, z_b2)))

            y_b1 = np.linspace(y_f[num][-1], y_f[num][-1] - step_dist, int(samples / 2))
            y_b2 = np.linspace(y_f[num][-1] - step_dist, y_f[num][0], int(samples / 2))
            y_b.append(np.ndarray.tolist(np.append(y_b1, y_b2)))

            if pair == "r_legs":
                present_pos = self.legs[num].motors_angles()
                if num == 2:
                    psi_a.append(np.ndarray.tolist(np.linspace(present_pos[0], 210/0.088, samples)))
                elif num == 5:
                    psi_a.append(np.ndarray.tolist(np.linspace(present_pos[0], 140/0.088, samples)))

        pos_fix = []
        max_samples = 0
        pos = [x_f, y_b, z_b]

        for leg_num in walk_legs:
            present_pos = self.legs[leg_num].motors_angles()
            des_pos = self.legs[leg_num].loc2deg(pos[0][leg_num][0], pos[1][leg_num][0], pos[2][leg_num][0])
            if pair == "r_legs":
                if leg_num == 2:
                    des_pos[0] = round(psi_a[0][0])
                else:
                    des_pos[0] = round(psi_a[1][0])
            max_values = max(abs(present_pos[0] - des_pos[0]), abs(present_pos[1] - des_pos[1]),
                              abs(present_pos[2] - des_pos[2]))
            pos_fix.append(np.ndarray.tolist(np.linspace(present_pos, des_pos, int(max_values / resolution)).astype(int)))
            if max_values > max_samples:
                max_samples = max_values
        
        for it in range(int(max_samples / 8)):
            for leg_num in range(2): 
                if it < len(pos_fix[leg_num]):
                    self.legs[walk_legs[leg_num]].set_leg_pos(pos_fix[leg_num][it])
            time.sleep(0.01)

        if pair == "m_legs":

            for it in range(samples-1):
                for leg_num, i, j, k in zip(range(6), x_f, y_b, z_b):
                    if leg_num in walk_legs:
                        self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                    else:
                        pass
                time.sleep(0.01)
        
        elif pair == "r_legs":
            
            for it in range(samples-1):
                for leg_num, i, j, k in zip(range(6), x_f, y_b, z_b):
                    if leg_num in walk_legs:
                        pos = self.legs[leg_num].loc2deg(i[it], j[it], k[it])
                        if leg_num == 2:
                            pos[0] = round(psi_a[0][it])
                        else:
                            pos[0] = round(psi_a[1][it])
                        self.legs[leg_num].set_leg_pos(pos)
                    else:
                        pass
                time.sleep(0.01)

    def climb_gait(self, steps_num, direction='forward', y=250, z=200, step_size=150, resolution=2, step_height=100,
                   step_dist=10, pair=""):

        # step_size will be as D_climb = m + d
        h_stair = 50
        D_climb = int((step_size ** 2 - (h_stair) ** 2) ** 0.5)
        m, d = (D_climb / 2, D_climb / 2)
        step_size = D_climb
        samples = int(step_size / resolution)
        
        x_f = []  # array of x value sets for six legs. 'f' stands for 'forward motion'. leg0 = 0
        x_f1 = []
        x_f2 = []
        y_f = []  # array of y value sets for six legs (etc.)
        y_f1 = []
        y_f2 = []
        z_f = []  # array of z value sets for six legs (etc.)
        z_f1 = []

        x_b = []
        y_b1 = []
        y_b2 = []                                            # array of x value sets for six legs. 'b' stands for 'backwards motion'. leg0 = 0
        y_b = []                                            # etc..
        z_b1 = []
        z_b2 = []
        z_b = []

        if pair == "f_legs":
            climb_legs = [0, 3]
        elif pair == "m_legs":
            climb_legs = [1, 4]
        else:
            climb_legs = [2, 5]

        legs_tip = self.find_tips()
            
        for num, leg in zip(range(6), self.legs):  # creating value sets

            if pair == "f_legs":
                
                if num in climb_legs:

                    x_f.append(np.ndarray.tolist(np.linspace(legs_tip[num][0][0], legs_tip[num][0][0] + int(step_size/2), samples)))
                    y_f.append([legs_tip[num][1][0]] * samples)

                    z_f1 = np.ndarray.tolist(np.linspace(abs(legs_tip[num][2][0])-40, h_stair - 20, int(samples / 2)))
                    z_f2 = np.ndarray.tolist(np.linspace(h_stair - 20, h_stair, int(samples / 2)))
                    z_f.append(np.append(z_f1, z_f2))

                else:

                    x_f.append(leg.x_forward(step_size, resolution))  # calculating x values for forward movement
                    y_f.append([y] * samples)                    
                    z_f.append([z] * samples)

                    x_f_f, y_f_f = leg.pos_fix(x_f[num], y_f[num])  # rotating x and z values to leg axis system
                    x_f[num] = x_f_f
                    y_f[num] = y_f_f
            
            elif pair == "m_legs":
                
                x_f1.append([legs_tip[num][0][0]] * samples)
                x_f.append(np.ndarray.tolist(np.linspace(legs_tip[num][0][0], legs_tip[num][0][0] + step_size, samples)))
                x_f2.append([legs_tip[num][0][0] + step_size] * samples)

                y_f.append([y] * samples)
                z_f.append([z] * samples)

                x_f_f, y_f_f = leg.pos_fix(x_f1[num], y_f[num])  # rotating x and z values to leg axis system
                x_f1[num] = x_f_f

                x_f_f, y_f_f = leg.pos_fix(x_f[num], y_f[num])  # rotating x and z values to leg axis system
                x_f[num] = x_f_f
                y_f[num] = y_f_f

                x_f_f, y_f_f = leg.pos_fix(x_f2[num], y_f[num])  # rotating x and z values to leg axis system
                x_f2[num] = x_f_f

                z_b1.append(np.ndarray.tolist(np.linspace(abs(legs_tip[num][2][0]), h_stair - 50, samples)))
                z_b.append([h_stair - 50] * samples)
                z_b2.append(np.ndarray.tolist(np.linspace(h_stair - 50, h_stair, samples)))

                y_b1.append(np.ndarray.tolist(np.linspace(y_f[num][-1], y_f[num][-1] - step_dist, samples)))
                y_b.append([y_f[num][-1] - step_dist] * samples)
                y_b2.append(np.ndarray.tolist(np.linspace(y_f[num][-1] - step_dist, y_f[num][0], samples)))

            else:
                pass                          
                    
        if pair == "f_legs":

            pos_fix = []
            max_samples = 0
            pos = [x_f, y_f, z_f]
            resolution = 8

            for leg_num in range(6):
                present_pos = self.legs[leg_num].motors_angles()
                des_pos = self.legs[leg_num].loc2deg(pos[0][leg_num][0], pos[1][leg_num][0], pos[2][leg_num][0])
                if leg_num == 0:
                    des_pos[0] = 190 / 0.088
                elif leg_num == 3:
                    des_pos[0] = 170 / 0.088
                else:
                    pass
                    
                max_values = max(abs(present_pos[0] - des_pos[0]), abs(present_pos[1] - des_pos[1]),
                                abs(present_pos[2] - des_pos[2]))
                pos_fix.append(np.ndarray.tolist(np.linspace(present_pos, des_pos, int(max_values / resolution)).astype(int)))
                if max_values > max_samples:
                    max_samples = max_values
            
            for it in range(int(max_samples / resolution)):
                for leg_num in climb_legs: 
                    if it < len(pos_fix[leg_num]):
                        present_pos = self.legs[leg_num].motors_angles()
                        pos_fix[leg_num][it][0] = present_pos[0]
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                time.sleep(0.01)

            for it in range(samples - 1):
                for leg_num, i, j, k in zip(range(6), x_f, y_f, z_f):
                    if leg_num in climb_legs:
                        pos = self.legs[leg_num].loc2deg(i[it], j[it], k[it])
                        present_pos = self.legs[leg_num].motors_angles()
                        pos[0] = present_pos[0]
                        self.legs[leg_num].set_leg_pos(pos)
                    else: 
                        pass
                time.sleep(0.01)
            
            for it in range(int(max_samples / resolution)):
                for leg_num in (5,2,1,4):
                    if it < len(pos_fix[leg_num]):
                        self.legs[leg_num].set_leg_pos(pos_fix[leg_num][it])
                time.sleep(0.01)

        elif pair == "m_legs":

            pos_fix = []
            max_samples = 0
            pos = [x_f1, y_b, z_b1]

            for leg_num in climb_legs:
                present_pos = self.legs[leg_num].motors_angles()
                des_pos = self.legs[leg_num].loc2deg(pos[0][leg_num][0], pos[1][leg_num][0], pos[2][leg_num][0])
                max_values = max(abs(present_pos[0] - des_pos[0]), abs(present_pos[1] - des_pos[1]), abs(present_pos[2] - des_pos[2]))
                pos_fix.append(np.ndarray.tolist(np.linspace(present_pos, des_pos, int(max_values / resolution)).astype(int)))
                if max_values > max_samples:
                    max_samples = max_values

            for it in range(int(max_samples / 8)):
                for leg_num in (range(2)): 
                    if it < len(pos_fix[leg_num]):
                        self.legs[climb_legs[leg_num]].set_leg_pos(pos_fix[leg_num][it])
                time.sleep(0.01)

            if pair == "m_legs":

                for it in range(samples-1):
                    for leg_num, i, j, k in zip(range(6), x_f1, y_b1, z_b1):
                        if leg_num in climb_legs:
                            self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                        else:
                            pass
                    time.sleep(0.01)

                for it in range(samples-1):
                    for leg_num, i, j, k in zip(range(6), x_f, y_b, z_b):
                        if leg_num in climb_legs:
                            self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                        else:
                            pass
                    time.sleep(0.01)

                for it in range(samples-1):
                    for leg_num, i, j, k in zip(range(6), x_f2, y_b2, z_b2):
                        if leg_num in climb_legs:
                            self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                        else:
                            pass
                    time.sleep(0.01)

        else:
                
            psi_a = []
            theta_a = []
            phi_a = []

            for leg_num in climb_legs:
                present_pos = self.legs[leg_num].motors_angles()
                angle1 = [present_pos[0]] * samples
                if leg_num == 2:
                    angle2 = np.ndarray.tolist(np.linspace(present_pos[0], round(186/0.088), samples))
                else:
                    angle2 = np.ndarray.tolist(np.linspace(present_pos[0], round(170/0.088), samples))
                psi_a.append(angle1 + angle2)

                if leg_num == 2:
                    angle1 = np.ndarray.tolist(np.linspace(present_pos[1], round(207/0.088), samples))
                    angle2 = np.ndarray.tolist(np.linspace(round(207/0.088), round(180/0.088), samples))
                else:
                    angle1 = np.ndarray.tolist(np.linspace(present_pos[1], round(153/0.088), samples))
                    angle2 = np.ndarray.tolist(np.linspace(round(153/0.088), round(180/0.088), samples))
                theta_a.append(angle1 + angle2)

                angle1 = [present_pos[2]] * samples
                if leg_num == 2:
                    angle2 = np.ndarray.tolist(np.linspace(present_pos[2], round(261/0.088), samples))
                else:
                    angle2 = np.ndarray.tolist(np.linspace(present_pos[2], round(95/0.088), samples))
                phi_a.append(angle1 + angle2)

            for it in range(2*samples-1):
                for leg_num, alpha, beta, gamma in zip(range(2), psi_a, theta_a, phi_a):
                    pos_next = [round(alpha[it]), round(beta[it]), round(gamma[it])]
                    self.legs[climb_legs[leg_num]].set_leg_pos(pos_next)
                time.sleep(0.01)
                
            
    def COM(self, pair="", count=1):
        
        h_stair = 50
        angles_to_motors = [math.radians(45), math.radians(90), math.radians(135), math.radians(-45), math.radians(-90), math.radians(-135)]
        distance_to_motor = [118.5, 112.75, 118.5, 118.5, 112.75, 118.5]  # mm
        angels_limits = {0: [(round(125/0.088), round(225/0.088)), (round(225/0.088), round(75/0.088)), (round(270/0.088), round(70/0.088))],
                        1: [(round(135/0.088), round(225/0.088)), (round(75/0.088), round(225/0.088)), (round(270/0.088), round(70/0.088))],
                        2: [(round(125/0.088), round(230/0.088)), (round(75/0.088), round(225/0.088)), (round(270/0.088), round(70/0.088))],
                        3: [(round(135/0.088), round(225/0.088)), (round(270/0.088), round(135/0.088)), (round(280/0.088), round(90/0.088))],
                        4: [(round(135/0.088), round(225/0.088)), (round(135/0.088), round(270/0.088)), (round(280/0.088), round(90/0.088))],
                        5: [(round(125/0.088), round(230/0.088)), (round(135/0.088), round(270/0.088)), (round(280/0.088), round(90/0.088))]}

        legs_tip = self.find_tips()

        # for leg_num, legs_tip in enumerate(legs_tip):
        #     x_coord, y_coord, z_coord, _ = legs_tip.flatten()
        #     print(f"The coordinates of leg number {leg_num+1} are: x = {x_coord:.2f}, y = {y_coord:.2f}, z = {z_coord:.2f}")

        f_legs = [1, 2, 4, 5]
        m_legs = [0, 2, 3, 5]
        r_legs = [0, 1, 3, 4]

        if pair == "f_legs":
            chosen_legs = f_legs
        elif pair == "m_legs":
            chosen_legs = m_legs
        else:
            chosen_legs = r_legs
            
        x_sum = 0
        y_sum = 0
        z_sum = 0

        # quad_coordinates = []

        for leg_num in chosen_legs:
            x_sum += legs_tip[leg_num][0][0]
            y_sum += legs_tip[leg_num][1][0]
            z_sum += legs_tip[leg_num][2][0]

            # x = legs_tip[leg_num][0][0]
            # y = legs_tip[leg_num][1][0]
            # z = legs_tip[leg_num][2][0]

            # coordinate = [x,y,z]

            # quad_coordinates.append(coordinate)

        center_x = x_sum / 4
        center_y = y_sum / 4
        center_z = z_sum / 4

        centroid = [center_x, center_y, center_z]
        # print(f"The centroid of the support polygon is: x = {centroid[0]:.2f}, y = {centroid[1]:.2f}")

        # quad_coordinates.sort(key=lambda coord: math.atan2(coord[1]-center_y, coord[1]-center_x))

        # x_coords = [coord[0] for coord in quad_coordinates]
        # y_coords = [coord[1] for coord in quad_coordinates]
        # z_coords = [coord[2] for coord in quad_coordinates]

        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.set_xlabel('X')
        # ax.set_ylabel('Y')
        # ax.set_zlabel('Z')

        # ax.plot(x_coords + [x_coords[0]], y_coords + [y_coords[0]], z_coords + [z_coords[0]], '-g', label='Support polygon')
        # ax.scatter(x_coords, y_coords, z_coords, s=50 ,color='b', label='Legs tips')
        # ax.scatter(centroid[0], centroid[1], 0, s=50 ,color='g', label='Centroid', marker='H')
        # ax.scatter(0, 0, 0, s=50 ,color='r', label='COM', marker='*')
        # ax.legend()
        # plt.show()

        T = [[1, 0, 0, centroid[0]],
             [0, 1, 0, centroid[1]],
             [0, 0, 1, 0],
             [0, 0, 0, 1]]
        # print("marix T is:", T)

        T_tag = inv(T)
        # print("matrix T_tag is:", T_tag)

        psi_a = []
        theta_a = []
        phi_a = []
        
        samples = 200

        for leg_num in range(6):

            present_pos = self.legs[leg_num].motors_angles()

            if pair == "f_legs":
                tip = [[legs_tip[leg_num][0][0]], [legs_tip[leg_num][1][0]], [legs_tip[leg_num][2][0]], [1]]
                
            else:
                if count == 3:
                    tip = [[legs_tip[leg_num][0][0]], [legs_tip[leg_num][1][0]], [legs_tip[leg_num][2][0]], [1]]
                else: 
                    if leg_num in (0, 3):
                        tip = [[legs_tip[leg_num][0][0]], [legs_tip[leg_num][1][0]], [260], [1]]
                    else:
                        tip = [[legs_tip[leg_num][0][0]], [legs_tip[leg_num][1][0]], [legs_tip[leg_num][2][0]], [1]]

            tip_B_tag = T_tag.dot(tip)

            A = [[math.cos(angles_to_motors[leg_num]), -math.sin(angles_to_motors[leg_num]), 0,
                    distance_to_motor[leg_num] * math.cos(angles_to_motors[leg_num])],
                    [math.sin(angles_to_motors[leg_num]), math.cos(angles_to_motors[leg_num]), 0,
                    distance_to_motor[leg_num] * math.sin(angles_to_motors[leg_num])],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]]

            A_tag = inv(A)

            tip_O_tag = A_tag.dot(tip_B_tag)
            
            # print(f"for leg number {leg_num+1}: tip_B_tag =", tip_B_tag, "T_B_O =", A_tag ,"tip_O_tag =", tip_O_tag)

            if pair == "f_legs":
                pos = self.legs[leg_num].loc2deg(tip_O_tag[0][0], tip_O_tag[1][0], -tip_O_tag[2][0])
                # print(f"The desired angles of each motor for leg number {leg_num+1} are: theta_1 = {round(pos[0]*0.088):.2f}, theta_2 = {round(pos[1]*0.088):.2f}, theta_3 = {round(pos[2]*0.088):.2f}")

            elif pair == "r_legs":

                if count == 1:
                    if leg_num in (0,3):
                        pos = self.legs[leg_num].loc2deg(tip_O_tag[0][0], tip_O_tag[1][0], h_stair)
                    else:
                        pos = self.legs[leg_num].loc2deg(tip_O_tag[0][0], tip_O_tag[1][0], -tip_O_tag[2][0])

                elif count == 2:
                    if leg_num in chosen_legs:
                        pos = self.legs[leg_num].loc2deg(tip_O_tag[0][0], tip_O_tag[1][0], h_stair)
                    else:
                        pos = self.legs[leg_num].loc2deg(tip_O_tag[0][0], tip_O_tag[1][0], -tip_O_tag[2][0])
                        if leg_num == 2:
                            pos[2] = round(195/0.088)
                        else:
                            pos[2] = round(165/0.088)
                                
                else:                
                    pos = self.legs[leg_num].loc2deg(tip_O_tag[0][0], tip_O_tag[1][0], -tip_O_tag[2][0])
                                       
            else:
                pass

            if pair == "f_legs":

                if pos[0] < angels_limits[leg_num][0][0] or pos[0] > angels_limits[leg_num][0][1]:
                    if angels_limits[leg_num][0][0] - pos[0] > pos[0] - angels_limits[leg_num][0][1]:
                        pos[0] = angels_limits[leg_num][0][0]
                    else:
                        pos[0] = angels_limits[leg_num][0][1]
                else:
                    pass

                psi_a.append(np.ndarray.tolist(np.linspace(present_pos[0], pos[0], samples)))
                theta_a.append(np.ndarray.tolist(np.linspace(present_pos[1], pos[1], samples)))
                phi_a.append(np.ndarray.tolist(np.linspace(present_pos[2], pos[2], samples)))

            elif pair == "r_legs":

                pos[0] = 4095 - pos[0]

                if pos[0] < angels_limits[leg_num][0][0] or pos[0] > angels_limits[leg_num][0][1]:
                    if angels_limits[leg_num][0][0] - pos[0] > pos[0] - angels_limits[leg_num][0][1]:
                        pos[0] = angels_limits[leg_num][0][0]
                    else:
                        pos[0] = angels_limits[leg_num][0][1]
                else:
                    pass

                if count == 3:
                    if leg_num in chosen_legs:
                        psi_a.append(np.ndarray.tolist(np.linspace(present_pos[0], pos[0], samples)))
                    else:
                        if leg_num == 2:
                            psi_a.append(np.ndarray.tolist(np.linspace(present_pos[0], round(160/0.088), samples)))
                        else:
                            psi_a.append(np.ndarray.tolist(np.linspace(present_pos[0], round(200/0.088), samples)))


                
                else:
                    if leg_num in (0,3):
                        if pos[1] < angels_limits[leg_num][1][0] or pos[1] > angels_limits[leg_num][1][1]:
                            if angels_limits[leg_num][1][0] - pos[1] > pos[1] - angels_limits[leg_num][1][1]:
                                pos[1] = angels_limits[leg_num][1][1]
                            else:
                                pos[1] = angels_limits[leg_num][1][0]
                        else:    
                            pass
                    else:
                        pass

                    if pos[2] > angels_limits[leg_num][2][0] or pos[2] < angels_limits[leg_num][2][1]:
                        if angels_limits[leg_num][2][0] - pos[2] > pos[2] - angels_limits[leg_num][2][1]:
                            pos[2] = angels_limits[leg_num][2][1]
                        else:
                            pos[2] = angels_limits[leg_num][2][0]
                    else:
                        pass
                
                    if leg_num in (0,3):
                        psi_a.append([present_pos[0]] * samples)
                    else:
                        psi_a.append(np.ndarray.tolist(np.linspace(present_pos[0], pos[0], samples)))

                theta_a.append(np.ndarray.tolist(np.linspace(present_pos[1], pos[1], samples)))
                phi_a.append(np.ndarray.tolist(np.linspace(present_pos[2], pos[2], samples)))

            else:
                pass

        if pair == "f_legs": # movement of COM for lift front legs

            for it in range(samples-1):
                for leg_num, alpha, beta, gamma in zip(range(6), psi_a, theta_a, phi_a):
                    pos_next = [round(alpha[it]), round(beta[it]), round(gamma[it])]
                    self.legs[leg_num].set_leg_pos(pos_next)
                time.sleep(0.01)

            psi_a = []
            theta_a = []
            phi_a = []

            for leg_num in range(6):

                if leg_num not in chosen_legs:

                    present_pos = self.legs[leg_num].motors_angles()

                    psi_a.append([present_pos[0]] * samples)

                    if leg_num == 0: 
                        theta_a.append(np.ndarray.tolist(np.linspace(present_pos[1], 225 / 0.088, samples)))
                        phi_a.append(np.ndarray.tolist(np.linspace(present_pos[2], 275 / 0.088, samples)))
                    else:
                        theta_a.append(np.ndarray.tolist(np.linspace(present_pos[1], 135 / 0.088, samples)))
                        phi_a.append(np.ndarray.tolist(np.linspace(present_pos[2], 85 / 0.088, samples)))

                else:
                    pass
            
            # fix of front legs for movement COM backwards 
            for it in range(samples-1):
                for leg_num, alpha, beta, gamma in zip((0,3), psi_a, theta_a, phi_a):
                    pos_next = [round(alpha[it]), round(beta[it]), round(gamma[it])]
                    self.legs[leg_num].set_leg_pos(pos_next)
                time.sleep(0.01)

        elif pair == "m_legs": # movement of COM for lift middle legs

            for it in range(samples-1):
                for leg_num, alpha, beta, gamma in zip(range(6), psi_a, theta_a, phi_a):
                    if leg_num not in chosen_legs:
                        pos_next = [round(alpha[it]), round(beta[it]), round(gamma[it])]
                        self.legs[leg_num].set_leg_pos(pos_next)
                    else:
                        pass
                time.sleep(0.01)

            for it in range(samples-1):
                for leg_num, alpha, beta, gamma in zip(range(6), psi_a, theta_a, phi_a):
                    if leg_num in chosen_legs:
                        pos_next = [round(alpha[it]), round(beta[it]), round(gamma[it])]
                        self.legs[leg_num].set_leg_pos(pos_next)
                    else:
                        pass
                time.sleep(0.01)
                
        else:
            if count == 3:
                for it in range(samples-1):
                    for leg_num, alpha, beta, gamma in zip(range(6), psi_a, theta_a, phi_a):
                        pos_next = [round(alpha[it]), round(beta[it]), round(gamma[it])]
                        self.legs[leg_num].set_leg_pos(pos_next)
                    time.sleep(0.01)
            elif count == 2:
                for it in range(int(samples/2)-1):
                    for leg_num, alpha, beta, gamma in zip(range(6), psi_a, theta_a, phi_a):
                        pos_next = [round(alpha[it]), round(beta[it]), round(gamma[it])]
                        self.legs[leg_num].set_leg_pos(pos_next)
                    time.sleep(0.01)
            else:
                for it in range(int(samples/2)-1):
                    for leg_num, alpha, beta, gamma in zip(range(6), psi_a, theta_a, phi_a):
                        pos_next = [round(alpha[it]), round(beta[it]), round(gamma[it])]
                        self.legs[leg_num].set_leg_pos(pos_next)
                    time.sleep(0.01)

    def reach_stair(self,steps_num=1, direction='forward', y=250, z=200, step_size=175, resolution=8, step_height=100, step_dist=10):
        
        h_stair = 50
        D_climb = int((step_size ** 2 - (h_stair) ** 2) ** 0.5)
        max_step_size = 100
        w = 500
        m, d = (D_climb / 2, D_climb / 2)
        D_walk = w - D_climb
        print(D_walk)
        
        # step_size is changed depends on the distance from line of climbing point, D_walk
        while D_walk > 0:

            if D_walk > 2 * max_step_size:
                step_size = max_step_size
                resolution = 4
            elif max_step_size < D_walk <= 2 * max_step_size:
                step_size = D_walk / 2
                resolution = 2
            elif D_walk <= max_step_size:
                step_size = D_walk
                resolution = 2
            
            D_walk = D_walk - step_size

            print(D_walk)

            samples = int(step_size/resolution)

            x_f = []                                            # array of x value sets for six legs. 'f' stands for 'forward motion'. leg0 = 0
            y_f = []                                            # array of y value sets for six legs (etc.)
            z_f = []                                            # array of z value sets for six legs (etc.)

            x_b = []                                            # array of x value sets for six legs. 'b' stands for 'backwards motion'. leg0 = 0
            y_b = []                                            # etc..
            z_b = []

            for num, leg in zip(range(6), self.legs):  # creating value sets
                if direction == 'forward':
                    x_f.append(leg.x_forward(int(step_size), resolution))  # calculating x values for forward movement
                else:
                    print('walking direction is defined wrong (should be "forward"/"backwards" only). Please re-define')
                    quit()

                y_f.append([y] * samples)
                z_f.append([z] * samples)

                x_f_f, y_f_f = leg.pos_fix(x_f[num], y_f[num])  # rotating x and z values to leg axis system
                x_f[num] = x_f_f
                y_f[num] = y_f_f

                x_b.append(np.flip(x_f[num]))  # creating values set for x backwards movement (opposite of x_f)

                z_b1 = np.linspace(z, z - step_height, int(samples / 2))
                z_b2 = np.linspace(z - step_height, z, int(samples / 2))
                z_b.append(np.ndarray.tolist(np.append(z_b1, z_b2)))

                y_b1 = np.linspace(y_f[num][-1], y_f[num][-1] - step_dist, int(samples / 2))
                y_b2 = np.linspace(y_f[num][-1] - step_dist, y_f[num][0], int(samples / 2))
                y_b.append(np.ndarray.tolist(np.append(y_b1, y_b2)))

                if num % 2 != 0:                                # switching double legs (0,2,4) values
                    x_f[num], x_b[num] = x_b[num], x_f[num]
                    y_f[num], y_b[num] = y_b[num], y_f[num]
                    z_f[num], z_b[num] = z_b[num], z_f[num]

                if direction == 'backwards':
                    x_f[num], x_b[num] = x_b[num], x_f[num]
                    y_f[num], y_b[num] = y_b[num], y_f[num]
                    z_f[num], z_b[num] = z_b[num], z_f[num]

            self.pos_fix([x_f, y_f, z_f], 8, 6)

            for step in range(steps_num):
                #print('\nFirst half cycle')
                for it in range(samples-1):
                    for leg_num, i, j, k in zip(range(6), x_f, y_f, z_f):
                        #print('{} {} {}'.format(round(i[it]), round(j[it]), round(k[it])))
                        self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                    time.sleep(0.01)

                #print('\nSecond half cycle')
                for it in range(samples-1):
                    for leg_num, i, j, k in zip(range(6), x_b, y_b, z_b):
                        #print('{} {} {}'.format(round(i[it]),round(j[it]),round(k[it])))
                        self.legs[leg_num].set_leg_pos(self.legs[leg_num].loc2deg(i[it], j[it], k[it]))
                    time.sleep(0.001)

    def rise_position(self, z1 = 200, z2 = 310, y=200, resolution=2, parts=1):
        # Changes the robot's height

        x_f = []                                            # array of x value sets for six legs. 'f' stands for 'forward motion'. leg0 = 0
        y_f = []                                            # array of y value sets for six legs (etc.)
        z_f = []                                            # array of z value sets for six legs (etc.)

        samples = int(100/resolution)

        floor = [2, 5]

        for num, leg in zip(range(6), self.legs):  # creating value sets
            x_f.append([0] * samples)  # calculating x values for forward movement
            y_f.append([y] * samples)
            if num in floor:
                z_f.append([z2] * samples)
            else:
                z_f.append([z1] * samples)

        self.pos_fix([x_f, y_f, z_f], resolution*2, parts)





