import numpy as np
import math
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Control table address
ADDR_PRO_TORQUE_ENABLE = 64  # Control table address is different in Dynamixel model
ADDR_PRO_LED_RED = 65
ADDR_PRO_GOAL_POSITION = 116
ADDR_PRO_PRESENT_POSITION = 132

# Data Byte Length
LEN_PRO_LED_RED = 1
LEN_PRO_CURRENT_LIMIT = 2
LEN_PRO_GOAL_POSITION = 4
LEN_PRO_PRESENT_POSITION = 4
LEN_PRO_VELOCITY_PROFILE = 4

# Protocol version
PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque

VELOCITY_PROFILE = 100   # motor speed



def body_loc1():
    dx = dy = 90.87
    dy_mid = 122.75
    dots = [[dx, 0, -dx, -dx, 0, dx, dx],
            [dy, dy_mid, dy, -dy, -dy_mid, -dy, dy],
            [0, 0, 0, 0, 0, 0, 0]]
    return dots


def forward_trapzoid (y,z,step_height, step_size, legs):
    x_f = []  # array of x value sets for six legs. 'f' stands for 'forward motion'. leg0 = 0
    y_f = []  # array of y value sets for six legs (etc.)
    z_f = []  # array of z value sets for six legs (etc.)

    x_b = []  # array of x value sets for six legs. 'b' stands for 'backwards motion'. leg0 = 0
    y_b = []  # etc..
    z_b = []

    x_t = []
    y_t = []
    z_t = []

    for num in (range(6)):  # creating value sets
        # x_f.append(legs[num].x_forward2(step_size, resolution)) # calculating x values for forward movement
        x_f.append(legs[num].x_1(step_size))

        samples = len(x_f[num])

        z_f.append([z] * samples)
        y_f.append([y] * samples)

        x_b.append(np.flip(x_f[num]))  # creating values set for x backwards movement (opposite of x_f)


        #############trapzoid profile########3

        z_b1 = np.linspace(z, z + step_height, int(samples / 2))
        z_b2 = np.flip(z_b1)
        z_b.append(np.ndarray.tolist(np.append(z_b1, z_b2)))
        if len(z_b[num]) < len(x_b[num]):
            z_b[num].append(z)
        y_b.append([y] * samples)
        if len(y_b[num]) < len(x_b[num]):
            y_b[num].append(y)

        ##############################################################################

        if num % 2 != 0:  # switching double legs (0,2,4) values
            x_f[num], x_b[num] = x_b[num], x_f[num]
            y_f[num], y_b[num] = y_b[num], y_f[num]
            z_f[num], z_b[num] = z_b[num], z_f[num]

        x_t.append(list(x_f[num]) + list(x_b[num]))
        y_t.append(list(y_f[num]) + list(y_b[num]))
        z_t.append(list(z_f[num]) + list(z_b[num]))
    return x_t, y_t, z_t





class LEG2:
    def __init__(self, portHandler, motor_ID, side, position='middle'):
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupBulkRead = GroupBulkRead(portHandler, self.packetHandler)
        self.groupBulkWrite = GroupBulkWrite(portHandler, self.packetHandler)
        self.portHandler = portHandler
        self.motor_ID = motor_ID
        self.side = side
        self.position = position

        # Enabling motors torque
        for i in range(3):
            self.torque(self.motor_ID[i], TORQUE_ENABLE)

            dxl_addparam_result = self.groupBulkRead.addParam(self.motor_ID[i], ADDR_PRO_PRESENT_POSITION,
                                                              LEN_PRO_PRESENT_POSITION)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkRead addparam failed" % self.motor_ID[i])
                quit()

        # Defining side correction (for left/right legs)
        if self.side == 'left':
            self.side_corr = 1
        elif self.side == 'right':
            self.side_corr = -1
        else:
            print('Leg side is defined wrong. Please re-define')
            quit()

        # Defining position correction (for front/back legs)
        if self.position == 'front':
            self.pos_corr = np.deg2rad(22.5)
            self.x_offset = 90.87
        elif self.position == 'middle':
            self.pos_corr = 0
            self.x_offset = 0
        elif self.position == 'back':
            self.pos_corr = np.deg2rad(-22.5)
            self.x_offset = -90.87
        else:
            print('Leg position is defined wrong. Please re-define')
            quit()

    def torque(self, motor_id, enable):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_PRO_TORQUE_ENABLE, enable)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % motor_id)


    def torque_off(self):
        for i in range(3):
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.motor_ID[i], ADDR_PRO_TORQUE_ENABLE, 0)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def set_leg_pos(self, pos):
        # pos in tick dimensions

        for i in range(3):
            param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(pos[i])), DXL_HIBYTE(DXL_LOWORD(pos[i])), DXL_LOBYTE(DXL_HIWORD(pos[i])), DXL_HIBYTE(DXL_HIWORD(pos[i]))]
            dxl_addparam_result1 = self.groupBulkWrite.addParam(self.motor_ID[i], ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position1)

            if dxl_addparam_result1 != True:
                print("[ID:%03d] groupBulkWrite addparam failed" % self.motor_ID[i])
                quit()

        dxl_comm_result = self.groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear bulkwrite parameter storage
        self.groupBulkWrite.clearParam()


    def motors_angles(self):
        # Check if groupbulkread data of motor is available

        motor_angles = []
        for i in range(3):
            dxl_comm_result = self.groupBulkRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # Get present position value
            dxl_getdata_result = self.groupBulkRead.isAvailable(self.motor_ID[i], ADDR_PRO_PRESENT_POSITION,
                                                                 LEN_PRO_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % self.motor_ID[i])
                quit()

            dxl_present_position = self.groupBulkRead.getData(self.motor_ID[i], ADDR_PRO_PRESENT_POSITION,
                                                              LEN_PRO_PRESENT_POSITION)
            motor_angles.append(dxl_present_position)
        return motor_angles

    def pos_fix(self, x_f, y_f):
        x_f_f = []  # stands for x_forward_fixed
        y_f_f = []  # stands for y_forward_fixed

        for i, k in zip(x_f, y_f):
            x_f_f.append(i * np.cos(self.pos_corr) - k * np.sin(self.pos_corr))
            y_f_f.append(i * np.sin(self.pos_corr) + k * np.cos(self.pos_corr))

        return x_f_f, y_f_f

    def x_forward(self, step_size, resolution):
        # Creates x values for forward movement (one leg at a time).
        front_offset = 80
        back_offset = -1 * front_offset

        x = []

        if self.position == 'front':
            start = step_size + front_offset
            finish = front_offset
        elif self.position == 'back':
            start = back_offset
            finish = -step_size + back_offset
        else:
            start = round(step_size / 2)
            finish = -round(step_size / 2)

        x_edge = []
        x_other_edge = []
        counter = 0
        counter_sum = 0
        acceleration = 1

        while counter < resolution:
            x_edge.append(start - (counter + counter_sum))
            x_other_edge.append(finish + (counter + counter_sum))
            counter_sum = counter_sum + counter
            counter = counter + acceleration

        x_other_edge = np.flip(x_other_edge)
        x = range(x_edge[-1] - resolution, x_other_edge[0], -resolution)

        x = np.append(x_edge, x)
        x = np.append(x, x_other_edge)
        return x

    def x_backwards(self, step_size, resolution):
        # Creates x values for forward movement (one leg at a time).
        front_offset = 75
        back_offset = -1 * front_offset
        if self.position == 'front':
            return range(front_offset, step_size + front_offset, resolution)
        elif self.position == 'back':
            return range(-step_size + back_offset, back_offset, resolution)
        else:
            return range(-round(step_size / 2), round(step_size / 2), resolution)

    def loc2deg(self, x, y, z):
        # converts x,y,z in world axis system to engines' degrees.
        # NOTE: while calculating this program assumes:
        # #1 outer leg edge is lower then the body
        # #2 inner leg edge is higher then outer leg edge
        # #3 x is forward, y is upwards and z is outwards
        # #4 y is given in opposite direction (greater y = higher body (=lower leg edge) ).
        # function fixes coordinates according to leg side (left/right)

        a = np.sqrt(x ** 2 + y ** 2)

        psi = np.arctan2(x / a, y / a)

        b = np.sqrt(x ** 2 + y ** 2 + z ** 2)

        if b <= 170:
            b = 170
        elif b > 500:
            b = 500
        else:
            pass
                
        l1 = 210.54
        l2 = 372.4

        C = (l1 ** 2 + b ** 2 - l2 ** 2) / (2 * l1 * b)
        D = (l1 ** 2 + l2 ** 2 - b ** 2) / (2 * l1 * l2)

        alpha = np.arcsin(z / b)

        theta = np.arctan2(np.sqrt(1 - C ** 2), C) - alpha
        phi = np.arctan2(np.sqrt(1 - D ** 2), D)

        # rad to deg
        psi_d = np.degrees(psi)
        theta_d = np.degrees(theta)
        phi_d = np.degrees(phi)

        psi_t = psi_d * 2048 / 180  # deg to relative ticks
        theta_t = theta_d * 2048 / 180
        phi_t = phi_d * 2048 / 180

        psi_a = round(2048 + self.side_corr*psi_t)  # relative ticks to absolute ticks (world axis system)
        theta_a = round(2048 + self.side_corr*(theta_t - 512))
        phi_a = round(2048 - self.side_corr*(phi_t - 1536))

        pos = [psi_a, theta_a, phi_a]
        return pos





    def loc2degm(self, x, y, z):
        # converts x,y,z in world axis system to engines' degrees.
        # NOTE: while calculating this program assumes:
        # #1 outer leg edge is lower then the body
        # #2 inner leg edge is higher then outer leg edge
        # #3 x is forward, z is upwards and y is outwards
        # #4 z is given in opposite direction (greater z = higher body (=lower leg edge) ).
        # function fixes coordinates according to leg side (left/right)

        l1 = 51.5
        l2 = 210.54
        l3 = 372.4

        psi = -np.arctan2(x, y)
        b = np.sqrt((y / math.cos(psi) - l1) ** 2 + z ** 2)

        C = (l2 ** 2 + b ** 2 - l3 ** 2) / (2 * l2 * b)
        D = (l2 ** 2 + l3 ** 2 - b ** 2) / (2 * l2 * l3)

        alpha = abs(np.arctan2(z, y / math.cos(psi) - l1))

        theta = np.arctan2(np.sqrt(1 - C ** 2), C) - alpha
        phi = np.arctan2(np.sqrt(1 - D ** 2), D)

        # rad to deg
        if z > 0:
            psi_d = psi * 180 / math.pi
            theta_d = -theta * 180 / math.pi
            phi_d = -phi * 180 / math.pi
        else:
            psi_d = psi * 180 / math.pi
            theta_d = theta * 180 / math.pi
            phi_d = phi * 180 / math.pi

        psi_t = psi_d * 2048 / 180  # deg to relative ticks
        theta_t = theta_d * 2048 / 180
        phi_t = phi_d * 2048 / 180

        psi_a = round(2048 + self.side_corr * psi_t)  # relative ticks to absolute ticks (world axis system)
        theta_a = round(2048 + self.side_corr * (theta_t - 512))
        phi_a = round(2048 - self.side_corr * (phi_t - 1536))

        pos = [psi_a, theta_a, phi_a]
        return pos

    def x_2(self, step_size):
        x = []
        if self.position == 'middle':
            x = np.linspace(-step_size/2, step_size/2, 20)
        else:
            x = np.linspace(self.x_offset, self.x_offset + step_size, 20)
        return x

    def x_1(self, step_size,samples):

        front_offset = (step_size / 2) + 10
        back_offset = -1 * front_offset
        x = []
        if self.position == 'front':
            x = np.linspace(front_offset+step_size, front_offset, samples)
        elif self.position == 'back':
            x = np.linspace(back_offset, back_offset + step_size, samples)
        else:
            x = np.linspace(step_size / 2, -step_size / 2, samples)
        return x

    def z_forward(self,z,step_height,resolution):

        z_edge = []
        z_other_edge = []
        z1 = []
        finish = z - step_height
        counter = 0
        counter_sum = 0
        acceleration = 1

        while counter < resolution:
            z_edge.append(z - (counter + counter_sum))
            z_other_edge.append(finish + (counter + counter_sum))
            counter_sum = counter_sum + counter
            counter = counter + acceleration

        z_other_edge = np.flip(z_other_edge)
        z1 = range(z_edge[-1] - resolution, z_other_edge[0], -resolution)

        z1 = np.append(z1, z_other_edge)
        z1 = np.append(z_edge, z1)
        z1 = np.append(z1, np.flip(z))
        z_f = z1 * (-1)
        return z_f

