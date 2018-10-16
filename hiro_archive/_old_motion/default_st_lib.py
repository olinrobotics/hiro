import serial as s
import shlex
import time

# Use this one for Mac/Linux
DEFAULT_DEV = '/dev/tty.KeySerial1'

# Use this one for PC
DEFAULT_DEV = 'COM7'
DEFAULT_BAUD_RATE = 19200
DEFAULT_TIMEOUT = 0.1

# Roboforth Strings
CR = '\r'
LF = '\n'

PURGE = 'PURGE'
ROBOFORTH = 'ROBOFORTH'
DECIMAL = 'DECIMAL'
START = 'START'
JOINT = 'JOINT'
CALIBRATE = 'CALIBRATE'
HOME = 'HOME'
WHERE = 'WHERE'
CARTESIAN = 'CARTESIAN'
SPEED = 'SPEED'
ACCEL = 'ACCEL'
MOVETO = 'MOVETO'
HAND = 'HAND'
WRIST = 'WRIST'
ENERGIZE = 'ENERGIZE'           #Start motors
DE_ENERGIZE = 'DE-ENERGIZE'     #power down motors
QUERY = ' ?'
IMPERATIVE = ' !'
TELL = 'TELL'
MOVE = 'MOVE'
CARTESIAN_NEW_ROUTE = 'CARTESIAN NEW ROUTE'
RESERVE = 'RESERVE'

OK = 'OK'


class StPosCart():

    def __init__(self, pos=[0, 0, 0, 0, 0]):
        self.set(pos)

    def set(self, pos):
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        self.pitch = pos[3]
        self.roll = pos[4]

    def __repr__(self):
        return 'X=%smm, Y=%smm, Z=%smm, Pitch=%sdeg, Roll=%sdeg' % (self.x/10.0,
                                                         self.y/10.0,
                                                         self.z/10.0,
                                                         self.pitch/10.0,
                                                         self.roll/10.0)


class StArm():
    '''Class for controlling the 5-axis R17 arm from ST Robotics'''

    '''
    Description:
    Create a serial connection and open it.

    Inputs:
        dev_name: The name of the serial device. For Macs/Linux, use
        /dev/tty.somestringofcharsandnums and for PCs use COMX where
        X is the COM port number the serial connector for the arm is
In [36]:
        connected to.
    '''

    def __init__(self, dev=DEFAULT_DEV, baud=DEFAULT_BAUD_RATE,
                 init=True, to=DEFAULT_TIMEOUT, debug=False):
        self.cxn = s.Serial(dev, baudrate=baud, timeout=to)
        # TODO
        # Check and parse return values of all ROBOFORTH methods called.
        self.debug = debug
        self.curr_pos = StPosCart()
        self.prev_pos = StPosCart()
        self.block_timeout = 10
        if init:
            self.cxn.flushInput()
            self.purge()
            self.roboforth()
            self.joint()
            self.start()
            self.calibrate()
            self.home()
            self.cartesian()
            self.where()

        self.tool_length = 0

    def set_tool_length(self, length):
        self.tool_length = length

    def purge(self):
        cmd = PURGE
        print('Purging...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def roboforth(self):
        cmd = ROBOFORTH
        print('Starting RoboForth...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def decimal(self):
        print('Setting decimal mode...')
        self.cxn.flushInput()
        self.cxn.write(DECIMAL + CR)

    def start(self):
        cmd = START
        print('Starting...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def joint(self):
        cmd = JOINT
        print('Setting Joint mode...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def calibrate(self):
        cmd = CALIBRATE
        print('Calibrating...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def home(self):
        cmd = HOME
        print('Homing...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def cartesian(self):
        cmd = CARTESIAN
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def block_on_result(self, cmd):
        t = time.time()
        s = self.cxn.read(self.cxn.inWaiting())

        while s[-5:-3] != OK:
            #Match '>' only at the end of the string
            if s[-1:] == '>':
                if self.debug:
                    print('Command ' + cmd + ' completed without ' +
                          'verification of success.')
                raise Exception('Arm command failed to execute as expected.', s)
            s += self.cxn.read(self.cxn.inWaiting())

        if self.debug:
            print('Command ' + cmd + ' completed successfully.')
        return s

    def get_status(self):
        if self.cxn.isOpen():
            self.cxn.write('' + CR)

    def get_speed(self):
        cmd = SPEED + QUERY
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        result = self.block_on_result(cmd)
        return int(result.split(' ')[-2])

    def set_speed(self, speed):
        cmd = str(speed) + ' ' + SPEED + IMPERATIVE
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def get_accel(self):
        cmd = ACCEL + QUERY
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        result = self.block_on_result(cmd)
        return int(result.split(' ')[-2])

    def set_accel(self, accel):
        cmd = str(accel) + ' ' + ACCEL + IMPERATIVE
        print('Setting acceleration to %d' % accel)
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def move_to(self, pos, block=True):
        cmd = str(pos[0]) + ' ' + str(pos[1]) + ' ' + str(pos[2]) + ' MOVETO'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        if block:
            self.block_on_result(cmd)
            self.where()

    def move(self, del_pos):
        cmd = str(del_pos[0]) + ' ' + str(del_pos[1]) + ' ' + str(del_pos[2]) + ' MOVE'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.where()

    def rotate_wrist(self, roll):
        cmd = TELL + ' ' + WRIST + ' ' + str(roll) + ' ' + MOVETO
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def rotate_wrist_rel(self, roll_inc):
        cmd = TELL + ' ' + WRIST + ' ' + str(roll_inc) + ' ' + MOVE
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.cartesian()
        self.where()

    def rotate_hand(self, pitch):
        cmd = TELL + ' ' + HAND + ' ' + str(pitch) + ' ' + MOVETO
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.cartesian()
        self.where()

    def rotate_hand_rel(self, pitch_inc):
        cmd = TELL + ' ' + HAND + ' ' + str(pitch_inc) + ' ' + MOVE
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.cartesian()
        self.where()

    def energize(self):
        cmd = ENERGIZE
        print('Powering motors...')
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def de_energize(self):
        cmd = DE_ENERGIZE
        print('Powering down motors...')
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def where(self):
        self.cartesian()
        cmd = WHERE
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        res = self.block_on_result(cmd)
        try:
            lines = res.split('\r\n')
            #TODO: Need to account for possibility that arm is in decimal mode

            cp = [int(x.strip().replace('.', ''))
                  for x in shlex.split(lines[2])]
            pp = [int(x.strip().replace('.', ''))
                  for x in shlex.split(lines[3])[1:]]

            self.curr_pos.set(cp)
            self.prev_pos.set(pp)
        except RuntimeError, e:
            print('Exception in where.')
            print(e)
            self.curr_pos.set([0, 0, 0, 0, 0])
            self.prev_pos.set([0, 0, 0, 0, 0])

        return (self.curr_pos, self.prev_pos)