#!/usr/bin/env python
from __future__ import division
from __future__ import absolute_import
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String
import time
import serial as s
import re
import shlex

# Use this one for Mac/Linux
# DEFAULT_DEV = '/dev/tty.KeySerial1'

#Robofourth manual
#http://www.strobotics.com/manuals/manual15.htm
#
# """
# Speed may be changed during the progress of a route with \'LEARN e.g. \'LEARN SPEED 7500 (or other value or other variable)
# Associated commands are:
# (line number) \'REPLACE SPEED (new value)
# (line number) \'INSERT SPEED (new value)
# """

# Use this one for PC
DEFAULT_DEV = '/dev/ttyUSB0'
#DEFAULT_DEV = '/dev/tty.KeySerial1'
DEFAULT_BAUD_RATE = 19200
DEFAULT_TIMEOUT = 0.05

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
HAND = 'L-HAND'
WRIST = 'WRIST'
ELBOW = 'ELBOW'
SHOULDER = 'SHOULDER'
WAIST = 'WAIST'
ENERGIZE = 'ENERGIZE'
DE_ENERGIZE = 'DE-ENERGIZE'
QUERY = ' ?'
IMPERATIVE = ' !'
TELL = 'TELL'
MOVE = 'MOVE'
ALIGN = 'ALIGN'
NONALIGN = 'NONALIGN'
CONTINUOUS = 'CONTINUOUS'
RUN = 'RUN'
STARTOVER = 'STARTOVER'
RESERVE = 'RESERVE'
ROUTE = 'ROUTE'
LEARN = 'LEARN'
START_HERE = 'START_HERE'
DRY = 'DRY'
SMOOTH = 'SMOOTH'
ADJUST = 'ADJUST'
NEW = 'NEW'
INSERT = 'INSERT'

OK = 'OK'


class StPosCart(object):

    def __init__(self, pos=[0, 0, 0, 0, 0]):
        self.set(pos)

    def set(self, pos):
        self.x = pos[0]
        self.y = pos[1]
        self.z = pos[2]
        self.pitch = pos[3]
        self.roll = pos[4]

    def __repr__(self):
        return '(x=%s, y=%s, z=%s, pitch=%s roll=%s)' % (self.x, self.y,
                                                         self.z,
                                                         self.pitch,
                                                         self.roll)


class StArm(object):
    '''Class for controlling the 5-axis R17 arm from ST Robotics'''

    '''
    Description:
    Create a serial connection and open it.
    Inputs:
        dev_name: The name of the serial device. For Macs/Linux, use
        /dev/tty.somestringofcharsandnums and for PCs use COMX where
        X is the COM port number the serial connector for the arm is
        connected to.
    '''

    def __init__(self, baud=DEFAULT_BAUD_RATE, to=DEFAULT_TIMEOUT):
        rospy.init_node('robot_arm', anonymous=True)

        self.debug = True

        possiblePorts = ['/dev/ttyUSB0', '/dev/ttyUSB1','/dev/ttyUSB2', '/dev/ttyUSB3', '/dev/ttyUSB4']
        self.pub = rospy.Publisher('arm_debug', String, queue_size=10)

        for port in possiblePorts:
            try:
                self.cxn = s.Serial(port, baudrate=baud, timeout=to)
                print "Connected to port: ", port
                return
            except:
                print "Couldn't connect to port: ", port
                pass


    def initial_calibration(self):
        self.cxn.flushInput()
        # self.purge() #this command seems to block the robot
        self.roboforth()
        self.joint()
        self.start()
        self.calibrate()
        self.home()
        self.cartesian()

        self.curr_pos = StPosCart()
        self.prev_pos = StPosCart()
        self.where()

    def purge(self):
        cmd = PURGE
        print 'Purging...'
        self.cxn.flushInput()
        print 'flush'
        self.cxn.write(cmd + CR)
        print "wrote: ", cmd+CR
        print 'write'
        self.block_on_result(cmd)

    def roboforth(self):
        cmd = ROBOFORTH
        print 'Starting RoboForth...'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def decimal(self):
        print 'Setting decimal mode...'
        self.cxn.flushInput()
        self.cxn.write(DECIMAL + CR)

    def start(self):
        cmd = START
        print 'Starting...'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def execute_command(self, cmd):
        print 'Running custom command...'
        self.cxn.write(cmd + CR)
        result = self.block_on_result(cmd)
        print result

    def continuous(self):
        cmd = CONTINUOUS
        print 'Setting mode to continuous...'
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def segmented(self):
        cmd = SEGMENTED
        print 'Setting mode to segmented...'
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def joint(self):
        cmd = JOINT
        print 'Setting Joint mode...'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    '''
    CARTESIAN NEW ROUTE hat 4 RESERVE hat hat LEARN DECIMAL CF
    hat 1 INSERT DECIMAL CF
    hat 1 INSERT DECIMAL CF
    hat 1 INSERT DECIMAL CF
    DECIMAL 0 0 900 400 10 20 hat 1 LINE DLD
    DECIMAL 0 0 900 200 20 200 hat 2 LINE DLD
    DECIMAL 0 0 900 100 30 300 hat 3 LINE DLD
    DECIMAL 0 0 900 20 100 100 hat 4 LINE DLD
    '''

    def create_route(self, route_name, commands, debug = False):
        '''
        Commands should be a list of lists [[x,y,z],[x,y,z],...]
        Lists must be [x,y,z] to use default pitch, yaw, roll or [x,y,z,pitch,yaw,roll] to specify

        Output format for one point:
            CARTESIAN NEW ROUTE route_name 4 RESERVE route_name route_name LEARN DECIMAL CF
            route_name 1 INSERT DECIMAL CF
            DECIMAL roll yaw pitch z y x route_name 1 LINE DLD
        '''

        # Initiaze in Cartesian mode and declare new route
        cmd = '  ' + CARTESIAN + ' ' + NEW + ' ' + ROUTE + ' ' + route_name
        # Reserve correct ammount of memory since default is 20
        print "LEN COMMANDS IS: ", len(commands)
        cmd += ' ' + str(len(commands)) + ' ' + RESERVE + ' ' + route_name

        # Put arm in Learning mode
        cmd += ' ' +route_name + ' ' + LEARN + ' ' + DECIMAL + ' CF'

        print "Creating route " + route_name

        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd, debug)

        # Required to run more than one pose at once
        for x in xrange(len(commands)-1):
            cmd = route_name + ' ' + str(x+1) + ' INSERT DECIMAL CF'

            self.cxn.flushInput()
            self.cxn.write(cmd + CR)
            self.block_on_result(cmd, debug)

        index = 0
        for cmd in commands:
            index += 1
            point = str(cmd[2]) + ' ' + str(cmd[1]) + ' ' + str(cmd[0])

            print "CMD IS: ", cmd
            print "LEN: ", len(cmd)

            if len(cmd) == 6:
                print "MAKING EULER ANGLES"
                euler_angles = str(cmd[5]) + ' ' + str(cmd[4]) + ' ' + str(cmd[3])
            else:
                print "DEFAULT EULER ANGLES"
                euler_angles = '0 0 0'

            cmd_tosend = DECIMAL + ' ' + euler_angles + ' ' + point +  ' ' + route_name + ' ' + str(index) + ' LINE DLD'
            print cmd_tosend
            print "Adding [" + point + ' ' + euler_angles + "] to route"
            self.cxn.flushInput()
            self.cxn.write(cmd_tosend + CR)
            self.block_on_result(cmd_tosend, debug)

    def calibrate(self):
        cmd = CALIBRATE
        print 'Calibrating...'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def home(self):
        cmd = HOME
        print 'Homing...'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def cartesian(self, block=False):
        cmd = CARTESIAN
        print 'Setting mode to Cartesian...'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def block_on_result(self, cmd, debug=False):
        t = time.time()
        print "in block_on_result"
        s = self.cxn.read(self.cxn.inWaiting())
        print s
        while s[-5:-3] != OK:
            # print "got: ", s
            # time.sleep(1)
            #Match '>' only at the end of the string
            if s[-1:] == '>':
                rospy.loginfo(" s is : %s" , s)
                if self.debug:
                    print " "
                    print "------------------"
                    print " "
                    print ('Command ' + cmd + ' completed without ' +
                          'verification of success.')
                    print "FAILED:  " + s
                    print "------------------"
                    print " "

                self.pub.publish(cmd + " FAILED: " + s)
                # if "Too tight" in s:
                #     print "CATCHING TOO TIGHT ERROR"
                #     return s
                # else:
                raise Exception('Arm command failed to execute as expected.', s)
            s += self.cxn.read(self.cxn.inWaiting())

        if self.debug:
            print ('Command ' + cmd + ' completed successfully.')
        return s

    def get_status(self):
        if self.cxn.isOpen():
            self.cxn.write('' + CR)

    def get_speed(self):
        cmd = SPEED + QUERY
        print 'Getting current speed setting...'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        result = self.block_on_result(cmd)
        print result
        return int(result.split(' ')[-2])

    def set_speed(self, speed):
        print ('Setting speed to %d' % speed)
        cmd = str(speed) + ' ' + SPEED + IMPERATIVE
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def set_point(self, name):
        cmd = "POINT " + name
        print 'Getting current point'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def get_accel(self):
        cmd = ACCEL + QUERY
        print 'Getting current acceleration setting...'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        result = self.block_on_result(cmd)
        return int(result.split(' ')[-2])

    def set_accel(self, accel):
        cmd = str(accel) + ' ' + ACCEL + IMPERATIVE
        print ('Setting acceleration to %d' % accel)
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def run_route(self, route):
        cmd = SMOOTH + ' ' + route + ' ' + RUN
        # self.set_accel(6000)
        # cmd = route + ' ' + RUN
        print ('Running route %s' % route)
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        res = self.block_on_result(cmd)
        # if "Too tight in res":
            # raise Exception('Arm command failed to execute as expected.', res)

        time.sleep(1)
        self.where()

    # def run_route(self, route):
    #     too_fast = True
    #     tries = 1
    #     # orig_accel = self.get_accel()
    #     # orig_speed = self.get_speed()
    #
    #     while too_fast:
    #         print('Doing dry run of route %s' % route)
    #         cmd = DRY + ' ' + route + ' ' + RUN
    #         self.cxn.flushInput()
    #         self.cxn.write(cmd + CR)
    #         res = self.block_on_result(cmd)
    #
    #         if "Too tight" in res:
    #             print "CATCHING TOO TIGHT ERROR"
    #             # if tries < 2:
    #             #     #TODO: Fix the acceleration scaling
    #             #     self.set_accel(orig_accel+1500*(tries))
    #             #     tries += 1
    #             # elif tries < 5:
    #             #     self.set_speed(orig_speed-1000*(tries-1))
    #             #     tries += 1
    #             # else:
    #             #     too_fast = False
    #             #     self.set_accel(orig_accel)
    #             #     self.set_speed(orig_speed)
    #             raise Exception('Arm command failed to execute as expected.', res)
    #
    #             time.sleep(1)
    #         else:
    #             too_fast = False
    #
    #     # route is string name of learned path
    #     cmd = SMOOTH + ' ' + route + ' ' + RUN
    #     # self.set_accel(6000)
    #     # cmd = route + ' ' + RUN
    #     print('Running route %s' % route)
    #     self.cxn.flushInput()
    #     self.cxn.write(cmd + CR)
    #     self.block_on_result(cmd)
    #     # self.set_accel(orig_accel)
    #     # self.set_speed(orig_speed)
    #     self.where()

    def move_to(self, x, y, z, debug=False, block=True):
        if debug:
            cmd = str(x) + ' ' + str(y) + ' ' + str(z) + ' MOVETO'
            print ('Moving to cartesian coords: (' + str(x) + ', ' +
                  str(y) + ', ' + str(z) + ')')
        self.cartesian()
        self.cxn.flushInput()
        self.cxn.write(str(x) + ' ' + str(y) + ' ' + str(z) + ' MOVETO' + CR)
        if block:
            self.block_on_result(cmd)
            self.where()

    def rotate_wrist(self, roll):
        cmd = TELL + ' ' + WRIST + ' ' + str(roll) + ' ' + MOVETO
        print ('Rotating wrist to %s' % roll)
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def rotate_wrist_rel(self, roll_inc):
        cmd = TELL + ' ' + WRIST + ' ' + str(roll_inc) + ' ' + MOVE
        print ('Rotating wrist by %s.' % roll_inc)
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.cartesian()
        self.where()

    def rotate_hand(self, pitch):
        cmd = TELL + ' ' + HAND + ' ' + str(pitch) + ' ' + MOVETO
        print ('Rotating hand to %s.' % pitch)
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.where()

    def rotate_elbow(self, pitch):
        cmd = TELL + ' ' + ELBOW + ' ' + str(pitch) + ' ' + MOVETO
        print ('Rotating hand to %s.' % pitch)
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.where()

    def rotate_shoulder(self, pitch):
        cmd = TELL + ' ' + SHOULDER + ' ' + str(pitch) + ' ' + MOVETO
        print ('Rotating hand to %s.' % pitch)
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.where()

    def rotate_waist(self, pitch):
        cmd = TELL + ' ' + WAIST + ' ' + str(pitch) + ' ' + MOVETO
        print ('Rotating hand to %s.' % pitch)
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.where()

    def rotate_waist_rel(self, pitch):
        cmd = TELL + ' ' + WAIST + ' ' + str(pitch) + ' ' + MOVE
        print ('Rotating hand to %s.' % pitch)
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.cartesian()
        self.where()


    def rotate_hand_rel(self, pitch_inc):
        cmd = TELL + ' ' + HAND + ' ' + str(pitch_inc) + ' ' + MOVE
        print ('Rotating hand by %s' % pitch_inc)
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
        self.cartesian()
        self.where()

    def move_hand(self, roll):
        self.rotate_hand(roll)

    def energize(self):
        cmd = ENERGIZE
        print 'Powering motors...'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def de_energize(self):
        cmd = DE_ENERGIZE
        print 'Powering down motors...'
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)

    def where(self):
        self.cartesian()
        cmd = WHERE
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        res = self.block_on_result(cmd)
        print "I'M LOCATED"
        print res
        self.pub.publish(str(res))
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
            self.pub.publish("EXCEPT IN WHERE: " + str(e))
            print 'Exception in where.'
            print e
            self.curr_pos.set([0, 0, 0, 0, 0])
            self.prev_pos.set([0, 0, 0, 0, 0])

        return (self.curr_pos, self.prev_pos)
    def check_if_done(self):
        cmd = TELL + 'FOOBAR'        # Not a real command. If it responds with an error, other processes are done.
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        return self.block_on_result(cmd)

    def dummy(self):
        return "no yo're a dummy!"

    def get_joint_states(self):
        # WAIST SHOULDER ELBOW L-HAND R-H/WR
        #Wrist 1:3 (300 = 9 deg.) CCW
        #Hand 1:1 (300 = 3 deg.) CCW, CW for WRIST
        #Elbow 1:1 (300 = 3 deg.) CW
        #Shoulder 2:1 (600 = 3 deg.) CCW
        #Waist 1:1 (eyeball) CW
        cmd = "WHERE"
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        res = self.block_on_result(cmd)
        lines = res.split('\r\n')
        joints = [int(j) for j in lines[2].split()]
        joints[1] /= 2.  #600 = 3 deg
        joints[4] *= 3.  #300 = 9 deg
        joints[4] -= joints[3] # counteract
        joints[2], joints[4] = -joints[2], -joints[4] #opposite turning direction
        joints = [j/100*3.1415/180 for j in joints] #to radians
        return joints


    def test(self):
        res = []
        for r in ['B-RATIO', 'S-RATIO', 'E-RATIO', 'W-RATIO', 'T-RATIO']:
            cmd = r + QUERY
            self.cxn.flushInput()
            self.cxn.write((cmd + CR).encode)
            res.append(self.block_on_result(cmd))
        print res

    def lock_wrist_angle(self,TF = True):
        if TF:
            cmd = ALIGN
        else:
            cmd = NONALIGN
        print 'Locking gripper orientation...'
        self.cxn.flushInput()
        self.cxn.write(cmd + CR)
        self.block_on_result(cmd)
