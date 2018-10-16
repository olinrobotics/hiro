#!/usr/bin/env python

import st
import time
import serial
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy

#to subscribe to and publish images
from std_msgs.msg import String, Bool, UInt16
from tetris_arm.msg import TetArmArray

class ArmWrapper():
	def __init__(self, arm = 0):

		# Set up talkers and listeners
		self.pieceSub = rospy.Subscriber("armCommand", TetArmArray, self.goXYTH)
		self.downSub = rospy.Subscriber("downCmd", String, self.down)
		self.donePub = rospy.Publisher("inPosition", String)
		self.actuatorPub = rospy.Publisher("actuated", String)
		self.gripperPub = rospy.Publisher("gripperSize", String)
		self.move_to_pub = rospy.Publisher("move_to_cmd", TetArmArray)
		self.move_to_listener = rospy.Subscriber("move_to_cmd", TetArmArray, self.move_to_sub)
		self.printOut = rospy.Publisher("print", String)
		print 'set up pubsubs'

		if arm == 0:
			#self.arm = st.StArm(dev = '/dev/ttyUSB0', init = False, to = 0.1)
			# Initializes comunication to arm (tests reasonable possible port names)
			try:
				self.arm = st.StArm(dev = '/dev/ttyUSB1', init = False, to = 0.1)
			except:
				self.arm = st.StArm(dev = '/dev/ttyUSB0', init = False, to = 0.1)

			self.arm.start()
		else: self.arm = arm

		# make sure that the arm is started
		res = self.arm.cxn.readline()
		while res == '':
			res = self.arm.cxn.readline()
			print 'nope'
			self.arm.start()
		time.sleep(0.2)
		print 'res =', res

		# initialize arduino serial communication  (tests reasonable possible port names)
		try:
			self.ser = serial.Serial('/dev/ttyACM0', 9600)
		except:
			self.ser = serial.Serial('/dev/ttyACM1', 9600)

		# set up arm starting position
		self.arm.cartesian()
		self.x = 100
		self.y = 6000
		self.arm.move_to(self.x, self.y, 0)

		# start hand and wrist at the right angle
		self.arm.rotate_hand(1800)
		self.gripperOrientation = 0
		self.printOut.publish('gripperOrientation is %d' %self.gripperOrientation)
		self.arm.rotate_wrist(1000)   #vertical	
		self.arm.lock_wrist_angle()

		# start gripper open
		self.size = 'open'
		self.gripperPub.publish(self.size)

		# wait off the board
		self.arm.move_to(3000,3000,0)	


	

	def goXYTH(self, data):
		data = data.data
		self.x, self.y, th = data
		z = 0
		self.printOut.publish('lowlevel.goXYTH: Recieved /armCommand %f %f %f' %(self.x, self.y, th))
		if self.y < 2700:
			self.y = 2700
		if self.y > 7000:
			self.y = 7000
		self.arm.move_to(self.x,self.y,z)
		self.rotate_gripper(th)
		self.printOut.publish('lowlevel: sending /inPosition 0')
		self.donePub.publish(0)


	def down(self, data):
		size = data.data
		self.printOut.publish('lowlevel.down: Recieved /downCmd %s' %size)
		z = 500
		if size == 'fake':
			doneMsg = 'wait'
			self.size = 'open'
			size = 'open'
			z = 0
		if size == 'open':			#open
			doneMsg = 'released'
			if self.size == 'big':
				z = -700
			if self.size == 'small':
				z = -550		
		if size == 'big':
			doneMsg = 'grabbed'
			z = -700
		if size == 'small':
			doneMsg = 'grabbed'
			z = -550

		#should start to close before dropping when picking up a piece, but vice-versa when dropping
		if size == 'open':  #drop piece
			self.move_to_pub.publish([self.x, self.y, z])	# up	# down
			time.sleep(.5)
			self.gripperPub.publish(size)   # start to close gripper
		else:   #pick up piece
			self.move_to_pub.publish([self.x, self.y, z])	# down
			time.sleep(.5)
			self.gripperPub.publish(size)   # start to close gripper

		self.printOut.publish('lowlevel.down: Sending /gripperSize %s' %size)
		self.size = size
		time.sleep(.15)				# give time to pick up piece! TODO (if you can read from servo, make self.up)
		self.arm.move_to(self.x, self.y, 0)	# up
		self.printOut.publish('lowlevel.down: Sending /actuated %s' %doneMsg)
		self.actuatorPub.publish(doneMsg)

	def move_to_sub(self, data):
		x, y, z = data.data
		self.arm.move_to(x, y, z)
		self.printOut.publish('lowlevel.move_to_sub: Moving to position %d %d %d' %(x, y, z))



	def rotate_gripper(self, orientation):
		if orientation == self.gripperOrientation:
			return
		if orientation < self.gripperOrientation:
			direction = -1
		else: direction = 1
		self.arm.rotate_wrist_rel(3000*direction*abs(orientation - self.gripperOrientation))
		self.gripperOrientation = orientation
		self.arm.lock_wrist_angle()

		'''
		if orientation == 1:		# vertical
			if self.gripperOrientation == 1:
				pass
			else: 
				self.arm.rotate_wrist_rel(-3000)
				self.gripperOrientation = 1
				self.arm.lock_wrist_angle()
		if orientation == 0:		# horizontal
			if self.gripperOrientation == 0:
				pass
			else:
				self.arm.rotate_wrist_rel(3000)
				self.gripperOrientation = 0
				self.arm.lock_wrist_angle()
		'''

def main(args):
	rospy.init_node('lowlevel', anonymous=True)
	aw = ArmWrapper()
	try:
		print "spin"
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
