#!/usr/bin/env python
import roslib; roslib.load_manifest('jimmy')
import rospy
from std_msgs.msg import String
from jimmy.msg import *

def callback(data):
    global x_pos, y_pos
    print data
 
    raw_data = str(data)
    data1 = raw_data[6:]
    positions = data1.split(',')
    x_pos = int(positions[0])
    y_pos = int(positions[1])
    print [x_pos, y_pos]

    
def face_listen():
    global x_pos, y_pos
    global change_rate_x, change_rate_y
    global ideal_position_x, ideal_position_y
    rospy.init_node('face_listen', anonymous=True)
    r = rospy.Rate(200)
    pub = rospy.Publisher("jimmy_move_servo", jimmy_servo, queue_size = 10)
    rospy.Subscriber("face_location", String, callback)
    position_x = 0
    position_y = 0
    while not rospy.is_shutdown():
        msg = jimmy_servo()
        msg.servo_names.append("HeadPan")
        position_x -= (x_pos - ideal_position_x)*change_rate_x
        msg.positions.append(-position_x) # ideal_position_y)
        msg.servo_names.append("HeadTilt")
        position_y -= (y_pos - ideal_position_y)*change_rate_y
        msg.positions.append(position_y)
        msg.servo_names.append("HeadTilt2")
        msg.positions.append(position_y)
        pub.publish(msg)
#        print "in here"
        r.sleep()
#    rospy.spin()
        
if __name__ == '__main__':
    try:
        x_pos = 215
        y_pos = 120
        ideal_position_x = 215
        #150 old
        ideal_position_y = 120
        #108 old
        #change_rate_x = 0.00002 #you can change this to make arm move faster or slower
        change_rate_x = 0.00002
        #change_rate_y = 0.00004
        change_rate_y = 0.00004
        face_listen()

    except rospy.ROSInterruptException: pass