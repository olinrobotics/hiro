#!/usr/bin/env python

import rospy
import rospkg
from irl.msg import Real_Cube, Grid_Cube, Real_Structure, Grid_Structure, Cube_Structures
from std_msgs.msg import String

import time
import smtplib
from email.mime.text import MIMEText

'''
arms.irl2018@gmail.com
Password: PolluxCastor2018
'''

class MessageSender(object):
    """
    The Message Sender class that listens to the build_cmd class and send the
    message over email
    """
    def __init__(self):
        rospy.init_node("message_sender")
        tcp_ip = rospy.get_param("~robot_ip")
        arm_dict = {'10.42.0.175':'pollux','10.42.0.54':'castor'}
        self.name = arm_dict[tcp_ip]

        # only castor sends the message
        if self.name == 'castor':
            self.cmd_sub = rospy.Subscriber("/build_cmd_castor", Cube_Structures, self.send_cube_msg, queue_size=1)

        if self.name =='castor':
            self.castor_status_sub = rospy.Subscriber("/coordination_status_castor", String, self.send_castor_status_msg, queue_size=1)
        elif self.name == 'pollux':
            self.pollux_status_sub = rospy.Subscriber("/coordination_status_pollux", String, self.send_pollux_status_msg, queue_size=1)
        self.is_sending = False
        self.cube_building = Cube_Structures()

    def send_castor_status_msg(self,data):

        smtp_ssl_host = 'smtp.gmail.com'  # smtp.mail.yahoo.com
        smtp_ssl_port = 465
        username = 'arms.irl2018@gmail.com'
        password = 'PolluxCastor2018'
        sender = 'arms.irl2018@gmail.com'
        targets = ['arms.irl2018@gmail.com']

        info = '/coordination_status_castor' + ',' + str(data.data)

        msg = MIMEText(info)
        msg['Subject'] = 'arm_msg ' + time.ctime()
        msg['From'] = sender
        msg['To'] = ', '.join(targets)

        server = smtplib.SMTP_SSL(smtp_ssl_host, smtp_ssl_port)
        server.login(username, password)
        server.sendmail(sender, targets, msg.as_string())
        print ('Castor Status Message Sent Successfully!')
        server.quit()

    def send_pollux_status_msg(self,data):

        smtp_ssl_host = 'smtp.gmail.com'  # smtp.mail.yahoo.com
        smtp_ssl_port = 465
        username = 'arms.irl2018@gmail.com'
        password = 'PolluxCastor2018'
        sender = 'arms.irl2018@gmail.com'
        targets = ['arms.irl2018@gmail.com']

        info = '/coordination_status_pollux' + ',' + str(data.data)

        msg = MIMEText(info)
        msg['Subject'] = 'arm_msg ' + time.ctime()
        msg['From'] = sender
        msg['To'] = ', '.join(targets)

        server = smtplib.SMTP_SSL(smtp_ssl_host, smtp_ssl_port)
        server.login(username, password)
        server.sendmail(sender, targets, msg.as_string())
        print ('Pollux Status Message Sent Successfully!')
        server.quit()

    def send_cube_msg(self, data):
        self.cube_building = data
        print("Data: ")
        print(str(self.cube_building))

        grid_building = self.cube_building.grid_building.building
        real_building = self.cube_building.real_building.building
        smtp_ssl_host = 'smtp.gmail.com'  # smtp.mail.yahoo.com
        smtp_ssl_port = 465
        username = 'arms.irl2018@gmail.com'
        password = 'PolluxCastor2018'
        sender = 'arms.irl2018@gmail.com'
        targets = ['arms.irl2018@gmail.com']

        info = '/build_cmd,'
        for cube in real_building:
            info = info + str(cube.x) + ','
            info = info + str(cube.y) + ','
            info = info + str(cube.z) + ','
        for cube in grid_building:
            info = info + str(cube.x) + ','
            info = info + str(cube.y) + ','
            info = info + str(cube.z) + ','

        msg = MIMEText(info)
        msg['Subject'] = 'arm_msg ' + time.ctime()
        msg['From'] = sender
        msg['To'] = ', '.join(targets)

        server = smtplib.SMTP_SSL(smtp_ssl_host, smtp_ssl_port)
        server.login(username, password)
        server.sendmail(sender, targets, msg.as_string())
        print ('Cube Message Sent Successfully!')
        server.quit()


    def run(self):
        print("Message Sender Running")

        while not rospy.is_shutdown():
            try:
                rospy.sleep(5)
            except KeyboardInterrupt:
                break

if __name__=="__main__":
    ms = MessageSender()
    ms.run()
