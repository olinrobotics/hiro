import rospy
from std_msgs.msg import Int32

class Arm:
    def __init__(self):
        rospy.init_node('grip')
        self.pub_status = rospy.Publisher('/grab_cmd', Int32, queue_size = 10)

    def give_input(self):
        while True:
            question = int(raw_input("In/Out?"))
            self.pub_status.publish(question)

if __name__ == '__main__':
    gemini = Arm()
    gemini.give_input()
