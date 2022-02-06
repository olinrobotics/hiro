#!/usr/bin/env python3
"""
By Khang Vu, 2019

This script will combine multiple JointState nodes into one
and publish it to /joint_states. This is helpful when we want
to combine left_gripper, right_gripper, left_arm and right_arm
joint states all together.

Current usage: xamyab_ur_common.launch
"""
import rospy
from sensor_msgs.msg import JointState


class CombinedJointStates:
    def __init__(self, subscribed_topics, node_name='CombinedJointStates', publish_topic='/joint_states'):
        rospy.init_node(node_name, anonymous=True)
        self.msg_list = [None] * len(subscribed_topics)
        for i, topic in enumerate(subscribed_topics):
            if topic:
                rospy.Subscriber(topic, JointState, self.join_states_cb, callback_args=i, queue_size=10)
        self.publisher = rospy.Publisher(publish_topic, JointState, queue_size=10)

    def join_states_cb(self, msg, index):
        self.msg_list[index] = msg

    def publish(self):
        while not rospy.is_shutdown():
            joint_state_msg = JointState()
            for msg in self.msg_list:
                self.put_msg_to_joint_states(joint_state_msg, msg)

            self.publisher.publish(joint_state_msg)

    @staticmethod
    def put_msg_to_joint_states(joint_states_msg, msg):
        if msg:
            joint_states_msg.header = msg.header
            joint_states_msg.name.extend(msg.name)
            joint_states_msg.position.extend(msg.position)
            joint_states_msg.velocity.extend(msg.velocity)
            joint_states_msg.effort.extend(msg.effort)


if __name__ == '__main__':
    CombinedJointStates(subscribed_topics=['/left_arm/joint_states',
                                           '/right_arm/joint_states',
                                           '/left_gripper/joint_states',
                                           '/right_gripper/joint_states']).publish()