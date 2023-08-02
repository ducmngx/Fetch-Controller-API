#!/usr/bin/env python
import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from lib.params import ARM_AND_TORSO_JOINTS
from src.fetch_controller_python.fetch_robot import FetchRobot

if __name__ == '__main__':
    rospy.init_node("example_repub")
    robot = FetchRobot()
    # print("hahaha")

    print(robot.get_joint_states())

    rospy.sleep(2)
    robot.getReady()

    print(robot.get_joint_states())
    rospy.sleep(2)

    print(robot.get_joint_states())
    rospy.sleep(2)

    print(robot.get_joint_states())


# def joint_states_callback(message):
#     # filter out joint0 position:
#     print('---------------------------------------------------------')
#     for i,name in enumerate(message.name):
#         pos = message.position[i]
#         if name in joints.keys():
#             joints[name] = pos   

#     print(joints)
    

# if __name__ == '__main__':
#     rospy.init_node("example_repub")
#     joints = {ARM_AND_TORSO_JOINTS[i]: 0.0 for i in range(len(ARM_AND_TORSO_JOINTS))}
#     # pub = rospy.Publisher("joint0_topic", Float64, queue_size=1)
#     rospy.Subscriber("joint_states", JointState, joint_states_callback, queue_size=1)
#     rospy.spin()