#!/usr/bin/env python3
import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("./src/fetch_controller_python"))
from fetch_gripper import Gripper
from fetch_move import FetchBaseMove
from fetch_manipulator import FetchManipulator
from lib.utils import Algorithms
from sensor_msgs.msg import JointState
from lib.params import ARM_AND_TORSO_JOINTS, JOINT_STATES

'''
This is the wrapper of all Fetch's functionalities
'''
class FetchRobot:

    def __init__(self):
        # rospy.init_node("control_fetch_robot")
        self.driver = FetchBaseMove()
        self.manipulator = FetchManipulator() 
        self.gripper = Gripper()
        self.joints = {ARM_AND_TORSO_JOINTS[i]: 0.0 for i in range(len(ARM_AND_TORSO_JOINTS))}
        rospy.Subscriber(JOINT_STATES, JointState, self.joint_states_callback, queue_size=1)
        # rospy.spin()

    def joint_states_callback(self, message):
        '''
        Add comments
        '''
        for i,name in enumerate(message.name):
            pos = message.position[i]
            if name in self.joints.keys():
                self.joints[name] = pos   
    
    def get_joint_states(self):
        '''
        Add comments
        '''
        return self.joints

    def getVisual(self):
        pass

    def execute(self, manipulation_matrix):
        '''
        This method sends the manipulation command to fetch
        '''
        self.manipulator.execute(manipulation_matrix)

    def go(self, linear_x, angular_z):
        '''
        This method moves the base(Freight) of Fetch
        '''
        self.driver.move(linear_x, angular_z)

    def grasp(self):
        '''
        This method enables Fetch to grab
        '''
        self.gripper.close()

    def release(self):
        '''
        This method releases Fetch's grip 
        '''
        self.gripper.open()

    def lookAt(self, manipulation_matrix, execution_time=1):
        '''
        This method changes the angle of Fetch's camera by moving its head position
        '''
        self.manipulator.lookAt(manipulation_matrix, execution_time)

    def getReady(self):
        '''
        This method moves Fetch to a predefined initial pose
        '''
        self.manipulator.reset()
        self.gripper.open()

    def getUnReady(self):
        '''
        This method moves Fetch to a predefined rest pose
        It should be run before turning Fetch off
        '''
        self.manipulator.rest()
        self.gripper.open()
