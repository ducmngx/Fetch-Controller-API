#!/usr/bin/env python3
import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("./src/fetch_controller_python"))
from fetch_gripper import Gripper
from fetch_move import FetchBaseMove
from fetch_manipulator import FetchManipulator
from lib.utils import Algorithms
# from lib import utils

'''
This is the wrapper of all Fetch's functionalities
'''
class FetchRobot:

    def __init__(self):
        # rospy.init_node("control_fetch_robot")
        self.driver = FetchBaseMove()
        self.manipulator = FetchManipulator() 
        self.gripper = Gripper()

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
