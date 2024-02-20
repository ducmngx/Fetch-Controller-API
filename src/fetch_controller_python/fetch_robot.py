#!/usr/bin/env python3
import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("./src/fetch_controller_python"))
from subs.fetch_gripper import Gripper
from subs.fetch_move import FetchBaseMove
from subs.fetch_manipulator import FetchManipulator
from lib.utils import Algorithms, MatrixOperation
from sensor_msgs.msg import JointState
from lib.params import *
from lib.board_tracker import *
import json

'''
This is the wrapper of all Fetch's functionalities
'''
class FetchRobot:

    def __init__(self, base=BASE_POS):
        '''
        Add comments
        Only create private objects for security purpose
        base_position: Fetch's base position in odom frame
        '''
        self.__op = MatrixOperation()
        self.__base_position = self.__op.vectorizeXYZ(base)
        self.__driver = FetchBaseMove()
        self.__manipulator = FetchManipulator() 
        self.__gripper = Gripper()
        self.__head_joints = {HEAD_JOINTS[i]: 0.0 for i in range(len(HEAD_JOINTS))}
        self.__joints = {ARM_AND_TORSO_JOINTS[i]: 0.0 for i in range(len(ARM_AND_TORSO_JOINTS))}
        self.__base_transform = self.__op.getTransformMatrix() # ones matrix
        self.lookAt([[0.0 for _ in range(2)], [0.0 for _ in range(2)], [0.0 for _ in range(2)]])
        rospy.Subscriber(JOINT_STATES, JointState, self.__joint_states_callback, queue_size=1)

    def report_fetch_state(self):
        print("**********BEGIN**********\n")
        print(f"**********BASE**********\n(0, 0, 0) in odom frame is {self.__base_position}")
        print(f"Transform matrix from base to odom now is: \n {self.__base_transform}")
        print(f"**********HEAD**********\n Joints' states:")
        print(json.dumps(self.__head_joints, indent = 4))
        print(f"**********BODY**********\n Joints' states:")
        print(json.dumps(self.__joints, indent = 4))
        print("**********END**********\n\n")


    def __update_base_transform(self, new_transform) -> None:
        '''
        Update base transform matrix
        '''
        updated_transform = self.__op.combineTransform(new_transform, self.__base_transform )
        self.__base_transform = updated_transform

    def __update_base(self, linear_x, angular_z) -> None:
        '''
        Update base position vector
        '''
        M = self.__op.getTransformMatrix(rotation=(0.0, 0.0, angular_z), translation=(linear_x, 0.0, 0.0))
        new_base_position = self.__op.transform(self.__base_position, M)

        # update base position
        self.__base_position = new_base_position

        # update base transform
        self.__update_base_transform(new_transform=M)

    def fromBase2Odom(self, pos):
        '''
        This method assumes Fetch starts at (0,0,0) in odom frame
        '''
        inverse_transform = self.__op.getInverseMatrix(self.__base_transform)
        
        return self.__op.transform(pos, inverse_transform)

    def __joint_states_callback(self, message):
        '''
        Add comments
        '''
        for i,name in enumerate(message.name):
            pos = message.position[i]
            if name in self.__joints.keys():
                self.__joints[name] = pos   
            if name in self.__head_joints.keys():
                self.__head_joints[name] = pos   
    
    def get_joint_states(self):
        '''
        Add comments
        '''
        return self.__joints.copy()
    
    def get_head_states(self):
        '''
        Add comments
        '''
        return self.__head_joints.copy()

    def execute(self, manipulation_matrix):
        '''
        This method sends the manipulation command to fetch
        '''
        self.__manipulator.execute(manipulation_matrix)

    def go(self, linear_x, angular_z):
        '''
        This method moves the base(Freight) of Fetch
        '''
        self.__driver.move(linear_x, angular_z)
        self.__update_base(linear_x, angular_z)

    def grasp(self):
        '''
        This method enables Fetch to grab
        '''
        self.__gripper.close()

    def release(self):
        '''
        This method releases Fetch's grip 
        '''
        self.__gripper.open()

    def lookAt(self, manipulation_matrix, execution_time=1):
        '''
        This method changes the angle of Fetch's camera by moving its head position
        '''
        self.__manipulator.lookAt(manipulation_matrix, execution_time)

    def getReady(self):
        '''
        This method moves Fetch to a predefined initial pose
        '''
        self.__manipulator.reset()
        self.__gripper.open()

    def getUnReady(self):
        '''
        This method moves Fetch to a predefined rest pose
        It should be run before turning Fetch off
        '''
        self.__manipulator.rest()
        self.__gripper.open()

    def getVisual(self):
        '''
        Add comment
        '''
        pass

    def getGripperLocation(self):
        '''
        Add comment
        '''
        M = self.__getTransformationBase2Gripper()
        gripper_location = self.__op.transform(self.__base_position, M)

        return gripper_location
    
    def convertGrip2Base(self, grip_location):
        '''
        Add comment
        '''
        M = self.__getTransformationBase2Gripper()
        computed_base_location = self.__op.transform(grip_location, self.__op.getInverseMatrix(M))

        return computed_base_location

    def __getTransformationBase2Gripper(self):
        '''
        Add comment
        '''
        current_joints = self.get_joint_states()
        final_matrix = self.__op.getTransformMatrix()

        # transform from base to torso
        base2torso_translation = self.__op.getTransformMatrix(rotation=(0,0,0), translation=BASE2TORSO)
        torso_action = self.__op.getTransformMatrix(rotation=(0,0,0), translation=(0, 0, current_joints['torso_lift_joint']))
        base2torso_matrix = self.__op.combineTransform(torso_action, base2torso_translation)
        final_matrix = self.__op.combineTransform(base2torso_matrix, final_matrix)

        # transform from torso to shoulder pan
        torso2shoulderpan_matrix = self.__op.getTransformMatrix(rotation=(0,0,current_joints['shoulder_pan_joint']), translation=TORSO2SHOULDERPAN)
        final_matrix = self.__op.combineTransform(torso2shoulderpan_matrix, final_matrix)

        # transform from shoulder pan to should lift
        shoulderpan2shoulderlift_matrix = self.__op.getTransformMatrix(rotation=(0, current_joints['shoulder_lift_joint'], 0), translation=SHOULDERPAN2SHOULDERLIFT)
        final_matrix = self.__op.combineTransform(shoulderpan2shoulderlift_matrix, final_matrix)

        # transform shoulder lift to upperarm roll
        shoulderlift2upperarm_matrix = self.__op.getTransformMatrix(rotation=(current_joints['upperarm_roll_joint'], 0, 0), translation=SHOULDERLIFT2UPPERARM)
        final_matrix = self.__op.combineTransform(shoulderlift2upperarm_matrix, final_matrix)

        # transform upperarm to elbow
        upperarm2elbow_matrix = self.__op.getTransformMatrix(rotation=(0, current_joints['elbow_flex_joint'], 0), translation=UPPERARM2ELBOW)
        final_matrix = self.__op.combineTransform(upperarm2elbow_matrix, final_matrix)

        # transform elbow to forearm
        elbow2forearm_matrix = self.__op.getTransformMatrix(rotation=(current_joints['forearm_roll_joint'], 0, 0), translation=ELBOW2FOREARM)
        final_matrix = self.__op.combineTransform(elbow2forearm_matrix, final_matrix)

        # transform forearm to wrist flex
        forearm2wflex = self.__op.getTransformMatrix(rotation=(0, current_joints['wrist_flex_joint'], 0), translation=FOREARM2WRISTFLEX)
        final_matrix = self.__op.combineTransform(forearm2wflex, final_matrix)

        # transform wrist flex to wrist roll
        wflex2wroll = self.__op.getTransformMatrix(rotation=(current_joints['wrist_roll_joint'], 0, 0), translation=WRISTFLEX2WRISTROLL)
        final_matrix = self.__op.combineTransform(wflex2wroll, final_matrix)

        # transform wrist roll to gripper
        wroll2gripper = self.__op.getTransformMatrix(rotation=(0, 0, 0), translation=WRISTROLL2GRIPPER)
        final_matrix = self.__op.combineTransform(wroll2gripper, final_matrix)

        return final_matrix