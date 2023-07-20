import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from lib.utils import *
from fetch_controller_python.fetch_robot import FetchRobot
import rospy

if __name__ == '__main__':
    '''
    This is where we subcribe to the Fetch state node, 
    run the optimizer algorithm to plan the optimal move
    to execute on fetch_controller
    '''
    pass