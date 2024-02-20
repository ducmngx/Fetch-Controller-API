import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from src.fetch_controller_python.fetch_robot import FetchRobot

if __name__ == '__main__':
    rospy.init_node("robot_test_2")
    robot = FetchRobot()

    robot.getReady()

    rospy.sleep(3)

    robot.getUnReady()

    # robot.go(-4,5)

    # rospy.sleep(10)

    # robot.go(-3,5)

    # rospy.sleep(2)

    # robot.go(4,0)

    # rospy.sleep(2)

    # robot.go(2,0)

    # rospy.sleep(2)