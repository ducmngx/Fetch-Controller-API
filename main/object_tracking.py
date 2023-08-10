
import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from lib.params import VISION_IMAGE_TOPIC
from lib.board_tracker import BoardTracker
from src.fetch_controller_python.fetch_robot import FetchRobot
# from src.fetch_controller_python.fetch_gripper import Gripper
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import std_msgs
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

class PoseTracker:
    def __init__(self):
        self.prev_tf = None
        self.cameraPoseTilt = 0
        self.cameraPosePan = 0

# Initialize the CvBridge class
bridge = CvBridge()

def execute(T):
    if ptk.prev_tf is None:
        ptk.prev_tf = T
    else:
        xx, xy, xz = ptk.prev_tf
        nx, ny, nz = T

    # tilt [-0.785 (U), 1.5708 (D) rad] = [-45, 90] 
    # pan [-1.5708 (R), 1.5708 (L) rad] = [-90, 90] 

        h_threshold = 40
        v_threshold = 30

        if nx > h_threshold:
            # move right
            alpha_horizon = -.005
        elif nx  < -h_threshold:
            # move left
            alpha_horizon = .005
        else:
            # stay
            alpha_horizon = 0

        if ny > v_threshold:
            # move up
            alpha_vertical = .005
        elif ny < -v_threshold:
            # move down
            alpha_vertical = -.005
        else:
            # stay
            alpha_vertical = 0

        newHorizon = ptk.cameraPosePan + alpha_horizon

        if np.abs(newHorizon) > 1.5708:
            newHorizon = ptk.cameraPosePan

        newVertical = ptk.cameraPoseTilt + alpha_vertical

        if newVertical > 1.5708 or newVertical < -0.785:
            newVertical = ptk.newVertical

        inputMatrix = [[newHorizon, newVertical], [0.0 for _ in range(2)], [0.0 for _ in range(2)]]
        robot.lookAt(inputMatrix)

        # update
        ptk.prev_tf = T
        ptk.cameraPosePan = newHorizon
        ptk.cameraPoseTilt = newVertical


 # Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    # rospy.loginfo(img_msg.header)

     # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    try:
        # Show the converted image
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        results = tracker.detect(gray)

        # print(results)
        rospy.loginfo(f"Number of tags detected is: {len(results)}")
        rospy.loginfo(f"Current head pos: pan-{ptk.cameraPosePan} and tilt-{ptk.cameraPoseTilt}")
        # if len(results) < 1:
        #     print('*************************************************************************')
        #     print("No APRILTAG detected")
        #     print('*************************************************************************')
        #     my_msg = Float32MultiArray()  
        #     my_msg.data = list(np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]))
        #     publisher.publish(my_msg)
        # else:    
        M, R, T = tracker.getTransformation(results[0])
        print('*************************************************************************')
        print(M)
        execute(T)
        print('*************************************************************************')
        my_msg = M
        publisher.publish(my_msg)
    except:
        print('*************************************************************************')
        print("No APRILTAG detected")
        print('*************************************************************************')
        my_msg = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        publisher.publish(my_msg)

if __name__ == '__main__':
    rospy.init_node('head_movement_tracking')
    # Initialize a tracker
    last_translate = None
    robot = FetchRobot()

    tracker = BoardTracker()
    ptk = PoseTracker()

    # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
    sub_image = rospy.Subscriber(VISION_IMAGE_TOPIC, Image, image_callback)

    # Init a publisher
    # publisher = rospy.Publisher('board2cam', Float32MultiArray, queue_size=10)
    publisher = rospy.Publisher('board2cam', numpy_msg(Floats), queue_size=10)

    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    while not rospy.is_shutdown():
        rospy.spin()