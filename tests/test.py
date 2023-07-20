
import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from lib.params import ARM_AND_TORSO_JOINTS
from fetch_controller_python.fetch_robot import FetchRobot
# from src.fetch_controller_python.fetch_gripper import Gripper
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        print("try")

    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image.jpeg', cv2_img)
        # cv2.imshow('camera_image', cv2_img)
        print("save")

if __name__ == "__main__":
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/joint_states" 
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    
    rospy.spin()
    # while not rospy.is_shutdown():
    #     rospy.spin()

