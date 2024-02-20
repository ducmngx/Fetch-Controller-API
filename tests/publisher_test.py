import sys, time, os
sys.path.insert(1, os.path.abspath("."))
import rospy
from lib.params import *
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32MultiArray, Float64MultiArray

if __name__ == '__main__':
    rospy.init_node('head_movement_tracking')
    # Initialize a tracker
    # last_translate = None
    # robot = FetchRobot()

    # tracker = BoardTracker()
    # ptk = PoseTracker()

    # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
    # sub_image = rospy.Subscriber(VISION_IMAGE_TOPIC, Image, image_callback)

    # Init a publisher
    # publisher = rospy.Publisher('board2cam', numpy_msg(Floats), queue_size=10)
    
    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    # while True:
    #     my_msg = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    #     # publisher.publish(my_msg)

    #     rospy.loginfo("Published")
    #     # rospy.spin()

    #     pub = rospy.Publisher('my_topic', Float64MultiArray, queue_size=10)
    #     # my_msg = Float64MultiArray()  
    #     # my_msg.data = [[1.2354567, 99.7890, 67.654236], [67.875, 90.6543, 76.5689], [65.3452, 45.873, 67.8956]]
    #     # pub.publish(my_msg)

    #     my_msg = Float64MultiArray( data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #     #d=[[1.2354567, 99.7890, 67.654236], [67.875, 90.6543, 76.5689], [65.3452, 45.873, 67.8956]]
    #     #d=[[float(d[i][j]) for j in range(len(d))] for i in range(len(d[0]))]

    #     pub.publish(my_msg)

    my_msg = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

    print(my_msg)
    print(my_msg.shape)

    flatted_msg =  my_msg.reshape(-1)
    print(flatted_msg)
    print()
    print(flatted_msg.reshape(4, 4))