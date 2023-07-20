### GRIPPER Params
CLOSED_POS = 0.0   # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).
MIN_EFFORT = 35   # Min grasp force, in Newtons
MAX_EFFORT = 100  # Max grasp force, in Newtons
GRIPPER_CONTROL_GROUP = 'gripper_controller/gripper_action'

### MANIPULATOR Params
# ARM_AND_TORSO
ARM_REST_POSITION = [0.05, -0.6074564456939697, 1.2425246238708496, 0.8049564361572266,
                     1.6225686073303223, -2.9548306465148926, -1.582684874534607, -1.0243149995803833]
ARM_READY_POSITION = [1.0, -0.6074564456939697, 1.2425246238708496, 0.8049564361572266,
                      1.6225686073303223, -2.9548306465148926, -1.582684874534607, -1.0243149995803833]
MAX_JOINT_VEL = [0.1, 1.25, 1.45, 1.57, 1.52, 1.57, 2.26, 2.26]
ARM_AND_TORSO_JOINTS = ['torso_lift_joint',
                        'shoulder_pan_joint',
                        'shoulder_lift_joint',
                        'upperarm_roll_joint',
                        'elbow_flex_joint',
                        'forearm_roll_joint',
                        'wrist_flex_joint',
                        'wrist_roll_joint']
# HEAD
HEAD_JOINTS = ['head_pan_joint',
               'head_tilt_joint']

ARM_TORSO_CONTROL_GROUP = 'arm_with_torso_controller/follow_joint_trajectory'
HEAD_CONTROL_GROUP = 'head_controller/follow_joint_trajectory'

### MOVE Params
# /cmd_vel allows controller commandd to take over the robot
MOVE_NODE = '/cmd_vel' # /cmd_vel OR /teleop/cmd_vel OR /base_controller/command 
CONTROL_RATE = 10
MAX_LINEAR_VELOCITY = 4.8

### VISION PARAM
VISION_IMAGE_TOPIC = "/head_camera/rgb/image_raw" # "/head_camera/rgb/image_rect_color"
VISION_CAMERA_INFO_TOPIC = "/head_camera/rgb/camera_info"
