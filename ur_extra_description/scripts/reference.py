#! /usr/bin/env python3

import rospy
import time

from std_msgs.msg import Bool
from ur_msgs.srv import SetIO

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

# Function to open the gripper
def open_gripper(self):
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    # open gripper
    spawn_srv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    spawn_srv(fun=1, pin=16, state=0.0)
    rospy.sleep(0.5)


# Function to close the gripper
def close_gripper(self):
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    # close gripper
    spawn_srv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    spawn_srv(fun=1, pin=16, state=1.0)
    rospy.sleep(0.5)


# Function to set joints of the UR5 arm using the topic /set_joint_value_target_wait_topic
def set_joint():
    pub_joint_pose_wait_ack = rospy.Publisher("/set_joint_value_target_wait_topic", Float32MultiArray, queue_size=10)

    array_temp = Float32MultiArray()
    array_temp.data = [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
    pub_joint_pose_wait_ack.publish(array_temp)


# Function to set the world coordinate of the end effector using the topic /set_ee_pose_target_wait_topic
def set_pose():
    pub_ee_pose_wait_ack = rospy.Publisher("/set_ee_pose_target_wait_topic", Pose, queue_size=10)

    ee_pose = Pose()
    ee_pose.position.x = x-coordinate
    ee_pose.position.y = y-coordinate
    ee_pose.position.z = z-coordinate
    ee_pose.orientation.x = x-quaternion
    ee_pose.orientation.y = y-quaternion
    ee_pose.orientation.z = z-quaternion
    ee_pose.orientation.w = w-quaternion

    pub_ee_pose_wait_ack.publish(ee_pose)


# Callback function for the topic /set_ee_pose_target_wait_ack_topic
def joint_state_topic_wait_cb(msg):
    '''
    msg = 1: If arm succesfully plans the path
    msg = 0: If arm fails to plan the path
    Add your code here
    '''
sub_joint_pose = rospy.Subscriber("/set_ee_pose_target_wait_ack_topic", Bool, joint_state_topic_wait_cb, queue_size=10)


# Perception callback function

# RGB Imagee callback function
sub_color_image = rospy.Subscriber(
    "/camera/aligned_depth_to_color/image_raw", Image, image_dp, queue_size=100)
def image_cb(data):
    np_arr = np.frombuffer(data.data, np.uint8)
    rgb_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    '''
    Use this rgb_frame for further computation
    '''

# Depth Image callback function
sub_depth_image = rospy.Subscriber("/camera/color/image_raw/compressed",
                        CompressedImage, image_cb, queue_size=100)
def image_dp(data):
    depth_frame = bridge.imgmsg_to_cv2(data, "passthrough")
    '''
    Use this depth_frame for futher computation
    '''
