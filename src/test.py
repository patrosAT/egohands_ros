#!/usr/bin/env python

# -- IMPORT --
import numpy as np
import cv2
# Ros std packages
import rospy
import actionlib
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
# Ros additional packages
from egohands_ros.srv import SemSegHandSrv
from egohands_ros.msg import SemSegHandActAction, SemSegHandActGoal


# -- PARAMETER --
TEST_PUBLISHER = True
TEST_SERVICE = False
TEST_ACTION = False

PATH_IMAGE = rospy.get_param('/egohands/camera/rgb')
INTERFACE_PUB = rospy.get_param('/egohands/interface/topic')
INTERFACE_SRV = rospy.get_param('/egohands/interface/service')
INTERFACE_ACT = rospy.get_param('/egohands/interface/action')


# -- INITIALIZATION --
bridge = CvBridge()


# -- FUNCTIONS --
def display(mask):

    mask_disp = np.zeros((mask.shape[0],mask.shape[1],3))
    mask_disp[:,:,0][mask == 1] = 255
    mask_disp[:,:,1][mask == 2] = 255
    mask_disp[:,:,2][mask == 3] = 255
    mask_disp[:,:,0][mask == 4] = 255
    mask_disp[:,:,1][mask == 4] = 255
    mask_disp[:,:,0][mask == 5] = 255
    mask_disp[:,:,2][mask == 5] = 255
    mask_disp[:,:,1][mask == 6] = 255
    mask_disp[:,:,2][mask == 6] = 255
    cv2.imshow('res', mask_disp)
    cv2.waitKey(1)

# -- CALLBACKS --
def callback_sub(msg):
    mask = bridge.compressed_imgmsg_to_cv2(msg)
    display(mask)

def callback_srv(msg):
    mask_c = servicecall(image=msg)
    mask = bridge.compressed_imgmsg_to_cv2(mask_c.mask)
    display(mask)

def callback_act(msg):
    goal = SemSegHandActGoal(image=msg)
    action.send_goal(goal)
    action.wait_for_result()
    res = action.get_result()
    mask = bridge.compressed_imgmsg_to_cv2(res.mask)
    display(mask)


# -- MAIN --
if __name__ == '__main__':

    rospy.init_node('test')

    if TEST_PUBLISHER:
        sub = rospy.Subscriber(INTERFACE_PUB, CompressedImage, callback_sub, queue_size=2, buff_size=2**29)

    if TEST_SERVICE:
        servicecall = rospy.ServiceProxy(INTERFACE_SRV, SemSegHandSrv)
        rospy.Subscriber(PATH_IMAGE, CompressedImage, callback_srv, queue_size=2)

    if TEST_ACTION:
        action = actionlib.SimpleActionClient(INTERFACE_ACT, SemSegHandActAction)
        action.wait_for_server()
        rospy.Subscriber(PATH_IMAGE, CompressedImage, callback_act, queue_size=2)
    
    rospy.spin()
