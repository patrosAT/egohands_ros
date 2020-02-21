#!/usr/bin/env python3

# -- IMPORT --
import numpy as np
import cv2
import torch
# Ros
import rospy
import cv_bridge
import actionlib
# Ros egohands
from egohands_ros.msg import SemSegHandActAction, SemSegHandActResult
from helper_CSAILVision.lib.segmentation import hand_segmentation, module_init


class Egohands:
    def __init__(self):
        # Parameter
        self.cam_rgb = rospy.get_param('/egohands/camera/rgb')
        self.interface = rospy.get_param('/egohands/interface/action')
        self.cpu = rospy.get_param('/egohands/gpu')

        # Init
        self.bridge = cv_bridge.CvBridge()
        self.segmentation_module = module_init()
        torch.cuda.set_device(self.cpu)

        # Action server
        self.server = actionlib.SimpleActionServer(self.interface, SemSegHandActAction, self._callback, False)
        self.server.start()

        # Feedback
        print("Hand segmentation action server up and running")

    # Callback function
    def _callback(self, goal):

        t_start = rospy.get_time()

        image = cv2.cvtColor(self.bridge.compressed_imgmsg_to_cv2(goal.image), cv2.COLOR_BGR2RGB)
        
        mask = hand_segmentation(image, self.segmentation_module)
        
        print('Hand detection successful. Current Hz-rate:\t' + str(1/(rospy.get_time() - t_start)))
        self.server.set_succeeded(SemSegHandActResult(mask=self.bridge.cv2_to_compressed_imgmsg(mask)))


if __name__ == '__main__':

    rospy.init_node('egohands_action_server')
    body = Egohands()    
    rospy.spin()