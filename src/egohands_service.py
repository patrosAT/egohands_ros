#!/usr/bin/env python3

# -- IMPORT --
import numpy as np
import cv2
import torch
# Ros
import rospy
import cv_bridge
# Ros bodyparts
from egohands_ros.srv import SemSegHandSrv, SemSegHandSrvResponse
from helper_CSAILVision.lib.segmentation import hand_segmentation, module_init


class Egohands:
    def __init__(self):
        # Parameter
        self.cam_rgb = rospy.get_param('/egohands/camera/rgb')
        self.interface = rospy.get_param('/egohands/interface/service')

        # Init
        self.bridge = cv_bridge.CvBridge()
        self.segmentation_module = module_init()
        torch.cuda.set_device(0)

        # Service
        self.pub_mask = rospy.Service(self.interface, SemSegHandSrv, self._callback)

        # Feedback
        print("Hand segmentation service up and running")

    # Callback function
    def _callback(self, request):

        t_start = rospy.get_time()

        image = cv2.cvtColor(self.bridge.compressed_imgmsg_to_cv2(request.image), cv2.COLOR_BGR2RGB)
        
        mask = hand_segmentation(image, self.segmentation_module)
        
        print('Hand detection successful. Current Hz-rate:\t' + str(1/(rospy.get_time() - t_start)))
        return SemSegHandSrvResponse(mask=self.bridge.cv2_to_compressed_imgmsg(mask))


if __name__ == '__main__':

    rospy.init_node('egohands_service')
    body = Egohands()
    rospy.spin()