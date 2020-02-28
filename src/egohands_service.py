#!/usr/bin/env python3

# -- IMPORT --
import numpy as np
import cv2
import torch
# Ros
import rospy
import cv_bridge
import ros_numpy
from sensor_msgs.msg import Image, CompressedImage
# Ros bodyparts
from egohands_ros.srv import SemSegHandSrv, SemSegHandSrvResponse
from helper_CSAILVision.lib.segmentation import hand_segmentation, module_init


class Egohands:
    def __init__(self):
        # Parameter
        self.camera_topic = rospy.get_param('/egohands/camera/topic')
        self.interface_topic = rospy.get_param('/egohands/interface/service')
        self.visualization_topic = rospy.get_param('/egohands/visualization/topic')
        self.visualization_activated = rospy.get_param('/egohands/visualization/activated')

        # Init
        self.bridge = cv_bridge.CvBridge()
        self.segmentation_module = module_init()
        torch.cuda.set_device(0)

        # Service
        rospy.Service(self.interface_topic, SemSegHandSrv, self._callback)

        # -- Visualization
        if self.visualization_activated:
            self.pub_visualization = rospy.Publisher(self.visualization_topic, Image, queue_size=1)

        # Feedback
        print("Hand segmentation service up and running")

    # Callback function
    def _callback(self, request):

        t_start = rospy.get_time()
        
        # Get image
        image = cv2.cvtColor(self.bridge.compressed_imgmsg_to_cv2(request.image), cv2.COLOR_BGR2RGB)

        # Calculate Mask                 
        mask = hand_segmentation(image, self.segmentation_module)

        # Visualize results
        if self.visualization_activated:
            image[:,:,0][mask == 0] = 0
            image[:,:,1][mask == 0] = 0
            image[:,:,2][mask == 0] = 0
            self.pub_visualization.publish(ros_numpy.msgify(Image, image, encoding='8UC3'))

        # Publish results
        print('Body detection successful. Current Hz-rate:\t' + str(1/(rospy.get_time() - t_start)))
        return SemSegHandSrvResponse(mask=self.bridge.cv2_to_compressed_imgmsg(mask, dst_format='png'))


if __name__ == '__main__':

    rospy.init_node('egohands_service')
    body = Egohands()
    rospy.spin()