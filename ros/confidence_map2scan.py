#!/usr/bin/env python3

# add absolute path to lib
import sys
sys.path.insert(0, '/home/kbkn/miniconda3/envs/data-science/lib/python3.8/site-packages')

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from .scan import Scan
from node_init import NodeInit

init_params = {
    'node_name': 'confidence_map2scan',
    'sub_name': 'confidence_map',
    'sub_type': Image,
    'pub_name': 'scan/line_detection',
    'pub_type': Scan,
}


class ConfidenceMap2Scan(NodeInit):
    def __init__(self, init_params):
        self.bridge = CvBridge()
        super().__init__(init_params)

    def f(self):
        return None

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'mono8')
        except CvBridgeError as e:
            print(e)

        cv2.imshow('test', image)
        cv2.waitKey(1)

        # write code here.
        scan = self.f()

        self.pub.publish(scan)


if __name__ == '__main__':
    ld = ConfidenceMap2Scan(init_params)