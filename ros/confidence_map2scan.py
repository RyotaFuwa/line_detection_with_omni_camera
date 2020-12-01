import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from .scan import Scan

from lib.vector import Vector2D
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

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'mono8')
        except CvBridgeError as e:
            print(e)

        # write code here.
        scan = self.f()

        self.pub.publish(scan)

    def transform_to_scan(image, thresh_length=3, height=1.8):
        scan = [float('inf') for _ in range(720)] # 0 - 720, -180 - 180
        center = (63, 63)
        for row in range(len(image)):
            for col in range(len(image[0])):
                if row == center[0] and col == center[0]:
                    continue
                if not image[row][col]:
                    continue
                length, fai = calc(row, col, center, image)
                fai = fai
                scan[fai] = min(scan[fai], length)
        for fai in range(len(scan)):
            if(scan[fai] > thresh_length):
                scan[fai] = 0
        return scan


if __name__ == '__main__':
    ld = ConfidenceMap2Scan(init_params)
