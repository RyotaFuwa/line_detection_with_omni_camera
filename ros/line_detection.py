#!/usr/bin/env python3

# add absolute path to lib
import sys
sys.path.insert(0, '/home/kbkn/miniconda3/envs/data-science/lib/python3.8/site-packages')

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from node_init import NodeInit

init_params = {
    'node_name': 'line_detection',
    'sub_name': 'cv_camera/image_raw',
    'sub_type': Image,
    'pub_name': 'confidence_map',
    'pub_type': Image,
}


class LineDetectionModel(NodeInit):
    def __init__(self, init_params):
        self.bridge = CvBridge()
        # self.model = import_model()
        super().__init__(init_params)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'rgb8')
        except CvBridgeError as e:
            print(e)

        # cv2.imshow('test', image)
        # cv2.waitKey(1)

        # image = crop_and_resize(image)
        # image = self.model.predict(image)
        # image = (image * 255).astype('uint8')

        try:
            image = self.bridge.cv2_to_imgmsg(image, 'mono8')
        except CvBridgeError as e:
            print(e)
        self.pub.publish(image)

        def import_model(self, model_file=''):
            return None

        # empirical crop
        def crop_and_resize(self, image):
            image = image[14: 636, 5: 637, :]
            image = cv2.resize(image, (128, 128))
            return image


if __name__ == '__main__':
    ld = LineDetectionModel(init_params)