import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from lib.node_init import NodeInit
from tensorflow.keras.models import load_model
import cv2

init_params = {
    'node_name': 'line_detection',
    'sub_name': 'cv_camera/image_raw',
    'sub_type': Image,
    'pub_name': 'confidence_map',
    'pub_type': Image,

    # other params
    'model_filepath': "data/models/v1.h5",
    'debug': True,
}


class LineDetectionModel(NodeInit):
    def __init__(self, init_params):
        self.bridge = CvBridge()
        self.model = load_model(init_params['model_filepath'])
        super().__init__(init_params)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'rgb8')
        except CvBridgeError as e:
            print(e)

        if self.init_params['debug']:
            cv_image = image[:, :, [2, 1, 0]]
            cv2.imshow('original', cv_image)

        image = np.expand_dims(self.crop_and_resize(image), 0) / 255.0
        image = self.model.predict(image)[0]
        image = (image * 255).astype('uint8')

        if self.init_params['debug']:
            cv2.imshow('confidence_map', image)
            cv2.waitKey(1)

        try:
            image = self.bridge.cv2_to_imgmsg(image, 'mono8')
        except CvBridgeError as e:
            print(e)
        self.pub.publish(image)

    # empirical crop
    def crop_and_resize(self, image):
        image = image[14: 636, 5: 637, :]
        image = cv2.resize(image, (128, 128))
        return image


if __name__ == '__main__':
    ld = LineDetectionModel(init_params)
    ld.run()