import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from lib.node_init import NodeInit
from lib.scan import Scan
from lib.vector import Vector2D

# constants
START_LINE = Vector2D(-1, 0)
RAD_TO_ANGLE = 180 / np.pi
fai_dash = np.pi
theta_ratio = 0.5 * np.pi / 64.0

# environment variables
HEIGHT = 1.8  # height of the camera from the ground.
CENTER = Vector2D(65, 68)  # row, col (determined empirically)
PIXEL_LENGTH = (Vector2D(10, 10) - CENTER).length()

init_params = {
    'node_name': 'confidence_map2scan',
    'sub_name': 'confidence_map',
    'sub_type': Image,
    'pub_name': 'scan/line_detection',
    'pub_type': Scan,

    # kind of like hyper parameters
    'threshold_for_p': 0.4,
    'max_distance': 7.0,

    'debug': True,
}


class ConfidenceMap2Scan(NodeInit):
    def __init__(self, init_params):
        self.bridge = CvBridge()
        super().__init__(init_params)

    def callback(self, data):
        try:
            confidence_map = self.bridge.imgmsg_to_cv2(data, 'mono8')
        except CvBridgeError as e:
            print(e)

        scan = Scan()

        # get coordinates on which line is detected, and then calculate scan value for each coordinate.
        segment = confidence_map > self.init_params['threshold_for_p'] * 255
        xs, ys = np.where(segment)
        for x, y in zip(xs, ys):
            vec = Vector2D(x, y)
            r, fai = self.r_fai(vec)
            idx = self.get_idx_from_fai(fai)
            if 0 <= idx < len(scan.ranges):
                scan.ranges[idx] = min(scan.ranges[idx], r)

        # set distance at the angles where a line wasn't detected to 0
        for idx, distance in enumerate(scan.ranges):
            if distance > self.init_params['max_distance']:
                scan.ranges[idx] = 0

        self.pub.publish(scan)

    @staticmethod
    def r_fai(vec):
        diff = vec - CENTER
        fai = diff.arg(START_LINE)

        theta = diff.length() * (np.pi / 2) / PIXEL_LENGTH  # make sure 0 <= theta < np.pi / 2
        if theta >= np.pi / 2:
            return float('inf'), fai
        return HEIGHT * np.tan(theta), fai

    @staticmethod
    def get_idx_from_fai(fai):
        fai = fai * RAD_TO_ANGLE
        if fai < -180 or fai >= 180:
            return -1
        decimal, i = np.modf(fai)
        idx = int(i * 2 + (1 if decimal >= 0.5 else 0))
        return idx + 360


if __name__ == '__main__':
    cms = ConfidenceMap2Scan(init_params)
    cms.run()
