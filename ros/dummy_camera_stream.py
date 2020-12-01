import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from node_init import NodeInit
import pickle

init_params = {
    'node_name': 'dummy_camera.py',
    'pub_name': 'cv_camera/image_raw',
    'pub_type': Image,

    # other params
    'ds_path': 'data/datasets/train_ds'
}


class DummyCameraStream(NodeInit):
    def __init__(self, init_params):
        super().__init__(init_params)
        self.bridge = CvBridge()
        self.image_list = self.load_ds_as_list(init_params['ds_path'])
        self.rate = rospy.Rate(15)

    def load_ds_as_list(self, ds_path):
        with open('data/misc/image_list', 'rb') as fin:
            return pickle.load(fin)
        # element_spec = {
        #     "image": tf.TensorSpec(shape=(None, 128, 128, 3), dtype=tf.float32, name=None),
        #     "label": tf.TensorSpec(shape=(None, 128, 128, 1), dtype=tf.float32, name=None),
        # }
        # return list(tf.data.experimental.load(ds_path, element_spec).as_numpy_iterator())

    def publish(self):
        i = 0
        while not rospy.is_shutdown():
            if i == len(self.image_list):
                i = 0
            image = self.image_list[i]
            image = self.bridge.cv2_to_imgmsg(image, 'rgb8')
            self.pub.publish(image)
            self.rate.sleep()
            i += 1


if __name__ == '__main__':
    dcs = DummyCameraStream(init_params).run()
