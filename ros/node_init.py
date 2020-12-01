import sys
import numpy as np
import rospy
from std_msgs.msg import String
from abc import ABC, abstractmethod

# global variable


class Dummy:
    def __init__(self, name):
        self.name = name


class NodeInit(ABC):
    """Node initialization class
       In __init__ of child class, call __init__ of this parent class (i.e. NodeInit) at the end
       because this constructor calls rospy.spin(), in turn, get into callback loop.
    """

    def __init__(self, init_params):
        """initialize a ros node
        Args:
            init_params: {
              node_name: node name
              sub_name: name of topic you'd like to subscribe to
              sub_type: type of subscribe topic
              pub_name: name of topic you'd like to publish data to
              pub_type: type of publish topic
            }

        Returns:
            None

        Attributes:
            self.pub: ros publisher (you can use this in callback)

        """
        self.init_params = init_params
        try:
            rospy.init_node(init_params['node_name'])
            try:
                self.sub = rospy.Subscriber(init_params['sub_name'], init_params['sub_type'], self.callback)
            except:
                self.sub = None

            try:
                self.pub = rospy.Publisher(init_params['pub_name'], init_params['pub_type'], queue_size=5)
            except:
                self.pub = None
        except:
            raise BaseException("Node Initialization Failed.")

        print('Node Created Successfully!')

    def run(self):
        print("=========")
        print("node : {}".format(self.init_params['node_name']))
        print("<-----")
        if self.sub is not None:
            print("subscriber : {} ".format(self.sub.name))
        else:
            print("subscriber : {} ".format('---'))
        print("----->")
        if self.pub is not None:
            print("publisher  : {} ".format(self.pub.name))
        else:
            print("publisher  : {} ".format('---'))
        print("=========")
        if self.sub is not None:
            try:
                rospy.spin()
            except KeyboardInterrupt:
                rospy.signal_shutdown("Keyboard Interrupt Occured")
        if self.pub is not None:
            self.publish()

    def callback(self, data):
        """override this function when you inherit this class
        """

        # process here
        """Here, write the process
        """
        pass

        # publish
        pass

    def publish(self, *args, **kwargs):
        """override this function when you want to publish sth without subscriber"""
        pass