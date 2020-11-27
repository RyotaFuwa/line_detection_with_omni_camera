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

    def __init__(self, init_param):
        """initialize a ros node
        Args:
            init_param: {
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

        try:
            rospy.init_node(init_param['node_name'])
            try:
                self.sub = rospy.Subscriber(init_param['sub_name'], init_param['sub_type'], self.callback)
            except:
                self.sub = Dummy('Not Defined')

            try:
                self.pub = rospy.Publisher(init_param['pub_name'], init_param['pub_type'], queue_size=5)
            except:
                self.pub = Dummy('Not Defined')
        except:
            raise BaseException("Node Initialization Failed.")
        finally:
            print("=========")
            print("node : {}".format(init_param['node_name']))
            print("<-----")
            print("subscriber : {} ".format(self.sub.name))
            print("----->")
            print("publisher  : {} ".format(self.pub.name))
            print("----->")
            print("=========")
            if type(self.sub) != Dummy:
                try:
                    rospy.spin()
                except KeyboardInterrupt:
                    rospy.signal_shutdown("Keyboard Interrupt Occured")

    @abstractmethod
    def callback(self, data):
        """override this function when you inherit this class
        """

        # process here
        """Here, write the process
        """
        pass

        # publish
        pass