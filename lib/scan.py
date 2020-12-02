import math
import rospy
from sensor_msgs.msg import LaserScan


class Scan(LaserScan):
    def __init__(self):
        super().__init__()
        self.header.stamp = rospy.Time.now()
        self.header.frame_id = 'map'
        self.angle_min = - math.pi
        self.angle_max = math.pi
        self.angle_increment = -math.pi / 360
        self.time_increment = 1.0 / 40 / 720
        self.range_min = 0.3
        self.range_max = 30.0
        self.ranges = [float('inf') for _ in range(720)]