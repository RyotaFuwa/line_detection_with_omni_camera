import math
from sensor_msgs.msg import LaserScan


class Scan(LaserScan):
    def __init__(self):
        super().__init__()
        self.header.stamp = 0.0
        self.header.frame_id = 'map'
        self.angle_min = - math.pi
        self.angle_max = math.pi
        self.angle_increment = -math.pi * 2 / 720
        self.time_increment = 1.0 / 40 / 720
        self.scan_time = 0.1
        self.range_min = 0.3
        self.range_max = 30.0
        self.ranges = [0.0 for _ in range(720)]