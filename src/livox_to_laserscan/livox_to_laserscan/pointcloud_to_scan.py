from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
import numpy as np
import rclpy
from rclpy.node import Node
import math


class CloudToScan(Node):
    def __init__(self):
        super().__init__('cloud_to_scan')
        self.sub = self.create_subscription(PointCloud2, '/livox/points', self.cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(0.25)
        self.range_min = 0.1
        self.range_max = 40.0
        self.z_min = 0.02   # m (相对 livox_frame)
        self.z_max = 1.00   # m

        self.n = int((self.angle_max - self.angle_min) / self.angle_increment)

    def cb(self, msg):
        # numpy 向量化处理, 避免逐点 Python 循环 (MID360 ~2万点/帧)
        pts = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)

        z = pts[:, 2]
        mask = (z >= self.z_min) & (z <= self.z_max)
        x = pts[mask, 0]
        y = pts[mask, 1]
        if x.size == 0:
            return

        r = np.hypot(x, y)
        mask = (r > self.range_min) & (r < self.range_max)
        if not np.any(mask):
            return
        r = r[mask]
        angle = np.arctan2(y[mask], x[mask])
        idx = ((angle - self.angle_min) / self.angle_increment).astype(np.int64)
        mask = (idx >= 0) & (idx < self.n)
        idx = idx[mask]
        r = r[mask]

        ranges = np.full(self.n, np.inf)
        # 同一扇区取最近点
        np.minimum.at(ranges, idx, r)

        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = 'livox_frame'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = CloudToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
