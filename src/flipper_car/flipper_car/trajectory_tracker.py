#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Path, '/robot_trajectory', 10)
        
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.max_poses = 1000  # Buffer length

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_poses:
            self.path.poses.pop(0)

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()