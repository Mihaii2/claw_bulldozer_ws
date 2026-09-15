#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class LatencyTester(Node):
    def __init__(self):
        super().__init__('latency_tester')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.pub = self.create_publisher(Imu, '/imu', 10)
        self.last_sent_time = 0.0
        self.latencies = []
        self.timer = self.create_timer(0.1, self.send_ping)

    def send_ping(self):
        msg = Imu()
        # Mock rear-stance vertical tilt to trigger ESP32 PID
        msg.linear_acceleration.x = -9.8
        msg.linear_acceleration.z = 0.0
        msg.angular_velocity.y = 0.1
        self.last_sent_time = time.time()
        self.pub.publish(msg)

    def cmd_cb(self, msg: Twist):
        if self.last_sent_time > 0:
            rtt_ms = (time.time() - self.last_sent_time) * 1000.0
            self.latencies.append(rtt_ms)
            if len(self.latencies) > 20:
                self.latencies.pop(0)
            avg = sum(self.latencies) / len(self.latencies)
            print(f"\rRound-Trip Latency: {rtt_ms:5.1f} ms | Avg (last 20): {avg:5.1f} ms", end="")

def main():
    rclpy.init()
    node = LatencyTester()
    rclpy.spin(node)

if __name__ == '__main__':
    main()