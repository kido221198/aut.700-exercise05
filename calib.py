# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import numpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, Point, PoseArray


class Server(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.position = Point()
        self.particles = PoseArray()
        self.tolerance = 1.0
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.particle_cloud_subscription = self.create_subscription(PoseArray, 'particlecloud', self.particle_cloud_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        # print("odom message")
        self.position = msg.pose.pose.position

    def particle_cloud_callback(self, msg):
        print("particle message")
        self.particles = msg
        # print(self.particles)

    def publish(self, stop):
        try:
            msg = Twist()

            if stop:
                msg.angular.z = 0

            else:
                msg.angular.z = 0.5
            self.get_logger().info('Publishing...')
            self.publisher_.publish(msg)
        except KeyError:
            pass


def ros_spin(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.001)


def main(args=None):
    rclpy.init(args=args)

    server = Server()
    # ros_spin(server)
    rclpy.spin(server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()