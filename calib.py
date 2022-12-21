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
from math import sqrt, pow
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, Point, PoseArray


class Server(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.position = Point()
        self.particles = PoseArray()
        self.tolerance = 0.2
        self.msg = Twist()
        self.error = 0.3
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        qos_sensor = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                depth=1)

        self.particlecloud_subscription = self.create_subscription(PoseArray, 'particlecloud',
                                                                     self.particlecloud_callback, qos_sensor)
        self.particle_cloud_subscription = self.create_subscription(ParticleCloud, 'particle_cloud',
                                                                    self.particle_cloud_callback, qos_sensor)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        # print("odom message")
        self.position = msg.pose.pose.position
        if self.error < self.tolerance:
            self.publish(True)
            quit()
        else:
            self.publish(False)

    def particle_cloud_callback(self, msg):
        particles = msg.particles
        current_position = self.position
        error = 0.0
        for particle in particles:
            error = error + particle.weight * self.dist_calc(particle.pose.position, current_position)
        self.error = error
        print("W/ Weight:", self.error)

    def particlecloud_callback(self, msg):
        particles = msg.poses
        current_position = self.position
        error = 0.0
        for particle in particles:
            error = error + self.dist_calc(particle.position, current_position)

        self.error = error / len(particles)
        print("W/O Weight:", self.error)

    def dist_calc(self, particle, pose):
        return sqrt(pow(particle.x - pose.x, 2) + pow(particle.y - pose.y, 2))

    def publish(self, stop):
        try:
            msg = Twist()

            if stop:
                msg.angular.z = 0.0

            else:
                diff = self.error - self.tolerance
                if diff < 0.1:
                    msg.angular.z = 0.1

                elif diff > 0.6:
                    msg.angular.z = 0.6

                else:
                    msg.angular.z = diff

            self.publisher_.publish(msg)
        except KeyError:
            pass

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