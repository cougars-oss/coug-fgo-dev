# Copyright (c) 2026 BYU FRoSt Lab
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
from rclpy.node import Node
from nav_msgs.msg import Odometry


class DepthConverterNode(Node):
    """
    Converts depth data from HoloOcean to an odometry message.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("depth_converter_node")

        self.declare_parameter("input_topic", "auv0/DepthSensor")
        self.declare_parameter("output_topic", "odometry/depth")
        self.declare_parameter("depth_frame", "depth_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("override_covariance", True)
        self.declare_parameter("noise_sigma", 0.02)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.depth_frame = (
            self.get_parameter("depth_frame").get_parameter_value().string_value
        )
        self.map_frame = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )
        self.override_covariance = (
            self.get_parameter("override_covariance").get_parameter_value().bool_value
        )
        self.noise_sigma = (
            self.get_parameter("noise_sigma").get_parameter_value().double_value
        )

        self.subscription = self.create_subscription(
            Odometry, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(Odometry, output_topic, 10)

        self.get_logger().info(
            f"Depth converter started. Listening on {input_topic} and "
            f"publishing on {output_topic}."
        )

    def listener_callback(self, msg: Odometry):
        """
        Process depth sensor data (Odometry).

        :param msg: Odometry message containing depth data.
        """
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.depth_frame

        if self.override_covariance:
            msg.pose.covariance[14] = self.noise_sigma * self.noise_sigma

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    depth_converter_node = DepthConverterNode()
    try:
        rclpy.spin(depth_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        depth_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
