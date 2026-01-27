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
from geometry_msgs.msg import TwistWithCovarianceStamped


class DvlConverterNode(Node):
    """
    Converts DVL data from HoloOcean to a twist with covariance message.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("dvl_converter_node")

        self.declare_parameter("input_topic", "auv0/DVLSensorVelocity")
        self.declare_parameter("output_topic", "dvl/twist")
        self.declare_parameter("dvl_frame", "dvl_link")
        self.declare_parameter("override_covariance", True)
        self.declare_parameter("noise_sigma", 0.02)
        self.declare_parameter("simulate_dropout", False)
        self.declare_parameter("dropout_frequency", 1.0 / 30.0)
        self.declare_parameter("dropout_duration", 5.0)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.dvl_frame = (
            self.get_parameter("dvl_frame").get_parameter_value().string_value
        )
        self.override_covariance = (
            self.get_parameter("override_covariance").get_parameter_value().bool_value
        )
        self.noise_sigma = (
            self.get_parameter("noise_sigma").get_parameter_value().double_value
        )
        self.simulate_dropout = (
            self.get_parameter("simulate_dropout").get_parameter_value().bool_value
        )
        self.dropout_frequency = (
            self.get_parameter("dropout_frequency").get_parameter_value().double_value
        )
        self.dropout_duration = (
            self.get_parameter("dropout_duration").get_parameter_value().double_value
        )

        self.subscription = self.create_subscription(
            TwistWithCovarianceStamped, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped, output_topic, 10
        )

        self.get_logger().info(
            f"DVL converter started. Listening on {input_topic} and publishing on {output_topic}."
        )

    def listener_callback(self, msg: TwistWithCovarianceStamped):
        """
        Process DVL sensor data (TwistWithCovarianceStamped).

        :param msg: TwistWithCovarianceStamped message containing DVL data.
        """
        if self.simulate_dropout and self.dropout_frequency > 0:
            current_time = self.get_clock().now().nanoseconds / 1e9

            cycle_period = 1.0 / self.dropout_frequency
            if (current_time % cycle_period) < self.dropout_duration:
                self.get_logger().warn(
                    "Simulating DVL dropout...",
                    throttle_duration_sec=cycle_period,
                )
                return

        msg.header.frame_id = self.dvl_frame
        if self.override_covariance:
            covariance = self.noise_sigma * self.noise_sigma
            msg.twist.covariance[0] = covariance  # Vx
            msg.twist.covariance[7] = covariance  # Vy
            msg.twist.covariance[14] = covariance  # Vz

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    dvl_converter_node = DvlConverterNode()
    try:
        rclpy.spin(dvl_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
