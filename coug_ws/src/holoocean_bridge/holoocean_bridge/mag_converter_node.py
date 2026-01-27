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
from sensor_msgs.msg import MagneticField


class MagConverterNode(Node):
    """
    Converts Magnetic data from HoloOcean to a standard MagneticField message.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("mag_converter_node")

        self.declare_parameter("input_topic", "auv0/MagnetometerSensor")
        self.declare_parameter("output_topic", "imu/mag")
        self.declare_parameter("mag_frame", "imu_link")
        self.declare_parameter("override_covariance", True)
        self.declare_parameter("noise_sigma", 0.003)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.mag_frame = (
            self.get_parameter("mag_frame").get_parameter_value().string_value
        )
        self.override_covariance = (
            self.get_parameter("override_covariance").get_parameter_value().bool_value
        )
        self.noise_sigma = (
            self.get_parameter("noise_sigma").get_parameter_value().double_value
        )

        self.subscription = self.create_subscription(
            MagneticField, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(MagneticField, output_topic, 10)

        self.get_logger().info(
            f"Mag converter started. Listening on {input_topic} and publishing on {output_topic}."
        )

    def listener_callback(self, msg: MagneticField):
        """
        Process Magnetic sensor data (MagneticField).

        :param msg: MagneticField message.
        """
        msg.header.frame_id = self.mag_frame
        if self.override_covariance:
            variance = self.noise_sigma * self.noise_sigma

            msg.magnetic_field_covariance[0] = variance
            msg.magnetic_field_covariance[4] = variance
            msg.magnetic_field_covariance[8] = variance

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    mag_converter_node = MagConverterNode()
    try:
        rclpy.spin(mag_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        mag_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
