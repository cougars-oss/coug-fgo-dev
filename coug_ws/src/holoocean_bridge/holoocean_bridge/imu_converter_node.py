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
from sensor_msgs.msg import Imu


class ImuConverterNode(Node):
    """
    Converts IMU data from HoloOcean to a standard IMU message.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("imu_converter_node")

        self.declare_parameter("input_topic", "auv0/IMUSensor")
        self.declare_parameter("output_topic", "imu/data")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("override_covariance", True)
        self.declare_parameter("accel_noise_sigma", 0.0078)
        self.declare_parameter("gyro_noise_sigma", 0.0012)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.imu_frame = (
            self.get_parameter("imu_frame").get_parameter_value().string_value
        )
        self.override_covariance = (
            self.get_parameter("override_covariance").get_parameter_value().bool_value
        )
        self.accel_noise_sigma = (
            self.get_parameter("accel_noise_sigma").get_parameter_value().double_value
        )
        self.gyro_noise_sigma = (
            self.get_parameter("gyro_noise_sigma").get_parameter_value().double_value
        )

        self.subscription = self.create_subscription(
            Imu, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(Imu, output_topic, 10)

        self.get_logger().info(
            f"IMU converter started. Listening on {input_topic} and publishing on {output_topic}."
        )

    def listener_callback(self, msg: Imu):
        """
        Process IMU sensor data (Imu).

        :param msg: Imu message containing IMU data.
        """
        msg.header.frame_id = self.imu_frame
        if self.override_covariance:
            accel_covariance = self.accel_noise_sigma * self.accel_noise_sigma
            gyro_covariance = self.gyro_noise_sigma * self.gyro_noise_sigma

            msg.linear_acceleration_covariance[0] = accel_covariance
            msg.linear_acceleration_covariance[4] = accel_covariance
            msg.linear_acceleration_covariance[8] = accel_covariance

            msg.angular_velocity_covariance[0] = gyro_covariance
            msg.angular_velocity_covariance[4] = gyro_covariance
            msg.angular_velocity_covariance[8] = gyro_covariance

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    imu_converter_node = ImuConverterNode()
    try:
        rclpy.spin(imu_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
