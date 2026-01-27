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

import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3Stamped


class AhrsConverterNode(Node):
    """
    Converts rotation sensor data from HoloOcean to a standard IMU message.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("ahrs_converter_node")

        self.declare_parameter("input_topic", "auv0/RotationSensor")
        self.declare_parameter("output_topic", "imu/ahrs")
        self.declare_parameter("ahrs_frame", "modem_link")
        self.declare_parameter("yaw_noise_sigma", 0.01745)
        self.declare_parameter("roll_pitch_noise_sigma", 0.00349)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.ahrs_frame = (
            self.get_parameter("ahrs_frame").get_parameter_value().string_value
        )
        self.yaw_noise_sigma = (
            self.get_parameter("yaw_noise_sigma").get_parameter_value().double_value
        )
        self.roll_pitch_noise_sigma = (
            self.get_parameter("roll_pitch_noise_sigma")
            .get_parameter_value()
            .double_value
        )

        self.subscription = self.create_subscription(
            Vector3Stamped, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(Imu, output_topic, 10)

        self.get_logger().info(
            f"AHRS converter started. Listening on {input_topic} and "
            f"publishing on {output_topic}."
        )

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles to a Quaternion message.

        :param roll: Roll angle in radians.
        :param pitch: Pitch angle in radians.
        :param yaw: Yaw angle in radians.
        :return: Populated geometry_msgs/Quaternion message.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def listener_callback(self, msg: Vector3Stamped):
        """
        Process rotation sensor data (Vector3Stamped).

        :param msg: Vector3Stamped message containing Euler angles.
        """
        imu_msg = Imu()
        imu_msg.header.stamp = msg.header.stamp
        imu_msg.header.frame_id = self.ahrs_frame

        roll_deg = msg.vector.x
        pitch_deg = msg.vector.y
        yaw_deg = msg.vector.z

        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)

        # HoloOcean doesn't add noise, so we add it ourselves
        roll_rad += random.gauss(0, self.roll_pitch_noise_sigma)
        pitch_rad += random.gauss(0, self.roll_pitch_noise_sigma)
        yaw_rad += random.gauss(0, self.yaw_noise_sigma)

        imu_msg.orientation = self.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)

        yaw_variance = self.yaw_noise_sigma**2
        roll_pitch_variance = self.roll_pitch_noise_sigma**2

        imu_msg.orientation_covariance[0] = roll_pitch_variance
        imu_msg.orientation_covariance[4] = roll_pitch_variance
        imu_msg.orientation_covariance[8] = yaw_variance

        self.publisher.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    ahrs_converter_node = AhrsConverterNode()
    try:
        rclpy.spin(ahrs_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        ahrs_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
