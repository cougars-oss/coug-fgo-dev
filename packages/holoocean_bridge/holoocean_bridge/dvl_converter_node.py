# Copyright (c) 2026 BYU FROST Lab
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
from dvl_msgs.msg import DVL


class DvlConverterNode(Node):
    """
    Converts DVL data from HoloOcean to a Waterlinked DVL message.

    :author: Nelson Durrant
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("dvl_converter_node")

        self.declare_parameter("input_topic", "auv0/DVLSensorVelocity")
        self.declare_parameter("output_topic", "dvl/data")
        self.declare_parameter("dvl_frame", "dvl_link")
        self.declare_parameter("override_covariance", True)
        self.declare_parameter("noise_sigma", 0.02)

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

        self.subscription = self.create_subscription(
            TwistWithCovarianceStamped, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(DVL, output_topic, 10)

        self.get_logger().info(
            f"DVL converter started. Listening on {input_topic} and publishing on {output_topic}."
        )

    def listener_callback(self, msg: TwistWithCovarianceStamped):
        """
        Process DVL sensor data (TwistWithCovarianceStamped).

        :param msg: TwistWithCovarianceStamped message containing DVL data.
        """
        msg.header.frame_id = self.dvl_frame

        dvl_msg = DVL()
        dvl_msg.header = msg.header
        dvl_msg.header.frame_id = self.dvl_frame

        dvl_msg.velocity.x = msg.twist.twist.linear.x
        dvl_msg.velocity.y = msg.twist.twist.linear.y
        dvl_msg.velocity.z = msg.twist.twist.linear.z

        dvl_msg.velocity_valid = True

        # Convert nanoseconds to microseconds
        dvl_msg.time_of_validity = int(
            msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3
        )

        if self.override_covariance:
            covariance = self.noise_sigma * self.noise_sigma
            dvl_msg.covariance = [0.0] * 9
            dvl_msg.covariance[0] = covariance
            dvl_msg.covariance[4] = covariance
            dvl_msg.covariance[8] = covariance
        else:
            dvl_msg.covariance = [0.0] * 9
            dvl_msg.covariance[0] = msg.twist.covariance[0]
            dvl_msg.covariance[1] = msg.twist.covariance[1]
            dvl_msg.covariance[2] = msg.twist.covariance[2]
            dvl_msg.covariance[3] = msg.twist.covariance[6]
            dvl_msg.covariance[4] = msg.twist.covariance[7]
            dvl_msg.covariance[5] = msg.twist.covariance[8]
            dvl_msg.covariance[6] = msg.twist.covariance[12]
            dvl_msg.covariance[7] = msg.twist.covariance[13]
            dvl_msg.covariance[8] = msg.twist.covariance[14]

        self.publisher.publish(dvl_msg)


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
