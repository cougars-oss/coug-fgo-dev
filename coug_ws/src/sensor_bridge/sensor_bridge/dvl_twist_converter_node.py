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
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistWithCovarianceStamped
from builtin_interfaces.msg import Time
from dvl_msgs.msg import DVL


class DvlTwistConverterNode(Node):
    """
    Converts raw DVL velocity data to a twist message.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("dvl_twist_converter_node")

        self.declare_parameter("input_topic", "dvl/data")
        self.declare_parameter("output_topic", "dvl/twist")
        self.declare_parameter("dvl_frame", "dvl_link")
        self.declare_parameter("simulate_dropout", False)
        self.declare_parameter("dropout_frequency", 1.0 / 30.0)
        self.declare_parameter("dropout_duration", 5.0)
        self.declare_parameter("use_fom_covariance", False)
        self.declare_parameter("fom_covariance_scale", 4.0)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.dvl_frame = (
            self.get_parameter("dvl_frame").get_parameter_value().string_value
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
        self.use_fom_covariance = (
            self.get_parameter("use_fom_covariance").get_parameter_value().bool_value
        )
        self.fom_covariance_scale = (
            self.get_parameter("fom_covariance_scale").get_parameter_value().double_value
        )

        self.subscription = self.create_subscription(
            DVL, input_topic, self.listener_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped, output_topic, 10
        )

        self.get_logger().info(
            f"DVL twist converter started. Listening on {input_topic} "
            f"and publishing on {output_topic}."
        )

    def listener_callback(self, msg: DVL):
        """
        Process DVL data (DVL message).

        :param msg: DVL message containing velocity data.
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

        if not msg.velocity_valid:
            self.get_logger().warn(
                "Received invalid DVL velocity", throttle_duration_sec=1.0
            )
            return

        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.frame_id = self.dvl_frame
        twist_msg.header.stamp = self.time_from_microseconds(msg.time_of_validity)

        twist_msg.twist.twist.linear.x = msg.velocity.x
        twist_msg.twist.twist.linear.y = msg.velocity.y
        twist_msg.twist.twist.linear.z = msg.velocity.z

        if self.use_fom_covariance:
            cov_val = msg.fom * self.fom_covariance_scale
            twist_msg.twist.covariance[0] = cov_val
            twist_msg.twist.covariance[7] = cov_val
            twist_msg.twist.covariance[14] = cov_val
        else:
            twist_msg.twist.covariance[0:3] = msg.covariance[0:3]
            twist_msg.twist.covariance[6:9] = msg.covariance[3:6]
            twist_msg.twist.covariance[12:15] = msg.covariance[6:9]

        self.publisher.publish(twist_msg)

    @staticmethod
    def time_from_microseconds(us: int) -> Time:
        """
        Convert microseconds to builtin_interfaces/Time.

        :param us: Time in microseconds.
        :return: Time message.
        """
        t = Time()
        t.sec = us // 1_000_000
        t.nanosec = (us % 1_000_000) * 1_000
        return t


def main(args=None):
    rclpy.init(args=args)
    dvl_twist_converter_node = DvlTwistConverterNode()
    try:
        rclpy.spin(dvl_twist_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_twist_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
