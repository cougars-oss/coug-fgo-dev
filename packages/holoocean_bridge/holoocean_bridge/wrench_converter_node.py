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
from geometry_msgs.msg import WrenchStamped
from holoocean_interfaces.msg import AgentCommand, ControlCommand


class WrenchConverterNode(Node):
    """
    Converts HoloOcean agent/control command messages back to WrenchStamped messages.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("wrench_converter_node")

        self.declare_parameter("agent_topic", "/command/agent")
        self.declare_parameter("control_topic", "ControlCommand")
        self.declare_parameter("output_topic", "cmd_wrench")
        self.declare_parameter("wrench_frame", "base_link")

        agent_topic = (
            self.get_parameter("agent_topic").get_parameter_value().string_value
        )
        control_topic = (
            self.get_parameter("control_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.wrench_frame = (
            self.get_parameter("wrench_frame").get_parameter_value().string_value
        )

        self.agent_sub = self.create_subscription(
            AgentCommand, agent_topic, self.agent_callback, 10
        )
        self.control_sub = self.create_subscription(
            ControlCommand, control_topic, self.control_callback, 10
        )
        self.publisher = self.create_publisher(WrenchStamped, output_topic, 10)

        self.get_logger().info(
            f"Wrench converter started. Listening on {agent_topic} and {control_topic}, "
            f"publishing on {output_topic}."
        )

    def agent_callback(self, msg: AgentCommand):
        """
        Process agent command messages (BlueROV2).

        :param msg: AgentCommand message containing thruster values.
        """
        cmd = msg.command

        # From the BlueROV2.cpp file
        GEO_FACTOR = 0.70710678
        fwd = (cmd[4] + cmd[5] + cmd[6] + cmd[7]) * GEO_FACTOR
        lat = (cmd[4] - cmd[5] + cmd[6] - cmd[7]) * GEO_FACTOR
        vert = (cmd[0] + cmd[1] + cmd[2] + cmd[3])

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = self.wrench_frame

        wrench_msg.wrench.force.x = fwd
        wrench_msg.wrench.force.y = lat
        wrench_msg.wrench.force.z = vert
        self.publisher.publish(wrench_msg)

    def control_callback(self, msg: ControlCommand):
        """
        Process control command messages (CougUV).

        :param msg: ControlCommand message containing control surface/thruster values.
        """
        thruster_rpm = msg.cs[3]

        # From the fossen_dynamics/actuator.py file
        rho = 1026.0
        D_prop = 0.14
        t_prop = 0.1
        KT_0 = 0.4566

        n_rps = thruster_rpm / 60.0
        abs_n_rps = abs(n_rps)

        # IMPORTANT! Assuming advance velocity is 0 and no spool up/down delays
        X_prop = rho * pow(D_prop, 4) * KT_0 * abs_n_rps * n_rps

        force_x = (1.0 - t_prop) * X_prop

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = self.wrench_frame
        wrench_msg.wrench.force.x = force_x
        self.publisher.publish(wrench_msg)


def main(args=None):
    rclpy.init(args=args)
    wrench_converter_node = WrenchConverterNode()
    try:
        rclpy.spin(wrench_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        wrench_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
