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
from sensor_msgs.msg import NavSatFix
import math


class GpsConverterNode(Node):
    """
    Converts GPS data from HoloOcean to a navigation satellite fix message.

    :author: Nelson Durrant (w Gemini 3 Pro)
    :date: Jan 2026
    """

    def __init__(self):
        super().__init__("gps_converter_node")

        self.declare_parameter("input_topic", "auv0/GPSSensor")
        self.declare_parameter("output_topic", "gps/fix")
        self.declare_parameter("gps_frame", "com_link")
        self.declare_parameter("origin_latitude", 40.23890)
        self.declare_parameter("origin_longitude", -111.74212)
        self.declare_parameter("origin_altitude", 1412.0)
        self.declare_parameter("override_covariance", True)
        self.declare_parameter("position_noise_sigma", 0.015)
        self.declare_parameter("altitude_noise_sigma", 0.025)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.gps_frame = (
            self.get_parameter("gps_frame").get_parameter_value().string_value
        )
        self.origin_lat = (
            self.get_parameter("origin_latitude").get_parameter_value().double_value
        )
        self.origin_lon = (
            self.get_parameter("origin_longitude").get_parameter_value().double_value
        )
        self.origin_alt = (
            self.get_parameter("origin_altitude").get_parameter_value().double_value
        )
        self.override_covariance = (
            self.get_parameter("override_covariance").get_parameter_value().bool_value
        )
        self.position_noise_sigma = (
            self.get_parameter("position_noise_sigma")
            .get_parameter_value()
            .double_value
        )
        self.altitude_noise_sigma = (
            self.get_parameter("altitude_noise_sigma")
            .get_parameter_value()
            .double_value
        )

        self.subscription = self.create_subscription(
            Odometry, input_topic, self.listener_callback, 10
        )
        self.publisher = self.create_publisher(NavSatFix, output_topic, 10)

        self.get_logger().info(
            f"GPS converter started. Listening on {input_topic} and publishing on {output_topic}."
        )

    def listener_callback(self, msg: Odometry):
        """
        Process GPS sensor data (Odometry).

        :param msg: Odometry message containing GPS data.
        """
        navsat_msg = NavSatFix()
        navsat_msg.header = msg.header
        navsat_msg.header.frame_id = self.gps_frame
        navsat_msg.status.status = navsat_msg.status.STATUS_FIX
        navsat_msg.status.service = navsat_msg.status.SERVICE_GPS

        d_east = msg.pose.pose.position.x
        d_north = msg.pose.pose.position.y

        lat, lon = self.calculate_inverse_haversine(
            self.origin_lat, self.origin_lon, d_north, d_east
        )

        navsat_msg.latitude = lat
        navsat_msg.longitude = lon
        navsat_msg.altitude = self.origin_alt + msg.pose.pose.position.z

        if self.override_covariance:

            position_covariance = self.position_noise_sigma * self.position_noise_sigma
            altitude_covariance = self.altitude_noise_sigma * self.altitude_noise_sigma

            navsat_msg.position_covariance[0] = position_covariance  # East
            navsat_msg.position_covariance[4] = position_covariance  # North
            navsat_msg.position_covariance[8] = altitude_covariance  # Up
            navsat_msg.position_covariance_type = (
                navsat_msg.COVARIANCE_TYPE_DIAGONAL_KNOWN
            )

        else:

            navsat_msg.position_covariance[0] = msg.pose.covariance[0]
            navsat_msg.position_covariance[1] = msg.pose.covariance[1]
            navsat_msg.position_covariance[2] = msg.pose.covariance[2]
            navsat_msg.position_covariance[3] = msg.pose.covariance[6]
            navsat_msg.position_covariance[4] = msg.pose.covariance[7]
            navsat_msg.position_covariance[5] = msg.pose.covariance[8]
            navsat_msg.position_covariance[6] = msg.pose.covariance[12]
            navsat_msg.position_covariance[7] = msg.pose.covariance[13]
            navsat_msg.position_covariance[8] = msg.pose.covariance[14]
            navsat_msg.position_covariance_type = navsat_msg.COVARIANCE_TYPE_KNOWN

        self.publisher.publish(navsat_msg)

    def calculate_inverse_haversine(self, ref_lat, ref_lon, d_north, d_east):
        """
        Calculate the latitude and longitude given a reference point and displacement.

        :param ref_lat: Reference latitude in degrees.
        :param ref_lon: Reference longitude in degrees.
        :param d_north: Displacement north in meters.
        :param d_east: Displacement east in meters.
        :return: Tuple of (latitude, longitude) in degrees.
        """
        earth_radius = 6378137.0
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)

        d = math.sqrt(d_north**2 + d_east**2)
        theta = math.atan2(d_east, d_north)

        lat_rad = math.asin(
            math.sin(ref_lat_rad) * math.cos(d / earth_radius)
            + math.cos(ref_lat_rad) * math.sin(d / earth_radius) * math.cos(theta)
        )
        lon_rad = ref_lon_rad + math.atan2(
            math.sin(theta) * math.sin(d / earth_radius) * math.cos(ref_lat_rad),
            math.cos(d / earth_radius) - math.sin(ref_lat_rad) * math.sin(lat_rad),
        )
        lat = math.degrees(lat_rad)
        lon = math.degrees(lon_rad)

        return lat, lon


def main(args=None):
    rclpy.init(args=args)
    gps_converter_node = GpsConverterNode()
    try:
        rclpy.spin(gps_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
