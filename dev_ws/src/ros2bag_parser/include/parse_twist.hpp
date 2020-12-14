#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <iostream>
#include <fstream>

#include "read_and_deserialize.hpp"
#include "topic_types.hpp"

void parse_twist(rosbag2_cpp::readers::SequentialReader& reader,
                 std::filesystem::path output_dir) {
    std::ofstream out((output_dir / "vel.csv").string());
    out << "timestamp,v_x,v_y,v_z,w_x,w_y,w_z" << std::endl;
    geometry_msgs::msg::TwistStamped msg_buf;
    while (reader.has_next()) {
        read_and_deserialize_msg(reader, msg_buf, TWIST);
        out << rclcpp::Time(msg_buf.header.stamp).nanoseconds() << ","
            << std::setprecision(16)
            << msg_buf.twist.linear.x << "," << msg_buf.twist.linear.y << ","
            << msg_buf.twist.linear.z << "," << msg_buf.twist.angular.x << ","
            << msg_buf.twist.angular.y << "," << msg_buf.twist.angular.z
            << std::endl;
    }
}