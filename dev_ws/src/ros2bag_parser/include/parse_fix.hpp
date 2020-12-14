#pragma once

#include <iostream>
#include <fstream>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "read_and_deserialize.hpp"
#include "topic_types.hpp"

void parse_fix(rosbag2_cpp::readers::SequentialReader& reader,
               std::filesystem::path output_dir) {
    std::ofstream out((output_dir / "fix.csv").string());
    out << "timestamp,"
        << "latitude,longitude,altitude,"
        << "position_covariance[0],"
        << "position_covariance[1],"
        << "position_covariance[2],"
        << "position_covariance[3],"
        << "position_covariance[4],"
        << "position_covariance[5],"
        << "position_covariance[6],"
        << "position_covariance[7],"
        << "position_covariance[8],"
        << "position_covariance_type" << std::endl;

    sensor_msgs::msg::NavSatFix msg_buf;
    while (reader.has_next()) {
        read_and_deserialize_msg(reader, msg_buf, GNSS_FIX);

        out << rclcpp::Time(msg_buf.header.stamp).nanoseconds() << ","
            << std::setprecision(16)
            << msg_buf.latitude << "," << msg_buf.longitude << ","
            << msg_buf.altitude << ",";
        for (double value : msg_buf.position_covariance) {
            out << value << ",";
        }
        out << static_cast<int>(msg_buf.position_covariance_type) << std::endl;
    }

    return;
}