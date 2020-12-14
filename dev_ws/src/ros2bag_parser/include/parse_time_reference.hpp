#pragma once

#include <iostream>
#include <fstream>
#include <sensor_msgs/msg/time_reference.hpp>

#include "read_and_deserialize.hpp"
#include "topic_types.hpp"

void parse_time_reference(rosbag2_cpp::readers::SequentialReader& reader,
                          std::filesystem::path output_dir) {
    std::ofstream out((output_dir / "time_ref.csv").string());
    out << "timestamp,time_reference" << std::endl;

    sensor_msgs::msg::TimeReference msg_buf;
    while (reader.has_next()) {
        read_and_deserialize_msg(reader, msg_buf, TIME_REF);
        out << rclcpp::Time(msg_buf.header.stamp).nanoseconds() << ","
            << msg_buf.time_ref.sec << std::endl;
    }
    return;
}