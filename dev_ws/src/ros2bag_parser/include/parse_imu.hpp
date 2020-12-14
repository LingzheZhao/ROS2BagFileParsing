#pragma once

// STD
#include <filesystem>
#include <fstream>
#include <iostream>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Project
#include "read_and_deserialize.hpp"
#include "topic_types.hpp"

void parse_imu(rosbag2_cpp::readers::SequentialReader& reader,
               std::filesystem::path output_dir) {
    std::ofstream out((output_dir / "imu.csv").string());
    out << "timestamp,"
        << "q_x,q_y,q_z,q_w,"
        << "a_x,a_y,a_z,"
        << "w_x,w_y,w_z,"
        << "orientation_covariance[0],"
        << "orientation_covariance[1],"
        << "orientation_covariance[2],"
        << "orientation_covariance[3],"
        << "orientation_covariance[4],"
        << "orientation_covariance[5],"
        << "orientation_covariance[6],"
        << "orientation_covariance[7],"
        << "orientation_covariance[8],"
        << "linear_acceleration_covariance[0],"
        << "linear_acceleration_covariance[1],"
        << "linear_acceleration_covariance[2],"
        << "linear_acceleration_covariance[3],"
        << "linear_acceleration_covariance[4],"
        << "linear_acceleration_covariance[5],"
        << "linear_acceleration_covariance[6],"
        << "linear_acceleration_covariance[7],"
        << "linear_acceleration_covariance[8],"
        << "angular_velocity_covariance[0],"
        << "angular_velocity_covariance[1],"
        << "angular_velocity_covariance[2],"
        << "angular_velocity_covariance[3],"
        << "angular_velocity_covariance[4],"
        << "angular_velocity_covariance[5],"
        << "angular_velocity_covariance[6],"
        << "angular_velocity_covariance[7],"
        << "angular_velocity_covariance[8]" << std::endl;

    sensor_msgs::msg::Imu msg_buf;
    while (reader.has_next()) {
        read_and_deserialize_msg(reader, msg_buf, DATA_IMU);
        out << rclcpp::Time(msg_buf.header.stamp).nanoseconds() << ","
            << std::setprecision(16)
            << msg_buf.orientation.x << "," << msg_buf.orientation.y << ","
            << msg_buf.orientation.z << "," << msg_buf.orientation.w << ","
            << msg_buf.linear_acceleration.x << ","
            << msg_buf.linear_acceleration.y << ","
            << msg_buf.linear_acceleration.z << ","
            << msg_buf.angular_velocity.x << "," << msg_buf.angular_velocity.y
            << "," << msg_buf.angular_velocity.z << ",";
        for (double value : msg_buf.orientation_covariance) {
            out << value << ",";
        }
        for (double value : msg_buf.linear_acceleration_covariance) {
            out << value << ",";
        }
        for (size_t i = 0; i < 9; i++) {
            out << msg_buf.angular_velocity_covariance[i];
            if (8 != i) {
                out << ",";
            } else {
                out << std::endl;
            }
        }
    }
    return;
}