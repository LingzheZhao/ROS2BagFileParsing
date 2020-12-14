#pragma once

// STD
#include <filesystem>
#include <iostream>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

// Project
#include "read_and_deserialize.hpp"
#include "topic_types.hpp"

void parse_image(rosbag2_cpp::readers::SequentialReader& reader,
                 std::filesystem::path output_dir) {
    sensor_msgs::msg::Image msg_buf;
    std::filesystem::path image_path;
    while (reader.has_next()) {
        image_path =
            output_dir / "data" /
            (std::to_string(rclcpp::Time(msg_buf.header.stamp).nanoseconds()) +
             ".png");
        read_and_deserialize_msg(reader, msg_buf, IMAGE);
        // std::cout << "height\t" << msg_buf.height << std::endl
        //           << "width\t" << msg_buf.width << std::endl
        //           << "step\t" << msg_buf.step << std::endl
        //           << "size\t" << msg_buf.data.size() << std::endl;
        uint8_t bits = msg_buf.step / msg_buf.width;
        bits = 0 == bits ? 1 : bits;
        cv::Mat img(msg_buf.height, msg_buf.width, CV_MAKETYPE(CV_8U, bits),
                    msg_buf.data.data());
        if (img.empty()) {
            std::cout << "Could not read the image: " << image_path
                      << std::endl;
        } else {
            // std::cout << "Got image(" << img.size() << "): " << image_path
            //           << std::endl;
        }
        // cv::imshow("Preview", img);
        cv::imwrite(image_path.string(), img);
    }
    return;
}