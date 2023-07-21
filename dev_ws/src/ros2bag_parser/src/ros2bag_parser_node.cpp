// C/C++
#include <cstdio>

// STD
#include <iostream>
#include <filesystem>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>

// PROJECT
#include "parse_fix.hpp"
#include "parse_image.hpp"
#include "parse_imu.hpp"
#include "parse_time_reference.hpp"
#include "parse_twist.hpp"
#include "topic_types.hpp"

using rosbag2_cpp::converter_interfaces::SerializationFormatConverter;



rosbag2_cpp::readers::SequentialReader reader;

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <db3 file path> <output dir>" << std::endl;
        return EXIT_FAILURE;
    }

    rosbag2_storage::StorageOptions storage_options{};
    storage_options.uri = argv[1];
    storage_options.storage_id = "sqlite3";
    std::filesystem::path output_dir(argv[2]);

    rosbag2_cpp::ConverterOptions converter_options{};
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    reader.open(storage_options, converter_options);

    auto topics = reader.get_all_topics_and_types();

    // about metadata
    std::cout << "\n[INFO] Detected meta data file." << std::endl
              << "[INFO] Topics count:\t" << topics.size() << std::endl
              << std::endl;

    // assert only one topic in db
    if (1 != topics.size()) {
        std::cerr << "Please record topics into separate databases."
                  << std::endl;
        return EXIT_FAILURE;
    }

    auto topic = topics[0];
    auto topic_name = topic.name;
    auto topic_type = topic.type;
    auto serialization_format = topic.serialization_format;

    std::cout << "Topic name:\t\t" << topic_name << std::endl
              << "Topic type:\t\t" << topic_type << std::endl
              << "serialization_format \t" << serialization_format << std::endl;

    // assert format is cdr
    if ("cdr" != serialization_format) {
        std::cerr << "Only CDR serialization_format is supported." << std::endl;
        return EXIT_FAILURE;
    }

    if (TIME_REF == topic_type) {
        parse_time_reference(reader, output_dir);
    } else if (GNSS_FIX == topic_type) {
        parse_fix(reader, output_dir);
    } else if (DATA_IMU == topic_type) {
        parse_imu(reader, output_dir);
    } else if (TWIST == topic_type) {
        parse_twist(reader, output_dir);
    } else if (IMAGE == topic_type) {
        parse_image(reader, output_dir);
    } else {
        std::cout << "Unsupported topic." << std::endl;
    }

    return EXIT_SUCCESS;
}
