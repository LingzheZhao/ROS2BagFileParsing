#pragma once

#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"

template <typename T>
void read_and_deserialize_msg(rosbag2_cpp::readers::SequentialReader& reader,
                              T& msg_buf, const std::string& topic_type) {
    // serialized data
    auto serialized_message = reader.read_next();

    // deserialization and conversion to ros message
    auto ros_message =
        std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
    ros_message->time_stamp = 0;
    ros_message->allocator = rcutils_get_default_allocator();
    ros_message->message = &msg_buf;
    auto library = rosbag2_cpp::get_typesupport_library(
        topic_type, "rosidl_typesupport_cpp");
    auto type_support = rosbag2_cpp::get_typesupport_handle(
        topic_type, "rosidl_typesupport_cpp", library);

    rosbag2_cpp::SerializationFormatConverterFactory factory;
    std::unique_ptr<
        rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer>
        cdr_deserializer_;
    cdr_deserializer_ = factory.load_deserializer("cdr");
    cdr_deserializer_->deserialize(serialized_message, type_support,
                                   ros_message);
    return;
}