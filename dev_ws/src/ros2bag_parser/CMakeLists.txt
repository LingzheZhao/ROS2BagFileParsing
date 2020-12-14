cmake_minimum_required(VERSION 3.5)
project(ros2bag_parser)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(OpenCV 4 REQUIRED)



include_directories(
  include
  ${ros2bag_parser_INCLUDE_DIRS}
)

add_executable(ros2bag_parser_node src/ros2bag_parser_node.cpp)

target_include_directories(ros2bag_parser_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  target_link_libraries(ros2bag_parser_node ${OpenCV_LIBS})

ament_target_dependencies(
  ros2bag_parser_node
  "rclcpp"
  "rosbag2_cpp"
  "rosbag2_storage"
)
  
install(TARGETS ros2bag_parser_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
