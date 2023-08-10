# ROS2BagFileParsing

# Getting started

1. Install ros2 foxy
2. Install rosbag2 packages

    ```bash
    sudo apt install ros-foxy-rosbag2* ros-foxy-ros2bag* ros-foxy-sensor-msgs
    ```
3. Build
    
    ```bash
    cd dev_ws
    rosdep install --from-paths src -y --ignore-src
    colcon build
    ```

4. Run package

    ```bash
    . install/setup.bash
    ros2 run ros2bag_parser ros2bag_parser_node
    ```
