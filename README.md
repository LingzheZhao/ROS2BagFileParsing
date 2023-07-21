# ROS2BagFileParsing

# Getting started

1. Install ros2 humble
2. Install rosbag2 packages

    ```bash
    sudo apt install ros-humble-rosbag2* ros-humble-ros2bag* ros-humble-sensor-msgs
    ```
3. Build
    
    ```bash
    cd dev_ws
    colcon build
    ```

4. Run package

    ```bash
    . install/setup.bash
    ros2 run ros2bag_parser ros2bag_parser_node
    ```
