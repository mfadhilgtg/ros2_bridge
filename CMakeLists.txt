cmake_minimum_required(VERSION 2.8.3)
project(ros2_bridge)

find_package(catkin REQUIRED)

catkin_package()

set(ROS2_DISTRO dashing)

# Build
if(EXISTS /opt/ros/${ROS2_DISTRO})
  add_custom_target(ros2_ws
    ALL
    COMMAND . /opt/ros/${ROS2_DISTRO}/setup.sh && colcon build
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/ros2_ws
  )
endif()

# Testing
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_config.test)
endif()
