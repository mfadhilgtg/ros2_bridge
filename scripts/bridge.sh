#!/bin/bash

project_dir=$(rospack find ros2_bridge)
name=$1
config_file=$2
common_config_file=$3
use_jpl_mm=${4:-false}

# Auto-generate config file
rosrun ros2_bridge generate_config.sh $use_jpl_mm

# Set environmental variables
unset ROS_DISTRO
source /opt/ros/dashing/local_setup.bash
source $project_dir/ros2_ws/install/local_setup.bash

# Choose DDS vendor
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_connext_cpp

# Run bridge node
cd $project_dir/config/dds && \
ros2 run ros1_bridge_subt static_bridge \
    __params:=$common_config_file \
    __params:=$config_file \
    __name:=$name \
    _capability_group:=/CommBridge