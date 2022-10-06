#!/usr/bin/env python
"""Launch bridge node."""

import subprocess

import rospy


def main():
    rospy.init_node("ros2_bridge_launcher")

    config_file = rospy.get_param("~config_file")
    common_config_file = rospy.get_param("~common_config_file")
    name = rospy.get_param("~name")
    use_jpl_mm = rospy.get_param("~use_jpl_mm", False)

    simulation = rospy.get_param("/use_sim_time", False)  # Hack

    if simulation:
        rospy.loginfo("Launching bridge without DDS configuration")
        subprocess.call(
            [
                "rosrun",
                "ros2_bridge",
                "bridge_noconf.sh",
                name,
                config_file,
                common_config_file,
            ]
        )
    else:
        rospy.loginfo("Launching bridge with DDS configuration")
        subprocess.call(
            [
                "rosrun",
                "ros2_bridge",
                "bridge.sh",
                name,
                config_file,
                common_config_file,
                str(use_jpl_mm),
            ]
        )


if __name__ == "__main__":
    main()
