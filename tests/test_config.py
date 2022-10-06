#!/usr/bin/env python

import unittest
import subprocess

import yaml

import rospy
import rostest


class ConfigTest(unittest.TestCase):
    def setUp(self):
        config_file = rospy.get_param("~config_file")
        yamlfile = open(config_file, "r")
        self.config_dict = yaml.load(yamlfile)
        yamlfile.close()
        self.config_dict = self.config_dict["ros_bridge_subt"]["ros__parameters"]

    def test_valid_value(self):
        for topic_direction in self.config_dict:
            if topic_direction in ["substitutions", "robots"]:
                continue
            for topic_category in self.config_dict[topic_direction]:
                topic_profile = self.config_dict[topic_direction][topic_category]
                # Assert the value if the topic exist
                if "history" in topic_profile.keys():
                    self.assertIn(topic_profile["history"], ["KEEP_LAST", "KEEP_ALL"])
                if "durability" in topic_profile.keys():
                    self.assertIn(
                        topic_profile["durability"], ["VOLATILE", "TRANSIENT_LOCAL"]
                    )
                if "reliability" in topic_profile.keys():
                    self.assertIn(
                        topic_profile["reliability"], ["BEST_EFFORT", "RELIABLE"]
                    )
                if "liveliness" in topic_profile.keys():
                    self.assertIn(
                        topic_profile["liveliness"],
                        ["SYSTEM_DEFAULT", "AUTOMATIC", "MANUAL_BY_NODE"],
                    )
                if "queue_size" in topic_profile.keys():
                    self.assertIsInstance(topic_profile["queue_size"], int)
                if "deadline" in topic_profile.keys():
                    self.assertIsInstance(topic_profile["deadline"], float)
                if "lifespan" in topic_profile.keys():
                    self.assertIsInstance(topic_profile["lifespan"], float)
                if "liveliness_duration" in topic_profile.keys():
                    self.assertIsInstance(topic_profile["liveliness_duration"], float)

    def test_autocode(self):
        subprocess.check_call("rosrun ros2_bridge generate_config.sh", shell=True)


if __name__ == "__main__":
    rospy.init_node("test_config")
    rostest.rosrun("ros2_bridge", rospy.get_name(), ConfigTest)
