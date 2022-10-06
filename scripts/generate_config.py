#!/usr/bin/env python
"""Auto-generate ROS2 config files and ROS1 launch file."""

import argparse
import copy
import logging
import os
import yaml

from lxml import etree


log = logging.getLogger(__name__)

use_jpl_mm = False

keywords = {
    "base": "_BASE_",
    "robot": "_ROBOT_",
    "other_robots": "_OTHER_ROBOTS_",
    "all_robots": "_ALL_ROBOTS_",
}


class Topic(object):
    def __init__(self, name, data):
        self.name = name
        self.type = data.get("type")
        self.rate = data.get("rate", None)
        self.bandwidth = data.get("bandwidth", None)
        self.include_attr = {}

    def __eq__(self, other):
        return self.name == other.name

    def __hash__(self):
        return hash(self.name)

    def __contains__(self, keyword):
        return keyword in self.name

    def copy(self):
        return copy.deepcopy(self)

    def has_ns(self, ns):
        return self.name.startswith("/{}".format(ns))

    def without_ns(self):
        new_self = copy.deepcopy(self)
        new_self.name = "/".join(self.name.split("/")[2:])
        return new_self

    def get(self):
        return self.name

    def get_compressed(self):
        return self.name + "/compressed"

    def get_compressed_reliable(self):
        return self.name + "/compressed/reliable"
    
    def get_ack(self):
        return self.name + "/ack"

    def replace(self, src, dst):
        self.name = self.name.replace(src, dst)
        return self

    def get_encoder_element(self):
        elem = etree.Element(
            "include",
            dict(
                file="$(find ros2_bridge)/launch/include/encode.launch",
                **self.include_attr
            ),
        )
        elem.append(
            etree.Element(
                "arg",
                {
                    "name": "robot_namespace",
                    "value": "$(arg robot_namespace)",
                },
            )
        )
        elem.append(
            etree.Element(
                "arg",
                {
                    "name": "type",
                    "value": self.type,
                },
            )
        )
        elem.append(
            etree.Element(
                "arg",
                {
                    "name": "input",
                    "value": self.name,
                },
            )
        )
        if self.rate:
            elem.append(
                etree.Element(
                    "arg",
                    {
                        "name": "rate",
                        "value": str(self.rate),
                    },
                )
            )
        if self.bandwidth:
            elem.append(
                etree.Element(
                    "arg",
                    {
                        "name": "bandwidth",
                        "value": str(self.bandwidth),
                    },
                )
            )
        elem.append(etree.Element("arg", {"name": "suffix", "value": "$(arg suffix)"}))
        return elem

    def get_decoder_element(self):
        elem = etree.Element(
            "include",
            dict(
                file="$(find ros2_bridge)/launch/include/decode.launch",
                **self.include_attr
            ),
        )
        elem.append(
            etree.Element(
                "arg",
                {
                    "name": "robot_namespace",
                    "value": "$(arg robot_namespace)",
                },
            )
        )
        elem.append(
            etree.Element(
                "arg",
                {
                    "name": "type",
                    "value": self.type,
                },
            )
        )
        elem.append(
            etree.Element(
                "arg",
                {
                    "name": "output",
                    "value": self.name,
                },
            )
        )
        elem.append(etree.Element("arg", {"name": "suffix", "value": "$(arg suffix)"}))
        return elem

    def add_launch_condition(self, cond):
        self.include_attr["if"] = cond


class Agent(object):
    robot_types = ["husky", "telemax", "spot"]

    def __init__(self, name):
        self.name = name

    def is_robot(self):
        return self.name == "robot" or self.name in self.robot_types

    def is_base(self):
        return not self.is_robot()


def generate_robot_ros2_params(filename, config, robots):
    topics_in = {}
    topics_out = {}

    for config_name, c in config.iteritems():
        # Generate ROS2 configs
        names_in = []
        names_out = []
        pub = Agent(c["publisher"])
        sub = Agent(c["subscriber"])

        if pub.is_robot() and sub.is_robot():  # Inter-robot
            for name, data in c["topics"].iteritems():
                topic = Topic(name, data)

                assert keywords["base"] not in topic
                assert keywords["robot"] not in topic
                assert keywords["all_robots"] not in topic

                if topic.has_ns(keywords["other_robots"]):
                    names_in.append(topic)

                    # Remove OTHER_ROBOTS keyword only out outbound topic
                    names_out.append(topic.without_ns())
                else:
                    names_in.append(topic)
                    names_out.append(topic)

        elif pub.is_robot():  # Base <-- Robot
            for name, data in c["topics"].iteritems():
                topic = Topic(name, data)

                assert keywords["robot"] not in topic or topic.has_ns(keywords["robot"])
                assert keywords["other_robots"] not in topic
                assert keywords["all_robots"] not in topic

                # Remove only _ROBOT_ keyword as it will be prepended by ROS_NAMESPACE
                # Other keywords will be handled in ros2 bridge
                if topic.has_ns(keywords["robot"]):
                    names_out.append(topic.without_ns())
                else:
                    names_out.append(topic)

        elif sub.is_robot():  # Base --> Robot
            for name, data in c["topics"].iteritems():
                topic = Topic(name, data)

                assert keywords["robot"] not in topic or topic.has_ns(keywords["robot"])
                assert keywords["other_robots"] not in topic
                assert keywords["all_robots"] not in topic

                # Remove only _ROBOT_ keyword as it will be prepended by ROS_NAMESPACE
                # Other keywords will be handled in ros2 bridge
                if topic.has_ns(keywords["robot"]):
                    names_in.append(topic.without_ns())
                else:
                    names_in.append(topic)

        else:
            raise ValueError("Invalid publisher/subscriber pair: {}".format(c))

        # Store in container
        qos = c["qos"]
        topics_in.setdefault(qos, {"topic_names": []})
        topics_out.setdefault(qos, {"topic_names": []})
        topics_in[qos]["topic_names"] += names_in
        topics_out[qos]["topic_names"] += names_out

    # Remove duplicates
    for topic_config in topics_in.values():
        topic_config["topic_names"] = list(set(topic_config["topic_names"]))

    for topic_config in topics_out.values():
        topic_config["topic_names"] = list(set(topic_config["topic_names"]))

    # Sort by topic name
    for topic_config in topics_in.values():
        topic_config["topic_names"].sort(key=lambda obj: obj.name)

    for topic_config in topics_out.values():
        topic_config["topic_names"].sort(key=lambda obj: obj.name)

    # Add compresion, reliable and ack suffix
    temp_topics_in = copy.deepcopy(topics_in)
    temp_topics_out = copy.deepcopy(topics_out)

    for qos in topics_in.keys():
        if(qos == "reliable"):
            temp_topics_in[qos]["topic_names"] = []

            for t in topics_in[qos]["topic_names"]:
                temp_topics_in[qos]["topic_names"].append(t.get_compressed_reliable())

            for t in topics_out[qos]["topic_names"]:
                temp_topics_in["default"]["topic_names"].append(t.get_ack())
        else:
            temp_topics_in[qos]["topic_names"] = []

            for t in topics_in[qos]["topic_names"]:
                temp_topics_in[qos]["topic_names"].append(t.get_compressed())

    for qos in topics_out.keys():
        if(qos == "reliable"):
            temp_topics_out[qos]["topic_names"] = []

            for t in topics_out[qos]["topic_names"]:
                temp_topics_out[qos]["topic_names"].append(t.get_compressed_reliable())
            
            for t in topics_in[qos]["topic_names"]:
                temp_topics_out["default"]["topic_names"].append(t.get_ack())
        else:
            temp_topics_out[qos]["topic_names"] = []

            for t in topics_out[qos]["topic_names"]:
                temp_topics_out[qos]["topic_names"].append(t.get_compressed())

    topics_in = temp_topics_in
    topics_out = temp_topics_out

    # Hack! Temporarily disable ROS2 transmission for reliable topics
    # to use external method
    if use_jpl_mm:
        log.warn("Temporarily disable ROS2 transmission for reliable topics")
        for config in topics_out.values():
            config["topic_names"] = [
                topic for topic in config["topic_names"] if 'compressed/reliable' not in topic
            ]
        for config in topics_in.values():
            config["topic_names"] = [
                topic for topic in config["topic_names"] if 'compressed/reliable' not in topic
            ]

    # Hack! Avoid empty topic list to prevent ros1-2 bridge crash
    # TODO: Fix on the bridge node
    for config in topics_out.values():
        if not config["topic_names"]:
            config["topic_names"] = [
                "ros2_dummy"
            ]
    for config in topics_in.values():
        if not config["topic_names"]:
            config["topic_names"] = [
                "ros2_dummy"
            ]

    # Write to file
    log.info("Writing param file to %s", filename)
    data = {
        "ros_bridge_subt": {
            "ros__parameters": {
                "topics_1_to_2": topics_out,
                "topics_2_to_1": topics_in,
            }
        }
    }
    with open(filename, "w") as f:
        f.write("# This file is auto-generated. Do not edit.\n")
        yaml.dump(data, f)


def generate_base_ros2_params(filename, config, robots):
    topics_in = {}
    topics_out = {}

    for c in config.values():
        names_in = []
        names_out = []
        pub = Agent(c["publisher"])
        sub = Agent(c["subscriber"])

        if pub.is_base():  # Base --> Robot
            for name, data in c["topics"].iteritems():
                topic = Topic(name, data)

                assert keywords["other_robots"] not in topic

                # Replace robot keywords by all robots
                topic.replace(keywords["robot"], keywords["all_robots"])
                names_out.append(topic)

        elif sub.is_base():  # Base <-- Robot
            for name, data in c["topics"].iteritems():
                topic = Topic(name, data)

                assert keywords["other_robots"] not in topic

                # Replace robot keywords by all robots
                topic.replace(keywords["robot"], keywords["all_robots"])
                names_in.append(topic)

        elif pub.is_robot() and sub.is_robot():
            pass

        else:
            raise ValueError("Invalid publisher/subscriber pair: {}".format(c))

        # Store in container
        qos = c["qos"]
        topics_in.setdefault(qos, {"topic_names": []})
        topics_out.setdefault(qos, {"topic_names": []})
        topics_in[qos]["topic_names"] += names_in
        topics_out[qos]["topic_names"] += names_out

    # Remove duplicates
    for topic_config in topics_in.values():
        topic_config["topic_names"] = list(set(topic_config["topic_names"]))

    for topic_config in topics_out.values():
        topic_config["topic_names"] = list(set(topic_config["topic_names"]))

    # Sort by topic name
    for topic_config in topics_in.values():
        topic_config["topic_names"].sort(key=lambda obj: obj.name)

    for topic_config in topics_out.values():
        topic_config["topic_names"].sort(key=lambda obj: obj.name)

    # Add compresion, reliable and ack suffixes
    temp_topics_in = copy.deepcopy(topics_in)
    temp_topics_out = copy.deepcopy(topics_out)

    for qos in topics_in.keys():
        if(qos == "reliable"):
            temp_topics_in[qos]["topic_names"] = []

            for t in topics_in[qos]["topic_names"]:
                temp_topics_in[qos]["topic_names"].append(t.get_compressed_reliable())

            for t in topics_out[qos]["topic_names"]:
                temp_topics_in["default"]["topic_names"].append(t.get_ack())
        else:
            temp_topics_in[qos]["topic_names"] = []

            for t in topics_in[qos]["topic_names"]:
                temp_topics_in[qos]["topic_names"].append(t.get_compressed())

    for qos in topics_out.keys():
        if(qos == "reliable"):
            temp_topics_out[qos]["topic_names"] = []

            for t in topics_out[qos]["topic_names"]:
                temp_topics_out[qos]["topic_names"].append(t.get_compressed_reliable())

            for t in topics_in[qos]["topic_names"]:
                temp_topics_out["default"]["topic_names"].append(t.get_ack())
        else:
            temp_topics_out[qos]["topic_names"] = []

            for t in topics_out[qos]["topic_names"]:
                temp_topics_out[qos]["topic_names"].append(t.get_compressed())

    topics_in = temp_topics_in
    topics_out = temp_topics_out

    # Write to file
    log.info("Writing param file to %s", filename)
    data = {
        "ros_bridge_subt": {
            "ros__parameters": {
                "topics_1_to_2": topics_out,
                "topics_2_to_1": topics_in,
            }
        }
    }
    with open(filename, "w") as f:
        f.write("# This file is auto-generated. Do not edit.\n")
        yaml.dump(data, f)


def generate_robot_launch_file(filename, config, robots):
    encoders = []
    decoders = []
    for c in config.values():
        pub = Agent(c["publisher"])
        sub = Agent(c["subscriber"])

        if pub.is_robot() and sub.is_robot():  # Robot <-> Robot
            for name, data in c["topics"].iteritems():
                topic = Topic(name, data)

                assert keywords["base"] not in topic
                assert keywords["robot"] not in topic
                assert keywords["all_robots"] not in topic

                for robot in robots:
                    _topic = topic.without_ns()
                    encoders.append(_topic)

                    _topic = topic.copy()
                    _topic.replace(keywords["other_robots"], robot)
                    _topic.add_launch_condition(
                        "$(eval robot_namespace != '{other}' and '{other}' in robot_list)".format(
                            other=robot
                        )
                    )

                    decoders.append(_topic)

        elif pub.is_robot():  # Base <-- Robot
            for name, data in c["topics"].iteritems():
                topic = Topic(name, data)

                assert keywords["other_robots"] not in topic
                assert keywords["all_robots"] not in topic

                if topic.has_ns(keywords["robot"]):
                    topic = topic.without_ns()
                if topic.has_ns(keywords["base"]):
                    topic.replace(keywords["base"], "$(arg base_namespace)")

                encoders.append(topic)

        elif sub.is_robot():  # Base --> Robot
            for name, data in c["topics"].iteritems():
                topic = Topic(name, data)

                assert keywords["other_robots"] not in topic
                assert keywords["all_robots"] not in topic

                if topic.has_ns(keywords["robot"]):
                    topic = topic.without_ns()
                if topic.has_ns(keywords["base"]):
                    topic.replace(keywords["base"], "$(arg base_namespace)")

                decoders.append(topic)

        else:
            raise ValueError("Invalid publisher/subscriber pair: {}".format(c))

    # Remove duplicate
    encoders = list(set(encoders))
    decoders = list(set(decoders))

    # Sort by topic name
    encoders.sort(key=lambda obj: obj.name)
    decoders.sort(key=lambda obj: obj.name)

    # Compose launch file xml
    launch = etree.Element("launch")
    launch.append(etree.Comment("This file is auto-generated. Do not edit."))
    launch.append(etree.Element("arg", {"name": "robot_namespace"}))
    launch.append(etree.Element("arg", {"name": "base_namespace"}))
    launch.append(etree.Element("arg", {"name": "robot_list"}))
    launch.append(etree.Element("arg", {"name": "suffix", "default": ""}))

    for enc in encoders:
        launch.append(enc.get_encoder_element())
    for dec in decoders:
        launch.append(dec.get_decoder_element())

    # Write to file
    log.info("Writing ros2 param in %s", filename)
    with open(filename, "w") as f:
        f.write(etree.tostring(launch, pretty_print=True))


def generate_base_launch_file(filename, config, robots):
    encoders = []
    decoders = []
    for c in config.values():
        pub = Agent(c["publisher"])
        sub = Agent(c["subscriber"])

        if pub.is_robot() and sub.is_robot():  # Robot <-> Robot
            pass

        elif pub.is_base():  # Base --> Robot
            for name, data in c["topics"].iteritems():
                topic = Topic(name, data)

                assert keywords["other_robots"] not in topic
                assert keywords["all_robots"] not in topic

                if topic.has_ns(keywords["robot"]):
                    # /<robot>/some/topic
                    for robot in robots:
                        _topic = topic.copy()
                        _topic.replace(keywords["robot"], robot)
                        if robot.startswith(sub.name) or sub.name == "robot":
                            _topic.add_launch_condition(
                                "$(eval '{}' in robot_list)".format(robot)
                            )
                            encoders.append(_topic)
                else:
                    # /<base>/some/topic
                    _topic = topic.without_ns()
                    encoders.append(_topic)

        elif sub.is_base():  # Base --> Robot
            for name, data in c["topics"].iteritems():
                topic = Topic(name, data)

                assert keywords["other_robots"] not in topic
                assert keywords["all_robots"] not in topic

                if topic.has_ns(keywords["robot"]):
                    for robot in robots:
                        _topic = topic.copy()
                        _topic.replace(keywords["robot"], robot)
                        if sub.name != "robot":
                            _topic.add_launch_condition(
                                "$(eval robot_namespace.startswith('{}') and '{}' in robot_list)".format(
                                    sub.name, robot
                                )
                            )
                        else:
                            _topic.add_launch_condition(
                                "$(eval '{}' in robot_list)".format(robot)
                            )
                        decoders.append(_topic)
                else:
                    # /<base>/some/topic
                    _topic = topic.without_ns()
                    decoders.append(_topic)

        else:
            raise ValueError("Invalid publisher/subscriber pair: {}".format(c))

    # Remove duplicate
    encoders = list(set(encoders))
    decoders = list(set(decoders))

    # Sort by topic name
    encoders.sort(key=lambda obj: obj.name)
    decoders.sort(key=lambda obj: obj.name)

    # Compose launch file xml
    launch = etree.Element("launch")
    launch.append(etree.Comment("This file is auto-generated. Do not edit."))
    launch.append(etree.Element("arg", {"name": "robot_namespace"}))
    launch.append(etree.Element("arg", {"name": "base_namespace"}))
    launch.append(etree.Element("arg", {"name": "robot_list"}))
    launch.append(etree.Element("arg", {"name": "suffix", "default": ""}))

    for enc in encoders:
        launch.append(enc.get_encoder_element())
    for dec in decoders:
        launch.append(dec.get_decoder_element())

    # Write to file
    log.info("Writing ros2 param in %s", filename)
    with open(filename, "w") as f:
        f.write(etree.tostring(launch, pretty_print=True))


def generate_topic_list(filename, config, robots):
    # Create a list of topics (without namespace)
    topics = []
    for c in config.values():
        for name, data in c["topics"].iteritems():
            topic = Topic(name, data)
            topics.append(topic.without_ns().name)

    # Remove duplicate
    topics = list(set(topics))

    # Write to file
    with open(filename, "w") as f:
        f.write("# This file is auto-generated. Do not edit.\n")
        yaml.dump(topics, f)


def main():
    global use_jpl_mm

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-r", "--robots", nargs="+", required=True, help="list of robots"
    )
    parser.add_argument(
        "-c", "--config", required=True, help="path to master config file"
    )
    parser.add_argument(
        "-o",
        "--output-directory",
        required=True,
        help="path to config output directory",
    )
    parser.add_argument(
        "--use-jpl-mm",
        help="use JPL MM for reliable topic transfer",
        default='false',
    )
    args = parser.parse_args()

    # Handle string boolean
    if args.use_jpl_mm.lower() in ['true', '1']:
        use_jpl_mm = True
    else:
        use_jpl_mm = False

    # Read master config
    with open(args.config) as f:
        config = yaml.load(f)

    # Create output directory
    try:
        os.makedirs(args.output_directory)
    except OSError:
        pass

    # Auto-generage ROS2 config files
    generate_robot_ros2_params(
        os.path.join(args.output_directory, "robot.yaml"), config, robots=args.robots
    )
    generate_base_ros2_params(
        os.path.join(args.output_directory, "base.yaml"), config, robots=args.robots
    )
    generate_robot_launch_file(
        os.path.join(args.output_directory, "robot_topics.launch"),
        config,
        robots=args.robots,
    )
    generate_base_launch_file(
        os.path.join(args.output_directory, "base_topics.launch"),
        config,
        robots=args.robots,
    )
    generate_topic_list(
        os.path.join(args.output_directory, "topics.yaml"), config, robots=args.robots
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
