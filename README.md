ROS 2 Bridge
============

This repository contains a package to bridge ROS1 and ROS2 messages. The bridge implementation is in ROS2(inside ros2_ws directory) and this ROS1 package wraps the ROS2 workspace.

This package supports bridging common messages listed in [ros2/common_interfaces repository](https://github.com/ros2/common_interfaces), tf2_msgs and other custom messages.

More detailed information about the ROS2 bridge implementation is in the README inside ros2_ws.


## Installation
1. Update the subtenv and install ROS2 from subtenv
```bash
subtenv update
subtenv install ros2
```
2. Clone this repo inisde ROS1 workspace
3. Build the ROS1 package, the CMake file in ROS1 package will build the ROS2 workspace.
```bash
catkin build ros2_bridge
```

## Configuration Files
### 1. Topic Configuration (config directory)
The config files configure the list of topics to be bridged and respective Quality of Service(QoS) policies.

More information about QoS can be found [here](https://design.ros2.org/articles/qos.html)(basic from ROS2 perspective) and [here](https://fast-rtps.docs.eprosima.com/en/v1.8.2/xmlprofiles.html#commonqos)(in depth).

This example with comments provides a working configuration:
```yaml
ros_bridge_subt: # ROS2 param syntax, keep it like this
  ros__parameters: # ROS2 param syntax
    substitutions:
      namespace: # all topics name with _namespace_ will be substituted by each of name on this list
        - husky1
        - husky2
    topics_1_to_2: # List topics to be bridge from ROS1 to ROS2
      default:
        topic_names:
          - vehicle_status/compressed
          - lamp/octree_map_downsampled/compressed
          - lamp/lamp_pose/compressed
        ros1_type_name: sensor_msgs/CompressedImage
        ros2_type_name: sensor_msgs/msg/CompressedImage # ROS2 type name has '/msg/' compared to the ROS1
        history: KEEP_LAST # QoS history: KEEP_LAST or KEEP_ALL
        durability: VOLATILE # QoS durability: VOLATILE or TRANSIENT_LOCAL
        reliability: BEST_EFFORT # QoS reliability: BEST_EFFORT or RELIABLE
        queue_size: 1 # uint, QoS queue_size or depth
        deadline: 0.0 # float, QoS deadline
        lifespan: 0.0 # float, QoS lifespan
        liveliness: SYSTEM_DEFAULT # QoS liveliness: SYSTEM_DEFAULT or AUTOMATIC or MANUAL_BY_NODE
        liveliness_duration: 0.0 # float, QoS liveliness duration
    topics_2_to_1: # List topics to be bridge from ROS2 to ROS1
      robot_pose:
        topic_names:
          - /_namespace_/odometry/dummy
        ros1_type_name: nav_msgs/Odometry
        ros2_type_name: nav_msgs/msg/Odometry
        history: KEEP_LAST
        durability: VOLATILE
        reliability: BEST_EFFORT
        queue_size: 1
        deadline: 0.0
        lifespan: 0.0
        liveliness: SYSTEM_DEFAULT
        liveliness_duration: 0.0
```

To verify the yaml file, run unittest:
```bash
rostest ros2_bridge test_config.test
```

### 2. DDS Configuration (config_dds directory)
DDS Configuration is used to setup up Unicast communication for ROS2 and set Flowcontroller. Currently we use Fast-RTPS DDS because it's more stable than RTI version.

In the config_dds directory, the config file of the Fast-RTPS is DEFAULT_FASTRTPS_PROFILES.xml. The rest is config files for RTI.

Following is the example of Fast-RTPS config file with commented description. For more information about other configuration setting, please refer to [Fast-RTPS documentation](https://fast-rtps.docs.eprosima.com/en/v1.8.2/xmlprofiles.html)

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles>
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>veelpeers</transport_id> <!-- string -->
            <type>UDPv4</type> <!-- string -->
            <maxInitialPeersRange>10</maxInitialPeersRange> <!-- uint32 The maximum number of guessed initial peers to try to connect. Keep it LOW NUMBER! because it effects the stability of initial discovery-->
            <interfaceWhiteList> <!-- Add all address in the network -->
              <address>127.0.0.1</address>
              <address>192.168.2.201</address>
              <address>192.168.2.104</address>
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="participant_somename" is_default_profile="true">
        <rtps>
            <builtin>
                <metatrafficUnicastLocatorList>
                    <locator/>
                </metatrafficUnicastLocatorList>
                <domainId>4</domainId>
                <initialPeersList> <!-- Add all address in the network for initial peer discovery-->
                    <locator>
                        <udpv4>
                                <address>192.168.2.101</address> <!-- husky1 -->
                        </udpv4>
                    </locator>
                    <locator>
                        <udpv4>
                                <address>192.168.2.201</address> <!-- base1 -->
                        </udpv4>
                    </locator>
                    <locator>
                        <udpv4>
                                <address>127.0.0.1</address>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
            <userTransports>
            <transport_id>veelpeers</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
    <publisher profile_name="publisher_profile" is_default_profile="true">
      <qos>
        <publishMode>
          <kind>ASYNCHRONOUS</kind>
        </publishMode>
        <reliability>
          <max_blocking_time> <!-- Period of allowable blocking time of publisher before throwing an error(Fast-RTPS 1.8.2) -->
            <sec>5</sec>
            <nanosec>10000000</nanosec>
          </max_blocking_time>
        </reliability>
      </qos>
      <throughputController> <!-- Flowcontroller -->
        <bytesPerPeriod>125000</bytesPerPeriod> <!-- Packet size in bytes that this controller will allow in a given period. -->
        <periodMillisecs>1000</periodMillisecs> <!-- Window of time in which no more than <bytesPerPeriod> bytes are allowed. -->
      </throughputController>
    </publisher>
    <subscriber profile_name="subscriber_profile" is_default_profile="true">
      <qos>
        <reliability>
          <max_blocking_time> <!-- Period of allowable blocking time of subscriber before throwing an error(Fast-RTPS 1.8.2) -->
            <sec>5</sec>
            <nanosec>10000000</nanosec>
          </max_blocking_time>
        </reliability>
      </qos>
    </subscriber>
</profiles>
```

### 3. Launch file configuration to bridge custom messages
To bridge custom type messages, we use [blob_tools](http://wiki.ros.org/blob_tools) to encode the message to CompressedImage message type.

For example, if we want to bridge a custom message from husky to the base station. Add these files in the launch files:

To encode the message, in husky.launch file:
```xml
  <include file="$(arg encode)">
    <arg name="robot_namespace" value="$(arg robot_namespace)"/>
    <arg name="type" value="pose_graph_msgs/PoseGraph"/>
    <arg name="input" value="lamp/pose_graph"/>
    <arg name="rate" value="0.1"/>  <!-- Hz -->
    <arg name="bandwidth" value="4000000"/>  <!-- Bytes -->
```

and to decode the message, in husky_base.launch file:
```xml
  <include file="$(arg decode)">
    <arg name="robot_namespace" value="$(arg robot_namespace)"/>
    <arg name="type" value="pose_graph_msgs/PoseGraph"/>
    <arg name="output" value="lamp/pose_graph"/>
  </include>
```
