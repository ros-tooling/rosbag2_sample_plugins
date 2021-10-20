# Rosbag2 Sample Plugins

This repository contains supplementary materials for the ROSWorld2021 talk "rosbag2 for power users" by Emerson Knapp and Adam Dabrowski.

Packages in this repository:
* `rosbag2_sample_plugins`: a metapackage containing demonstrations of these sample plugins
* `rosbag2_compression_zlib`: a Rosbag2 compression plugin implemented using the ubiquitous `zlib` compression library
* `rosbag2_converter_yaml`: a Rosbag2 serialization format converter plugin that serializes messages as YAML using dynamic typesupport introspection
* `rosbag2_storage_simplefile`: a Rosbag2 storage plugin implementing a bag file as a very simple unindexed binary format

## Getting started

This branch is kept up to date with ROS Rolling Ridley. Backports to ROS 2 distros Galactic and Foxy are in the works.

* First, follow the general instructions to install Rolling in the environment of your choice.
* Then, create a `colcon` workspace containing the `workspace.repos` repositories

```
mkdir rosbag2_sample_ws
cd rosbag2_sample_ws
mkdir src
wget https://raw.githubusercontent.com/ros-tooling/rosbag2_sample_plugins/master/workspace.repos
vcs import src < workspace.repos
```

* `colcon build` - this will build everything in the workspace


NOTE: the repos file is mainly necessary for the `dynamic_message_introspection` package version used for YAML serialization, which is not yet released to Rolling. We are also building rosbag2 from source to get a few small tweaks that are in the process of being backported. In a future update of this demo, only this repository will be needed.

## Running demos

```
# Start viewer and data publisher (separate shells)
rviz2 -d $(ros2 pkg prefix rosbag2_sample_plugins)/config/sample.rviz
ros2 run rosbag2_sample_plugins pub_sample_data

# Look in RViz, you can see the fake odometry

# Put bags in some common location
mkdir data
cd data
```

# Demos

Use the following commands to try out the various plugin types (building on top of one another)

```
# Storage
ros2 bag record --storage simplefile -o simple_chat /chat

# Converter
ros2 bag record --storage simplefile --serialization-format yaml -o simple_odom_yaml /odom

# Compression
ros2 bag record --storage simplefile --serialization-format yaml --compression-format zlib --compression-mode file -o simple_odom_yaml_zlib /odom
```

# Explore the outputs

Look at binary format - you can see the topic names, text payloads, and metadata:

```
vim simple_chat/simple_chat0.simple
```

Now you can see the serialized YAML in the format

```
vim simple_odom_yaml/simple_odom_yaml0.simple
```

Check out how much smaller the compressed file is:

```
ls -hal ./*
```

Playback a bag - take a look at the rviz

```
ros2 bag play --storage simplefile simple_odom_yaml_zlib
```
