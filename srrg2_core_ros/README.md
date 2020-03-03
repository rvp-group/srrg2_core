# Package `srrg2_core_ros`

This package contains all ROS-dependent components of `srrg2` core libraries and utilities, namely:

* `rosbag` <--> `BOSS` converters
* message reader/sorter/synchronizer (that operates directly on `rosbags`)
* core functionalities of our multi-process viewing system

## How to build
All our software is tested both with Ubuntu 18.04 and 16.04 (GCC 5 and 7), still the remaining of this guide refers to Ubuntu 18.04.

The steps are the following:

1. build the `srrg2_core` package following the `readme`
2. install ROS referring to the [official guide](http://wiki.ros.org/ROS/Installation) [**MANDATORY**]
3. link this package in `<srrg2_core_ws>/src`
4. build using Catkin (we recommend using `catkin build`)

## How to use
The message readers/sorter/synchronizer expose the same API as their base class in the `srrg2_core` package, therefore no explicit examples are provided here.
We provide two application to convert messages from/to `rosbag` to/from `BOSS`.
