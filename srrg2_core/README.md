# Package `srrg2_core`

This package contains some essential libraries and utilities extensively used in
the `srrg2` packages. These include:
* base defines (2D/3D transforms, elementary linear algebra operations ...)
* point-cloud and basic point-cloud operations (voxelization, transformation, projection/unprojection ...)
* core data structures (Matrix, KT-Trees, Images ...)
* computing 2D distance maps for nearest neighbor search
* serialization/deserialization library (BOSS)
* module's parameter machinery (Property, Configurable, Configurable Manager)
* base system utilities (tic/toc, class profiler, shell colors, automatic command line parser)
* ROS messages conversions for the most common message types (odometry, LiDAR 2D/3D, Images ...)
* core functionalities for our custom viewing system

## How to build
The build requirements are rather minimal.

- Catkin build system
- OpenCV (tested on 3.3.1)
- Eigen (tested on Eigen3)
- SuiteSparse
- Miscellaneous deb packages

All our software is tested both with Ubuntu 18.04 and 16.04 (GCC 5 and 7), still the remaining of this guide refers to Ubuntu 18.04,

The required steps are the following:
1. install deb packages
```bash
sudo apt install \
  build-essential \
  libeigen3-dev \
  libsuitesparse-dev \
  ninja-build \
  libncurses5-dev \
  libwebsockets-dev \
  libreadline-dev \
  libudev-dev \
  libgtest-dev \
  arduino \
  arduino-mk \
  python-catkin-tools
```
2. install ROS following the [official guide](http://wiki.ros.org/ROS/Installation) [OPTIONAL]
3. create a new catkin workspace
4. link this package in `<your_catkin_ws>/src`
5. build (we suggest using `catkin build`)

## How to use
We also provide some self-explanatory examples of the tools included in this package.
These are located in the [`example`](src/examples) directory.
