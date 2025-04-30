# RMF2 Task Scheduler

[![CI](https://github.com/ros-industrial/rmf_scheduler/actions/workflows/build.yml/badge.svg?branch=develop/v1)](https://github.com/ros-industrial/rmf_scheduler/actions/workflows/build.yml)
[![doc](https://app.readthedocs.org/projects/rmf-scheduler/badge/?version=develop-v1)](https://app.readthedocs.org/projects/rmf-scheduler/)
[![codecov](https://codecov.io/gh/ros-industrial/rmf_scheduler/branch/develop/v1/graph/badge.svg?token=pKmw3Ifwft)](https://codecov.io/gh/ros-industrial/rmf_scheduler)

[![support level: consortium / vendor](https://img.shields.io/badge/support%20level-consortium-brightgreen.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)


Manages task schedules for RMF and RMF2.

## Requirements

* ROS 2 Humble
* ROS 2 Jazzy


## Documentation

See the [documentation](https://rmf-scheduler.readthedocs.io/en/develop-v1) on how to use it

## Quick Setup

Create a colcon workspace.

```bash
export COLCON_WS=~/colcon_ws
mkdir -p $COLCON_WS/src
cd $COLCON_WS
```

Download the source code.

```bash
cd src
git clone https://github.com/ros-industrial/rmf_scheduler.git --branch develop/v1
```

Install dependencies.

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Build.
```bash
cd ..
colcon build
```

### Quick Demo

- [Python API Server Demo](./rmf2_scheduler_server_py)

## Support

This repository is developed by ROS Industrial Consortium Asia Pacific

## Contributing
Guidelines on contributing to this repo can be found [here](CONTRIBUTING.md).
