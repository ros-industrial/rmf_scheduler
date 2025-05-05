# RMF Task Scheduler

[![support level: consortium / vendor](https://img.shields.io/badge/support%20level-consortium-brightgreen.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

Manages task schedules for OpenRMF.

## Requirements

* ROS 2 Humble
* OpenRMF (More information [here](https://github.com/open-rmf/rmf))

## Documentation

Refer to the [documentation](https://rmf-scheduler.readthedocs.io/) for usage instructions.

## Quick Setup

Full setup instructions with OpenRMF can be found in the documentation above.  
Create a colcon workspace.

```bash
export COLCON_WS=~/colcon_ws
mkdir -p $COLCON_WS/src
cd $COLCON_WS
```

Download the source code.

```bash
cd src
git clone https://github.com/ros-industrial/rmf_scheduler.git
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

## Support

This repository is developed by ROS Industrial Consortium Asia Pacific

## Contributing
Guidelines on contributing to this repo can be found [here](CONTRIBUTING.md).
