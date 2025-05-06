# RMF Task Scheduler

[![CI](https://github.com/ros-industrial/rmf_scheduler/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/ros-industrial/rmf_scheduler/actions/workflows/build.yml)
[![doc](https://app.readthedocs.org/projects/rmf-scheduler/badge/?version=latest)](https://app.readthedocs.org/projects/rmf-scheduler/)
[![codecov](https://codecov.io/gh/ros-industrial/rmf_scheduler/branch/main/graph/badge.svg?token=pKmw3Ifwft)](https://codecov.io/gh/ros-industrial/rmf_scheduler)

[![support level: consortium / vendor](https://img.shields.io/badge/support%20level-consortium-brightgreen.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

Manages task schedules for Open-RMF.

> This version (`0.2.x`) serves as a proof-of-concept.
> A future stable version (`1.0.0`) is under development within the RMF2 project,
> the progress can be found under [**develop/v1**](https://github.com/ros-industrial/rmf_scheduler/tree/develop/v1).
> For more info, please checkout <https://github.com/ros-industrial/rmf_scheduler/issues/9>.

## Requirements

* ROS 2 Humble
* Open-RMF (More information [here](https://github.com/open-rmf/rmf))

## Documentation

Refer to the [documentation](https://rmf-scheduler.readthedocs.io/) for usage instructions.

## Quick Setup

Full setup instructions with Open-RMF can be found in the documentation above.
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
