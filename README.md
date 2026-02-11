# RMF2 Task Scheduler

[![CI](https://github.com/ros-industrial/rmf_scheduler/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/ros-industrial/rmf_scheduler/actions/workflows/build.yml)
[![doc](https://app.readthedocs.org/projects/rmf-scheduler/badge/?version=main)](https://app.readthedocs.org/projects/rmf-scheduler/)
[![codecov](https://codecov.io/gh/ros-industrial/rmf_scheduler/branch/main/graph/badge.svg?token=pKmw3Ifwft)](https://codecov.io/gh/ros-industrial/rmf_scheduler)

[![support level: consortium / vendor](https://img.shields.io/badge/support%20level-consortium-brightgreen.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)


Manages task schedules for RMF and RMF2.

## Requirements

* ROS 2 Humble
* ROS 2 Jazzy


## Documentation

See the [documentation](https://rmf-scheduler.readthedocs.io/en/main) on how to use it

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
git clone https://github.com/ros-industrial/rmf_scheduler.git --branch main
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

## Docker

### Docker Build
After cloning the repository, run the following commands in the same directory:
``` bash
cd ./rmf2_scheduler
docker build . -t rmf2_scheduler:local
```

### Docker Run
After the image is built, you can run and access the container using:
``` bash
docker run -it --net=host rmf2_scheduler:local bash
```

After which you can run the modules from the packages, for example `rmf2_scheduler_server_py`.
``` bash
rmf2_scheduler_server_py
```
The server should be accessible on your local device on `localhost:8000` as the command was ran with `--net=host`. You can open a browser and navigate to `http://localhost:8000/docs` to see if the swagger webpage is accessible.

## Support

This repository is developed by ROS Industrial Consortium Asia Pacific

## Contributing
Guidelines on contributing to this repo can be found [here](CONTRIBUTING.md).
