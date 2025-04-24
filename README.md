# RMF Task Scheduler

[![CI](https://github.com/ros-industrial/rmf_scheduler/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/ros-industrial/rmf_scheduler/actions/workflows/build.yml)
[![doc](https://app.readthedocs.org/projects/rmf-scheduler/badge/?version=latest)](https://app.readthedocs.org/projects/rmf-scheduler/)
[![codecov](https://codecov.io/gh/ros-industrial/rmf_scheduler/branch/main/graph/badge.svg?token=pKmw3Ifwft)](https://codecov.io/gh/ros-industrial/rmf_scheduler)

[![support level: consortium / vendor](https://img.shields.io/badge/support%20level-consortium-brightgreen.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)


Manages task schedules for RMF.

## Requirements

* ROS 2 Humble
* RMF (More information [here](https://github.com/open-rmf/rmf))

## Quick Setup

Full setup instructions can be found in the Sphinx documentation.  
Create a colcon workspace and download the source code:

```bash
export COLCON_WS=~/colcon_ws
mkdir -p $COLCON_WS/src
cd $COLCON_WS/src
git clone git@github.com:ros-industrial/rmf_scheduler.git
```

Install dependencies.
```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Build.
```bash
cd $COLCON_WS
colcon build
```

## Documentation

### Sphinx Documentation

Install dependencies.

```bash
sudo apt install python3-pip python3-dev
pip3 install --user --upgrade setuptools
pip3 install --user --upgrade sphinx sphinx-rtd-theme myst_parser recommonmark sphinxcontrib-jquery
```

Generate the documentation.

```bash
cd $COLCON_WS/src/rmf_scheduler/
make -C docs/sphinx html
```

Open the documentation with your favourite web browser.

```bash
firefox docs/sphinx/build/html/index.html
```

## Support

This repository is developed by ROS Industrial Consortium Asia Pacific

## Contributing
Guidelines on contributing to this repo can be found [here](CONTRIBUTING.md).
