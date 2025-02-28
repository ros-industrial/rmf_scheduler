# RMF Task Scheduler

[![support level: consortium / vendor](https://img.shields.io/badge/support%20level-consortium-brightgreen.svg)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)


Manages task schedules for RMF.

# Requirements

* ROS Humble

## RMF

More about RMF, including build instructions, can be found [here](https://github.com/open-rmf/rmf).

## Documentation

- [![sphinx-doc](https://shields.io/badge/General%20Doc-sphinx-blue)](https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/cag_p3/-/jobs/artifacts/main/file/docs/sphinx/build/html/index.html?job=sphinx-docs)

## Build Documentation

### Sphinx Documentation

Install python3 pip

```bash
sudo apt install python3-pip python3-dev
```

Install Sphinx

```bash
pip3 install --user --upgrade setuptools
pip3 install --user --upgrade sphinx sphinx-rtd-theme myst_parser recommonmark sphinxcontrib-jquery
```

Generate sphinx documentation

```bash
make -C docs/sphinx html
```

Open the documentation with your favourite web browser

```bash
firefox docs/sphinx/build/html/index.html
```

## Support

This repository is developed by ROS Industrial Consortium Asia Pacific

## Contributing
Guidelines on contributing to this repo can be found [here](CONTRIBUTING.md).
