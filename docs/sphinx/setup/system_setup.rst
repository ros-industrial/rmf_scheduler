System Setup
============

System Requirement
------------------

* Ubuntu 22.04

* ROS2 Humble

* Cyclone DDS


Install ROS2 Humble
-------------------

Follow the `official documentation`__ to install the latest binary release of ROS2.

Install all necessary additional dependencies:

.. code-block:: bash

   sudo apt install -y python3-colcon-common-extensions \
                       python3-vcstool \
                       python3-rosdep

Remember to `initialize and update rosdep`__ if it is your first time installing ``rosdep``.

.. code-block:: bash

   sudo rosdep init
   rosdep update

__ https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
__ https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#how-do-i-use-the-rosdep-tool

.. note::

   :strong:`Append ROS2 environment in` ``.bashrc`` :strong:`(Optional)`

   To ensure that the ROS2 environment is sourced automatically when the terminal is started,
   append the following line to the end of the bash configuration (in ``~/.bashrc``):

   .. code-block:: bash

      source /opt/ros/humble/setup.bash

Install Cyclone DDS
-------------------

.. code-block:: bash

   sudo apt install ros-humble-rmw-cyclonedds-cpp

Add the following line to your ``~/.bashrc`` to always select Cyclone DDS by default.

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

Alternatively, follow the `official documentation`__ to build from source.

__ https://docs.ros.org/en/humble/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html
