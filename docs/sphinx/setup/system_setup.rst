System Setup
============

Prerequisite
------------

* Compiler: GCC 11+, Clang 14+
* OS: Ubuntu 22.04+
* Supported ROS2 distros

  * Humble

  * Jazzy


Install ROS2
------------

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

