Installation
============

Install from Source
-------------------

Create a colcon workspace.

.. code-block:: bash

   export COLCON_WS=~/colcon_ws
   mkdir -p $COLCON_WS/src
   cd $COLCON_WS

Download the source code

.. code-block:: bash

   cd src
   git clone https://github.com/ros-industrial/rmf_scheduler.git --branch develop/v1

Install dependencies.

.. code-block:: bash

   rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

Build.

.. code-block:: bash

   cd ..
   colcon build
