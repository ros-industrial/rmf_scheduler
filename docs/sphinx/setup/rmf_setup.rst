RMF Setup
=========

Install RMF from Source
-----------------------

.. warning::

   If you have any prior RMF installation using Debian binaries (``sudo apt install``),
   please use the following command to remove them.

   .. code-block:: bash

      sudo apt purge ros-humble-rmf*

Install rosdep
``````````````

``rosdep`` helps install dependencies for ROS packages across various distributions.
It can be installed with:

.. code-block:: bash

   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update

Download the source code
````````````````````````

Create a colcon workspace:

.. code-block:: bash

   export COLCON_WS=~/colcon_ws
   mkdir -p $COLCON_WS/src
   cd $COLCON_WS

Download the source code:

.. code-block:: bash

   cd src
   git clone https://github.com/ros-industrial/rmf_scheduler.git

In the workspace pull in additional RMF repositories using ``vcs``:

.. code-block:: bash

   vcs import . < rmf_scheduler/rmf.repos

Ensure all ROS 2 prerequisites are fulfilled:

.. code-block:: bash

   source /opt/ros/humble/setup.bash
   rosdep install --from-paths . --ignore-src --rosdistro "$ROS_DISTRO" -yr
   sudo apt install clang lldb lld
   sudo apt install python3-pip python3-setuptools python3-dev
   python3 -m pip install -U \
      datamodel-code-generator \
      websockets \
      fastapi \
      flask-socketio \
      uvicorn

Setup colcon mixin:

.. code-block:: bash

   sudo apt install python3-colcon-mixin
   colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
   colcon mixin update default


Compiling Instructions
``````````````````````

On ``Ubuntu 22.04``:

.. code-block:: bash

   cd $COLCON_WS
   source /opt/ros/humble/setup.bash
   export CXX=clang++
   export CC=clang
   colcon build --mixn release lld

.. note::

   The first time the build occurs, many simulation models will be downloaded from Ignition Fuel to populate the scene when the simulation is run.
   As a result, the first build can take a very long time depending on the server load and your Internet connection.
