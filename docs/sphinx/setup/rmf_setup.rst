OpenRMF Setup
=============

Install Cyclone DDS
-------------------

.. code-block:: bash

   sudo apt install ros-humble-rmw-cyclonedds-cpp

Add the following line to your ``~/.bashrc`` to always select Cyclone DDS by default.

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

Alternatively, follow the `official documentation`__ to build from source.

__ https://docs.ros.org/en/humble/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html


Install OpenRMF from Source
---------------------------

.. warning::

   If you have any prior OpenRMF installation using Debian binaries (``sudo apt install``),
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

In the workspace set up from the instructions in README.md, pull in additional repositories using ``vcs``:

.. code-block:: bash

   cd $COLCON_WS
   wget https://raw.githubusercontent.com/ros-industrial/rmf_scheduler/refs/heads/main/rmf.repos
   vcs import src < rmf.repos

Ensure all ROS 2 prerequisites are fulfilled:

.. code-block:: bash

   cd $COLCON_WS
   source /opt/ros/humble/setup.bash
   rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -yr
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


