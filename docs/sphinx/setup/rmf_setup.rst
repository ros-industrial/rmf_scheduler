RMF Setup
=========

Install RMF from Source
-----------------------

.. warning::

   If you have any prior RMF installation using Debian binaries (``sudo apt install``),
   please use the following command to remove them.

   .. code-block:: bash

      sudo apt purge ros-humble-rmf*

Install ROSDEP
``````````````

``rosdep`` helps install dependencies for ROS packages across various distributions.
It can be installed with:

.. code-block:: bash

   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update

Download the source code
````````````````````````

Setup a new ROS 2 workspace and pull in the demo repositories using ``vcs``,

.. note::

   You will need a `personal access token`__ to download the ``.repos`` file.
   Otherwise, download the ``.repos`` file manually `here`__.

__ https://docs.gitlab.com/ee/user/profile/personal_access_tokens.html#create-a-personal-access-token
__ https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/cag_p3/-/blob/main/.docker/rmf-sim.repos

.. code-block:: bash

   mkdir -p ~/rmf_ws/src
   cd ~/rmf_ws
   curl --header "PRIVATE-TOKEN: <your-access-token>" "https://gitlab.com/api/v4/projects/45233178/repository/files/rmf.repos/raw?ref=0.2.0" | tee rmf.repos
   vcs import src < rmf.repos

Ensure all ROS 2 prerequisites are fulfilled,

.. code-block:: bash

   cd ~/rmf_ws
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

Setup colcon mixin

.. code-block:: bash

   sudo apt install python3-colcon-mixin
   colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
   colcon mixin update default


Compiling Instructions
``````````````````````

On ``Ubuntu 22.04``:

.. code-block:: bash

   cd ~/rmf_ws
   source /opt/ros/humble/setup.bash
   export CXX=clang++
   export CC=clang
   colcon build --mixn release lld

.. note::

   The first time the build occurs, many simulation models will be downloaded from Ignition Fuel to populate the scene when the simulation is run.
   As a result, the first build can take a very long time depending on the server load and your Internet connection.
