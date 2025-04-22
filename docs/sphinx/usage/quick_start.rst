Quick Start
===========

The first step is to start the RMF2 Scheduler Server.

The RMF2 Scheduler provides a **Sample Python Server** for the user to try out.

.. note::

   The Sample Python Server requires ``fastapi>=0.101.0``.

   You can check the version using the following command

   .. code-block:: bash

      pip show fastapi

   For **Ubuntu 22.04** users,
   the system installed FastAPI needs to be upgraded using ``pip``.

   .. code-block:: bash

      pip install -U fastapi

   This is **NOT NEEDED** for **Ubuntu 24.04**.


To start the API server, run the following command.

.. code-block:: bash

   rmf2_scheduler_server_py

Other options include

::

    -h, --help         show this help message and exit
    --host HOST        Server Host
    --port PORT        Server Port
    --debug            Debug Mode

Next you can try out some of our :ref:`Basic Tutorials`.
