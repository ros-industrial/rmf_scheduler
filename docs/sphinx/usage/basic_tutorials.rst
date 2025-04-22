Basic Tutorials
===============

Create a task using REST Endpoints
----------------------------------

.. note::

   Please make sure a RMF2 Scheduler Server is running.
   For more, checkout :ref:`Quick Start`.


Let's create a **Task** in the scheduler using the ``POST /schedule/edit`` API
and the ``TASK_ADD`` **ScheduleAction**.

This tutorial requires some utility command line tools.
Run the following command to install them.

.. code-block:: bash

   sudo apt install coreutils curl uuid-runtime jq


**Step 1 - Dry run**

Let's do a **dry-run** of the REST API query.
This checks if the query is valid.
No changes are made to the schedule stored.

Run the following cURL command.

.. code-block:: bash

   curl --location 'localhost:8000/schedule/edit' \
        --header 'Content-Type: application/json' \
        --data-raw '{
          "type": "TASK_ADD",
          "task": {
            "id": "'$(uuidgen)'",
            "start_time": "'$(date -u -Iseconds)'",
            "type": "rmf2/dummy"
          }
        }'

You should receive a successful response as follows.

::

    {"message":"Dry run successfully."}


**Step 2 - add task**

Let's run the command with **dry-run disabled**.
This command changes the schedule stored.
Simply append query parameter ``dry_run=false`` to the URL.

``localhost:8000/schedule/edit?dry_run=false``

The full cURL command is as follows.

.. code-block:: bash

   curl --location 'localhost:8000/schedule/edit?dry_run=false' \
        --header 'Content-Type: application/json' \
        --data-raw '{
          "type": "TASK_ADD",
          "task": {
            "id": "'$(uuidgen)'",
            "start_time": "'$(date -u -Iseconds)'",
            "type": "rmf2/dummy"
          }
        }'

Upon success, you should receive the following response.

::

    {"message":"Schedule updated successfully!"}


**Step 3 - Verification**

Let's verify the task we have created.

To retrieve the task we have created,
you can use the ``GET /schedule/`` API.

Simply run the following cURL command.

.. code-block:: bash

   curl -sS localhost:8000/schedule/ | jq .

You should receive a response similar to the following.

.. code-block:: bash

   {
     "tasks": [
       {
         "type": "rmf2/dummy",
         "start_time": "2025-04-21T10:28:10Z",
         "id": "d37b295a-6fbc-431c-a87d-ece7607c9f89",
         "status": ""
       }
     ],
     "processes": []
   }

.. note::

   RMF2 scheduler interprets and stores time in UTC timezone by default.
   The Time output follows the `ISO8601 format`__.

.. __: https://en.wikipedia.org/wiki/ISO_8601
