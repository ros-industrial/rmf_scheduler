Basic Tutorials
===============

Create an event using ROS Topic
-------------------------------

.. note::

   Please make sure a RMF Scheduler ROS2 Node is running.
   For more, checkout :ref:`Quick Start`.


Let's create an **Event** in the scheduler using the ``/rmf_scheduler_api_request`` topic
and the ``add`` action.

This tutorial requires some utility command line tools.
Run the following command to install them.

.. code-block:: bash

   sudo apt install coreutils uuid-runtime jq


**Terminal 1 - Start Listener**

First let's listen to the response from the scheduler
from the ``/rmf_scheduler_api_response`` topic.

.. code-block:: bash

   ros2 topic echo --full-length /rmf_scheduler_api_responses


**Terminal 2 - Add Event**

Let's create an **Event** like the following.

.. code-block:: javascript

   {
     "type": "rmf/go_to_place",
     "description": "Test Event",
     "start_time": <current-time>,
     "event_details": {}
   }

Run the following command to add the Event.
This command changes the schedule stored.

.. code-block:: bash

   ros2 topic pub -1 /rmf_scheduler_api_requests rmf_task_msgs/msg/ApiRequest \
     '{
       "request_id": "'$(uuidgen)'",
       "json_msg": "{
         \"type\": \"add\",
         \"payload\": {
           \"events\": {
             \"'$(uuidgen)'\": {
               \"type\": \"rmf/go_to_place\",
               \"description\": \"Test Go to place\",
               \"start_time\": '$(date "+%s")',
               \"event_details\": {}
             }
           },
           \"dependencies\": {},
           \"series\": {}
         }
       }"
     }'

Upon success, you should receive the following response from **Terminal 1**.

::

    type: 0
    json_msg: '{"detail":"","value":0}'
    request_id: 2c3a5018-ae1a-4789-a568-bfbef24d1b78
    ---

In this response, ``"value": 0`` means ``SUCCESS``.

**Terminal 2 - Verification**

Let's verify the **Event** we have created.

To retrieve the **Event** we have created,
you can use the ``get`` action.

Simply run the following command.

.. code-block:: bash

   ros2 topic pub -1 /rmf_scheduler_api_requests rmf_task_msgs/msg/ApiRequest \
     '{
       "request_id": "'$(uuidgen)'",
       "json_msg": "{
         \"type\": \"get\",
         \"payload\": {}
       }"
     }'

You should receive a response similar to the following in **Terminal 1**.

::

    type: 0
    json_msg: '{"error_code":{"detail":"","value":0},"schedule":{"events":{"96210274-86e9-4992-a8e9-f3ae770108a1":{"description":"Test Go to place","event_details":{},"start_time":1745461792.0,"type":"rmf/go_to_place"}}}}'
    request_id: ad6c0091-ebdf-4770-864a-78f82c9e0e5e
    ---
