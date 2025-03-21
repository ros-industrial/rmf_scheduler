# Copyright 2024-2025 ROS Industrial Consortium Asia Pacific
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import List
import json

import pika

from rmf2_scheduler import (
    SchedulerOptions,
    ExecutorData,
    TaskExecutor,
    ProcessExecutor,
    TaskExecutorManager
)
from rmf2_scheduler.data import Task, Process
from rmf2_scheduler.utils import TreeConversion
from threading import Thread, Lock


class TOProcessExecutor(ProcessExecutor):
    def __init__(
        self,
        task_executor_manager: TaskExecutorManager,
        amqp_host: str = "localhost",
        amqp_port: int = 5672,
        amqp_exchange: str =  "@RECEIVE@",
        to_queue: str = "@RECEIVE@-event_mgr",
    ):
        super().__init__()
        self._task_executor_manager = task_executor_manager
        self._process_pushed = {}
        self._amqp_exchange = amqp_exchange
        self._to_queue = to_queue

        # Initialize AMPQ
        self._connection = pika.BlockingConnection(
            pika.ConnectionParameters(
                host=amqp_host,
                port=amqp_port
            )
        )

        self._channel = self._connection.channel()

        # Configure AMQP
        self._channel.exchange_declare(
            exchange=amqp_exchange,
            exchange_type="fanout",
            durable=True
        )
        self._amqp_listener_queue = ""


        self._publish_connection = pika.BlockingConnection(
            pika.ConnectionParameters(
                host=amqp_host,
                port=amqp_port
            )
        )
        self._publish_channel = self._publish_connection.channel()

        self._amqp_lock = Lock()
        self._pending_tasks = {}
        self._terminated = True
        self._spin_thread = None

    # Override
    def run_async(
        self,
        process: Process,
        tasks: List[Task]
    ):
        if process.id in self._process_pushed:
            return False, f"[{process.id}] already pushed"

        # print(process.json())
        self._process_pushed[process.id] = process

        for task in tasks:
            if not self._task_executor_manager.is_runnable(task.type):
                return False, f"[{task.id}]'s type [{task.type}] is not runnable."

        task_map = { task.id: task for task in tasks }
        bt_id = "urn:" + process.id
        bt_xml = TreeConversion().convert_to_tree(
            bt_id, process.graph,
            lambda task_id: self._generate_bt_node(task_map[task_id])
        )

        # strip </>
        bt_xml = bt_xml.replace("</>", "")
        print(bt_xml)

        with self._amqp_lock:
            # Add task to listener
            for task in tasks:
                self._pending_tasks[task.id] = task

            # Send task to amqp
            print("Sending BT")
            self._send_bt_amqp(bt_id, bt_xml)
            print("BT sent successfully")

        return True, None


    def start_listener(self):
        # Create a listener queue
        result = self._channel.queue_declare(queue='', exclusive=True)
        listener_queue = result.method.queue
        print(listener_queue)

        self._channel.queue_bind(
            exchange=self._amqp_exchange,
            queue=listener_queue
        )
        self._channel.basic_consume(
            queue=listener_queue,
            on_message_callback=self._on_message,
            auto_ack=True
        )

        self._terminated = False

        # start the spinner
        self._spin_thread = Thread(target=self._spin)
        self._spin_thread.start()

    def stop_listener(self):
        with self._amqp_lock:
            self._terminated = True

        if self._spin_thread is not None:
            self._spin_thread.join()
            self._channel.stop_consuming()
            self._connection.close()

    def _spin(self):
        while(self._ok()):
            self._connection.process_data_events()
            self._connection.sleep(1)

    def _ok(self):
        ok = False

        with self._amqp_lock:
            ok = not self._terminated

        return ok

    def _on_message(
        self,
        channel,
        method,
        properties,
        body
    ):
        try:
            body_j = json.loads(body)
        except Exception as e:
            print(str(e))
            return

        if "type" not in body_j:
            print("No type found")
            return

        message_type = body_j["type"]
        if message_type == "Schedule":
            # ignore
            return

        if message_type == "TaskStatus":
            print("TaskStatus:")
            print(body_j)
            task_id = str(body_j['id'])
            if task_id.startswith("urn:ngsi-ld:Task:urn"):
                task_id = task_id.lstrip("urn:ngsi-ld:Task:urn")[:36]
            elif task_id.startswith("urn:ngsi-ld:Task"):
                task_id = task_id.lstrip("urn:ngsi-ld:Task")[:36]

            action = None
            task_to_start = Task()
            with self._amqp_lock:
                if task_id in self._pending_tasks and body_j['status'] == "IN_PROGRESS":
                    action = "start"
                    task_to_start = self._pending_tasks[task_id]
                    del self._pending_tasks[task_id]
                elif body_j['status'] == "COMPLETED":
                    action = "completed"


            if action == "start":
                self._task_executor_manager.run_async(task_to_start)
            elif action == "completed":
                data = ExecutorData()
                self._task_executor_manager.notify_completion(task_id, True, data)


    def _generate_bt_node(self, task):
        _, _, data = self._task_executor_manager.dry_run(task);
        return data.get_data_as_string()

    def _send_bt_amqp(self, bt_id: str, bt_xml: str):
        obj = {}
        obj["id"] = bt_id
        obj["type"] = "Schedule"
        obj["scheduleType"] = "xml"
        obj["taskTime"] = ""
        obj["payload"] = bt_xml

        obj_str = json.dumps(obj)

        self._publish_channel.basic_publish(
            exchange=self._amqp_exchange,
            routing_key="",
            body=obj_str,
            properties=pika.spec.BasicProperties(
                content_type="application/json"
            )
        )
