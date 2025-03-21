import pika, sys, os

import json
from xml.etree import ElementTree as ET
from time import sleep
from threading import Thread, Lock
from functools import partial

class Listener:

    def __init__(self):
        self._connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self._channel = self._connection.channel()
        self._channel.exchange_declare(exchange='@RECEIVE@', exchange_type='fanout', durable=True)

        result = self._channel.queue_declare(queue='', exclusive=True)
        queue_name = result.method.queue
        # to_queue = "@RECEIVE@-event_mgr"

        self._channel.queue_bind(exchange='@RECEIVE@', queue=queue_name)
        self._channel.basic_consume(queue=queue_name, on_message_callback=self.callback, auto_ack=True)
        self._lk = Lock()
        self._terminated = True
        self._spin_thread = None
        self._worker_threads = []

    def run_task(self, task_id: str):
        obj = {}
        obj["id"] = f"urn:{task_id}"
        obj["type"] = "TaskStatus"
        obj["status"] = ""
        obj["taskType"] = ""

        task_connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        task_channel = task_connection.channel()

        obj["status"] = "IN_PROGRESS"
        in_progress_str = json.dumps(obj)
        print(f"starting task {task_id}")
        task_channel.basic_publish(
            exchange='@RECEIVE@',
            routing_key="",
            body=in_progress_str,
            properties=pika.spec.BasicProperties(
                content_type="application/json"
            )
        )

        sleep(2)
        print(f"task ongoing {task_id}")
        task_channel.basic_publish(
            exchange='@RECEIVE@',
            routing_key="",
            body=in_progress_str,
            properties=pika.spec.BasicProperties(
                content_type="application/json"
            )
        )

        sleep(5)
        obj["status"] = "COMPLETED"
        completed_str = json.dumps(obj)
        print(f"task completed {task_id}")
        task_channel.basic_publish(
            exchange='@RECEIVE@',
            routing_key="",
            body=completed_str,
            properties=pika.spec.BasicProperties(
                content_type="application/json"
            )
        )

    def callback(self, ch, method, properties, body):
        message_j = json.loads(body)
        if message_j['type'] == "Schedule":
            payload_j = message_j["payload"]
            bt_tree = ET.fromstring(payload_j)
            for child in bt_tree.iter('SubTree'):
                task_id = child.attrib["task_id"]
                work_thread = Thread(target=partial(self.run_task, task_id))
                work_thread.start()
                self._worker_threads.append(
                    work_thread
                )

        # print(f" [x] {payload_j}")

    def start(self):
        self._terminated = False
        self._spin_thread = Thread(target=self._spin)
        self._spin_thread.start()

    def stop(self):
        with self._lk:
            self._terminated = True

        if self._spin_thread is not None:
            self._spin_thread.join()

        self._connection.close()

        # Wait for worker thread to complete
        if self._worker_threads:
            for thread in self._worker_threads:
                thread.join()


    def _spin(self):
        print(' [*] Waiting for messages. To exit press CTRL+C')
        while(self._ok()):
            self._connection.process_data_events()
            self._connection.sleep(1)

    def _ok(self):
        ok = False
        with self._lk:
            ok = not self._terminated

        return ok



if __name__ == '__main__':
    listener = Listener()
    try:
        listener.start()
        while(True):
            sleep(1)
    except KeyboardInterrupt:
        print('Interrupted')
        listener.stop()
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)
