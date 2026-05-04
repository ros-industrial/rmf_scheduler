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

import argparse
from datetime import datetime
import json
from typing import List
from uuid import uuid4

import requests
from rmf2_scheduler.data import action_type, Duration, Graph, ScheduleAction, Task, Time
from rmf2_scheduler.fastapi.schemas import Dependency as DependencySchema
from rmf2_scheduler.fastapi.schemas import Edge as EdgeSchema
from rmf2_scheduler.fastapi.schemas import ProcessPayload as ProcessPayloadSchema
from rmf2_scheduler.fastapi.schemas import ScheduleAction as ScheduleActionSchema


def is_not_null_or_unset(value):
    return value is not None and value != ''


def remove_nulls(value):
    if isinstance(value, dict):
        return {k: remove_nulls(v) for k, v in value.items() if is_not_null_or_unset(v)}
    elif isinstance(value, list):
        return [remove_nulls(item) for item in value if is_not_null_or_unset(item)]
    else:
        return value


def generate_2_tasks(
    agf: str,
    agf_wp_s: str,
    agf_wp_e: str,
    warehouse_machine: str,
    start_time: Time
):
    process_id = str(uuid4())
    task_ids = [str(uuid4()) for i in range(2)]

    actions = []

    task1 = Task()
    task1.id = task_ids[0]
    task1.type = 'custom/go_to_amr'
    task1.start_time = start_time
    task1.resource_id = agf
    task1.task_details = {
        'coordinates': agf_wp_s
    }

    task2 = Task()
    task2.id = task_ids[1]
    task2.type = 'custom/warehouse_task'
    task2.start_time = start_time
    task2.resource_id = warehouse_machine

    graph = Graph()

    for task in [task1, task2]:
        action = ScheduleAction()
        action.type = action_type.TASK_ADD
        action.task = task

        actions.append(remove_nulls(action.json()))

        graph.add_node(task.id)

    graph.add_edge(task1.id, task2.id)

    dependencies: List[DependencySchema] = []
    for item in graph.json():
        edges: List[EdgeSchema] = []
        for e in item['needs']:
            edge = EdgeSchema(id=e['id'], type_=e['type'])
            edges.append(edge)
        dependencies.append(DependencySchema(id=item['id'], needs=edges))

    process = ProcessPayloadSchema(
        graph=dependencies, id=process_id, process_details={}
    )

    action = ScheduleActionSchema(type=action_type.PROCESS_ADD, process=process)
    actions.append(json.loads(action.model_dump_json()))
    print(json.dumps(actions, indent=2))
    return actions


def apply_batch_edit(url: str, batch_edit_json):
    # Send JSON
    try:
        response = requests.post(url, params=[('dry_run', False)], json=batch_edit_json)
    except requests.ConnectionError as e:
        print('\nConnection Error, is RMF2 Scheduler up?')
        print(str(e))
        return False

    if not response.ok:
        print('\nUnable to send request to RMF2 Scheduler, is the URL correct?')
        print(f'{response.status_code} - {response.reason}')
        print(f'{response.text}')
        return False

    success = True
    try:
        response_json = response.json()
        if response_json['message'] != 'Schedule updated successfully!':
            success = False
    except requests.JSONDecodeError:
        success = False

    if not success:
        print('\nUnable to send request to RMF2 Scheduler, is the URL correct?')
        print(f'{response.status_code} - {response.reason}')
        print(f'{response.text}')
        return False

    return True


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser()

    parser.add_argument('--host', type=str, default='localhost',
                        help='Server Host')
    parser.add_argument('--port', type=int, default=8000,
                        help='Server Port')
    parser.add_argument('-n', '--num-loops', type=int, default=1)
    args = parser.parse_args()

    rs_url = f'http://{args.host}:{args.port}/schedule/edit/batch'
    print(f'RMF2 Scheduler batch edit endpoint <{rs_url}>')

    now = Time(datetime.now())
    now += Duration.from_seconds(60*60*24)

    # ORDER 1
    print('', end='')
    for loop_idx in range(args.num_loops):
        start_time = now + Duration.from_seconds(loop_idx * 10 * 60)

        batch_edit_json = []
        print(f'\rSending Order 1 [{loop_idx + 1}/{args.num_loops}]', end='')
        for i in range(8):
            batch_edit_json += generate_2_tasks(
                agf=f'agf_{i + 1}',
                agf_wp_s=f'agf_incoming_s[{i}]',
                agf_wp_e=f'agf_incoming_e[{i}]',
                warehouse_machine=f'TS1_{i + 1}',
                start_time=start_time
            )

        if not apply_batch_edit(rs_url, batch_edit_json):
            return 1

    # New line
    print('')
    print('Orders created successfully')


if __name__ == '__main__':
    main()
