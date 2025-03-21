# Copyright 2025 ROS Industrial Consortium Asia Pacific
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

from datetime import datetime
from functools import partial
import json
from typing import List, Optional, Tuple
from uuid import uuid4

import amqp
from numpy.random import normal

from rmf2_scheduler.cache import Action, ActionPayload, ScheduleCache
from rmf2_scheduler.data import (
    action_type,
    Duration,
    Edge,
    Process,
    Task,
    Time,
    TimeWindow,
)
from rmf2_scheduler.storage import ScheduleStream
from rmf2_scheduler.utils import TreeConversion


def _add_tasks_and_processes(
    tasks: List[Task], processes: List[Process], cache: ScheduleCache
) -> Optional[ScheduleCache]:
    for task in tasks:
        # Add task
        task_action = Action.create(action_type.TASK_ADD, ActionPayload().task(task))
        result, error = task_action.validate(cache)
        if not result:
            print(error)
            return None

        task_action.apply()

    for process in processes:
        # Add process
        process_add_action = Action.create(
            action_type.PROCESS_ADD, ActionPayload().process(process)
        )
        result, error = process_add_action.validate(cache)
        if not result:
            print(error)
            return None

        process_add_action.apply()

        # Update process start time
        process_update_start_time_action = Action.create(
            action_type.PROCESS_UPDATE_START_TIME, ActionPayload().id(process.id)
        )
        result, error = process_update_start_time_action.validate(cache)
        if not result:
            print(error)
            return None

        process_update_start_time_action.apply()

    return cache


def generate_preset_manufacturing_tasks(
    mm_name: str,
    mm_id: str,
    mm_base_type: str,
    mm_arm_type: str,
    workcell_name: str,
    workcell_id: str,
    workcell_operations: [str],
    move_duration: float,
    dock_durations: Tuple[float, float],
    workcell_operations_durations: List[float],
    start_time: Time,
    status: str,
) -> List[Task]:
    dock_duration, undock_duration = dock_durations

    # MM move to workcell
    task1 = Task()
    task1.id = str(uuid4())
    task1.type = 'rmf2/go_to_place'
    task1.start_time = start_time
    task1.status = status
    task1.description = f'{mm_name} moves to {workcell_name}'
    task1.duration = Duration.from_seconds(move_duration)
    task1.resource_id = mm_id
    task1.task_details = {
        'end_location': workcell_id,
    }

    # MM dock to workcell
    task2 = Task()
    task2.id = str(uuid4())
    task2.type = 'dsvc/workcell_docking'
    task2.start_time = start_time
    task2.status = status
    task2.description = f'{mm_name} docks to {workcell_name}'
    task2.duration = Duration.from_seconds(dock_duration)
    task2.resource_id = mm_id
    task2.task_details = {
        'workcell_id': workcell_id,
        'mm_command_type': mm_base_type,
        'workflow_type': 'dock',
    }

    # Workcell operation
    workcell_tasks = []
    for i in range(len(workcell_operations)):
        task = Task()
        task.id = str(uuid4())
        task.type = 'dsvc/workcell_generic'
        task.start_time = start_time
        task.status = status
        task.description = f'{mm_name} {workcell_operations[i]} at {workcell_name}'
        task.duration = Duration.from_seconds(workcell_operations_durations[i])
        task.resource_id = mm_id
        task.task_details = {
            'workcell_id': workcell_id,
            'mm_command_type': mm_arm_type,
            'workflow_type': workcell_operations[i],
        }
        workcell_tasks.append(task)

    # Workcell undocking
    task3 = Task()
    task3.id = str(uuid4())
    task3.type = 'dsvc/workcell_docking'
    task3.start_time = start_time
    task3.status = status
    task3.description = f'{mm_name} undock from {workcell_name}'
    task3.duration = Duration.from_seconds(undock_duration)
    task3.resource_id = mm_id
    task3.task_details = {
        'workcell_id': workcell_id,
        'mm_command_type': mm_base_type,
        'workflow_type': 'undock',
    }

    return [task1, task2] + workcell_tasks + [task3]


def generate_preset_sequence1(
    start_time: Time, status: str
) -> Tuple[List[Task], Process]:
    # Parts Station
    move_to_parts_station_durations = normal(300, 10, 1)
    dock_to_parts_station_durations = normal(60, 2, 2)
    pick_at_parts_station_durations = normal(60, 2, 1)
    parts_station_tasks = generate_preset_manufacturing_tasks(
        'MM 01',
        'MM_01',
        'MIR',
        'UR10',
        'Parts Station',
        'parts_station',
        ['pick'],
        move_to_parts_station_durations[0],
        (dock_to_parts_station_durations[0], dock_to_parts_station_durations[1]),
        pick_at_parts_station_durations.tolist(),
        start_time,
        status,
    )

    # CNC
    move_to_cnc_durations = normal(300, 10, 1)
    dock_to_cnc_durations = normal(60, 2, 2)
    pick_at_cnc_durations = normal(60, 2, 2)
    cnc_tasks = generate_preset_manufacturing_tasks(
        'MM 01',
        'MM_01',
        'MIR',
        'UR10',
        'CNC Machine',
        'CNC',
        ['place', 'pick'],
        move_to_cnc_durations[0],
        (dock_to_cnc_durations[0], dock_to_cnc_durations[1]),
        pick_at_cnc_durations.tolist(),
        start_time,
        status,
    )

    # Robot Cell
    move_to_robot_cell_durations = normal(300, 10, 1)
    dock_to_robot_cell_durations = normal(60, 2, 2)
    pick_at_robot_cell_durations = normal(60, 2, 1)
    robot_cell_tasks = generate_preset_manufacturing_tasks(
        'MM 01',
        'MM_01',
        'MIR',
        'UR10',
        'Robot Cell',
        'robot_cell',
        ['place'],
        move_to_robot_cell_durations[0],
        (dock_to_robot_cell_durations[0], dock_to_robot_cell_durations[1]),
        pick_at_robot_cell_durations.tolist(),
        start_time,
        status,
    )

    all_tasks = parts_station_tasks + cnc_tasks + robot_cell_tasks

    # Process
    process = Process()
    process.id = str(uuid4())
    for task in all_tasks:
        process.graph.add_node(task.id)

    for i in range(1, len(all_tasks)):
        process.graph.add_edge(all_tasks[i - 1].id, all_tasks[i].id)

    return all_tasks, process


def generate_preset_sequence2(
    start_time: Time, status: str
) -> Tuple[List[Task], Process]:
    # CMM
    move_to_cmm_durations = normal(300, 10, 1)
    dock_to_cmm_durations = normal(60, 2, 2)
    pick_at_cmm_durations = normal(60, 2, 2)
    cmm_tasks = generate_preset_manufacturing_tasks(
        'MM 02',
        'MM_02',
        'MIR',
        'UR10',
        'CMM Machine',
        'CMM',
        ['place', 'pick'],
        move_to_cmm_durations[0],
        (dock_to_cmm_durations[0], dock_to_cmm_durations[1]),
        pick_at_cmm_durations.tolist(),
        start_time,
        status,
    )

    # Robot Cell
    move_to_robot_cell_durations = normal(300, 10, 1)
    dock_to_robot_cell_durations = normal(60, 2, 2)
    pick_at_robot_cell_durations = normal(60, 2, 1)
    robot_cell_tasks = generate_preset_manufacturing_tasks(
        'MM 02',
        'MM_02',
        'MIR',
        'UR10',
        'Robot Cell',
        'robot_cell',
        ['place'],
        move_to_robot_cell_durations[0],
        (dock_to_robot_cell_durations[0], dock_to_robot_cell_durations[1]),
        pick_at_robot_cell_durations.tolist(),
        start_time,
        status,
    )

    all_tasks = cmm_tasks + robot_cell_tasks

    # Process
    process = Process()
    process.id = str(uuid4())
    for task in all_tasks:
        process.graph.add_node(task.id)

    for i in range(1, len(all_tasks)):
        process.graph.add_edge(all_tasks[i - 1].id, all_tasks[i].id)

    return all_tasks, process


def generate_manufacturing_tasks(start_time: Time) -> Optional[ScheduleCache]:
    status = 'queued'

    tasks1, process1 = generate_preset_sequence1(start_time, status)
    tasks2, process2 = generate_preset_sequence2(start_time, status)

    cache = ScheduleCache()

    tasks = tasks1 + tasks2
    processes = [process1, process2]
    return _add_tasks_and_processes(tasks, processes, cache)


def to_bt_node(task_id: str, cache: ScheduleCache):
    task = cache.get_task(task_id)
    if task.type == 'rmf2/go_to_place':
        return (
            'SubTree ID="ReplaceMAPF" '
            + f"task_id=\"{'urn:ngsi-ld:Task:' + task.id}\" "
            + f'asset_name="{task.resource_id}" '
            + f"coordinates=\"{task.task_details['end_location']}\" "
            + 'connection="{connection}"'
        )

    if task.type == 'dsvc/workcell_docking':
        return (
            'SubTree ID="WORKCELLDocking" '
            + f"task_id=\"{'urn:ngsi-ld:Task:' + task.id}\" "
            + f"asset_name=\"{task.resource_id}_{task.task_details['mm_command_type']}\" "
            + f"workcell_name=\"{task.task_details['workcell_id']}\" "
            + f"workflow=\"{task.task_details['workflow_type']}\" "
            + 'connection="{connection}"'
        )

    if task.type == 'dsvc/workcell_generic':
        return (
            'SubTree ID="WORKCELLGeneric" '
            + f"task_id=\"{'urn:ngsi-ld:Task:' + task.id}\" "
            + f"asset_name=\"{task.resource_id}_{task.task_details['mm_command_type']}\" "
            + f"workcell_name=\"{task.task_details['workcell_id']}\" "
            + f"workflow=\"{task.task_details['workflow_type']}\" "
            + 'connection="{connection}"'
        )

    raise ValueError('Invalid task type found')


def main():
    start_time = Time(datetime.now())

    stream = ScheduleStream.create_default('http://localhost:9090/ngsi-ld')

    time_window = TimeWindow()
    time_window.start = Time(0)
    time_window.end = Time.max()

    cache = generate_manufacturing_tasks(start_time)
    if not cache:
        return

    print('TASK')
    for task in cache.get_all_tasks():
        print(task.json())

    print('PROCESS')
    for process in cache.get_all_processes():
        print(process.json())

    # Write the cache to stream
    print('WRITING TO STREAM')
    result, error = stream.write_schedule(cache, time_window)

    if not result:
        print(error)
        return

    # Convert to BT
    print('SEND FOR EXECUTION')
    bts = {}
    for process in cache.get_all_processes():
        bt_id = 'urn:' + process.id
        bt_xml = TreeConversion().convert_to_tree(
            'urn:' + process.id, process.graph, partial(to_bt_node, cache=cache)
        )
        print(bt_xml)
        bts[bt_id] = bt_xml

    queue_name = '@RECEIVE@-event_mgr'
    exchange_name = '@RECEIVE@'
    routing_key = ''

    with amqp.Connection('localhost:5672') as c:
        ch = c.channel()
        ch.queue_bind(queue_name, exchange_name, routing_key)

        for bt_id, bt_xml in bts.items():
            obj = {}
            obj['id'] = bt_id
            obj['type'] = 'Schedule'
            obj['scheduleType'] = 'xml'
            obj['taskTime'] = ''
            obj['payload'] = bt_xml

            obj_str = json.dumps(obj)

            message = amqp.Message(obj_str, content_type='application/json')

            ch.basic_publish(message, exchange=exchange_name, routing_key=routing_key)

    print('Done')


if __name__ == '__main__':
    main()
