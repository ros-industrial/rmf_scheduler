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
from typing import List, Optional, Tuple
from uuid import uuid4

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


def generate_warehouse_tasks(start_time: Time) -> Optional[ScheduleCache]:
    status = 'completed'
    forklift_durations = normal(300, 10, 1)
    palletizer_durations = normal(60, 2, 1)
    amr1_drop1_durations = normal(600, 30, 1)
    amr1_drop2_durations = normal(600, 30, 1)
    amr2_drop1_durations = normal(600, 15, 1)

    # Forklift to pickup
    task1 = Task()
    task1.id = str(uuid4())
    task1.type = 'rmf2/go_to_place'
    task1.start_time = start_time
    task1.status = status
    task1.description = 'Forklift 1 pickup'
    task1.duration = Duration.from_seconds(forklift_durations[0])
    task1.resource_id = 'forklift_1'
    task1.task_details = {
        'start_location': 'pickup_1',
        'end_location': 'palletizer_1',
    }

    # Palletization
    task2 = Task()
    task2.id = str(uuid4())
    task2.type = 'ihi/palletization'
    task2.start_time = start_time
    task2.status = status
    task2.description = 'Palletization'
    task2.duration = Duration.from_seconds(palletizer_durations[0])
    task2.resource_id = 'palletizer_1'
    task2.task_details = {
        'forklift_id': 'forklift_1',
    }

    # AMR 1 task
    task3 = Task()
    task3.id = str(uuid4())
    task3.type = 'rmf2/go_to_place'
    task3.start_time = start_time
    task3.status = status
    task3.description = 'AMR1 transport task'
    task3.duration = Duration.from_seconds(amr1_drop1_durations[0])
    task3.resource_id = 'amr_1'
    task3.task_details = {
        'start_location': 'palletizer_1',
        'end_location': 'drop_1',
    }

    # AMR 2 task
    task4 = Task()
    task4.id = str(uuid4())
    task4.type = 'rmf2/go_to_place'
    task4.start_time = start_time
    task4.status = status
    task4.description = 'AMR2 transport task'
    task4.duration = Duration.from_seconds(amr2_drop1_durations[0])
    task4.resource_id = 'amr_2'
    task4.task_details = {
        'start_location': 'palletizer_1',
        'end_location': 'drop_1',
    }

    # AMR 1 task
    task5 = Task()
    task5.id = str(uuid4())
    task5.type = 'rmf2/go_to_place'
    task5.start_time = start_time
    task5.status = status
    task5.description = 'AMR1 transport task'
    task5.duration = Duration.from_seconds(amr1_drop2_durations[0])
    task5.resource_id = 'amr_1'
    task5.task_details = {
        'start_location': 'palletizer_1',
        'end_location': 'drop_2',
    }

    # Process
    process = Process()
    process.id = str(uuid4())
    process.graph.add_node(task1.id)
    process.graph.add_node(task2.id)
    process.graph.add_node(task3.id)
    process.graph.add_node(task4.id)
    process.graph.add_node(task5.id)
    process.graph.add_edge(task1.id, task2.id)
    process.graph.add_edge(task2.id, task3.id, Edge('soft'))
    process.graph.add_edge(task2.id, task4.id, Edge('soft'))
    process.graph.add_edge(task2.id, task5.id, Edge('soft'))

    # Write tasks and process to cache
    cache = ScheduleCache()

    tasks = [task1, task2, task3, task4, task5]
    processes = [process]
    return _add_tasks_and_processes(tasks, processes, cache)


def main():
    start_time = Time.from_ISOtime('2025-01-01T10:15:00Z')

    stream = ScheduleStream.create_default('http://localhost:9090/ngsi-ld')

    time_window = TimeWindow()
    time_window.start = Time(0)
    time_window.end = Time.max()

    for i in range(50):
        cache = generate_warehouse_tasks(start_time)
        if not cache:
            return

        # Write the cache to stream
        result, error = stream.write_schedule(cache, time_window)

        if not result:
            print(error)
            return

    print('Done')


if __name__ == '__main__':
    main()
