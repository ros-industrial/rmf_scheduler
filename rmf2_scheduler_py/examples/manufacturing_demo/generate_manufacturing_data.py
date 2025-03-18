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

from typing import Optional, List, Tuple

from datetime import datetime
from uuid import uuid4

from numpy.random import normal
from rmf2_scheduler import TimeWindow
from rmf2_scheduler.data import Duration, Edge, Process, Task, Time, json_serializer
from rmf2_scheduler.cache import ScheduleCache
from rmf2_scheduler.cache import TaskAction, ProcessAction
from rmf2_scheduler.cache.task_action import Payload as TaskActionPayload
from rmf2_scheduler.cache.process_action import Payload as ProcessActionPayload
from rmf2_scheduler.cache.schedule_action import ActionType
from rmf2_scheduler.storage import ScheduleStream


def _add_tasks_and_processes(
    tasks: List[Task], processes: List[Process], cache: ScheduleCache
) -> Optional[ScheduleCache]:
    for task in tasks:
        # Add task
        task_action = TaskAction(ActionType.TASK_ADD, TaskActionPayload().task(task))
        result, error = task_action.validate(cache)
        if not result:
            print(error)
            return None

        task_action.apply(cache)

    for process in processes:
        # Add process
        process_add_action = ProcessAction(
            ActionType.PROCESS_ADD, ProcessActionPayload().process(process)
        )
        result, error = process_add_action.validate(cache)
        if not result:
            print(error)
            return None

        process_add_action.apply(cache)

        # Update process start time
        process_update_start_time_action = ProcessAction(
            ActionType.PROCESS_UPDATE_START_TIME, ProcessActionPayload().id(process.id)
        )
        result, error = process_update_start_time_action.validate(cache)
        if not result:
            print(error)
            return None

        process_update_start_time_action.apply(cache)

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
    task1.type = "rmf2/go_to_place"
    task1.start_time = start_time
    task1.status = status
    task1.description = f"{mm_name} moves to {workcell_name}"
    task1.duration = Duration.from_seconds(move_duration)
    task1.resource_id = mm_id
    task1.task_details = {
        "end_location": workcell_id,
    }

    # MM dock to workcell
    task2 = Task()
    task2.id = str(uuid4())
    task2.type = "dsvc/workcell_docking"
    task2.start_time = start_time
    task2.status = status
    task2.description = f"{mm_name} docks to {workcell_name}"
    task2.duration = Duration.from_seconds(dock_duration)
    task2.resource_id = "MM_01"
    task2.task_details = {
        "workcell_id": workcell_id,
        "mm_command_type": mm_base_type,
        "workflow_type": "dock",
    }

    # Workcell operation
    workcell_tasks = []
    for i in range(len(workcell_operations)):
        task = Task()
        task.id = str(uuid4())
        task.type = "dsvc/workcell_generic"
        task.start_time = start_time
        task.status = status
        task.description = f"{mm_name} {workcell_operations[i]} at {workcell_name}"
        task.duration = Duration.from_seconds(workcell_operations_durations[i])
        task.resource_id = mm_id
        task.task_details = {
            "workcell_id": workcell_id,
            "mm_command_type": mm_arm_type,
            "workflow_type": workcell_operations[i],
        }
        workcell_tasks.append(task)

    # Workcell undocking
    task3 = Task()
    task3.id = str(uuid4())
    task3.type = "dsvc/workcell_docking"
    task3.start_time = start_time
    task3.status = status
    task3.description = f"{mm_name} undock from {workcell_name}"
    task3.duration = Duration.from_seconds(undock_duration)
    task3.resource_id = mm_id
    task3.task_details = {
        "workcell_id": workcell_id,
        "mm_command_type": mm_base_type,
        "workflow_type": "undock",
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
        "MM 01",
        "MM_01",
        "MIR",
        "UR10",
        "Parts Station",
        "parts_station",
        ["pick"],
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
        "MM 01",
        "MM_01",
        "MIR",
        "UR10",
        "CNC Machine",
        "CNC",
        ["place", "pick"],
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
        "MM 01",
        "MM_01",
        "MIR",
        "UR10",
        "Robot Cell",
        "robot_cell",
        ["place"],
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
        "MM 02",
        "MM_02",
        "MIR",
        "UR10",
        "CNC Machine",
        "CNC",
        ["place", "pick"],
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
        "MM 01",
        "MM_01",
        "MIR",
        "UR10",
        "Robot Cell",
        "robot_cell",
        ["place"],
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


def generate_manufacturing_tasks(
    start_time: Time, task_types: List[str]
) -> Optional[ScheduleCache]:
    status = "queued"

    tasks1, process1 = generate_preset_sequence1(start_time, status)
    tasks2, process2 = generate_preset_sequence2(start_time, status)

    cache = ScheduleCache(task_types)

    tasks = tasks1 + tasks2
    processes = [process1, process2]
    return _add_tasks_and_processes(tasks, processes, cache)


def main():
    start_time = Time(datetime.now())
    task_types = ["rmf2/go_to_place", "dsvc/workcell_docking", "dsvc/workcell_generic"]

    stream = ScheduleStream.create_default("http://localhost:9090/ngsi-ld", task_types)

    time_window = TimeWindow()
    time_window.start = Time(0)
    time_window.end = Time.max()

    cache = generate_manufacturing_tasks(start_time, task_types)
    if not cache:
        return

    print("TASK")
    for task in cache.get_all_task():
        print(json_serializer.serialize(task))

    print("PROCESS")
    for process in cache.get_all_process():
        print(json_serializer.serialize(process))

    # Write the cache to stream
    print("WRITING TO STREAM")
    result, error = stream.write_schedule(cache, time_window)

    if not result:
        print(error)
        return

    print("Done")


if __name__ == "__main__":
    main()
