import argparse
from datetime import datetime
from typing import List
import json
from uuid import uuid4

import requests
from rmf2_scheduler.data import Time, Duration, Task, Process, ScheduleAction, action_type
from rmf2_scheduler.utils import TreeConversion

def is_not_null_or_unset(value):
    return value is not None and value != ""

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
    task_ids = [ str(uuid4()) for i in range(2) ]

    actions = []

    task1 = Task()
    task1.id = task_ids[0]
    task1.type = "ihi/go_to_amr"
    task1.start_time = start_time
    task1.resource_id = agf
    task1.task_details = {
        "coordinates": agf_wp_s
    }

    task2 = Task()
    task2.id = task_ids[1]
    task2.type = "ihi/warehouse_task"
    task2.start_time = start_time
    task2.resource_id = warehouse_machine

    process = Process()
    process.id = process_id

    for task in [task1, task2]:
        action = ScheduleAction()
        action.type = action_type.TASK_ADD
        action.task = task

        action_j = action.json()
        actions.append(remove_nulls(action.json()))

        process.graph.add_node(task.id)

    process.graph.add_edge(task1.id, task2.id);

    action = ScheduleAction()
    action.type = action_type.PROCESS_ADD
    action.process = process
    action_j = action.json()
    actions.append(remove_nulls(action.json()))

    # print(json.dumps(actions, indent=2))
    return actions


def apply_batch_edit(url: str, batch_edit_json):
    # Send JSON
    try:
        response = requests.post(url, params=[("dry_run", False)], json=batch_edit_json)
    except requests.ConnectionError as e:
        print("\nConnection Error, is RMF2 Scheduler up?")
        print(str(e))
        return False

    if not response.ok:
        print("\nUnable to send request to RMF2 Scheduler, is the URL correct?")
        print(f"{response.status_code} - {response.reason}")
        print(f"{response.text}")
        return False

    success = True
    try:
        response_json = response.json()
        if response_json["message"] != "Schedule updated successfully!":
            success = False
    except requests.JSONDecodeError as e:
        success = False

    if not success:
        print("\nUnable to send request to RMF2 Scheduler, is the URL correct?")
        print(f"{response.status_code} - {response.reason}")
        print(f"{response.text}")
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

    rs_url = f"http://{args.host}:{args.port}/schedule/edit/batch"
    print(f"RMF2 Scheduler batch edit endpoint <{rs_url}>")

    now = Time(datetime.now())

    # ORDER 1
    print("", end="")
    for loop_idx in range(args.num_loops):
        start_time = now + Duration.from_seconds(loop_idx * 10 * 60)

        batch_edit_json = []
        print(f"\rSending Order 1 [{loop_idx + 1}/{args.num_loops}]", end="")
        for i in range(8):
            batch_edit_json += generate_2_tasks(
                agf=f"agf_{i + 1}",
                agf_wp_s=f"agf_incoming_s[{i}]",
                agf_wp_e=f"agf_incoming_e[{i}]",
                warehouse_machine=f"TS1_{i + 1}",
                start_time=start_time
            )

        if not apply_batch_edit(rs_url, batch_edit_json):
            return 1

    # New line
    print("")
    print("Orders created successfully")


if __name__ == "__main__":
    main()
