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

def generate_order_1(
    agf: str,
    agf_wp_s: str,
    agf_wp_e: str,
    warehouse_machine: str,
    lamr: str,
    lamr_wp_s: str,
    lamr_wp_e: str,
    start_time: Time
):
    process_id = str(uuid4())
    task_ids = [ str(uuid4()) for i in range(7) ]

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
    task2.type = "ihi/wait_amr"
    task2.start_time = start_time
    task2.resource_id = agf

    task3 = Task()
    task3.id = task_ids[2]
    task3.type = "ihi/go_to_amr"
    task3.start_time = start_time
    task3.resource_id = agf
    task3.task_details = {
        "coordinates": agf_wp_e
    }

    task4 = Task()
    task4.id = task_ids[3]
    task4.type = "ihi/go_to_amr"
    task4.start_time = start_time
    task4.resource_id = lamr
    task4.task_details = {
        "coordinates": lamr_wp_s
    }

    task5 = Task()
    task5.id = task_ids[4]
    task5.type = "ihi/warehouse_task"
    task5.start_time = start_time
    task5.resource_id = warehouse_machine

    task6 = Task()
    task6.id = task_ids[5]
    task6.type = "ihi/go_to_amr"
    task6.start_time = start_time
    task6.resource_id = lamr
    task6.task_details = {
        "coordinates": lamr_wp_e
    }

    task7 = Task()
    task7.id = task_ids[6]
    task7.type = "ihi/wait_amr"
    task7.start_time = start_time
    task7.resource_id = lamr

    process = Process()
    process.id = process_id

    for task in [task1, task2, task3, task4, task5, task6, task7]:
        action = ScheduleAction()
        action.type = action_type.TASK_ADD
        action.task = task

        action_j = action.json()
        actions.append(remove_nulls(action.json()))

        process.graph.add_node(task.id)

    process.graph.add_edge(task1.id, task2.id);
    process.graph.add_edge(task2.id, task3.id);
    process.graph.add_edge(task2.id, task4.id);
    process.graph.add_edge(task3.id, task5.id);
    process.graph.add_edge(task4.id, task5.id);
    process.graph.add_edge(task5.id, task6.id);
    process.graph.add_edge(task6.id, task7.id);

    action = ScheduleAction()
    action.type = action_type.PROCESS_ADD
    action.process = process
    action_j = action.json()
    actions.append(remove_nulls(action.json()))

    # print(json.dumps(actions, indent=2))
    return actions

def generate_order_2(
    agf: str,
    samrs: List[str],
    start_time: Time
):
    assert(len(samrs) == 3)
    process_id = str(uuid4())
    task_ids = [ str(uuid4()) for i in range(14) ]
    dummy_task_ids = [ str(uuid4()) for i in range(4) ]

    actions = []

    task1 = Task()
    task1.id = task_ids[0]
    task1.type = "rmf2/mapf"
    task1.start_time = start_time
    task1.resource_id = samrs[0]
    task1.task_details = {
        "coordinates": "samr_s[0]"
    }

    task2 = Task()
    task2.id = task_ids[1]
    task2.type = "rmf2/mapf"
    task2.start_time = start_time
    task2.resource_id = samrs[0]
    task2.task_details = {
        "coordinates": "samr_e[0]"
    }

    task3 = Task()
    task3.id = task_ids[2]
    task3.type = "rmf2/mapf"
    task3.start_time = start_time
    task3.resource_id = samrs[1]
    task3.task_details = {
        "coordinates": "samr_s[0]"
    }

    task4 = Task()
    task4.id = task_ids[3]
    task4.type = "ihi/go_to_amr"
    task4.start_time = start_time
    task4.resource_id = agf
    task4.task_details = {
        "coordinates": "agf_outgoing_s[0]"
    }

    task5 = Task()
    task5.id = task_ids[4]
    task5.type = "rmf2/mapf"
    task5.start_time = start_time
    task5.resource_id = samrs[0]
    task5.task_details = {
        "coordinates": "samr_i[1]"
    }

    task6 = Task()
    task6.id = task_ids[5]
    task6.type = "rmf2/mapf"
    task6.start_time = start_time
    task6.resource_id = samrs[1]
    task6.task_details = {
        "coordinates": "samr_e[0]"
    }

    task7 = Task()
    task7.id = task_ids[6]
    task7.type = "rmf2/mapf"
    task7.start_time = start_time
    task7.resource_id = samrs[2]
    task7.task_details = {
        "coordinates": "samr_s[0]"
    }

    task8 = Task()
    task8.id = task_ids[7]
    task8.type = "rmf2/mapf"
    task8.start_time = start_time
    task8.resource_id = samrs[1]
    task8.task_details = {
        "coordinates": "samr_i[2]"
    }

    task9 = Task()
    task9.id = task_ids[8]
    task9.type = "rmf2/mapf"
    task9.start_time = start_time
    task9.resource_id = samrs[2]
    task9.task_details = {
        "coordinates": "samr_e[0]"
    }

    task10 = Task()
    task10.id = task_ids[9]
    task10.type = "rmf2/mapf"
    task10.start_time = start_time
    task10.resource_id = samrs[0]
    task10.task_details = {
        "coordinates": "samr_s[0]"
    }

    task11 = Task()
    task11.id = task_ids[10]
    task11.type = "rmf2/mapf"
    task11.start_time = start_time
    task11.resource_id = samrs[2]
    task11.task_details = {
        "coordinates": "samr_i[3]"
    }

    task12 = Task()
    task12.id = task_ids[11]
    task12.type = "rmf2/mapf"
    task12.start_time = start_time
    task12.resource_id = samrs[0]
    task12.task_details = {
        "coordinates": "samr_e[0]"
    }

    task13 = Task()
    task13.id = task_ids[12]
    task13.type = "rmf2/mapf"
    task13.start_time = start_time
    task13.resource_id = samrs[0]
    task13.task_details = {
        "coordinates": "samr_i[1]"
    }

    task14 = Task()
    task14.id = task_ids[13]
    task14.type = "ihi/go_to_amr"
    task14.start_time = start_time
    task14.resource_id = agf
    task14.task_details = {
        "coordinates": "agf_outgoing_e[0]"
    }

    dummy_task1 = Task()
    dummy_task1.id = dummy_task_ids[0]
    dummy_task1.type = "ihi/dummy"
    dummy_task1.start_time = start_time

    dummy_task2 = Task()
    dummy_task2.id = dummy_task_ids[1]
    dummy_task2.type = "ihi/dummy"
    dummy_task2.start_time = start_time

    dummy_task3 = Task()
    dummy_task3.id = dummy_task_ids[2]
    dummy_task3.type = "ihi/dummy"
    dummy_task3.start_time = start_time

    dummy_task4 = Task()
    dummy_task4.id = dummy_task_ids[3]
    dummy_task4.type = "ihi/dummy"
    dummy_task4.start_time = start_time

    process = Process()
    process.id = process_id

    for task in [
        task1, task2, task3, task4, task5, task6, task7,
        task8, task9, task10, task11, task12, task13, task14,
        dummy_task1, dummy_task2, dummy_task3, dummy_task4
    ]:
        action = ScheduleAction()
        action.type = action_type.TASK_ADD
        action.task = task

        action_j = action.json()
        actions.append(remove_nulls(action.json()))

        process.graph.add_node(task.id)

    process.graph.add_edge(task1.id, task2.id);
    process.graph.add_edge(task1.id, task3.id);
    process.graph.add_edge(task1.id, task4.id);
    process.graph.add_edge(task2.id, dummy_task1.id);
    process.graph.add_edge(task3.id, dummy_task1.id);
    process.graph.add_edge(task4.id, dummy_task1.id);
    process.graph.add_edge(dummy_task1.id, task5.id);
    process.graph.add_edge(dummy_task1.id, task6.id);
    process.graph.add_edge(dummy_task1.id, task7.id);
    process.graph.add_edge(task5.id, dummy_task2.id);
    process.graph.add_edge(task6.id, dummy_task2.id);
    process.graph.add_edge(task7.id, dummy_task2.id);
    process.graph.add_edge(dummy_task2.id, task8.id);
    process.graph.add_edge(dummy_task2.id, task9.id);
    process.graph.add_edge(dummy_task2.id, task10.id);
    process.graph.add_edge(task8.id, dummy_task3.id);
    process.graph.add_edge(task9.id, dummy_task3.id);
    process.graph.add_edge(task10.id, dummy_task3.id);
    process.graph.add_edge(dummy_task3.id, task11.id);
    process.graph.add_edge(dummy_task3.id, task12.id);
    process.graph.add_edge(task11.id, dummy_task4.id);
    process.graph.add_edge(task12.id, dummy_task4.id);
    process.graph.add_edge(dummy_task4.id, task13.id);
    process.graph.add_edge(dummy_task4.id, task14.id);

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
    parser.add_argument('-o1', '--num-o1-loops', type=int, default=10)
    parser.add_argument('-o2', '--num-o2-loops', type=int, default=10)
    args = parser.parse_args()

    rs_url = f"http://{args.host}:{args.port}/schedule/edit/batch"
    print(f"RMF2 Scheduler batch edit endpoint <{rs_url}>")

    now = Time(datetime.now())

    # ORDER 1
    print("", end="")
    for loop_idx in range(args.num_o1_loops):
        start_time = now + Duration.from_seconds(loop_idx * 10 * 60)

        batch_edit_json = []
        print(f"\rSending Order 1 [{loop_idx + 1}/{args.num_o1_loops}]", end="")
        for i in range(8):
            batch_edit_json += generate_order_1(
                agf=f"agf_{i + 1}",
                agf_wp_s=f"agf_incoming_s[{i}]",
                agf_wp_e=f"agf_incoming_e[{i}]",
                lamr=f"lamr_{i + 1}",
                warehouse_machine=f"TS1_{i + 1}",
                lamr_wp_s=f"lamr_s[{7 - i}]",
                lamr_wp_e=f"lamr_e[{7 - i}]",
                start_time=start_time
            )

        if not apply_batch_edit(rs_url, batch_edit_json):
            return 1

    print("")

    # ORDER 2
    print("", end="")
    for loop_idx in range(args.num_o2_loops):
        start_time = now + Duration.from_seconds(loop_idx * 10 * 60)

        print(f"\rSending Order 2 [{loop_idx + 1}/{args.num_o2_loops}]", end="")
        batch_edit_json = []
        for i in range(1):
            batch_edit_json += generate_order_2(
                agf=f"agf_{i + 16}",
                samrs=[
                    f"MiR_000{i + 1}",
                    f"MiR_000{i + 2}",
                    f"MiR_000{i + 3}",
                ],
                start_time=start_time
            )

        if not apply_batch_edit(rs_url, batch_edit_json):
            return 1


    # New line
    print("")
    print("Orders created successfully")


if __name__ == "__main__":
    main()
