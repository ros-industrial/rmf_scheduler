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
from datetime import datetime, timedelta
import json
from typing import List
from uuid import uuid4

import requests
from rmf2_scheduler.data import (
    action_type,
    Graph,
)
from rmf2_scheduler.fastapi.schemas import Occurrence as OccurrenceSchema
from rmf2_scheduler.fastapi.schemas import ProcessPayload as ProcessPayloadSchema
from rmf2_scheduler.fastapi.schemas import ScheduleAction as ScheduleActionSchema
from rmf2_scheduler.fastapi.schemas import Series as SeriesSchema
from rmf2_scheduler.fastapi.schemas import SeriesPayload as SeriesPayloadSchema
from rmf2_scheduler.fastapi.schemas import (
    SeriesUpdateOccurrenceTime as SeriesUpdateOccurrenceTimeSchema,
)
from rmf2_scheduler.fastapi.schemas import TaskPayload as TaskPayloadSchema


def is_not_null_or_unset(value):
    return value is not None and value != ''


def remove_nulls(value):
    if isinstance(value, dict):
        return {k: remove_nulls(v) for k, v in value.items() if is_not_null_or_unset(v)}
    elif isinstance(value, list):
        return [remove_nulls(item) for item in value if is_not_null_or_unset(item)]
    else:
        return value


def generate_tasks(places: List[str], robot: str, fleet: str) -> List[TaskPayloadSchema]:
    start_time_datetime = datetime.utcnow()
    tasks = []
    for p in places:
        task = TaskPayloadSchema(
            type='go_to_place',
            id=str(uuid4()),
            task_details={'fleet': fleet, 'robot': robot, 'place': p},
            start_time=start_time_datetime,
        )
        tasks.append(task)

    return tasks


def generate_process(tasks: List[TaskPayloadSchema]) -> ProcessPayloadSchema:
    process_id = str(uuid4())
    if tasks is None or len(tasks) == 0:
        return None

    # create graph
    graph = Graph()
    for i in range(len(tasks)):
        if i == 0:
            graph.add_node(str(tasks[i].id))
            continue
        graph.add_node(str(tasks[i].id))
        graph.add_edge(str(tasks[i - 1].id), destination=str(tasks[i].id))

    process_id = str(uuid4())
    return ProcessPayloadSchema(id=process_id, graph=graph.json())


def generate_series(
    process: ProcessPayloadSchema, start_time: datetime, freq: int
) -> SeriesPayloadSchema:
    # cron_string = f'{start_time.second}/10 * * * * ?' # every 10 seconds
    cron_string = f'{start_time.second} {start_time.minute}/{freq} * * * ?'  # every 1 minutes
    print(f'cron string is {cron_string}')

    return SeriesPayloadSchema(
        id=str(uuid4()),
        type='process',
        occurrences=[OccurrenceSchema(id=str(process.id), time=start_time)],
        cron=cron_string,
        timezone='SGT',
    )


def append_schedule_actions(
    tasks: List[TaskPayloadSchema], process: ProcessPayloadSchema, series: SeriesPayloadSchema
):
    tasks_actions = [
        json.loads(ScheduleActionSchema(type=action_type.TASK_ADD, task=t).model_dump_json())
        for t in tasks
    ]
    tasks_actions.append(
        json.loads(
            ScheduleActionSchema(type=action_type.PROCESS_ADD, process=process).model_dump_json()
        )
    )
    tasks_actions.append(
        json.loads(
            ScheduleActionSchema(type=action_type.SERIES_ADD, series_add=series).model_dump_json()
        )
    )
    return tasks_actions


def create_update_last_occurrence(series: SeriesSchema) -> SeriesUpdateOccurrenceTimeSchema:
    last_occ = series.occurrences[-1]
    print('\n')
    print('LAST OCCURRENCE')
    print(last_occ)
    print('\n')
    return SeriesUpdateOccurrenceTimeSchema(
        series_id=str(series.id),
        occurrence_id=last_occ.id,
        time=(last_occ.time + timedelta(hours=1)),
    )


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

    parser.add_argument('--host', type=str, default='localhost', help='Server Host')
    parser.add_argument('--port', type=int, default=8000, help='Server Port')
    parser.add_argument('--robot', '-R', type=str, default='tinyRobot1', help='Robot Name')
    parser.add_argument('--fleet', '-F', type=str, default='tinyRobot', help='Robot Name')
    parser.add_argument('--freq', '-f', type=int, default=1, help='frequency of series in minutes')
    parser.add_argument(
        '-p',
        '--places',
        nargs='+',  # Expect one or more arguments
        type=str,  # Convert each argument to an str
        help='A list of places each place is a go to place task in a Process',
        default=['pantry', 'coe'],
    )
    args = parser.parse_args()

    if args.freq > 59 or args.freq < 1:
        raise argparse.ArgumentTypeError(
            f'frequency has to be between 1-59. [{args.freq}] is not valid'
        )

    rs_url = f'http://{args.host}:{args.port}/schedule/edit/batch'

    print('', end='')

    tasks_payloads = generate_tasks(places=args.places, robot=args.robot, fleet=args.fleet)
    process_payload = generate_process(tasks=tasks_payloads)
    series_payload = generate_series(
        process=process_payload, start_time=tasks_payloads[0].start_time, freq=args.freq
    )
    schedule_actions = append_schedule_actions(
        tasks=tasks_payloads, process=process_payload, series=series_payload
    )

    apply_batch_edit(rs_url, schedule_actions)
    # New line
    print('')
    print('Series created successfully')


if __name__ == '__main__':
    main()
