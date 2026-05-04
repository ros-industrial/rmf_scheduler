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
from uuid import uuid4

import requests
from rmf2_scheduler.data import (
    action_type,
    Occurrence,
    ScheduleAction,
    Series,
    Task,
    Time,
)
from rmf2_scheduler.fastapi.schemas import Occurrence as OccurrenceSchema
from rmf2_scheduler.fastapi.schemas import Series as SeriesSchema
from rmf2_scheduler.fastapi.schemas import SeriesCreate as SeriesCreateSchema
from rmf2_scheduler.fastapi.schemas import (
    SeriesUpdateOccurrenceTime as SeriesUpdateOccurrenceTimeSchema,
)
from rmf2_scheduler.fastapi.schemas import Task as TaskSchema
from rmf2_scheduler.fastapi.schemas import TaskCreate as TaskCreateSchema


def is_not_null_or_unset(value):
    return value is not None and value != ''


def remove_nulls(value):
    if isinstance(value, dict):
        return {k: remove_nulls(v) for k, v in value.items() if is_not_null_or_unset(v)}
    elif isinstance(value, list):
        return [remove_nulls(item) for item in value if is_not_null_or_unset(item)]
    else:
        return value


def generate_series_task():
    actions = []

    start_time_datetime = datetime.now()
    cron_string = f'* {start_time_datetime.minute}/5 * * * ?'  # every 5 minutes
    print(f'cron string is {cron_string}')

    task_id = str(uuid4())
    task = Task()
    task.type = 'go_to_place'
    task.id = task_id
    task.start_time = Time(start_time_datetime)

    series = Series(
        id=str(uuid4()),
        type='task',
        occurrence=Occurrence(id=task.id, _time=task.start_time),
        cron=cron_string,
        timezone='SGT',
    )

    # Add task
    add_task_action = ScheduleAction()
    add_task_action.type = action_type.TASK_ADD
    add_task_action.task = task
    actions.append(remove_nulls(add_task_action.json()))

    add_series_action = ScheduleAction()
    add_series_action.type = action_type.SERIES_ADD
    add_series_action.series_add = series
    actions.append(remove_nulls(add_series_action.json()))

    print(f'Series Action f{add_series_action.json()}')

    # print(json.dumps(actions, indent=2))
    return actions


def post_request_json(url: str, batch_edit_json):
    # Send JSON
    try:
        response = requests.post(url, params=[('dry_run', False)], json=batch_edit_json)
    except requests.ConnectionError as e:
        print('\nConnection Error, is RMF2 Scheduler up?')
        print(str(e))
        return (False, None)

    if not response.ok:
        print('\nUnable to send request to RMF2 Scheduler, is the URL correct?')
        print(f'{response.status_code} - {response.reason}')
        print(f'{response.text}')
        return (False, None)

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
        return (False, None)

    return (True, response)


def post_request_data(url: str, batch_edit_json):
    # Send JSON
    try:
        response = requests.post(url, params=[('dry_run', False)], data=batch_edit_json)
    except requests.ConnectionError as e:
        print('\nConnection Error, is RMF2 Scheduler up?')
        print(str(e))
        return (False, None)

    if not response.ok:
        print('\nUnable to send request to RMF2 Scheduler, is the URL correct?')
        print(f'{response.status_code} - {response.reason}')
        print(f'{response.text}')
        return (False, None)

    return (True, response)


def create_task() -> TaskCreateSchema:
    return TaskCreateSchema(
        type='go_to_place',
        start_time=datetime.utcnow(),
        task_details={'fleet': 'tinyRobot', 'robot': 'tinyRobot1', 'place': 'pantry'},
    )


def put_request_json(url: str, batch_edit_json):
    # Send JSON
    try:
        response = requests.put(url, params=[('dry_run', False)], json=batch_edit_json)
    except requests.ConnectionError as e:
        print('\nConnection Error, is RMF2 Scheduler up?')
        print(str(e))
        return (False, None)

    if not response.ok:
        print('\nUnable to send request to RMF2 Scheduler, is the URL correct?')
        print(f'{response.status_code} - {response.reason}')
        print(f'{response.text}')
        return (False, None)

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
        return (False, None)

    return (True, response)


def create_series(task: TaskSchema) -> SeriesCreateSchema:
    cron_string = f'{task.start_time.second}/10 * * * * ?'  # every 10 seconds
    cron_string = f'{task.start_time.second} {task.start_time.minute} */1 * * ?'  # every 5 minutes

    return SeriesCreateSchema(
        id=str(uuid4()),
        type='task',
        occurrences=[OccurrenceSchema(id=task.id, time=task.start_time)],
        cron=cron_string,
        timezone='SGT',
    )


def create_update_last_occurrence(
    series: SeriesSchema,
) -> SeriesUpdateOccurrenceTimeSchema:
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


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser()

    parser.add_argument('--host', type=str, default='localhost', help='Server Host')
    parser.add_argument('--port', type=int, default=8000, help='Server Port')
    args = parser.parse_args()

    rs_url = f'http://{args.host}:{args.port}/schedule/edit/batch'
    rs_create_series_url = f'http://{args.host}:{args.port}/series/'
    rs_create_task_url = f'http://{args.host}:{args.port}/tasks/'
    rs_update_series_occurrence_time_url = (
        f'http://{args.host}:{args.port}/series/update_occurrence_time'
    )
    print(f'RMF2 Scheduler batch edit endpoint <{rs_url}>')

    # ORDER 1
    print('', end='')
    # batch_edit_json =  generate_series_task()
    # if not post_request(rs_url, batch_edit_json):
    #         return 1

    task_create_schema = create_task()
    print(task_create_schema.json())
    (success, response) = post_request_data(
        rs_create_task_url, remove_nulls(task_create_schema.json())
    )
    series_create_schema = create_series(TaskSchema(**response.json()))
    series_create_schema_json = remove_nulls(series_create_schema.json())
    print(series_create_schema_json)
    (success, response) = post_request_data(rs_create_series_url, series_create_schema_json)
    print('created series\n')
    print(response.json())
    update_series_occ_schema = create_update_last_occurrence(SeriesSchema(**response.json()))
    print(update_series_occ_schema)
    put_request_json(
        rs_update_series_occurrence_time_url,
        remove_nulls(json.loads(update_series_occ_schema.model_dump_json())),
    )

    # New line
    print('')
    print('Series created successfully')


if __name__ == '__main__':
    main()
