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

from datetime import datetime, timedelta
from typing import List, Optional
from uuid import uuid4

from fastapi import APIRouter, HTTPException

from rmf2_scheduler import LockedScheduleRO
from rmf2_scheduler.cache import Action, ActionPayload, ScheduleCache
from rmf2_scheduler.data import action_type, ScheduleAction, Task, Time
from rmf2_scheduler.fastapi.deps import SchedulerDep
from rmf2_scheduler.fastapi.schemas import Message
from rmf2_scheduler.fastapi.schemas import Schedule as ScheduleSchema
from rmf2_scheduler.fastapi.schemas import ScheduleAction as ScheduleActionSchema

router = APIRouter()


@router.get(
    '/',
    response_model=ScheduleSchema,
    response_model_exclude_unset=True,
    response_model_exclude_none=True,
)
def read_schedule(
    task_scheduler: SchedulerDep,
    start_time: Optional[datetime] = None,
    end_time: Optional[datetime] = None,
    offset: int = 0,
    limit: int = 100,
) -> ScheduleSchema:

    query_start_time = Time(start_time) if start_time else Time(datetime.now() - timedelta(days=1))
    query_end_time = Time(end_time) if end_time else Time(datetime.now() + timedelta(days=1))

    result, error, tasks, processes, _ = task_scheduler.get_schedule(
        query_start_time,
        query_end_time,
        offset,
        limit
    )

    if not result:
        raise HTTPException(status_code=404, detail=error)

    tasks_j = []
    for task in tasks:
        tasks_j.append(task.json())

    processes_j = []
    for process in processes:
        processes_j.append(process.json())

    return ScheduleSchema(tasks=tasks_j, processes=processes_j)


@router.post('/edit', response_model=Message)
def edit_schedule(
    task_scheduler: SchedulerDep, action: ScheduleActionSchema, dry_run: bool = True
) -> Message:
    """
    Edit Schedule.

    Allowed actions types and their payloads are as follows

    | type                      | payloads                                             |
    | ------------------------- | ---------------------------------------------------- |
    | EVENT_ADD                 | event                                                |
    | EVENT_UPDATE              | event                                                |
    | EVENT_DELETE              | id                                                   |
    | TASK_ADD                  | task                                                 |
    | TASK_UPDATE               | task                                                 |
    | TASK_DELETE               | id                                                   |
    | PROCESS_ADD               | process                                              |
    | PROCESS_UPDATE            | process                                              |
    | PROCESS_ATTACH_NODE       | id, node_id                                          |
    | PROCESS_DETACH_NODE       | id, node_id                                          |
    | PROCESS_UPDATE_START_TIME | id                                                   |
    | PROCESS_ADD_DEPENDENCY    | source_id, destination_id, edge_type                 |
    | PROCESS_DELETE_DEPENDENCY | source_id, destination_id                            |
    | PROCESS_DELETE            | id                                                   |
    | PROCESS_DELETE_ALL        | id                                                   |

    Below is an example of a selected action
    - type: **PROCESS_ADD_DEPENDENCY**
    - payloads: **source_id**, **destination_id**

    ```json
    {
        "type": "PROCESS_ADD_DEPENDENCY",
        "source_id": "<uuid>",
        "destination_id": "<uuid>"
    }
    ```
    """
    action_j = action.model_dump(mode='json')
    if 'task' in action_j and action_j['task']:
        action_j['task']['status'] = ''

    try:
        cache_action = Action.create(ScheduleAction.from_json(action_j))
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    if dry_run:
        result, error = task_scheduler.validate(cache_action)

        if not result:
            raise HTTPException(status_code=404, detail=error)

        return Message(message='Dry run successfully.')

    result, error = task_scheduler.perform(cache_action)

    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Schedule updated successfully!')


@router.post('/edit/batch', response_model=Message)
def batch_edit_schedule(
    task_scheduler: SchedulerDep,
    actions: List[ScheduleActionSchema],
    dry_run: bool = True,
) -> Message:
    cache_actions = []
    for action in actions:
        action_j = action.model_dump(mode='json')
        if 'task' in action_j and action_j['task']:
            action_j['task']['status'] = ''

        try:
            cache_action = Action.create(ScheduleAction.from_json(action_j))
        except Exception as e:
            raise HTTPException(status_code=400, detail=str(e))

        cache_actions.append(cache_action)

    if dry_run:
        result, error = task_scheduler.validate_batch(cache_actions)

        if not result:
            raise HTTPException(status_code=404, detail=error)

        return Message(message='Dry run successfully.')

    result, error = task_scheduler.perform_batch(cache_actions)

    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Schedule updated successfully!')
