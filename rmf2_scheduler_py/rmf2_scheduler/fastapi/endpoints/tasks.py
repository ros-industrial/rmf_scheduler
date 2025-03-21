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
from typing import List
from uuid import uuid4

from fastapi import APIRouter, HTTPException

from rmf2_scheduler import LockedScheduleRO
from rmf2_scheduler.cache import Action, ActionPayload, ScheduleCache
from rmf2_scheduler.data import action_type, Task, Time
from rmf2_scheduler.fastapi.deps import SchedulerDep
from rmf2_scheduler.fastapi.schemas import Message
from rmf2_scheduler.fastapi.schemas import Task as TaskSchema
from rmf2_scheduler.fastapi.schemas import TaskCreate as TaskCreateSchema
from rmf2_scheduler.fastapi.schemas import TaskUpdate as TaskUpdateSchema

router = APIRouter()


@router.get(
    '/',
    response_model=List[TaskSchema],
    response_model_exclude_unset=True,
    response_model_exclude_none=True,
)
def read_tasks(
    task_scheduler: SchedulerDep,
    start_time: datetime = datetime.now(),
    end_time: datetime = datetime.now() + timedelta(days=1),
    offset: int = 0,
    limit: int = 100,
) -> List[TaskSchema]:
    schedule_cache: ScheduleCache = LockedScheduleRO(task_scheduler).cache()

    tasks = schedule_cache.lookup_tasks(Time(start_time), Time(end_time))

    # if not result:
    #     raise HTTPException(status_code=404, detail=error)

    response = []
    for task in tasks:
        response.append(task.json())
    return response


@router.post(
    '/',
    response_model=TaskSchema,
    response_model_exclude_unset=True,
    response_model_exclude_none=True,
)
def create_task(
    task_scheduler: SchedulerDep,
    task_create: TaskCreateSchema,
) -> TaskSchema:
    task_j = task_create.model_dump(mode='json')
    task_j['id'] = str(uuid4())
    task_j['status'] = ''

    try:
        task_data = Task.from_json(task_j)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    action = Action.create(action_type.TASK_ADD, ActionPayload().task(task_data))
    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    schedule_cache: ScheduleCache = LockedScheduleRO(task_scheduler).cache()

    task = schedule_cache.get_task(task_data.id)
    return task.json()


@router.get(
    '/{uuid}',
    response_model=TaskSchema,
    response_model_exclude_unset=True,
    response_model_exclude_none=True,
)
def read_task(
    task_scheduler: SchedulerDep,
    uuid: str,
) -> TaskSchema:
    schedule_cache: ScheduleCache = LockedScheduleRO(task_scheduler).cache()

    if not schedule_cache.has_task(uuid):
        raise HTTPException(status_code=404, detail="Task doesn't exist")

    task = schedule_cache.get_task(uuid)
    response = task.json()

    return response


@router.put('/{uuid}', response_model=Message)
def update_task(
    task_scheduler: SchedulerDep,
    uuid: str,
    task_update: TaskUpdateSchema,
) -> Message:
    task_j = task_update.model_dump(mode='json')
    task_j['id'] = uuid
    task_j['status'] = ''

    try:
        task_data = Task.from_json(task_j)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    action = Action.create(action_type.TASK_UPDATE, ActionPayload().task(task_data))
    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Task updated successfully!')


@router.delete('/{uuid}', response_model=Message)
def delete_task(
    task_scheduler: SchedulerDep,
    uuid: str,
) -> Message:
    action = Action.create(action_type.TASK_DELETE, ActionPayload().id(uuid))
    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Task delete successfully!')
