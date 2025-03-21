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
from typing import Dict, List
from uuid import uuid4

from fastapi import APIRouter, HTTPException

from rmf2_scheduler import LockedScheduleRO
from rmf2_scheduler.cache import Action, ActionPayload, ScheduleCache
from rmf2_scheduler.data import action_type, Process, Time
from rmf2_scheduler.fastapi.deps import SchedulerDep
from rmf2_scheduler.fastapi.schemas import Message
from rmf2_scheduler.fastapi.schemas import Process as ProcessSchema
from rmf2_scheduler.fastapi.schemas import ProcessCreate as ProcessCreateSchema
from rmf2_scheduler.fastapi.schemas import ProcessUpdate as ProcessUpdateSchema

router = APIRouter()


@router.get(
    '/',
    response_model=List[ProcessSchema],
    response_model_exclude_unset=True,
    response_model_exclude_none=True,
)
def read_processes(
    task_scheduler: SchedulerDep,
    start_time: datetime = datetime.now(),
    end_time: datetime = datetime.now() + timedelta(days=1),
    offset: int = 0,
    limit: int = 100,
) -> List[ProcessSchema]:
    schedule_cache: ScheduleCache = LockedScheduleRO(task_scheduler).cache()

    tasks = schedule_cache.lookup_tasks(Time(start_time), Time(end_time))

    # if not result:
    #     raise HTTPException(status_code=404, detail=error)

    process_map: Dict[str, Process] = {}
    for task in tasks:
        if task.process_id is None or task.process_id in process_map:
            continue

        process_map[task.process_id] = schedule_cache.get_process(task.process_id)

    response = []
    for _, process in process_map.items():
        response.append(process.json())

    return response


@router.post(
    '/',
    response_model=ProcessSchema,
    response_model_exclude_unset=True,
    response_model_exclude_none=True,
)
def create_process(
    task_scheduler: SchedulerDep,
    process_create: ProcessCreateSchema,
) -> ProcessSchema:
    process_j = process_create.model_dump(mode='json')
    process_j['id'] = str(uuid4())

    try:
        process_data = Process.from_json(process_j)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    action = Action.create(
        action_type.PROCESS_ADD, ActionPayload().process(process_data)
    )
    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    schedule_cache: ScheduleCache = LockedScheduleRO(task_scheduler).cache()

    process = schedule_cache.get_process(process_data.id)
    return process.json()


@router.post(
    '/add_dependency',
    response_model=Message,
)
def add_dependency_to_process(
    task_scheduler: SchedulerDep,
    source_uuid: str,
    destination_uuid: str,
    edge_type: str = 'hard',
) -> Message:
    # Create payload
    payload = ActionPayload()
    payload.source_id(source_uuid)
    payload.destination_id(destination_uuid)
    payload.edge_type(edge_type)

    action = Action.create(action_type.PROCESS_ADD_DEPENDENCY, payload)

    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Process updated successfully!')


@router.post(
    '/delete_dependency',
    response_model=Message,
)
def delete_dependency_from_process(
    task_scheduler: SchedulerDep,
    source_uuid: str,
    destination_uuid: str,
) -> Message:
    # Create payload
    payload = ActionPayload()
    payload.source_id(source_uuid)
    payload.destination_id(destination_uuid)

    action = Action.create(action_type.PROCESS_DELETE_DEPENDENCY, payload)

    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Process updated successfully!')


@router.get(
    '/{uuid}',
    response_model=ProcessSchema,
    response_model_exclude_unset=True,
    response_model_exclude_none=True,
)
def read_process(task_scheduler: SchedulerDep, uuid: str) -> ProcessSchema:
    schedule_cache: ScheduleCache = LockedScheduleRO(task_scheduler).cache()

    if not schedule_cache.has_process(uuid):
        raise HTTPException(status_code=404, detail="Process doesn't exist")

    process = schedule_cache.get_process(uuid)
    response = process.json()

    return response


@router.post(
    '/{uuid}/attach_node',
    response_model=Message,
)
def attach_node_to_process(
    task_scheduler: SchedulerDep, uuid: str, node_uuid: str
) -> Message:
    action = Action.create(
        action_type.PROCESS_ATTACH_NODE, ActionPayload().id(uuid).node_id(node_uuid)
    )

    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Process updated successfully!')


@router.post(
    '/{uuid}/detach_node',
    response_model=Message,
)
def detach_node_from_process(
    task_scheduler: SchedulerDep, uuid: str, node_uuid: str
) -> Message:
    action = Action.create(
        action_type.PROCESS_DETACH_NODE, ActionPayload().id(uuid).node_id(node_uuid)
    )

    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Process updated successfully!')


@router.put('/{uuid}', response_model=Message)
def update_process(
    task_scheduler: SchedulerDep, uuid: str, process_update: ProcessUpdateSchema
) -> Message:
    process_j = process_update.model_dump(mode='json')
    process_j['id'] = uuid

    try:
        process_data = Process.from_json(process_j)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    action = Action.create(
        action_type.PROCESS_UPDATE, ActionPayload().process(process_data)
    )
    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Process updated successfully!')


@router.delete('/{uuid}', response_model=Message)
def delete_process(
    task_scheduler: SchedulerDep, uuid: str, delete_all: bool = False
) -> Message:
    action: Action
    if delete_all:
        action = Action.create(action_type.PROCESS_DELETE_ALL, ActionPayload().id(uuid))
    else:
        action = Action.create(action_type.PROCESS_DELETE, ActionPayload().id(uuid))

    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Process delete successfully!')
