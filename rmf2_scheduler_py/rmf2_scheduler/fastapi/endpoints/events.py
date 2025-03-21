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
from rmf2_scheduler.data import action_type, Event, Time
from rmf2_scheduler.fastapi.deps import SchedulerDep
from rmf2_scheduler.fastapi.schemas import Event as EventSchema
from rmf2_scheduler.fastapi.schemas import EventCreate as EventCreateSchema
from rmf2_scheduler.fastapi.schemas import EventUpdate as EventUpdateSchema
from rmf2_scheduler.fastapi.schemas import Message

router = APIRouter()


@router.get(
    '/',
    response_model=List[EventSchema],
    response_model_exclude_unset=True,
    response_model_exclude_none=True,
)
def read_events(
    task_scheduler: SchedulerDep,
    start_time: datetime = datetime.now(),
    end_time: datetime = datetime.now() + timedelta(days=1),
    offset: int = 0,
    limit: int = 100,
) -> List[EventSchema]:
    schedule_cache: ScheduleCache = LockedScheduleRO(task_scheduler).cache()

    events = schedule_cache.lookup_events(Time(start_time), Time(end_time))

    # if not result:
    #     raise HTTPException(status_code=404, detail=error)

    response = []
    for event in events:
        response.append(event.json())
    return response


@router.post(
    '/',
    response_model=EventSchema,
    response_model_exclude_unset=True,
    response_model_exclude_none=True,
)
def create_event(
    task_scheduler: SchedulerDep,
    event_create: EventCreateSchema,
) -> EventSchema:
    event_j = event_create.model_dump(mode='json')
    event_j['id'] = str(uuid4())

    try:
        event_data = Event.from_json(event_j)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    action = Action.create(action_type.EVENT_ADD, ActionPayload().event(event_data))
    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    schedule_cache: ScheduleCache = LockedScheduleRO(task_scheduler).cache()
    event = schedule_cache.get_event(event_data.id)
    return event.json()


@router.get(
    '/{uuid}',
    response_model=EventSchema,
    response_model_exclude_unset=True,
    response_model_exclude_none=True,
)
def read_event(uuid: str, task_scheduler: SchedulerDep) -> EventSchema:
    schedule_cache: ScheduleCache = LockedScheduleRO(task_scheduler).cache()

    if not schedule_cache.has_event(uuid):
        raise HTTPException(status_code=404, detail="Event doesn't exist")

    event = schedule_cache.get_event(uuid)
    return event.json()


@router.put('/{uuid}', response_model=Message)
def update_event(
    task_scheduler: SchedulerDep,
    uuid: str,
    event_update: EventUpdateSchema,
) -> Message:
    event_j = event_update.model_dump(mode='json')
    event_j['id'] = uuid

    try:
        event_data = Event.from_json(event_j)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    action = Action.create(action_type.EVENT_UPDATE, ActionPayload().event(event_data))
    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Event updated successfully!')


@router.delete('/{uuid}', response_model=Message)
def delete_event(
    task_scheduler: SchedulerDep,
    uuid: str,
) -> Message:
    action = Action.create(action_type.EVENT_DELETE, ActionPayload().id(uuid))
    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Event delete successfully!')
