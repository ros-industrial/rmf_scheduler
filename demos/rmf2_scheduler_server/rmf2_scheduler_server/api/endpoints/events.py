# Copyright 2024 ROS Industrial Consortium Asia Pacific
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
from rmf2_scheduler.data import Event, Time, json_serializer
from rmf2_scheduler.cache import EventAction
from rmf2_scheduler.cache.event_action import Payload as EventActionPayload
from rmf2_scheduler.cache.schedule_action import ActionType
from rmf2_scheduler_server.scheduler import task_scheduler
from rmf2_scheduler_server.schemas import (
    EventCreate as EventCreateSchema,
    Event as EventSchema,
)

router = APIRouter()

@router.get(
    '/',
    response_model=List[EventSchema],
    response_model_exclude_unset=True,
    response_model_exclude_none=True
)
def read_events(
    start_time: datetime = datetime.now(),
    end_time: datetime = datetime.now(),
    offset: int = 0,
    limit: int = 100
) -> List[EventSchema]:
    result, error, events, _, _, _ = task_scheduler.get_schedule(
        Time(start_time),
        Time(end_time),
        offset,
        limit
    );
    if not result:
        raise HTTPException(status_code=404, detail=error)

    response = []
    for event in events:
        response.append(json_serializer.serialize(event))
    return response

@router.post(
    '/',
    response_model=EventSchema,
    response_model_exclude_unset=True,
    response_model_exclude_none=True
)
def create_event(
    event_create: EventCreateSchema
) -> EventSchema:
    event_j = event_create.model_dump(mode='json')
    event_j["id"] = str(uuid4())

    event_data = Event()
    json_serializer.deserialize(event_j, event_data)
    result, error = task_scheduler.perform(EventAction(
        ActionType.EVENT_ADD,
        EventActionPayload().event(event_data)
    ))
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return json_serializer.serialize(event_data)
