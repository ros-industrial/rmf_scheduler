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

from __future__ import annotations

from datetime import datetime, timedelta
from typing import Any, Dict, List, Literal, Optional, Tuple

from fastapi._compat import PYDANTIC_V2
from pydantic import BaseModel, Extra, UUID4

if PYDANTIC_V2:
    from pydantic import JsonValue
else:
    import json

    def _model_dump(
        model: BaseModel, mode: Literal['json', 'python'] = 'python', **kwargs: Any
    ) -> Any:
        if mode == 'python':
            return model.dict(**kwargs)
        else:
            return json.loads(model.json())

    BaseModel.model_dump = _model_dump

    JsonValue = Dict[str, Any]


class EventBase(BaseModel):
    type: str  # noqa: A003
    description: Optional[str] = None
    start_time: datetime
    end_time: Optional[datetime] = None


class EventCreate(EventBase):
    class Config:
        extra = Extra.forbid


class EventUpdate(EventBase):
    class Config:
        extra = Extra.forbid


class Event(EventBase):
    id: str  # noqa: A003
    series_id: Optional[str] = None
    process_id: Optional[str] = None


class TaskBase(EventBase):
    resource_id: Optional[str] = None
    deadline: Optional[datetime] = None
    task_details: Optional[JsonValue] = None


class TaskCreate(TaskBase):
    class Config:
        extra = Extra.forbid


class TaskUpdate(TaskBase):
    class Config:
        extra = Extra.forbid


class Task(TaskBase):
    id: str  # noqa: A003
    series_id: Optional[str] = None
    process_id: Optional[str] = None
    status: str
    planned_start_time: Optional[datetime] = None
    planned_end_time: Optional[datetime] = None
    estimated_duration: Optional[timedelta] = None
    actual_start_time: Optional[datetime] = None
    actual_end_time: Optional[datetime] = None


class Edge(BaseModel):
    id: str  # noqa: A003
    type: str = 'hard'  # noqa: A003


class Dependency(BaseModel):
    id: str  # noqa: A003
    needs: List[Edge]


class ProcessBase(BaseModel):
    graph: List[Dependency]


class ProcessCreate(ProcessBase):
    class Config:
        extra = Extra.forbid


class ProcessUpdate(ProcessBase):
    class Config:
        extra = Extra.forbid
    status: Optional[str]
    current_events: List[str]


class Process(ProcessBase):
    id: str  # noqa: A003
    status: Optional[str] = ''
    series_id: Optional[str] = ''
    process_details: Optional[JsonValue] = None
    current_events: List[str] = []


class Schedule(BaseModel):
    class Config:
        extra = Extra.forbid

    tasks: List[Task]
    processes: List[Process]


class EventPayload(EventBase):
    class Config:
        extra = Extra.forbid

    id: UUID4  # noqa: A003


class TaskPayload(TaskBase):
    class Config:
        extra = Extra.forbid

    id: UUID4  # noqa: A003


class ProcessPayload(ProcessBase):
    class Config:
        extra = Extra.forbid

    id: UUID4  # noqa: A003
    process_details: Optional[JsonValue] = {}


class Occurrence(BaseModel):
    id: str  # noqa: A003
    time: datetime


class SeriesBase(BaseModel):
    type: str  # noqa: A003
    cron: str  # noqa: A003
    timezone: str  # noqa: A003
    until: Optional[datetime] = None


class Series(SeriesBase):
    class Config:
        extra = Extra.forbid

    occurrences: List[Occurrence]
    exception_ids: Optional[List[str]] = []
    id: UUID4  # noqa: A003


class SeriesPayload(SeriesBase):
    id: UUID4  # noqa: A003
    occurrences: List[Occurrence]


class SeriesCreate(SeriesPayload):
    class Config:
        extra = Extra.forbid


# Series helper classes
class SeriesExpandUntil(BaseModel):
    series_id: str  # noqa: A003


class SeriesUpdateCron(BaseModel):
    series_id: str  # noqa: A003
    cron_string: str  # noqa: A003


class SeriesUpdateOccurrenceTime(BaseModel):
    series_id: str  # noqa: A003
    occurrence_id: str  # noqa: A003
    time: datetime


class SeriesDeleteOccurrence(BaseModel):
    series_id: str  # noqa: A003
    occurrence_id: str  # noqa: A003


class ScheduleAction(BaseModel):
    class Config:
        extra = Extra.forbid

    type: str  # noqa: A003
    # Payload
    id: Optional[str] = None  # noqa: A003
    event: Optional[EventPayload] = None
    task: Optional[TaskPayload] = None
    process: Optional[ProcessPayload] = None
    node_id: Optional[str] = None
    source_id: Optional[str] = None
    destination_id: Optional[str] = None
    edge_type: Optional[str] = None
    series_add: Optional[SeriesPayload] = None
    series_expand_until: Optional[SeriesExpandUntil] = None
    series_update_cron: Optional[SeriesUpdateCron] = None
    series_update_occurrence_time: Optional[SeriesUpdateOccurrenceTime] = None
    series_delete_occurrence: Optional[SeriesDeleteOccurrence] = None
    series_delete: Optional[str] = None


class Message(BaseModel):
    """Schema for Message."""

    message: str  # noqa: A003
