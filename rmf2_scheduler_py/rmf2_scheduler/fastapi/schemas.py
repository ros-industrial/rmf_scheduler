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
from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Extra, JsonValue, UUID4


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
    series_id: Optional[str]
    process_id: Optional[str]
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


class Process(ProcessBase):
    id: str  # noqa: A003


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


class Message(BaseModel):
    """Schema for Message."""

    message: str
