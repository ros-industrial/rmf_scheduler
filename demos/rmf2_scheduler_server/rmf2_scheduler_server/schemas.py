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

from pydantic import BaseModel, Json, Extra


class EventBase(BaseModel):
    type: str
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

    id: str
    series_id: Optional[str] = None
    process_id: Optional[str] = None


class TaskBase(EventBase):

    resource_id: Optional[str] = None
    deadline: Optional[datetime] = None
    task_details: Optional[Json] = None


class TaskCreate(TaskBase):

    class Config:
        extra = Extra.forbid


class TaskUpdate(TaskBase):

    class Config:
        extra = Extra.forbid


class Task(TaskBase):

    id: str
    series_id: Optional[str]
    process_id: Optional[str]
    status: str
    planned_start_time: Optional[datetime] = None
    planned_end_time: Optional[datetime] = None
    estimated_duration: Optional[timedelta] = None
    actual_start_time: Optional[datetime] = None
    actual_end_time: Optional[datetime] = None


class ScheduleAction(BaseModel):
    class Config:
        extra = Extra.forbid

    type: int
    payload: Any
