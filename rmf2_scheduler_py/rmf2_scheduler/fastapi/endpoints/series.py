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
from rmf2_scheduler.data import Series, SeriesDeleteOccurrence, SeriesUpdateOccurrenceTime, Time
import rmf2_scheduler.data.action_type as action_type
from rmf2_scheduler.fastapi.deps import SchedulerDep
from rmf2_scheduler.fastapi.schemas import Message
from rmf2_scheduler.fastapi.schemas import Series as SeriesSchema
from rmf2_scheduler.fastapi.schemas import SeriesCreate as SeriesCreateSchema
from rmf2_scheduler.fastapi.schemas import SeriesDeleteOccurrence as SeriesDeleteOccurrenceSchema
from rmf2_scheduler.fastapi.schemas import SeriesUpdateOccurrenceTime \
    as SeriesUpdateOccurrenceTimeSchema

router = APIRouter()


@router.get(
    '/',
    response_model=List[SeriesSchema],
    response_model_exclude_unset=True,
    response_model_exclude_none=True,
)
def read_series(
    task_scheduler: SchedulerDep,
    start_time: Optional[datetime] = None,
    end_time: Optional[datetime] = None,
    offset: int = 0,
    limit: int = 100,
) -> List[SeriesSchema]:
    schedule_cache: ScheduleCache = LockedScheduleRO(task_scheduler).cache()
    query_start_time = Time(start_time) if start_time else Time(datetime.now() - timedelta(days=1))

    try:
        series_list = schedule_cache.lookup_series(query_start_time)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    return [series.json() for series in series_list]


@router.post(
    '/',
    response_model=SeriesSchema,
    response_model_exclude_unset=True,
    response_model_exclude_none=True,
)
def create_series(
    task_scheduler: SchedulerDep,
    series_create: SeriesCreateSchema,
) -> SeriesSchema:
    series_j = series_create.model_dump(mode='json')
    series_j['id'] = str(uuid4())
    print(f'SERIES ADD {series_j}')

    try:
        series_data = Series.from_json(series_j)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    action = Action.create(
        action_type.SERIES_ADD, ActionPayload().series(series_data)
    )
    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    schedule_cache: ScheduleCache = LockedScheduleRO(task_scheduler).cache()

    series = schedule_cache.get_series(series_data.id())
    return series.json()


@router.put('/update_occurrence_time', response_model=Message)
def update_occurrence_time(
    task_scheduler: SchedulerDep,
    series_id: str,
    occurrence_id: str,
    occurrence_time: datetime,
) -> Message:

    action = Action.create(action_type.SERIES_UPDATE_OCCURRENCE_TIME,
                           ActionPayload().id(series_id)
                           .occurrence_id(occurrence_id)
                           .occurrence_time(Time(occurrence_time)))
    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Task updated successfully!')


@router.delete('/{uuid}', response_model=Message)
def delete_series(
    task_scheduler: SchedulerDep,
    uuid: str,
) -> Message:
    action = Action.create(action_type.SERIES_DELETE, ActionPayload().id(uuid))
    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Series delete successfully!')


@router.delete('/occurrence', response_model=Message)
def delete_series_occurrence(
    task_scheduler: SchedulerDep,
    series_id: str,
    occurrence_id: str
) -> Message:
    action = Action.create(
        action_type.SERIES_DELETE_OCCURRENCE,
        ActionPayload().id(series_id).occurrence_id(occurrence_id))
    result, error = task_scheduler.perform(action)
    if not result:
        raise HTTPException(status_code=404, detail=error)

    return Message(message='Task delete successfully!')
