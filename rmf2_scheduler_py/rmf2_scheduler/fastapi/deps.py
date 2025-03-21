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

from typing import Generator

from fastapi import Depends

from rmf2_scheduler import Scheduler
from rmf2_scheduler.fastapi.scheduler import make_scheduler_session
from typing_extensions import Annotated


def _get_scheduler() -> Generator[Scheduler, None, None]:
    with make_scheduler_session() as scheduler:
        yield scheduler


SchedulerDep = Annotated[Scheduler, Depends(_get_scheduler)]
