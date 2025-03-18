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

from rmf2_scheduler import Scheduler, SchedulerOptions, SystemTimeExecutor
from rmf2_scheduler.storage import ScheduleStream

system_time_executor = SystemTimeExecutor()

task_scheduler = Scheduler(
    SchedulerOptions(),
    ScheduleStream.create_default(
        "http://localhost:9090/ngsi-ld",
        ["rmf2/go_to_place", "ihi/palletization"]
    ),
    system_time_executor,
)
