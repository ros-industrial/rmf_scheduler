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

from typing import List
import json

from rmf2_scheduler import (
    SchedulerOptions,
    ExecutorData,
    TaskExecutor,
    ProcessExecutor,
    TaskExecutorManager
)
from rmf2_scheduler.data import Task, Process
from rmf2_scheduler.storage import ScheduleStream
from rmf2_scheduler.utils import TreeConversion

class DummyTaskExecutor(TaskExecutor):
    def __init__(self):
        super().__init__()

    def build(self, task: Task):
        executor_data = ExecutorData();
        return True, None, executor_data

    def start(self, id: str, executor_data: ExecutorData):
        return True, "Undefined"

class MAPFTaskExecutor(TaskExecutor):
    def __init__(self):
        super().__init__()

    def build(self, task: Task):
        executor_data = ExecutorData();
        coorindates = task.task_details["coordinates"]
        asset_name = task.resource_id
        out = (
            "SubTree " +
            f"ID=\"ReplaceMAPF\" task_id=\"{task.id}\" " +
            "bt_id=\"{bt_id}\" task_status=\"{task_status}\" connection=\"{connection}\" " +
            f"coordinates=\"{coorindates}\" " +
            f"asset_name=\"{asset_name}\""
        )
        executor_data.set_data_as_string(out)
        return True, None, executor_data

    def start(self, id: str, executor_data: ExecutorData):
        return True, "Undefined"

class GoToAMRTaskExecutor(TaskExecutor):
    def __init__(self):
        super().__init__()

    def build(self, task: Task):
        executor_data = ExecutorData();
        coorindates = task.task_details["coordinates"]
        asset_name = task.resource_id
        out = (
            "SubTree " +
            f"ID=\"GoToAMR\" task_id=\"{task.id}\" " +
            "bt_id=\"{bt_id}\" task_status=\"{task_status}\" connection=\"{connection}\" " +
            f"coordinates=\"{coorindates}\" " +
            f"asset_name=\"{asset_name}\""
        )
        executor_data.set_data_as_string(out)
        return True, None, executor_data

    def start(self, id: str, executor_data: ExecutorData):
        return True, "Undefined"

class WaitAMRTaskExecutor(TaskExecutor):
    def __init__(self):
        super().__init__()

    def build(self, task: Task):
        executor_data = ExecutorData();
        asset_name = task.resource_id
        out = (
            "SubTree " +
            f"ID=\"WaitAMR\" task_id=\"{task.id}\" " +
            "bt_id=\"{bt_id}\" task_status=\"{task_status}\" connection=\"{connection}\" " +
            f"asset_name=\"{asset_name}\""
        )
        executor_data.set_data_as_string(out)
        return True, None, executor_data

    def start(self, id: str, executor_data: ExecutorData):
        return True, "Undefined"

class WareHouseTaskExecutor(TaskExecutor):
    def __init__(self):
        super().__init__()

    def build(self, task: Task):
        executor_data = ExecutorData();
        asset_name = task.resource_id
        out = (
            "SubTree " +
            f"ID=\"WareHouseTask\" task_id=\"{task.id}\" " +
            "bt_id=\"{bt_id}\" task_status=\"{task_status}\" connection=\"{connection}\" " +
            f"asset_name=\"{asset_name}\""
        )
        executor_data.set_data_as_string(out)
        return True, None, executor_data

    def start(self, id: str, executor_data: ExecutorData):
        return True, "Undefined"
