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

from contextlib import asynccontextmanager
from typing import List
import json

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from rmf2_scheduler import (
    SchedulerOptions,
    ExecutorData,
    TaskExecutor,
    ProcessExecutor,
    TaskExecutorManager
)
from rmf2_scheduler.data import Task, Process
from rmf2_scheduler.fastapi import scheduler_api_router
from rmf2_scheduler.fastapi.scheduler import make_scheduler
from rmf2_scheduler.storage import ScheduleStream
from rmf2_scheduler.utils import TreeConversion
from .task_orchestrator_process_executor import TOProcessExecutor
from .ihi_task_executors import (
    DummyTaskExecutor,
    MAPFTaskExecutor,
    GoToAMRTaskExecutor,
    WaitAMRTaskExecutor,
    WareHouseTaskExecutor,
)

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Define startup and shutdown actions."""
    print("Starting Scheduler")

    tem = TaskExecutorManager()
    task_executors = {
        "ihi/go_to_amr": GoToAMRTaskExecutor(),
        "ihi/wait_amr": WaitAMRTaskExecutor(),
        "ihi/warehouse_task": WareHouseTaskExecutor(),
        "rmf2/mapf": MAPFTaskExecutor(),
        "ihi/dummy": DummyTaskExecutor(),
    }
    for task_type, task_executor in task_executors.items():
        tem.load(task_type, task_executor);

    tpe = TOProcessExecutor(
        tem,
        amqp_host="localhost",
        amqp_port=5672,
        amqp_exchange="@RECEIVE@",
        to_queue="@RECEIVE@-event_mgr"
    )

    with make_scheduler(
        SchedulerOptions(),
        ScheduleStream.create_default(
            "http://localhost:9090/ngsi-ld",
        ),
        tpe,
        tem
    ):
        # Start AMQP listener
        tpe.start_listener()
        yield
        tpe.stop_listener()


app = FastAPI(
    title='RMF2 Task Scheduler Service',
    lifespan=lifespan,
)

# Add CORS Middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_credentials=True,
    allow_methods=['*'],
    allow_headers=['*'],
)

app.include_router(scheduler_api_router)
