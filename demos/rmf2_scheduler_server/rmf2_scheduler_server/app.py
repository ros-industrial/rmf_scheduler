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
from threading import Thread

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .api import api_router
from .scheduler import task_scheduler, system_time_executor

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Define startup and shutdown actions."""
    # Create a background thread for the task scheduler
    task_scheduler_thread = Thread(target=system_time_executor.spin)
    task_scheduler_thread.start()

    yield

    # Safely shut down the task scheduler
    system_time_executor.stop()
    task_scheduler_thread.join()


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

app.include_router(api_router)
