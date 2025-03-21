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

from contextlib import contextmanager
from threading import Thread
from typing import Optional

from rmf2_scheduler import (
    ProcessExecutor,
    Scheduler,
    SchedulerOptions,
    SystemTimeExecutor,
    TaskExecutorManager,
)
from rmf2_scheduler.storage import ScheduleStream

SCHEDULER: Optional[Scheduler] = None
SYSTEM_TIME_EXECUTOR: Optional[SystemTimeExecutor] = None
SCHEDULER_THREAD: Optional[Thread] = None


def init_scheduler(
    options: SchedulerOptions,
    stream: ScheduleStream,
    process_executor: ProcessExecutor,
    task_executor_manager: TaskExecutorManager,
):
    global SCHEDULER
    global SYSTEM_TIME_EXECUTOR

    SYSTEM_TIME_EXECUTOR = SystemTimeExecutor()

    SCHEDULER = Scheduler(
        options,
        stream,
        SYSTEM_TIME_EXECUTOR,
        process_executor=process_executor,
        task_executor_manager=task_executor_manager,
    )

    return SCHEDULER, SYSTEM_TIME_EXECUTOR


def start_scheduler():
    global SCHEDULER
    global SCHEDULER_THREAD
    global SYSTEM_TIME_EXECUTOR

    if SCHEDULER is None or SYSTEM_TIME_EXECUTOR is None:
        raise RuntimeError('Scheduler is not initialized!!')

    if SCHEDULER_THREAD:
        raise RuntimeError('Scheduler is already started!!')

    SCHEDULER_THREAD = Thread(target=SYSTEM_TIME_EXECUTOR.spin)
    SCHEDULER_THREAD.start()


def stop_scheduler():
    global SCHEDULER_THREAD
    global SYSTEM_TIME_EXECUTOR

    if SCHEDULER_THREAD is None:
        raise RuntimeError('Scheduler is not started!!')

    SYSTEM_TIME_EXECUTOR.stop()
    SCHEDULER_THREAD.join()


@contextmanager
def make_scheduler(
    options: SchedulerOptions,
    stream: ScheduleStream,
    process_executor: ProcessExecutor,
    task_executor_manager: TaskExecutorManager,
) -> Scheduler:
    scheduler, _ = init_scheduler(
        options, stream, process_executor, task_executor_manager
    )
    start_scheduler()
    yield scheduler
    stop_scheduler()


@contextmanager
def make_scheduler_session() -> Scheduler:
    global SCHEDULER
    global SCHEDULER_THREAD

    if SCHEDULER is None:
        raise RuntimeError('Scheduler is not initialized')

    if SCHEDULER_THREAD is None:
        raise RuntimeError('Scheduler is not started')

    yield SCHEDULER
