from __future__ import annotations

import datetime
import typing

from . import cache, data, storage, utils

__all__ = ['Estimator', 'ExecutorData', 'LockedScheduleRO', 'LockedScheduleRW', 'Optimizer', 'ProcessExecutor', 'Scheduler', 'SchedulerOptions', 'SystemTimeAction', 'SystemTimeExecutor', 'TaskExecutor', 'TaskExecutorManager', 'cache', 'data', 'storage', 'utils']
class Estimator:
    pass
class ExecutorData:
    def __init__(self) -> None:
        ...
    def get_data_as_json(self) -> json | None:
        ...
    def get_data_as_string(self) -> str:
        ...
    def set_data_as_json(self, arg0: json) -> None:
        ...
    def set_data_as_string(self, arg0: str) -> None:
        ...
class LockedScheduleRO:
    """
    
        Acquire read-only schedule cache from the scheduler
        
    """
    def __init__(self, scheduler: Scheduler) -> None:
        ...
    def cache(self) -> cache.ScheduleCache:
        ...
class LockedScheduleRW:
    """
    
        Acquire read-write schedule from the scheduler
        
    """
    def __init__(self, scheduler: Scheduler) -> None:
        ...
    def cache(self) -> cache.ScheduleCache:
        ...
class Optimizer:
    pass
class ProcessExecutor:
    def __init__(self) -> None:
        ...
    def run_async(self, arg0: data.Process, arg1: list[data.Task]) -> tuple[bool, str]:
        ...
class Scheduler:
    """
    
        Scheduler Class
        
    """
    def __init__(self, options: SchedulerOptions, stream: storage.ScheduleStream, system_time_executor: SystemTimeExecutor, optimizer: Optimizer = None, estimator: Estimator = None, process_executor: ProcessExecutor = None, task_executor_manager: TaskExecutorManager = None) -> None:
        ...
    def get_schedule(self, arg0: data.Time, arg1: data.Time, arg2: int, arg3: int) -> tuple:
        ...
    @typing.overload
    def perform(self, arg0: cache.Action) -> tuple:
        ...
    @typing.overload
    def perform(self, arg0: data.ScheduleAction) -> tuple:
        ...
    @typing.overload
    def perform_batch(self, arg0: list[cache.Action]) -> tuple:
        ...
    @typing.overload
    def perform_batch(self, arg0: list[data.ScheduleAction]) -> tuple:
        ...
    @typing.overload
    def validate(self, arg0: cache.Action) -> tuple:
        ...
    @typing.overload
    def validate(self, arg0: data.ScheduleAction) -> tuple:
        ...
    @typing.overload
    def validate_batch(self, arg0: list[cache.Action]) -> tuple:
        ...
    @typing.overload
    def validate_batch(self, arg0: list[data.ScheduleAction]) -> tuple:
        ...
class SchedulerOptions:
    """
    
        SchedulerOptions Class
        
    """
    def __init__(self) -> None:
        ...
    @typing.overload
    def runtime_tick_period(self, arg0: data.Duration) -> SchedulerOptions:
        ...
    @typing.overload
    def runtime_tick_period(self) -> data.Duration:
        ...
    @typing.overload
    def static_cache_period(self, arg0: data.Duration) -> SchedulerOptions:
        ...
    @typing.overload
    def static_cache_period(self) -> data.Duration:
        ...
class SystemTimeAction:
    """
    
        SystemTimeAction
        
    """
    time: data.Time
    work: typing.Callable[[], None]
    def __init__(self) -> None:
        ...
class SystemTimeExecutor:
    """
    
        SystemTimeExecutor Class
        
    """
    def __init__(self) -> None:
        ...
    def add_action(self, arg0: SystemTimeAction) -> str:
        ...
    def delete_action(self, arg0: str) -> None:
        ...
    def spin(self) -> None:
        ...
    def stop(self) -> None:
        ...
class TaskExecutor:
    def __init__(self) -> None:
        ...
    def build(self, arg0: data.Task) -> tuple[bool, str, ExecutorData]:
        ...
    def notify_completion(self, arg0: str, arg1: bool, arg2: ExecutorData) -> None:
        ...
    def start(self, arg0: str, arg1: ExecutorData) -> tuple[bool, str]:
        ...
    @typing.overload
    def update(self, arg0: str, arg1: data.Duration) -> None:
        ...
    @typing.overload
    def update(self, arg0: str, arg1: float) -> None:
        ...
    @typing.overload
    def update(self, arg0: str, arg1: datetime.timedelta) -> None:
        ...
class TaskExecutorManager:
    def __init__(self) -> None:
        ...
    def dry_run(self, arg0: data.Task) -> tuple:
        ...
    def is_runnable(self, arg0: str) -> bool:
        ...
    def load(self, arg0: str, arg1: TaskExecutor) -> None:
        ...
    def notify_completion(self, arg0: str, arg1: bool, arg2: ExecutorData) -> None:
        ...
    def run_async(self, arg0: data.Task) -> tuple:
        ...
    def unload(self, arg0: str) -> None:
        ...
    def update(self, arg0: str, arg1: data.Duration) -> None:
        ...
