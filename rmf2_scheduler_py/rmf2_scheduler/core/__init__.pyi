from __future__ import annotations
import typing
from . import cache
from . import data
from . import storage
__all__ = ['Estimator', 'Optimizer', 'ProcessExecutor', 'Scheduler', 'SchedulerOptions', 'SystemTimeAction', 'SystemTimeExecutor', 'TaskExecutor', 'cache', 'data', 'storage']
class Estimator:
    pass
class Optimizer:
    pass
class ProcessExecutor:
    pass
class Scheduler:
    """
    
        Scheduler Class
        
    """
    def __init__(self, options: SchedulerOptions, stream: storage.ScheduleStream, system_time_executor: SystemTimeExecutor, optimizer: Optimizer = None, estimator: Estimator = None, process_executor: ProcessExecutor = None, task_executor: TaskExecutor = None) -> None:
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
    pass
