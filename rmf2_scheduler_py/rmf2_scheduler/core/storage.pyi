from __future__ import annotations
import rmf2_scheduler.core.cache
import rmf2_scheduler.core.data
import typing
__all__ = ['ScheduleStream']
class ScheduleStream:
    """
    
        Stream for the schedule
        
    """
    @staticmethod
    def create_default(arg0: str) -> ScheduleStream:
        ...
    def read_schedule(self, arg0: rmf2_scheduler.core.cache.ScheduleCache, arg1: rmf2_scheduler.core.data.TimeWindow) -> tuple:
        ...
    @typing.overload
    def write_schedule(self, arg0: rmf2_scheduler.core.cache.ScheduleCache, arg1: rmf2_scheduler.core.data.TimeWindow) -> tuple:
        ...
    @typing.overload
    def write_schedule(self, arg0: rmf2_scheduler.core.cache.ScheduleCache, arg1: list[rmf2_scheduler.core.data.ScheduleChangeRecord]) -> tuple:
        ...
