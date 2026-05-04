from __future__ import annotations

import datetime
import typing

from . import action_type, record_action_type, record_data_type

__all__ = ['ChangeAction', 'Duration', 'Edge', 'Event', 'Graph', 'Node', 'Occurrence', 'Process', 'ScheduleAction', 'ScheduleChangeRecord', 'Series', 'SeriesDeleteOccurrence', 'SeriesExpandUntil', 'SeriesUpdateCron', 'SeriesUpdateOccurrenceTime', 'Task', 'Time', 'TimeWindow', 'action_type', 'record_action_type', 'record_data_type']
class ChangeAction:
    """
    
        ChangeActions for schedule
        
    """
    __hash__: typing.ClassVar[None] = None
    action: str
    id: str
    def __eq__(self, arg0: ChangeAction) -> bool:
        ...
    def __ne__(self, arg0: ChangeAction) -> bool:
        ...
class Duration:
    """
    
        Duration class
        
    """
    __hash__: typing.ClassVar[None] = None
    @staticmethod
    def from_json(arg0: json) -> Duration:
        ...
    @staticmethod
    def from_nanoseconds(nanoseconds: int) -> Duration:
        ...
    @staticmethod
    def from_seconds(seconds: float) -> Duration:
        ...
    @staticmethod
    def max() -> Duration:
        ...
    def __add__(self, arg0: Duration) -> Duration:
        ...
    def __eq__(self, arg0: Duration) -> bool:
        ...
    def __ge__(self, arg0: Duration) -> bool:
        ...
    def __gt__(self, arg0: Duration) -> bool:
        ...
    def __iadd__(self, arg0: Duration) -> Duration:
        ...
    def __imul__(self, arg0: float) -> Duration:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, seconds: int, nanoseconds: int) -> None:
        ...
    @typing.overload
    def __init__(self, nanoseconds: int) -> None:
        ...
    @typing.overload
    def __init__(self, duration: datetime.timedelta) -> None:
        ...
    def __isub__(self, arg0: Duration) -> Duration:
        ...
    def __le__(self, arg0: Duration) -> bool:
        ...
    def __lt__(self, arg0: Duration) -> bool:
        ...
    def __mul__(self, arg0: float) -> Duration:
        ...
    def __ne__(self, arg0: Duration) -> bool:
        ...
    def __sub__(self, arg0: Duration) -> Duration:
        ...
    def json(self) -> json:
        ...
    def nanoseconds(self) -> int:
        ...
    def seconds(self) -> float:
        ...
    def to_timedelta(self) -> datetime.timedelta:
        ...
class Edge:
    """
    
        Edge class
        
    """
    __hash__: typing.ClassVar[None] = None
    type: str
    def __eq__(self, arg0: Edge) -> bool:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, type: str) -> None:
        ...
    def __ne__(self, arg0: Edge) -> bool:
        ...
class Event:
    """
    
        Basic information about the Event
        
    """
    __hash__: typing.ClassVar[None] = None
    description: str | None
    duration: Duration | None
    id: str
    process_id: str | None
    series_id: str | None
    start_time: Time
    type: str
    @staticmethod
    def from_json(arg0: json) -> Event:
        ...
    def __eq__(self, arg0: Event) -> bool:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, id: str, type: str, start_time: Time) -> None:
        ...
    @typing.overload
    def __init__(self, id: str, type: str, description: str, start_time: Time, duration: Duration, series_id: str, process_id: str) -> None:
        ...
    def __ne__(self, arg0: Event) -> bool:
        ...
    def json(self) -> json:
        ...
class Graph:
    """
    
        Graph class
        
    """
    __hash__: typing.ClassVar[None] = None
    @staticmethod
    def from_json(arg0: json) -> Graph:
        ...
    def __eq__(self, arg0: Graph) -> bool:
        ...
    def __init__(self) -> None:
        ...
    def __ne__(self, arg0: Graph) -> bool:
        ...
    def add_edge(self, source: str, destination: str, edge: Edge = ...) -> None:
        ...
    def add_node(self, arg0: str) -> None:
        ...
    def delete_edge(self, source: str, destination: str) -> None:
        ...
    def delete_node(self, arg0: str) -> None:
        ...
    def dump(self) -> str:
        ...
    def empty(self) -> bool:
        ...
    def get_all_nodes(self) -> dict[str, Node]:
        ...
    def get_node(self, arg0: str) -> Node:
        ...
    def has_node(self, arg0: str) -> bool:
        ...
    def json(self) -> json:
        ...
    def prune(self) -> None:
        ...
    def update_node(self, arg0: str, arg1: str) -> None:
        ...
class Node:
    """
    
        Node class
        
    """
    __hash__: typing.ClassVar[None] = None
    def __eq__(self, arg0: Node) -> bool:
        ...
    def __init__(self, arg0: str) -> None:
        ...
    def __ne__(self, arg0: Node) -> bool:
        ...
    def id(self) -> str:
        ...
    def inbound_edges(self) -> dict[str, Edge]:
        ...
    def outbound_edges(self) -> dict[str, Edge]:
        ...
class Occurrence:
    """
    
        Additional information for a Process
        
    """
    __hash__: typing.ClassVar[None] = None
    @staticmethod
    def from_json(arg0: json) -> Occurrence:
        ...
    def __eq__(self, arg0: Occurrence) -> bool:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, _time: Time, id: str) -> None:
        ...
    def __ne__(self, arg0: Occurrence) -> bool:
        ...
    def json(self: Series) -> json:
        ...
class Process:
    """
    
        Additional information for a Process
        
    """
    __hash__: typing.ClassVar[None] = None
    current_events: set[str]
    graph: Graph
    id: str
    process_details: json
    status: str
    @staticmethod
    def from_json(arg0: json) -> Process:
        ...
    def __eq__(self, arg0: Process) -> bool:
        ...
    def __init__(self) -> None:
        ...
    def __ne__(self, arg0: Process) -> bool:
        ...
    def json(self) -> json:
        ...
class ScheduleAction:
    """
    
        Change Action to the schedule
        
    """
    __hash__: typing.ClassVar[None] = None
    destination_id: str | None
    edge_type: str | None
    event: Event
    id: str | None
    node_id: str | None
    process: Process
    series_add: Series
    series_delete: str | None
    series_delete_occurrence: SeriesDeleteOccurrence | None
    series_expand_until: SeriesExpandUntil | None
    series_update: Series | None
    series_update_cron: SeriesUpdateCron | None
    series_update_occurrence_process: tuple[str, Process] | None
    series_update_occurrence_task: tuple[str, Task] | None
    series_update_occurrence_time: SeriesUpdateOccurrenceTime | None
    series_update_until: tuple[str, Time] | None
    source_id: str | None
    task: Task
    type: str
    @staticmethod
    def from_json(arg0: json) -> ScheduleAction:
        ...
    def __eq__(self, arg0: ScheduleAction) -> bool:
        ...
    def __init__(self) -> None:
        ...
    def __ne__(self, arg0: ScheduleAction) -> bool:
        ...
    def json(self) -> json:
        ...
class ScheduleChangeRecord:
    """
    
        Records for the schedule changes
        
    """
    @staticmethod
    def squash(arg0: list[ScheduleChangeRecord]) -> ScheduleChangeRecord:
        ...
    def add(self, arg0: str, arg1: list[ChangeAction]) -> None:
        ...
    def get(self, arg0: str) -> list[ChangeAction]:
        ...
class Series:
    """
    
        Additional information for a Process
        
    """
    __hash__: typing.ClassVar[None] = None
    @staticmethod
    def from_json(arg0: json) -> Series:
        ...
    def __eq__(self, arg0: Series) -> bool:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, id: str, type: str, occurrence: ..., cron: str, timezone: str, until: Time = ...) -> None:
        ...
    def __ne__(self, arg0: Series) -> bool:
        ...
    def id(self) -> str:
        ...
    def json(self) -> json:
        ...
class SeriesDeleteOccurrence:
    """
    
        Information to delete an occurrence
        
    """
    __hash__: typing.ClassVar[None] = None
    @staticmethod
    def from_json(arg0: json) -> SeriesDeleteOccurrence:
        ...
    def __eq__(self, arg0: SeriesDeleteOccurrence) -> bool:
        ...
    def __init__(self) -> None:
        ...
    def __ne__(self, arg0: SeriesDeleteOccurrence) -> bool:
        ...
    def json(self) -> json:
        ...
class SeriesExpandUntil:
    """
    
        Information for expanding series
        
    """
    __hash__: typing.ClassVar[None] = None
    @staticmethod
    def from_json(arg0: json) -> SeriesExpandUntil:
        ...
    def __eq__(self, arg0: SeriesExpandUntil) -> bool:
        ...
    def __init__(self) -> None:
        ...
    def __ne__(self, arg0: SeriesExpandUntil) -> bool:
        ...
    def json(self) -> json:
        ...
class SeriesUpdateCron:
    """
    
        Information for updating a series' cron
        
    """
    __hash__: typing.ClassVar[None] = None
    @staticmethod
    def from_json(arg0: json) -> SeriesUpdateCron:
        ...
    def __eq__(self, arg0: SeriesUpdateCron) -> bool:
        ...
    def __init__(self) -> None:
        ...
    def __ne__(self, arg0: SeriesUpdateCron) -> bool:
        ...
    def json(self) -> json:
        ...
class SeriesUpdateOccurrenceTime:
    """
    
        Information to update an occurrence's time
        
    """
    __hash__: typing.ClassVar[None] = None
    @staticmethod
    def from_json(arg0: json) -> SeriesUpdateOccurrenceTime:
        ...
    def __eq__(self, arg0: SeriesUpdateOccurrenceTime) -> bool:
        ...
    def __init__(self) -> None:
        ...
    def __ne__(self, arg0: SeriesUpdateOccurrenceTime) -> bool:
        ...
    def json(self) -> json:
        ...
class Task(Event):
    """
    
        Additional information for a Task
        
    """
    __hash__: typing.ClassVar[None] = None
    actual_duration: Duration | None
    actual_start_time: Time | None
    deadline: Time | None
    estimated_duration: Duration | None
    planned_duration: Duration | None
    planned_start_time: Time | None
    resource_id: str | None
    status: str
    task_details: json
    @staticmethod
    def from_json(arg0: json) -> Task:
        ...
    def __eq__(self, arg0: Task) -> bool:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, id: str, type: str, start_time: Time, status: str) -> None:
        ...
    @typing.overload
    def __init__(self, event: Event, status: str) -> None:
        ...
    @typing.overload
    def __init__(self, id: str, type: str, description: str, start_time: Time, duration: Duration, series_id: str, process_id: str, resource_id: str, deadline: Time, status: str, planned_start_time: Time, planned_duration: Duration, estimated_duration: Duration, actual_start_time: Time, actual_duration: Duration, task_details: json) -> None:
        ...
    @typing.overload
    def __init__(self, event: Event, resource_id: str, deadline: Time, status: str, planned_start_time: Time, planned_duration: Duration, estimated_duration: Duration, actual_start_time: Time, actual_duration: Duration, task_details: json) -> None:
        ...
    def __ne__(self, arg0: Task) -> bool:
        ...
    def json(self) -> json:
        ...
class Time:
    """
    
        Time class
        
    """
    __hash__: typing.ClassVar[None] = None
    @staticmethod
    def from_ISOtime(arg0: str) -> Time:
        ...
    @staticmethod
    def from_json(arg0: json) -> Time:
        ...
    @staticmethod
    def from_localtime(localtime: str, timezone: str = 'UTC', fmt: str = '%b %d %H:%M:%S %Y') -> Time:
        ...
    @staticmethod
    def max() -> Time:
        ...
    def __add__(self, arg0: Duration) -> Time:
        ...
    def __eq__(self, arg0: Time) -> bool:
        ...
    def __ge__(self, arg0: Time) -> bool:
        ...
    def __gt__(self, arg0: Time) -> bool:
        ...
    def __iadd__(self, arg0: Duration) -> Time:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, seconds: int, nanoseconds: int) -> None:
        ...
    @typing.overload
    def __init__(self, nanoseconds: int) -> None:
        ...
    @typing.overload
    def __init__(self, time_point: datetime.datetime) -> None:
        ...
    def __isub__(self, arg0: Duration) -> Time:
        ...
    def __le__(self, arg0: Time) -> bool:
        ...
    def __lt__(self, arg0: Time) -> bool:
        ...
    def __ne__(self, arg0: Time) -> bool:
        ...
    @typing.overload
    def __sub__(self, arg0: Time) -> Duration:
        ...
    @typing.overload
    def __sub__(self, arg0: Duration) -> Time:
        ...
    def json(self) -> json:
        ...
    def nanoseconds(self) -> int:
        ...
    def seconds(self) -> float:
        ...
    def to_ISOtime(self) -> str:
        ...
    def to_datetime(self) -> datetime.timedelta:
        ...
    def to_localtime(self, timezone: str = 'UTC', fmt: str = '%b %d %H:%M:%S %Y') -> str:
        ...
class TimeWindow:
    """
    
        Time window
        
    """
    __hash__: typing.ClassVar[None] = None
    end: Time
    start: Time
    def __eq__(self, arg0: TimeWindow) -> bool:
        ...
    def __init__(self) -> None:
        ...
    def __ne__(self, arg0: TimeWindow) -> bool:
        ...
