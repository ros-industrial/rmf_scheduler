from __future__ import annotations
import rmf2_scheduler.core.data
import typing
__all__ = ['deserialize', 'serialize']
@typing.overload
def deserialize(arg0: json, arg1: rmf2_scheduler.core.data.Time) -> None:
    ...
@typing.overload
def deserialize(arg0: json, arg1: rmf2_scheduler.core.data.Duration) -> None:
    ...
@typing.overload
def deserialize(arg0: json, arg1: rmf2_scheduler.core.data.Event) -> None:
    ...
@typing.overload
def deserialize(arg0: json, arg1: rmf2_scheduler.core.data.Process) -> None:
    ...
@typing.overload
def serialize(arg0: rmf2_scheduler.core.data.Time) -> json:
    ...
@typing.overload
def serialize(arg0: rmf2_scheduler.core.data.Duration) -> json:
    ...
@typing.overload
def serialize(arg0: rmf2_scheduler.core.data.Event) -> json:
    ...
@typing.overload
def serialize(arg0: rmf2_scheduler.core.data.Graph) -> json:
    ...
@typing.overload
def serialize(arg0: rmf2_scheduler.core.data.Process) -> json:
    ...
