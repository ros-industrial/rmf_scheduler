from __future__ import annotations

import typing

import rmf2_scheduler.core.data

__all__ = ['TreeConversion']
class TreeConversion:
    """
    
        DAG to Behavior Tree
        
    """
    def __init__(self) -> None:
        ...
    @typing.overload
    def convert_to_tree(self, arg0: rmf2_scheduler.core.data.Graph, arg1: typing.Callable[[str], str]) -> str:
        ...
    @typing.overload
    def convert_to_tree(self, arg0: str, arg1: rmf2_scheduler.core.data.Graph, arg2: typing.Callable[[str], str]) -> str:
        ...
