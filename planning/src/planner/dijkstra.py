from ..map.map import Map
from typing import Tuple

from .planner import Planner


class Dijkstra(Planner):
    def __init__(self, m: Map, src: Tuple[int, int], target: Tuple[int, int]) -> None:
        super().__init__(m, src, target)
    
    def plan(self):
        pass