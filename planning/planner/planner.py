import time

import numpy as np
from ..map.map import Map
from typing import Tuple, List
from abc import ABC, abstractmethod

from ..map.type import Coor
from ..vis import vis_with_plotly


class Planner(ABC):
    name: str = "Planner"

    def __init__(self, m: Map) -> None:
        self.map = m
        self.path: List[Coor] = []
        self.visited: List[Coor] = []
        # src and target will be updated each time plan method is called. We store it for building visualization
        self.src = None
        self.target = None
        self.solved = False
        self.time_taken: float = 0  # in seconds

    def plan(self, src: Coor, target: Coor) -> Tuple[bool, List, List[Coor]]:
        start = time.time()
        result = self.plan_impl(src, target)
        self.time_taken = time.time() - start
        return result

    def plan_impl(self, src: Coor, target: Coor) -> Tuple[bool, List, List[Coor]]:
        """Plan Path

        :return: (solved, visited, path)
        :rtype: Tuple[bool, List, List[Coor]]
        """
        self.src = src
        self.target = target
        return False, [], []

    def vis(self, marker_opacity: float = 0.3, marker_size: float = 2, show_visited: bool = True):
        fig = vis_with_plotly(self.map.map, self.src, self.target, path_pts=self.path,
                              visited=self.visited if show_visited else [],
                              marker_size=marker_size,
                              marker_opacity=marker_opacity, z_axis_range_upper_bound=np.max(self.map.map),
                              background_injection_fn=None)
        return fig

    def verify(self) -> bool:
        """Check if path is valid by checking if points in path are connected and are not obstacles
        nonzero value in map is considered as obstacle
        :return: whether path is valid
        :rtype: bool
        """
        for pt in self.path:
            if self.map.map(*pt) != 0:
                return False
        return True
