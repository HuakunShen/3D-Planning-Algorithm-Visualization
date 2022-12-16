import numpy as np
from ..map.map import Map
from typing import Tuple, List
from abc import ABC, abstractmethod


class Planner(ABC):
    def __init__(self, m: Map, src: Tuple[int, int], target: Tuple[int, int]) -> None:
        self.map = m
        self.src = src
        self.target = target
        self.path: List[np.array] = []

    @abstractmethod
    def plan(self):
        raise NotImplementedError

    @abstractmethod
    def vis(self):
        raise NotImplementedError

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
