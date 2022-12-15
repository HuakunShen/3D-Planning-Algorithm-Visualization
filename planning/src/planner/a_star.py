from ..map.map import Coor
import heapq
from numba import njit
from typing import Tuple, Callable, List

from ..map.map import Map, BuildingMap
from .planner import Planner
from collections import defaultdict

from ..map.util import get_3d_neighbors, get_3d_neighbors_brute


class AStar(Planner):
    def __init__(self, m: Map, src: Tuple[int, int], target: Tuple[int, int]) -> None:
        super().__init__(m, src, target)

    def plan(self):
        pass


@njit
def manhattan_distance(coor1: Coor, coor2: Coor):
    return sum([abs(coor1[i] - coor2[i]) for i in range(len(coor1))])


@njit
def euclidean_distance(coor1: Coor, coor2: Coor):
    return sum([(coor1[i] - coor2[i]) ** 2 for i in range(len(coor1))]) ** 0.5


def solve_astar(bmap: BuildingMap, src: Tuple, target: Tuple, distance_fn: Callable = euclidean_distance,
                consider_corner: bool = True):
    """Solve 3D a star"""
    q = []
    heapq.heappush(q, (0, src))
    parents = {src: None}
    gscore = defaultdict(lambda: float('inf'))  # real cost
    gscore[src] = 0
    fscore = defaultdict(lambda: float('inf'))  # estimated cost
    fscore[src] = distance_fn(src, target)
    solved = False
    visited = set()
    visited.add(src)
    curr = None
    get_neighbor_fn = get_3d_neighbors_brute if consider_corner else get_3d_neighbors
    while len(q) != 0:
        # print(len(q))
        priority, curr = heapq.heappop(q)
        # print(len(q))
        if curr == target:
            solved = True
            break
        for neighbor in get_neighbor_fn(curr, bmap.shape):
            if neighbor not in visited and bmap.is_free(neighbor):
                visited.add(neighbor)
                tentative_gscore = gscore[curr] + distance_fn(curr, neighbor)
                gscore[neighbor] = tentative_gscore
                fscore[neighbor] = tentative_gscore + distance_fn(neighbor, target)
                if neighbor not in q:
                    parents[neighbor] = curr
                    heapq.heappush(q, (fscore[neighbor], neighbor))

    path: List[Coor] = []
    while curr is not None:
        path.append(curr)
        curr = parents[curr]

    return solved, visited, path
