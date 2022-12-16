from ..map.map import Coor
import heapq
from typing import Tuple, Callable, List

from ..map.map import Map, BuildingMap
from .planner import Planner
from collections import defaultdict

from ..map.util import get_3d_neighbors, get_3d_neighbors_brute, euclidean_distance


class AStarPlanner(Planner):
    name: str = "A*"

    def plan_impl(self, src: Tuple, target: Tuple, distance_fn: Callable = euclidean_distance,
                  consider_corner: bool = True):
        super().plan_impl(src, target)
        q = []
        heapq.heappush(q, (0, src))
        parents = {src: None}
        gscore = defaultdict(lambda: float('inf'))  # accumulated real cost
        gscore[src] = 0
        fscore = defaultdict(lambda: float('inf'))  # estimated cost
        fscore[src] = distance_fn(src, target)
        self.solved = False
        self.visited = set()
        self.visited.add(src)
        curr = None
        get_neighbor_fn = get_3d_neighbors_brute if consider_corner else get_3d_neighbors
        while len(q) != 0:
            priority, curr = heapq.heappop(q)
            if curr == target:
                self.solved = True
                break
            for neighbor in get_neighbor_fn(curr, self.map.shape):
                if neighbor not in self.visited and self.map.is_free(neighbor):
                    self.visited.add(neighbor)
                    tentative_gscore = gscore[curr] + distance_fn(curr, neighbor)
                    gscore[neighbor] = tentative_gscore
                    fscore[neighbor] = tentative_gscore + distance_fn(neighbor, target)
                    if neighbor not in q:
                        parents[neighbor] = curr
                        heapq.heappush(q, (fscore[neighbor], neighbor))

        self.path: List[Coor] = []
        while curr is not None:
            self.path.append(curr)
            curr = parents[curr]
        return self.solved, self.visited, self.path
