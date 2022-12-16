import heapq
from collections import defaultdict

from ..map.map import Map, BuildingMap
from typing import Tuple, List, Callable
import numpy as np
from .planner import Planner
from ..map.type import Coor
from ..map.util import get_3d_neighbors, get_3d_neighbors_brute, euclidean_distance
from numba import njit


class DijkstraPlanner(Planner):
    name: str = "Dijkastra"

    def plan_impl(self, src: Tuple, target: Tuple, consider_corner: bool = True):
        super().plan_impl(src, target)
        # neighbors = bmap.free_graph.neighbors(bmap.free_graph_pos_id_map[src])
        # print(len(list(neighbors)))
        queue = [src]
        self.visited = set()
        # pos_to_id_map = bmap.free_graph_pos_id_map
        pt = src
        parents = {pt: None}
        self.solved = False
        get_neighbor_fn = get_3d_neighbors_brute if consider_corner else get_3d_neighbors
        while len(queue) > 0:
            pt = queue.pop(0)
            if pt in self.visited:
                continue
            self.visited.add(pt)
            if (np.array(pt) == np.array(target)).all():
                self.solved = True
                break
            # for neighbor in bmap.free_graph.neighbors(bmap.free_graph_pos_id_map[pt]):
            #     neighbor_coor = bmap.free_graph_id_pos_map[neighbor]
            # for neighbor in get_3d_neighbors(pt, bmap.shape):
            for neighbor in get_neighbor_fn(pt, self.map.shape):
                if neighbor not in self.visited and self.map.is_free(neighbor):
                    parents[neighbor] = pt
                    queue.append(neighbor)
        # trace back and find the path
        cur = pt
        self.path: List[Coor] = []
        while cur is not None:
            self.path.append(cur)
            cur = parents[cur]
        return self.solved, self.visited, self.path
