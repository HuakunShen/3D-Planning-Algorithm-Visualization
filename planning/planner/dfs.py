from typing import Tuple, List, Callable, Set, Dict

import numpy as np

from planning.map.map import BuildingMap
from planning.map.type import Coor
from planning.map.util import get_3d_neighbors, get_3d_neighbors_brute, euclidean_distance
from planning.planner.planner import Planner


class DFSPlanner(Planner):
    name: str = "DFS"

    def plan_impl(self, src: Tuple, target: Tuple, consider_corner: bool = True):
        super().plan_impl(src, target)
        get_neighbor_fn = get_3d_neighbors_brute if consider_corner else get_3d_neighbors
        stack = [src]
        self.visited = set()
        self.solved = False
        parents = {src: None}
        curr = None
        while len(stack) != 0:
            curr = stack.pop()
            if curr == target:
                self.solved = True
                break
            if curr not in self.visited:
                self.visited.add(curr)
                for neighbor in get_neighbor_fn(curr, self.map.shape):
                    if neighbor not in self.visited and self.map.is_free(neighbor):
                        parents[neighbor] = curr
                        stack.append(neighbor)

        self.path: List[Coor] = []
        while curr is not None:
            self.path.append(curr)
            curr = parents[curr]
        return self.solved, self.visited, self.path


def depth_limited_search(node: Coor, depth: int, target: Coor, get_neighbor_fn: Callable, shape: np.array,
                         visited: Set, parents: Dict):
    if depth == 0:
        if node == target:
            return node, True
        else:
            return None, True
    else:
        any_remaining = False
        for neighbor in get_neighbor_fn(node, shape):
            if neighbor not in visited:
                visited.add(neighbor)
                parents[neighbor] = node
                found, remaining = depth_limited_search(neighbor, depth - 1, target, get_neighbor_fn, shape, visited,
                                                        parents)
                if found is not None:
                    return found, True
                if remaining:
                    any_remaining = True
        return None, any_remaining


def solve_iterative_dfs(bmap: BuildingMap, src: Tuple, target: Tuple, consider_corner: bool = True, depth: int = None):
    get_neighbor_fn = get_3d_neighbors_brute if consider_corner else get_3d_neighbors
    max_depth = np.sum(bmap.shape)
    depth = depth if depth is not None else max_depth
    visited = set()
    parents = {src: None}
    solved = False
    found = None
    for d in range(0, depth):
        parents = {src: None}
        visited = set()
        found, remaining = depth_limited_search(src, d, target, get_neighbor_fn, bmap.shape, visited, parents)
        if found is not None:
            solved = True
            break
        elif not remaining:
            break
    path: List[Coor] = []
    curr = found
    while curr is not None:
        print("find path")
        path.append(curr)
        curr = parents[curr]
    return solved, visited, path
