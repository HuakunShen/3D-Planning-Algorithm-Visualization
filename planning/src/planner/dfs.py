from typing import Tuple, List, Callable, Set, Dict

import numpy as np

from planning.src.map import BuildingMap, Coor
from planning.src.map.util import get_3d_neighbors, get_3d_neighbors_brute


def solve_dfs(bmap: BuildingMap, src: Tuple, target: Tuple, consider_corner: bool = True):
    get_neighbor_fn = get_3d_neighbors_brute if consider_corner else get_3d_neighbors
    stack = [src]
    discovered = set()
    solved = False
    parents = {src: None}
    curr = None
    while len(stack) != 0:
        curr = stack.pop()
        if curr == target:
            solved = True
            break
        if curr not in discovered:
            discovered.add(curr)
            for neighbor in get_neighbor_fn(curr, bmap.shape):
                if neighbor not in discovered:
                    parents[neighbor] = curr
                    stack.append(neighbor)

    path: List[Coor] = []
    while curr is not None:
        path.append(curr)
        curr = parents[curr]
    return solved, discovered, path


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
