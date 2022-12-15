from ..map.map import Map, BuildingMap
from typing import Tuple, List
import numpy as np
from .planner import Planner
from ..map.util import get_3d_neighbors, get_3d_neighbors_brute


class Dijkstra(Planner):
    def __init__(self, m: Map, src: Tuple[int, int], target: Tuple[int, int]) -> None:
        super().__init__(m, src, target)

    def plan(self):
        pass


# solave bfs
def solve_bfs(bmap: BuildingMap, src: Tuple, target: Tuple, consider_corner: bool=True):
    # neighbors = bmap.free_graph.neighbors(bmap.free_graph_pos_id_map[src])
    # print(len(list(neighbors)))
    queue = [src]
    visited = set()
    # pos_to_id_map = bmap.free_graph_pos_id_map
    pt = src
    parents = {pt: None}
    solved = False
    while len(queue) > 0:
        pt = queue.pop(0)
        if pt in visited:
            continue
        visited.add(pt)
        if (np.array(pt) == np.array(target)).all():
            solved = True
            break
        # for neighbor in bmap.free_graph.neighbors(bmap.free_graph_pos_id_map[pt]):
        #     neighbor_coor = bmap.free_graph_id_pos_map[neighbor]
        # for neighbor in get_3d_neighbors(pt, bmap.shape):
        if consider_corner:
            for neighbor in get_3d_neighbors_brute(pt, bmap.shape):
                if neighbor not in visited and bmap.is_free(neighbor):
                    parents[neighbor] = pt
                    queue.append(neighbor)
        else:
            for neighbor in get_3d_neighbors(pt, bmap.shape):
                if neighbor not in visited and bmap.is_free(neighbor):
                    parents[neighbor] = pt
                    queue.append(neighbor)

    # trace back and find the path
    cur = pt
    path: List[Tuple[int, int, int]] = []
    while cur is not None:
        path.append(cur)
        cur = parents[cur]
    return solved, visited, path
