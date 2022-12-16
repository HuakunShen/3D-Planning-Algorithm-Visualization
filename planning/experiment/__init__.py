from typing import List

import numpy as np

from planning.map import BuildingMapGenConfig, BuildingMap, Coor
from planning.planner import Planner
from planning.scenarios import Scenario


def run_planner(bmap: BuildingMap, planner: Planner, src: Coor, target: Coor, seed: int):
    solved, visited, path = planner.plan(src, target)
    return {
        "seed": seed,
        "solved": solved,
        "path_length": len(path),
        "n_visited": len(visited),
        "n_free": bmap.num_free,
        "width": bmap.config.width1,
        "length": bmap.config.width2,
        "height": bmap.config.height,
        "size": bmap.size,
        "free_ratio": bmap.free_percentage,
        "visited_ratio": len(visited) / bmap.num_free,
        "planner": planner.__class__.__name__,
        "time_taken": planner.time_taken
    }
