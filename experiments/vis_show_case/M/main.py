import os
import sys
from typing import List, Type

import numpy as np
import pandas as pd
from tqdm import tqdm

from planning.experiment import run_planner
from planning.map import BuildingMap, BuildingMapGenConfig
from planning.scenarios import ScenarioConfig, Scenario, ScenarioSize
from planning.map.util import manhattan_distance, euclidean_distance, clip
from planning.planner import AStarPlanner, DijkstraPlanner, DFSPlanner, RRTPlanner, RRTStarPlanner, Planner

if __name__ == "__main__":
    bmap, src, target = Scenario.get(0, ScenarioSize.M)
    astar_planner = AStarPlanner(bmap)
    dijkstra_planner = DijkstraPlanner(bmap)
    dfs_planner = DFSPlanner(bmap)

    # RRT specific
    max_streering_radius = clip(bmap.shape[0] // 20, 2, 10)
    destination_reached_radius = clip(bmap.shape[0] // 25, 5, 8)
    neighbor_radius = clip(bmap.shape[0] // 25, 5, 8)
    rrt_max_step = bmap.shape[0] * 50
    rrt_planner = RRTPlanner(bmap, max_streering_radius=max_streering_radius, max_steps=rrt_max_step,
                             destination_reached_radius=destination_reached_radius)
    rrt_star_planner = RRTStarPlanner(bmap, max_streering_radius=max_streering_radius, max_steps=rrt_max_step * 5,
                                      destination_reached_radius=destination_reached_radius,
                                      neighbor_radius=neighbor_radius, quit_early=False)
    # for planner in [rrt_star_planner]:
    for planner in [astar_planner, dijkstra_planner, dfs_planner, rrt_planner, rrt_star_planner]:
        print(planner.__class__.__name__)
        solved, visited, path = planner.plan(src, target)
        if not solved:
            print(f"Not Solved ({planner.__class__.__name__})")
            exit(0)
        fig = planner.vis()
        fig.show()
        fig.write_image(f"{planner.__class__.__name__}.png")
