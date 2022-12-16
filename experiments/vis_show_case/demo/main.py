import os
import sys
from typing import List, Type

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from tqdm import tqdm

from planning.experiment import run_planner
from planning.map import BuildingMap, BuildingMapGenConfig
from planning.scenarios import ScenarioConfig, Scenario, ScenarioSize
from planning.map.util import manhattan_distance, euclidean_distance, clip, path_total_length
from planning.planner import AStarPlanner, DijkstraPlanner, DFSPlanner, RRTPlanner, RRTStarPlanner, Planner
from planning.vis import vis_2d_histogram

if __name__ == "__main__":
    seed = 996
    np.random.seed(seed)
    # config = BuildingMapGenConfig(width1=150, width2=50, height=40, n_building=40, min_building_size=8,
    #                               max_building_size=16, max_building_height=30, sample_point_max_height=20)
    config = BuildingMapGenConfig(width1=20, width2=20, height=20, n_building=10, min_building_size=3,
                                  max_building_size=7, max_building_height=20, sample_point_max_height=15)
    bmap = BuildingMap(config)
    src, target = bmap.gen_random_state()
    # src, target = (14, 28, 8), (147, 8, 10)
    # bmap, src, target = Scenario.get(0, ScenarioSize.M)
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
    rrt_star_planner = RRTStarPlanner(bmap, max_streering_radius=max_streering_radius, max_steps=rrt_max_step,
                                      destination_reached_radius=destination_reached_radius,
                                      neighbor_radius=neighbor_radius, quit_early=False)
    # planner = astar_planner
    # planner = dijkstra_planner
    # planner = dfs_planner
    for planner in [astar_planner, dijkstra_planner, dfs_planner, rrt_planner, rrt_star_planner]:
        if isinstance(planner, RRTStarPlanner):
            planner.max_steps = rrt_planner.n_step * 3
        np.random.seed(seed)
        solved, visited, path = planner.plan(src, target)
        print(planner.__class__.__name__)
        # print(visited)
        if not solved:
            print("Not Solved")
            exit(0)
        print(path_total_length(path))
        fig = planner.vis(marker_size=4, marker_opacity=0.4)
        fig.show()
    # fig.write_image("vis.png")
    # vis_2d_histogram(bmap, src=src, target=target, path=path, visited=list(visited))
    # plt.show()
