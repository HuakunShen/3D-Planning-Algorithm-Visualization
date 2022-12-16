import time
from typing import List, Tuple
import matplotlib.pyplot as plt
from matplotlib import cm

from planning.map.map import BuildingMap, BuildingMapGenConfig, Map, Coor
import numpy as np

from planning.planner.a_star import AStarPlanner
from planning.planner.dfs import DFSPlanner
from planning.planner.dijkstra import DijkstraPlanner
from planning.planner.rrt import RRTPlanner
from planning.planner.rrt_star import RRTStarPlanner
from planning.scenarios.config import ScenarioConfig
from planning.vis import vis_with_plotly


def get_big_map():
    size = 50
    config = BuildingMapGenConfig(width1=size, width2=size, height=50, n_building=10, min_building_size=3,
                                  max_building_size=14, max_building_height=50, sample_point_max_height=30)
    bmap = BuildingMap(config)
    return bmap


def get_small_map():
    size = 5
    config = BuildingMapGenConfig(width1=size, width2=size, height=size, n_building=2, min_building_size=1,
                                  max_building_size=2, max_building_height=3, sample_point_max_height=3)
    bmap = BuildingMap(config)
    return bmap


def vis_2d_histogram(bmap: Map, path_pts: List[Coor] = None, figsize: Tuple[int, int] = (8, 8),
                     title: str = None, src: Tuple = None, target: Tuple = None) -> plt.figure:
    f = plt.figure(figsize=figsize)
    ax = plt.axes(projection='3d')
    _x = np.arange(bmap.shape[0])
    _y = np.arange(bmap.shape[1])
    _xx, _yy = np.meshgrid(_x, _y)
    x, y = _xx.ravel(), _yy.ravel()

    top = bmap.map.flatten()
    bottom = np.zeros_like(top)
    width = depth = 1

    cmap = cm.get_cmap()
    bar_plt = ax.bar3d(x, y, bottom, width, depth, top, shade=True, color=cmap(top / np.max(top)))

    # xline, yline, zline = path_pts[:, 0], path_pts[:, 1], path_pts[:, 2]
    # ax1.plot3D(xline, yline, zline, 'red')
    if path_pts is not None:
        edges = np.array([(path_pts[i], path_pts[i + 1]) for i in range(len(path_pts) - 1)])
        for edge in edges:
            xline, yline, zline = edge[:, 0], edge[:, 1], edge[:, 2]
            ax.plot3D(xline, yline, zline, 'red')
    if title is not None:
        ax.set_title(title)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    # plot source and target
    if src is not None and target is not None:
        print("scatter")
        data = np.array([src, target])
        ax.scatter(data[:, 0], data[:, 1], data[:, 2], marker="o", c="b")

    # plt.close()
    return f


if __name__ == "__main__":
    np.random.seed(0)
    # bmap = get_big_map()
    # config = BuildingMapGenConfig(width1=50, width2=50, height=50, n_building=20, min_building_size=8,
    #                               max_building_size=20, max_building_height=50, sample_point_max_height=30)

    bmap = BuildingMap(ScenarioConfig.medium_scenario(9))
    # bmap = get_small_map()
    bmap.overview_heatmap()
    # bmap.vis_map()
    plt.show()

    # start planning
    src, target = bmap.gen_random_state()
    print(src, target)
    start = time.time()
    # rrt_planner = RRTPlanner(bmap)
    # rrt_planner = RRTStarPlanner(bmap, 5)
    # solved, visited, path = solve_bfs(bmap, src, target, consider_corner=False)
    # dijkstra_planner = DijkstraPlanner(bmap)
    # solved, visited, path = dijkstra_planner.plan(src, target, consider_corner=True)
    # astar_planner = AStarPlanner(bmap)
    # solved, visited, path = astar_planner.plan(src, target, consider_corner=True)
    # solved, visited, path = solve_astar(bmap, src, target, consider_corner=True)

    dfs_planner = DFSPlanner(bmap)
    solved, visited, path = dfs_planner.plan_impl(src, target)
    dfs_planner.vis().show()

    # solved, visited, path = solve_dfs(bmap, src, target, consider_corner=True)
    # solved, visited, path = solve_iterative_dfs(bmap, src, target, consider_corner=True) # Not Working
    # solved, visited, path = solve_rrt(bmap, src, target, consider_corner=True)
    # solved, visited, path = rrt_planner.plan(src, target, 10000, 10, 10.0)
    # solved, visited, path = rrt_planner.plan(src, target, 1200, 10, 5)
    # rrt_planner.vis().show()
    # dijkstra_planner.vis().show()
    # print(f"Cost: {rrt_planner.path_cost}")
    print(path)

    print(f"Time Taken: {round(time.time() - start, 2)}s")
    print(f"Path Length: {len(path)}")  # path is a list of tuple
    # print(path[:3], path[-3:])  # path is a list of tuple
    # print(path)
    print(f"solved: {solved}")
    print(f"visited {len(visited)} nodes")
    if not solved:
        print("Not Solved")
        exit(1)

    # vis_2d_histogram(bmap, path_pts=path, src=src, target=target)
    vis_with_plotly(bmap.map, src, target, path_pts=path, visited=visited, marker_size=1, marker_opacity=0.1,
                    z_axis_range_upper_bound=80).show()
    plt.show()

    # # bmap.free_graph_pos_id_map[src]
    # # bmap.free_graph_pos_id_map[target]
    # path = nx.shortest_path(bmap.free_graph, bmap.free_graph_pos_id_map[src], bmap.free_graph_pos_id_map[target])
    # path_pts = [bmap.free_graph_id_pos_map[pt] for pt in path]
    # bfs_f = vis_2d_histogram(bmap, path_pts)
    # plt.show()

    # # path = nx.dfs_preorder_nodes(bmap.free_graph, bmap.free_graph_pos_id_map[src], bmap.free_graph_pos_id_map[target])
    # # path_pts = np.array([bmap.free_graph_id_pos_map[pt] for pt in path])
    # # dfs_f = vis_2d_histogram(bmap, path_pts)
    # path = nx.astar_path(bmap.free_graph, bmap.free_graph_pos_id_map[src], bmap.free_graph_pos_id_map[target])
    # path_pts = [bmap.free_graph_id_pos_map[pt] for pt in path]
    # astar_f = vis_2d_histogram(bmap, path_pts)
    # plt.show()
