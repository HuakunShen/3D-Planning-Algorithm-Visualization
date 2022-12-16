from collections import defaultdict
from typing import Tuple, List, Dict, Set

import numpy as np

from planning.map import BuildingMap, Coor
from planning.planner import RRTPlanner


class RRTConnectPlanner(RRTPlanner):
    name: str = "RRT*"

    def __init__(self, m: BuildingMap, max_streering_radius: int = 10,
                 max_steps: int = 200, destination_reached_radius: float = 4,
                 neighbor_radius: int = 5,
                 quit_early: bool = False):
        super().__init__(m, max_streering_radius, max_steps, destination_reached_radius)
        self.neighbor_radius = neighbor_radius
        self.quit_early = quit_early

    def plan_impl(self, src: Tuple, target: Tuple) -> Tuple[bool, List, List[Coor]]:
        super().plan_impl(src, target)
        assert self.is_free(src), "Source is not Free"
        assert self.is_free(target), "Target is not Free"

        parent_src = {src: None}
        parent_target = {target: None}
        tree_nodes_src = {src}
        tree_nodes_target = {target}

        self.parents = {src: None}
        cost = defaultdict(lambda: float('inf'))
        cost[src] = 0

        path = []
        solved = False
        self.q_new = None  # Global

        def NEAREST_NEIGHBOR(q: Coor, tree_nodes: Set[Coor]) -> Coor:
            # find the nearest neighbor in the tree
            tree_nodes_list = list(tree_nodes)
            dists = np.array([self.cost(q, node) for node in tree_nodes_list])
            return tree_nodes_list[np.argmin(dists)]

        def NEW_CONFIG(q_near: Coor, q_new: Coor) -> bool:
            return self.path_is_obstacle_free(q_near, q_new)

        def CONNECT(tree_nodes: Set[Coor], parent: Dict, q: Coor) -> str:
            while True:
                print("stuck")
                s = EXTEND(tree_nodes, parent, q)
                if s != 'Advanced':
                    break
            return s

        def EXTEND(tree_nodes: Set[Coor], parent: Dict, q: Coor) -> str:
            q_near = NEAREST_NEIGHBOR(q, tree_nodes)
            # qnew, dist = steer(self, qnear, q)
            self.q_new = self.steer_towards(q_near, q, self.max_streering_radius)
            if NEW_CONFIG(q_near, self.q_new):
                tree_nodes.add(self.q_new)
                parent[self.q_new] = q_near
                if self.q_new == q:
                    return 'Reached'
                else:
                    return 'Advanced'
            return 'Trapped'

        def build_path(parent_src: Dict, parent_target: Dict):
            path_src, path_target = [], []
            cur_src = self.q_new
            while cur_src is not None:
                path_src.append(cur_src)
                cur_src = parent_src[cur_src]
            cur_target = self.q_new
            while cur_target is not None:
                path_target.append(cur_target)
                cur_target = parent_target[cur_target]
            path_target.reverse()
            return path_target + path_src

        for i in range(self.max_steps):
            print(i)
            q_rand = self.sample_state()
            if EXTEND(tree_nodes_src, parent_src, q_rand) != "Trapped":
                if CONNECT(tree_nodes_target, parent_target, self.q_new) == 'Reached':
                    solved = True
                    path = build_path(parent_src, parent_target)
            # swap
            parent_src, parent_target = parent_target, parent_src
            tree_nodes_src, tree_nodes_target = tree_nodes_target, tree_nodes_src

        return solved, tree_nodes_src + tree_nodes_target, path

        # self.parents = {src: None}
        # cost = defaultdict(lambda: float('inf'))
        # cost[src] = 0
        # tree_nodes = set()
        # tree_nodes.add(src)
        # path = []
        # solved = False
        #
        #
        #
        #
        #
        #
        #
        #
        #
        # for self.n_step in range(self.max_steps):
        #     x_rand = self.sample_state()
        #     x_nearest = self.find_closest(list(tree_nodes), x_rand)
        #     x_new = self.steer_towards(x_nearest, x_rand, self.max_streering_radius)
        #     if x_new == src:
        #         # avoid cycle, never set parent for src
        #         continue
        #     # RRT * specific code
        #     if self.path_is_obstacle_free(x_nearest, x_new):
        #         tree_nodes.add(x_new)
        #         x_min = x_nearest
        #         # find all nodes within a radius of s_new
        #         neighbors = [node for node in tree_nodes if self.cost(node, x_new) < self.neighbor_radius]
        #         for x_near in neighbors:
        #             # if x_near == x_min:
        #             #     continue
        #             # find a neighboring node with even lower cost to x_new, similar to A*
        #             if self.path_is_obstacle_free(x_near, x_new):
        #                 tentative_cost = cost[x_near] + self.cost(x_near, x_new)
        #                 if tentative_cost < cost[x_new]:
        #                     x_min = x_near
        #         if x_new != x_min:  # avoid self cycle
        #             self.parents[x_new] = x_min  # connect x_new to the min-cost node in neighborhood
        #
        #         # go through neighborhood again to see if anyone can benefit from connecting to x_new (the new node)
        #         for x_near in neighbors:
        #             if x_near == x_min:
        #                 continue
        #             if self.path_is_obstacle_free(x_new, x_near) and \
        #                     cost[x_near] > cost[x_new] + self.cost(x_new, x_near):
        #                 self.parents[x_near] = x_new
        #
        #     # RRT * specific code
        # for child, parent in self.parents.items():
        #     if self.cost(child, target) < self.destination_reached_radius and self.path_is_obstacle_free(child, target):
        #         self.parents[target] = child
        #         break
        #     # if euclidean_distance(x_new, target) < self.destination_reached_radius:
        #     #     solved = True
        #     #     self.parents[target] = x_new
        #     #     if self.quit_early:
        #     #         break
        # if target in self.parents.keys():
        #     solved = True
        # if solved:
        #     path: List[Coor] = []
        #     cur = target
        #     count = 0
        #     while cur is not None:
        #         path.append(cur)
        #         cur = self.parents[cur]
        #         count += 1
        #         if count > 100_000:
        #             raise AssertionError("May Fall Into a Cycle")
        #     self.path = path
        # return solved, list(tree_nodes), path
