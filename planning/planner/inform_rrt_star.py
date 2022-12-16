from collections import defaultdict
from typing import Tuple, List

import numpy as np

from planning.map import BuildingMap, Coor
from planning.planner import RRTPlanner


class InformRRTStarPlanner(RRTPlanner):
    name: str = "RRT*"

    def __init__(self, m: BuildingMap, max_streering_radius: int = 10,
                 max_steps: int = 200, destination_reached_radius: float = 4,
                 neighbor_radius: int = 5,
                 quit_early: bool = False):
        super().__init__(m, max_streering_radius, max_steps, destination_reached_radius)
        self.neighbor_radius = neighbor_radius
        self.quit_early = quit_early

    def sample(self, x_start: Coor, x_goal: Coor, c_max: float):
        if c_max < float('inf'):
            x_start, x_goal = np.array(x_start), np.array(x_goal)
            c_min = self.cost(x_start, x_goal)
            x_center = np.array(x_start) + np.array(x_goal) / 2
            C = self.rotation_to_world_frame(x_start, x_goal)
            r = np.zeros(3)
            r[0] = c_max / 2
            for i in [1, 2]:
                r[i] = np.sqrt(c_max ** 2 - c_min ** 2) / 2
            L = np.diag(r)
            x_ball = self.sample_unit_n_ball()
            x = C @ L @ x_ball + x_center
            x_rand = None  # TODO, figure out what's next
        else:
            x_rand = self.sample_state()

    def sample_unit_n_ball(self):
        r = np.random.uniform(0.0, 1.0)
        theta = np.random.uniform(0, np.pi)
        phi = np.random.uniform(0, 2 * np.pi)
        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * np.cos(theta)
        return np.array([x, y, z])


    def rotation_to_world_frame(self, xstart: np.array, xgoal: np.array):
        # S0(n): such that the xstart and xgoal are the center points
        d = self.cost(xstart, xgoal)
        a1 = (xgoal - xstart) / d
        M = np.outer(a1, [1, 0, 0])
        U, S, V = np.linalg.svd(M)
        C = U @ np.diag([1, 1, np.linalg.det(U) * np.linalg.det(V)]) @ V.T
        return C

    def plan_impl(self, src: Tuple, target: Tuple) -> Tuple[bool, List, List[Coor]]:
        super().plan_impl(src, target)
        assert self.is_free(src), "Source is not Free"
        assert self.is_free(target), "Target is not Free"
        self.parents = {src: None}
        cost = defaultdict(lambda: float('inf'))
        cost[src] = 0
        tree_nodes = set()
        tree_nodes.add(src)
        path = []
        solved = False

        # Inform RRT* specific code
        x_sol = set()

        for self.n_step in range(self.max_steps):
            if len(x_sol) == 0:
                c_best = np.inf
            else:
                c_best = min([cost[x] for x in x_sol])
            x_rand = self.sample(src, target, c_best)
            x_rand = self.sample_state()
            x_nearest = self.find_closest(list(tree_nodes), x_rand)
            x_new = self.steer_towards(x_nearest, x_rand, self.max_streering_radius)
            if x_new == src:
                # avoid cycle, never set parent for src
                continue
            # RRT * specific code
            if self.path_is_obstacle_free(x_nearest, x_new):
                tree_nodes.add(x_new)
                x_min = x_nearest
                # find all nodes within a radius of s_new
                neighbors = [node for node in tree_nodes if self.cost(node, x_new) < self.neighbor_radius]
                for x_near in neighbors:
                    # if x_near == x_min:
                    #     continue
                    # find a neighboring node with even lower cost to x_new, similar to A*
                    if self.path_is_obstacle_free(x_near, x_new):
                        tentative_cost = cost[x_near] + self.cost(x_near, x_new)
                        if tentative_cost < cost[x_new]:
                            x_min = x_near
                if x_new != x_min:  # avoid self cycle
                    self.parents[x_new] = x_min  # connect x_new to the min-cost node in neighborhood

                # go through neighborhood again to see if anyone can benefit from connecting to x_new (the new node)
                for x_near in neighbors:
                    if x_near == x_min:
                        continue
                    if self.path_is_obstacle_free(x_new, x_near) and \
                            cost[x_near] > cost[x_new] + self.cost(x_new, x_near):
                        self.parents[x_near] = x_new

            # RRT * specific code
        for child, parent in self.parents.items():
            if self.cost(child, target) < self.destination_reached_radius and self.path_is_obstacle_free(child, target):
                self.parents[target] = child
                break
            # if euclidean_distance(x_new, target) < self.destination_reached_radius:
            #     solved = True
            #     self.parents[target] = x_new
            #     if self.quit_early:
            #         break
        if target in self.parents.keys():
            solved = True
        if solved:
            path: List[Coor] = []
            cur = target
            count = 0
            while cur is not None:
                path.append(cur)
                cur = self.parents[cur]
                count += 1
                if count > 100_000:
                    raise AssertionError("May Fall Into a Cycle")
            self.path = path
        return solved, list(tree_nodes), path
