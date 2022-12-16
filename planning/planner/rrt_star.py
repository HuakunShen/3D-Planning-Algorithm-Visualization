from collections import defaultdict
from typing import Tuple, List

from planning.map.map import BuildingMap
from planning.map.type import Coor
from planning.planner.a_star import euclidean_distance
from planning.planner.rrt import RRTPlanner
from tqdm import tqdm


# https://arxiv.org/pdf/1005.0416.pdf
# page 9: RRT* pseudocode
class RRTStarPlanner(RRTPlanner):
    def __init__(self, m: BuildingMap, neighbor_radius: int):
        super().__init__(m)
        self.neighbor_radius = neighbor_radius

    def plan(self, src: Tuple, target: Tuple, max_steps: int, max_streering_radius: float,
             destination_reached_radius: float, quit_early: bool = False) -> Tuple[bool, List, List[Coor]]:
        self.src = src
        self.target = target
        assert self.is_free(src), "Source is not Free"
        assert self.is_free(target), "Target is not Free"
        self.parents = {src: None}
        cost = defaultdict(lambda: float('inf'))
        cost[src] = 0
        tree_nodes = set()
        tree_nodes.add(src)
        path = []
        step_count = 0
        solved = False

        for self.n_step in tqdm(range(max_steps)):
            step_count += 1
            x_rand = self.sample_state()
            x_nearest = self.find_closest(list(tree_nodes), x_rand)
            x_new = self.steer_towards(x_nearest, x_rand, max_streering_radius)
            if x_new == src:
                # avoid cycle, never set parent for src
                continue
            # RRT * specific code
            if self.path_is_obstacle_free(x_nearest, x_new):
                tree_nodes.add(x_new)
                x_min = x_nearest
                # find all nodes within a radius of s_new
                neighbors = [node for node in tree_nodes if euclidean_distance(node, x_new) < self.neighbor_radius]
                for x_near in neighbors:
                    if x_near == x_min:
                        continue
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
            if euclidean_distance(x_new, target) < destination_reached_radius:
                solved = True
                self.parents[target] = x_new
                if quit_early:
                    break
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
        return solved, [], path
