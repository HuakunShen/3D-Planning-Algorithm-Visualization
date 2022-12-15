from collections import defaultdict
from typing import Tuple, List

from planning.src.map.map import BuildingMap
from planning.src.map.type import Coor
from planning.src.planner.a_star import euclidean_distance
from planning.src.planner.rrt import RRTPlanner


# https://arxiv.org/pdf/1005.0416.pdf
# page 9: RRT* pseudocode
class RRTStarPlanner(RRTPlanner):
    def __init__(self, m: BuildingMap, neighbor_radius: int):
        super().__init__(m)
        self.neighbor_radius = neighbor_radius

    def plan(self, src: Tuple, target: Tuple, max_steps: int, max_streering_radius: float,
             destination_reached_radius: float) -> Tuple[bool, List, List[Coor]]:
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
        for step in range(max_steps):
            step_count += 1
            x_rand = self.sample_state()
            x_nearest = self.find_closest(list(tree_nodes), x_rand)
            x_new = self.steer_towards(x_nearest, x_rand, max_streering_radius)

            # RRT * specific code
            if self.path_is_obstacle_free(x_nearest, x_new):
                tree_nodes.add(x_new)
                x_min = x_nearest
                # find all nodes within a radius of s_new
                neighbors = [node for node in tree_nodes if euclidean_distance(node, x_new) < self.neighbor_radius]
                for x_near in neighbors:
                    # find a neighboring node with even lower cost to x_new, similar to A*
                    if self.path_is_obstacle_free(x_near, x_new):
                        tentative_cost = cost[x_near] + self.cost(x_near, x_new)
                        if tentative_cost < cost[x_new]:
                            x_min = x_near
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
                path: List[Coor] = []
                cur = target
                while cur is not None:
                    path.append(cur)
                    cur = self.parents[cur]
                break
        self.path = path
        return solved, [], path
