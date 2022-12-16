from typing import Tuple, List

import numpy as np
from plotly import graph_objects as go

from planning.planner.planner import Planner
from planning.vis import vis_with_plotly
from planning.map.map import BuildingMap, Coor
from planning.map.util import euclidean_distance


def solve_rrt(bmap: BuildingMap, src: Tuple, target: Tuple, max_steps: int, max_streering_radius: float,
              destination_reached_radius: float, consider_corner: bool = True):
    assert bmap.is_free(src), "Source is not Free"
    assert bmap.is_free(target), "Target is not Free"
    tree_nodes = set()
    tree_nodes.add(src)
    plan = [src]
    step_count = 0
    for step in range(max_steps):
        step_count += 1


def steer_algorithm(s_from: Coor, s_to: Coor, max_radius: float) -> Coor:
    s_from = np.array(s_from)
    s_to = np.array(s_to)
    direction = s_to - s_from
    distance = np.linalg.norm(direction)
    offset = direction / distance * min(distance, max_radius)
    new = np.array([s_from[0] + offset[0], s_from[1] + offset[1], s_from[2] + offset[2]]).astype(np.int)
    return tuple(new)


class RRTPlanner(Planner):
    name: str = "RRT"

    def __init__(self, m: BuildingMap, max_streering_radius: int = 10,
                 max_steps: int = 200, destination_reached_radius: float = 4) -> None:
        super().__init__(m)
        self.map = m
        assert len(m.shape) == 3, "Map Shape Must be 3"
        self.max_resample_iterations = np.prod(self.map.shape)  # max number of times to try to sample a free point
        self.parents = {}
        self.max_checks = 10  # resolution of checking obstacle, should be dynamic based on size of map
        self.n_step = 0
        self.max_streering_radius = max_streering_radius
        self.max_steps = max_steps
        self.destination_reached_radius = destination_reached_radius

    def is_free(self, coor: Coor):
        return self.map.is_free(coor)

    def sample_state(self) -> Coor:
        for i in range(self.max_resample_iterations):
            rand_pt = tuple((np.random.randint(0, self.map.shape[i]) for i in range(len(self.map.shape))))
            if self.is_free(rand_pt):
                return rand_pt
        raise Exception(f"Cannot Find any free position in 1000 iterations")

    def find_closest(self, tree_nodes: List[Coor], pt: Coor) -> Coor:
        distances = [self.cost(pt, node) for node in tree_nodes]
        return tree_nodes[np.argmin(distances)]

    def steer_towards(self, s_nearest: Coor, s_rand: Coor, max_radius: float) -> Coor:
        if self.cost(s_nearest, s_rand) <= max_radius:
            return s_rand
        return steer_algorithm(s_nearest, s_rand, max_radius)

    def path_is_obstacle_free(self, s_from: Coor, s_to: Coor) -> bool:
        assert (self.is_free(s_from))
        if not (self.is_free(s_to)):
            return False
        if s_from == s_to:
            return True
        for i in range(1, self.max_checks):
            distance = float(i) / self.max_checks * self.cost(s_from, s_to)
            s_new = steer_algorithm(s_from, s_to, distance)
            if not self.is_free(s_new):
                return False
        return True

    def plan_impl(self, src: Tuple, target: Tuple) -> Tuple[bool, List, List[Coor]]:
        super().plan_impl(src, target)
        assert self.is_free(src), "Source is not Free"
        assert self.is_free(target), "Target is not Free"
        self.parents = {src: None}
        tree_nodes = set()
        tree_nodes.add(src)
        path = []
        step_count = 0
        solved = False
        for self.n_step in range(self.max_steps):
            step_count += 1
            s_rand = self.sample_state()
            s_nearest = self.find_closest(list(tree_nodes), s_rand)
            s_new = self.steer_towards(s_nearest, s_rand, self.max_streering_radius)
            if self.path_is_obstacle_free(s_nearest, s_new):
                self.parents[s_new] = s_nearest
                tree_nodes.add(s_new)
                if self.cost(s_new, target) < self.destination_reached_radius:
                    solved = True
                    self.parents[target] = s_new
                    path: List[Coor] = []
                    cur = target
                    while cur is not None:
                        path.append(cur)
                        cur = self.parents[cur]
                    break
        self.path = path
        return solved, list(tree_nodes), path

    def vis(self, marker_opacity: float = 0.3, marker_size: float = 2, show_visited: bool = False,
            display_exploring: bool = True):
        """
        visualize planning result
        :param show_visited: show visited nodes, we don't show for rrt
        :param marker_opacity: visited nodes visualization marker opacity
        :param marker_size: visited nodes visualization marker size
        :param display_exploring: decide weather to show the tree expansion
        :return: a plotly figure
        """

        def background_injection_fn(fig: go.Figure):
            path_edges = set([(self.path[i], self.path[i + 1]) for i in range(len(self.path) - 1)])
            for s_from, s_to in self.parents.items():
                # for all edges
                edge = (s_from, s_to)
                if s_from and s_to and edge not in path_edges:
                    path_data = np.array(edge)
                    # fig.add_trace(go.Scatter3d(x=path_data[:, 1], y=path_data[:, 0], z=path_data[:, 2]))
                    fig.add_scatter3d(x=path_data[:, 1], y=path_data[:, 0], z=path_data[:, 2], mode="lines+markers",
                                      marker=dict(
                                          color="yellow",
                                          size=3,
                                      ), name="RRT Expansion")

        injection = background_injection_fn if display_exploring else None
        fig = vis_with_plotly(self.map.map, self.src, self.target, path_pts=self.path, visited=[], marker_size=1,
                              marker_opacity=0.1, z_axis_range_upper_bound=np.max(self.map.map),
                              background_injection_fn=injection)
        return fig

    def cost(self, a: Coor, b: Coor):
        return euclidean_distance(a, b)

    @property
    def path_cost(self):
        return sum([self.cost(self.path[i], self.path[i + 1]) for i in range(len(self.path) - 1)])
