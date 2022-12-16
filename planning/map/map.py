from typing import Tuple, Type, Dict
import networkx as nx
import seaborn as sns
import numpy as np
from dataclasses import dataclass

from planning.map.type import Coor
from planning.vis import vis_2d_histogram


class Map:
    def __init__(self, shape: Coor, dtype: Type = np.int8) -> None:
        self.dtype = dtype
        self.shape = np.array(shape)
        self.map = np.zeros(self.shape, dtype=self.dtype)

    @property
    def num_free(self) -> int:
        """
        :return: number of free nodes
        :rtype: int
        """
        return np.count_nonzero(self.map == 0)

    @property
    def free_percentage(self) -> float:
        return self.num_free / self.size

    @property
    def size(self) -> int:
        return self.map.size

    def is_free(self, coor: Coor) -> bool:
        raise NotImplementedError

    def is_coordinate_in_map(self, coor: Coor) -> bool:
        if len(coor) != len(self.shape):
            return False
        return all([0 <= coor[dim] < self.shape[dim] for dim in range(len(coor))])

    def __str__(self) -> str:
        data = [
            ("Shape", self.map.shape),
            ("dtype", self.dtype),
            ("zero_percentage", self.free_percentage)
        ]
        return f"{self.__class__.__name__}" + "".join([f"\n\t{d[0]}: {d[1]}" for d in data])


class ProbabilityMap(Map):
    def __init__(self, shape: Tuple[int], probability: float) -> None:
        super().__init__(shape, dtype=np.int8)
        self.probability = probability
        self.reset()
        self.free_graph = build_graph(self.map, 0)
        self.obstacle_graph = build_graph(self.map, 1)

    def reset(self) -> None:
        prob_map = np.random.rand(*self.shape)
        self.map[prob_map < self.probability] = 1


# BuildingMapGenConfig = namedtuple("Point", "width1 width2 height n_building min_building_size max_building_size")
@dataclass
class BuildingMapGenConfig:
    width1: int
    width2: int
    height: int
    n_building: int
    min_building_size: int
    max_building_size: int
    max_building_height: int = None
    sample_point_max_height: int = None


def gen_map(config: BuildingMapGenConfig) -> np.ndarray:
    min_building_size, max_building_size = config.min_building_size, config.max_building_size

    if max_building_size is None:
        max_building_size = (config.width1 * config.width2) // 70 // config.n_building
    if min_building_size is None:
        min_building_size = (config.width1 * config.width2) // 100 // config.n_building

    # build 2D random building map, cell value is building height
    map2d = np.zeros((config.width1, config.width2))  # one means floor height
    max_building_height = min(config.max_building_height, config.height)
    building_heights: Dict[tuple, int] = {}
    building_centers = [(np.random.randint(0, config.width1), np.random.randint(0, config.width2)) for i in
                        range(config.n_building)]
    for idx, center in enumerate(building_centers):
        x, y = center
        building_heights[center] = np.random.randint(max(config.height // 4, 1), max_building_height)
        building_rand_width, building_rand_height = np.random.randint(min_building_size,
                                                                      max_building_size), np.random.randint(
            min_building_size, max_building_size)
        map2d[x:x + building_rand_width, y:y + building_rand_height] = building_heights[center]
        # map3D[x:x+building_rand_width, y:y+building_rand_height, 0:building_heights[center]] = 0
    # sns.heatmap(map2D)
    return map2d


class BuildingMap(Map):
    def __init__(self, config: BuildingMapGenConfig, build_graph: bool = False) -> None:
        super().__init__((config.width1, config.width2, config.height), dtype=np.int8)
        self.map = gen_map(config)
        self.config = config
        self.free_graph: nx.Graph = None
        self.obstacle_graph: nx.Graph = None
        self.free_graph_pos_id_map = {}
        self.obstacle_graph_pos_id_map = {}
        if build_graph:  # enable when networkx graph is needed
            self.free_graph, self.free_graph_pos_id_map = self.build_graph(True)
            self.obstacle_graph, self.obstacle_graph_pos_id_map = self.build_graph(False)
        self.src: Tuple[int] = None
        self.target: Tuple[int] = None

    @property
    def num_free(self) -> int:
        return int(np.sum(np.ones_like(self.map) * self.config.height - self.map))

    @property
    def size(self) -> int:
        return int(np.prod(self.shape))

    @property
    def free_graph_id_pos_map(self):
        return {v: k for k, v in self.free_graph_pos_id_map.items()}

    @property
    def obstacle_graph_id_pos_map(self):
        return {v: k for k, v in self.obstacle_graph_pos_id_map.items()}

    @property
    def cartesian_map(self) -> np.ndarray:
        """
        In IJK coordinate system, row corresponds to y in cartesian plan, and is flipped (i.e. 0 starts from top to
        bottom). So to better visualize the map in cartesian (xyz/flipped) form
        :return: map representation in numpy matrix and cartesian plan form
        """
        return np.flip(self.map, axis=0)

    def is_free(self, point: Coor) -> bool:
        return not self.is_occupied(point)

    def is_occupied(self, point: Coor) -> bool:
        """determine if a point is occupied
        since our map is a 2D representation of a 3D map (building), we check if the height of the point is less than the height of the building
        :param point: query point (y, x, z) or (row, column, height)
        :type point: Tuple[int]
        :return: whether the point is occupied
        :rtype: bool
        """
        return self.map[point[0]][point[1]] >= point[-1]

    def overview_heatmap(self):
        heatmap = sns.heatmap(self.cartesian_map)
        heatmap.set(xlabel="x", ylabel="y", title='Overview Heatmap', aspect=1)
        return heatmap

    @property
    def max_distance(self):
        n_row, n_col, n_layer = self.shape
        max_distance = np.sqrt(n_row ** 2 + n_col ** 2 + n_layer ** 2)
        return max_distance

    def gen_random_state(self, min_distance: int = None) -> Tuple:
        """get a random free state
        :param free: whether the state is free, defaults to True
        :type free: bool, optional
        :return: a random free state
        :rtype: Tuple[int]
        """
        n_row, n_col, n_layer = self.shape
        if min_distance is None:
            min_distance = self.max_distance / 2
        if min_distance > self.max_distance:
            raise ValueError(f"min_distance must be less than max_distance: {self.max_distance}")
        self.src, self.target = None, None

        def get_random_free_state():
            for i in range(np.prod((n_row, n_col, n_layer))):
                x, y, z = np.random.randint(0, n_row), np.random.randint(0, n_col), \
                    np.random.randint(1, min(self.config.sample_point_max_height, n_layer))
                if self.is_free((x, y, z)):
                    return x, y, z
            raise ValueError("No free state found")

        def get_pair_random_state():
            src = get_random_free_state()
            for i in range(np.prod((n_row, n_col, n_layer))):
                target = get_random_free_state()
                if target != src:
                    break
            return src, target

        for i in range(1000):
            src, target = get_pair_random_state()
            dist = np.linalg.norm(np.array(src) - np.array(target))
            if dist >= min_distance:
                return src, target
        raise ValueError("No free state found")

    def build_graph(self, free: bool) -> Tuple[nx.Graph, Dict]:
        row, col, height = self.shape
        count = 0
        # build a 3D graph
        G = nx.Graph()
        id_pos_map = {}
        for i in range(row):
            for j in range(col):
                for k in range(height):
                    if (free and self.is_free((i, j, k))) or (not free and self.is_occupied((i, j, k))):
                        G.add_node(count, pos=(i, j, k))
                        id_pos_map[(i, j, k)] = count
                        count += 1
        for i in range(row):
            for j in range(col):
                for k in range(height):
                    if (i, j, k) in id_pos_map.keys():
                        # if (free and self.is_free((i, j, k))) or (not free and self.is_occupied((i, j, k))):
                        for di in range(-1, 2):
                            for dj in range(-1, 2):
                                for dk in range(-1, 2):
                                    if (i + di, j + dj, k + dk) in id_pos_map.keys():
                                        # G.add_edge((i, j, k), (i + di, j + dj, k + dk))
                                        G.add_edge(id_pos_map[(i, j, k)], id_pos_map[(i + di, j + dj, k + dk)])
        return G, id_pos_map

    def vis_map(self, figsize=(10, 10)):
        return vis_2d_histogram(self.map, figsize=figsize)


def build_graph(map: np.ndarray, node_value: int):
    row, col, height = map.shape
    # build a 3D graph
    G = nx.Graph()
    for i in range(row):
        for j in range(col):
            for k in range(height):
                if map[i, j, k] == node_value:
                    G.add_node((i, j, k))
    for i in range(row):
        for j in range(col):
            for k in range(height):
                if map[i, j, k] == node_value:
                    for di in range(-1, 2):
                        for dj in range(-1, 2):
                            for dk in range(-1, 2):
                                if (i + di, j + dj, k + dk) in G.nodes:
                                    G.add_edge((i, j, k), (i + di, j + dj, k + dk))
    return G


if __name__ == "__main__":
    map_ = ProbabilityMap((5, 5), 0.5)
    print(map_)
