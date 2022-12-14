from collections import namedtuple
import numpy as np
from typing import Tuple, Type, Dict
import networkx as nx
import seaborn as sns
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator

from planning.src.vis import vis_2d_histogram




class Map:
    def __init__(self, shape: Tuple[int], dtype: Type = np.int8) -> None:
        self.dtype = dtype
        self.shape = shape
        self.map = np.zeros(self.shape, dtype=self.dtype)
        
    @property
    def zero_percentage(self) -> float:
        return np.count_nonzero(self.map == 0) / self.map.size

    @property
    def size(self) -> int:
        return self.map.size

    def __str__(self) -> str:
        data = [
            ("Shape", self.map.shape),
            ("dtype", self.dtype),
            ("zero_percentage", self.zero_percentage)
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

BuildingMapGenConfig = namedtuple("Point", "width1 width2 height n_building max_building_size min_building_size")




class BuildingMap(Map):
    def __init__(self, config: BuildingMapGenConfig) -> None:
        super().__init__((config.width1, config.width2, config.height), dtype=np.int8)
        self.map = self.gen_map(config)
        self.config = config
        self.free_graph = self.build_graph(0)
        self.obstacle_graph = self.build_graph(1)

    def gen_map(self, config: BuildingMapGenConfig) -> None:
        min_building_size, max_building_size = config.min_building_size, config.max_building_size

        if max_building_size is None:
            max_building_size = (config.width1 * config.width2) // 70 // config.n_building
        if min_building_size is None:
            min_building_size = (config.width1 * config.width2) // 100 // config.n_building

        # build 2D random building map, cell value is building height
        map2D = np.ones((config.width1, config.width2)) # one means floor height

        building_heights: Dict[tuple, int] = {}
        building_centers = [(np.random.randint(0, config.width1), np.random.randint(0, config.width2)) for i in range(config.n_building)]
        for idx, center in enumerate(building_centers):
            x, y = center
            building_heights[center] = np.random.randint(config.height//4, config.height)
            building_rand_width, building_rand_height = np.random.randint(min_building_size, max_building_size), np.random.randint(min_building_size, max_building_size)
            map2D[x:x+building_rand_width, y:y+building_rand_height] = building_heights[center]
            # map3D[x:x+building_rand_width, y:y+building_rand_height, 0:building_heights[center]] = 0
        # sns.heatmap(map2D)
        return map2D

    def is_occupied(self, point: Tuple[int]) -> bool:
        """determine if a point is occupied
        since our map is a 2D representation of a 3D map (building), we check if the height of the point is less than the height of the building
        :param point: query point (y, x, z) or (row, column, height)
        :type point: Tuple[int]
        :return: whether the point is occupied
        :rtype: bool
        """
        return self.map[point[:-1]] < point[-1]

    def overview_heatmap(self):
        sns.heatmap(self.map)

    def build_graph(self, free: bool) -> nx.Graph:
        row, col, height = self.shape
        # build a 3D graph
        G = nx.Graph()
        for i in range(row):
            for j in range(col):
                for k in range(height):
                    if free and not self.is_occupied((i, j, k)):
                        G.add_node((i, j, k))
        for i in range(row):
            for j in range(col):
                for k in range(height):
                    if not self.is_occupied((i, j, k)):
                        for di in range(-1, 2):
                            for dj in range(-1, 2):
                                for dk in range(-1, 2):
                                    if (i + di, j + dj, k + dk) in G.nodes:
                                        G.add_edge((i, j, k), (i + di, j + dj, k + dk))
        return G
    
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
