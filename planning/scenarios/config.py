from enum import Enum

import numpy as np

from planning.map.map import BuildingMapGenConfig


class ScenarioSize(Enum):
    XS = 1
    S = 2
    M = 3
    L = 4
    XL = 5


class ScenarioConfig:
    @staticmethod
    def gen_config(seed: int, size: ScenarioSize):
        if size == ScenarioSize.XS:
            return ScenarioConfig.extra_small_scenario(seed)
        elif size == ScenarioSize.S:
            return ScenarioConfig.small_scenario(seed)
        elif size == ScenarioSize.M:
            return ScenarioConfig.medium_scenario(seed)
        elif size == ScenarioSize.L:
            return ScenarioConfig.large_scenario(seed)
        elif size == ScenarioSize.XL:
            return ScenarioConfig.extra_large_scenario(seed)
        else:
            raise ValueError("size is invalid")

    @staticmethod
    def extra_small_scenario(seed: int) -> BuildingMapGenConfig:
        np.random.seed(seed)
        return BuildingMapGenConfig(width1=3, width2=3, height=2, n_building=1, min_building_size=1,
                                    max_building_size=2, max_building_height=2, sample_point_max_height=2)

    @staticmethod
    def small_scenario(seed: int) -> BuildingMapGenConfig:
        np.random.seed(seed)
        return BuildingMapGenConfig(width1=10, width2=10, height=10, n_building=8, min_building_size=1,
                                    max_building_size=4, max_building_height=10, sample_point_max_height=7)

    @staticmethod
    def medium_scenario(seed: int) -> BuildingMapGenConfig:
        np.random.seed(seed)
        return BuildingMapGenConfig(width1=50, width2=50, height=50, n_building=20, min_building_size=8,
                                    max_building_size=20, max_building_height=50, sample_point_max_height=30)

    @staticmethod
    def large_scenario(seed: int) -> BuildingMapGenConfig:
        np.random.seed(seed)
        return BuildingMapGenConfig(width1=100, width2=100, height=50, n_building=40, min_building_size=12,
                                    max_building_size=25, max_building_height=50, sample_point_max_height=30)

    @staticmethod
    def extra_large_scenario(seed: int) -> BuildingMapGenConfig:
        np.random.seed(seed)
        return BuildingMapGenConfig(width1=150, width2=150, height=60, n_building=50, min_building_size=12,
                                    max_building_size=25, max_building_height=60, sample_point_max_height=40)
