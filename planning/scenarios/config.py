import numpy as np

from planning.map.map import BuildingMapGenConfig


class ScenarioConfig:
    @staticmethod
    def extra_small_scenario(idx: int):
        np.random.seed(idx)
        return BuildingMapGenConfig(width1=3, width2=3, height=3, n_building=1, min_building_size=1,
                                    max_building_size=2, max_building_height=3, sample_point_max_height=3)

    @staticmethod
    def small_scenario(idx: int):
        np.random.seed(idx)
        return BuildingMapGenConfig(width1=10, width2=10, height=10, n_building=8, min_building_size=1,
                                    max_building_size=4, max_building_height=10, sample_point_max_height=7)

    @staticmethod
    def medium_scenario(idx: int):
        np.random.seed(idx)
        return BuildingMapGenConfig(width1=50, width2=50, height=50, n_building=20, min_building_size=8,
                                    max_building_size=20, max_building_height=50, sample_point_max_height=30)

    @staticmethod
    def large_scenario(idx: int):
        np.random.seed(idx)
        return BuildingMapGenConfig(width1=100, width2=100, height=50, n_building=40, min_building_size=12,
                                    max_building_size=25, max_building_height=50, sample_point_max_height=30)

    @staticmethod
    def extra_large_scenario(idx: int):
        np.random.seed(idx)
        return BuildingMapGenConfig(width1=150, width2=150, height=60, n_building=50, min_building_size=12,
                                    max_building_size=25, max_building_height=60, sample_point_max_height=40)
