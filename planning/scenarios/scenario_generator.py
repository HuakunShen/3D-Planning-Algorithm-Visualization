from typing import Tuple

from planning.map import BuildingMap, BuildingMapGenConfig, Coor
from planning.scenarios.config import ScenarioConfig, ScenarioSize


class Scenario:

    @staticmethod
    def get(seed: int, size: ScenarioSize) -> Tuple[BuildingMap, Coor, Coor]:
        config = ScenarioConfig.gen_config(seed, size)
        bmap = BuildingMap(config)
        src, target = bmap.gen_random_state()
        return bmap, src, target

    @staticmethod
    def get_with_config(config: BuildingMapGenConfig) -> Tuple[BuildingMap, Coor, Coor]:
        bmap = BuildingMap(config)
        src, target = bmap.gen_random_state()
        return bmap, src, target
