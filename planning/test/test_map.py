import numpy as np
from planning.map.map import ProbabilityMap, BuildingMap, BuildingMapGenConfig


def test_ProbabilityMap():
    np.random.seed(1)
    shape = (100, 100)
    map_ = ProbabilityMap((100, 100), 0.25)
    assert map_.free_percentage == 0.7461
    assert map_.size == 10000
    assert map_.shape == shape


def test_building_map_occupancy():
    np.random.seed(5)
    size = 3
    n_building = 1
    config = BuildingMapGenConfig(size, size, size, n_building, 1, 2)
    bmap = BuildingMap(config)
    for pos, idx in bmap.free_graph_pos_id_map.items():
        assert bmap.map[pos[0]][pos[1]] < pos[-1]
    for pos, idx in bmap.obstacle_graph_pos_id_map.items():
        assert bmap.map[pos[0]][pos[1]] >= pos[-1]
    assert bmap.free_graph.has_edge(0, 1)
