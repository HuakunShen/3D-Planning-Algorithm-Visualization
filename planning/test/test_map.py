import os
import sys
import numpy as np
from planning.src.map.map import ProbabilityMap


def test_ProbabilityMap():
    np.random.seed(1)
    shape = (100, 100)
    map_ = ProbabilityMap((100, 100), 0.25)
    assert map_.zero_percentage == 0.7461
    assert map_.size == 10000
    assert map_.shape == shape
