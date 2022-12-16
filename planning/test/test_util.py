import numpy as np

from planning.map.util import get_3d_neighbors_brute


def test_get_3d_neighbors_brute():
    neighbors = get_3d_neighbors_brute((0, 0, 0), np.array([10, 10, 10]))
    assert len(set(list(neighbors))) == 7
    neighbors = get_3d_neighbors_brute((3, 3, 3), np.array([10, 10, 10]))
    assert len(set(list(neighbors))) == 27 - 1


# if __name__ == "__main__":
#     test_get_3d_neighbors_brute()
