from typing import Tuple, List

import numpy as np
import networkx as nx
from numba import njit

from planning.src.map.type import Coor


def cartesian_plane_to_row_col_2D(x: int, y: int, shape: np.array):
    """Convert from cartesian plane coordinates to row, column coordinates
    :param coor: coordinates
    :type coor: np.array
    :param shape: shape of the map
    :type shape: np.array
    :return: row, column coordinates
    :rtype: np.array

    >>> cartesian_plane_to_row_col_2D(30, 15, np.array([20, 40]))
    array([ 5, 30])
    """
    n_row, n_col = shape
    return np.array([n_row - y, x])


def row_col_2_cartesion_plane_2D(row: int, col: int, shape: np.array):
    """Convert from row, column coordinates to cartesian plane coordinates
    :param coor: coordinates
    :type coor: np.array
    :param shape: shape of the map
    :type shape: np.array
    :return: cartesian plane coordinates
    :rtype: np.array

    >>> row_col_2_cartesion_plane_2D(5, 30, np.array([20, 40]))
    array([30, 15])
    """
    n_row, n_col = shape
    return np.array([col, n_row - row])


@njit
def get_2D_neighbors(pt: np.array, shape: np.array) -> np.ndarray:
    n_row, n_col = shape
    coor = []
    row, col = pt
    if row > 0:
        coor.append((row - 1, col))
    if row < n_row - 1:
        coor.append((row + 1, col))
    if col > 0:
        coor.append((row, col - 1))
    if col < n_col - 1:
        coor.append((row, col + 1))
    return np.array(coor)


@njit
def get_3d_neighbors(pt: Coor, shape: np.array) -> List[Coor]:
    n_row, n_col, n_height = shape
    coor = []
    row, col, height = pt
    if row > 0:  # front
        coor.append((row - 1, col, height))
    if row < n_row - 1:  # back
        coor.append((row + 1, col, height))
    if col > 0:  # left
        coor.append((row, col - 1, height))
    if col < n_col - 1:  # right
        coor.append((row, col + 1, height))
    if height > 0:  # down
        coor.append((row, col, height - 1))
    if height < n_height - 1:  # up
        coor.append((row, col, height + 1))
    return coor


@njit
def get_3d_neighbors_brute(pt: Coor, shape: np.array):
    """
    This get neighbors function doesn't do any error checking
    could return negative values or out of bound values
    :param pt: coordinates in 3D
    :param shape: shape of map
    :return:
    """
    n_row, n_col, n_layers = shape

    for i in range(-1, 2):
        for j in range(-1, 2):
            for k in range(-1, 2):
                if i == 0 and j == 0 and k == 0:
                    continue
                # if np.all([0 <= pt[dim] < shape[dim] for dim in range(len(pt))]):
                new_i, new_j, new_k = pt[0] + i, pt[1] + j, pt[2] + k
                if 0 <= new_i < n_row and 0 <= new_j < n_col and 0 <= new_k < n_layers:
                    yield new_i, new_j, new_k

@njit
def clip(x: float, min_val: float, max_val: float):
    if x < min_val:
        return min_val
    if x > max_val:
        return max_val
    return x
    

def intert_dict(d: dict):
    return {v: k for k, v in d.items()}


if __name__ == "__main__":
    import doctest

    doctest.testmod()
