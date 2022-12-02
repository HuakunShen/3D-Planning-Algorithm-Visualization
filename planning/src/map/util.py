import numpy as np
import networkx as nx
from numba import njit


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
def get_3D_neighbors(pt: np.array, shape: np.array) -> np.ndarray:
    n_row, n_col, n_height = shape
    coor = []
    row, col, height = pt
    if row > 0:
        coor.append((row - 1, col, height))
    if row < n_row - 1:
        coor.append((row + 1, col, height))
    if col > 0:
        coor.append((row, col - 1, height))
    if col < n_col - 1:
        coor.append((row, col + 1, height))
    if height > 0:
        coor.append((row, col, height - 1))
    if height < n_height - 1:
        coor.append((row, col, height + 1))
    return np.array(coor)


if __name__ == "__main__":
    import doctest
    doctest.testmod()
