import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
from typing import Tuple


def vis_2d_histogram(bmap: np.ndarray, figsize:Tuple[int, int]=(7, 7)) -> None:
    f = plt.figure(figsize=figsize)
    ax1 = plt.axes(projection='3d')
    _x = np.arange(bmap.shape[0])
    _y = np.arange(bmap.shape[1])
    _xx, _yy = np.meshgrid(_x, _y)
    x, y = _xx.ravel(), _yy.ravel()

    top = bmap.flatten()
    bottom = np.zeros_like(top)
    width = depth = 1

    cmap = cm.get_cmap()
    bar_plt = ax1.bar3d(x, y, bottom, width, depth, top, shade=True, color=cmap(top / np.max(top)))
    # ax1.set_title('Shaded')
    # ax1.zaxis.set_major_locator(LinearLocator(10))
    # bar_plt
    plt.close()
    return f

