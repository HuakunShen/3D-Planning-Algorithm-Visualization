import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
from typing import Tuple, List, Callable

from plotly import graph_objects as go

from planning.src.map.map import Coor


def vis_2d_histogram(bmap: np.ndarray, figsize: Tuple[int, int] = (7, 7)) -> None:
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


def vis_with_plotly(z_data: np.ndarray, src: Coor, target: Coor, path_pts: List[Coor] = [],
                    title: str = "Visualization", visited: List[Coor] = [], marker_size: float = 1,
                    marker_opacity: float = 0.3, z_axis_range_upper_bound: float = None,
                    background_injection_fn: Callable = None):
    # path_df = pd.DataFrame(np.array(path_pts), columns=['row', 'col', 'height'])
    # path_plot = px.line_3d(path_df, x="col", y="row", z="height")
    fig = go.Figure()
    fig.add_surface(z=z_data, showscale=False)
    if background_injection_fn is not None:
        background_injection_fn(fig)
    src_data = np.array([src])
    fig.add_scatter3d(x=src_data[:, 1], y=src_data[:, 0], z=src_data[:, 2], mode='markers',
                      marker=dict(
                          size=20,
                          color="lightgreen",
                          # color=z,  # set color to an array/list of desired values
                          colorscale='Viridis',  # choose a colorscale
                          opacity=0.8
                      ), name="Source")
    target_data = np.array([target])
    fig.add_scatter3d(x=target_data[:, 1], y=target_data[:, 0], z=target_data[:, 2], mode='markers',
                      marker=dict(
                          size=20,
                          color="cyan",
                          # color=z,  # set color to an array/list of desired values
                          colorscale='Viridis',  # choose a colorscale
                          opacity=0.8
                      ), name="Target")
    path_data = np.array(path_pts)
    if len(path_data) != 0:
        fig.add_scatter3d(x=path_data[:, 1], y=path_data[:, 0], z=path_data[:, 2], mode="lines+markers",
                          marker=dict(
                              size=8,
                              color="red",
                              colorscale='Viridis',  # choose a colorscale
                              opacity=1
                          ), name="Path")
    camera = dict(
        up=dict(x=0, y=0, z=1),
        center=dict(x=0, y=0, z=0),
        eye=dict(x=-1.5, y=-2, z=1.5)
    )
    fig.update_layout(
        scene_camera=camera,
        # width=400, height=400,
        title=title,
        margin=dict(t=40, r=0, l=20, b=20),
        scene=dict(
            zaxis=dict(nticks=4, range=[0, z_axis_range_upper_bound or z_data.shape[0]])),
    )
    fig.update_layout(scene_aspectmode='manual',
                      scene_aspectratio=dict(x=z_data.shape[1] / z_data.shape[0], y=1,
                                             z=np.max(z_data) / z_data.shape[0]))

    if len(visited):
        visited_data = np.array(list(visited))
        fig.add_scatter3d(x=visited_data[:, 1], y=visited_data[:, 0], z=visited_data[:, 2], mode='markers',
                          marker=dict(
                              size=marker_size,
                              # color=z,  # set color to an array/list of desired values
                              colorscale='Viridis',  # choose a colorscale
                              opacity=marker_opacity
                          ))
    return fig
