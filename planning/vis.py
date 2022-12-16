import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from typing import Tuple, List, Callable, Union, Set

from plotly import graph_objects as go

from planning.map import Coor, Map


def vis_2d_histogram(bmap: Map, path_pts: List[Coor] = None, figsize: Tuple[int, int] = (8, 8),
                     title: str = None, src: Tuple = None, target: Tuple = None,
                     path: List[Coor] = [], visited: List[Coor] = []) -> plt.figure:
    f = plt.figure(figsize=figsize)
    ax = plt.axes(projection='3d')
    _x = np.arange(bmap.shape[0])
    _y = np.arange(bmap.shape[1])
    _xx, _yy = np.meshgrid(_x, _y)
    x, y = _xx.ravel(), _yy.ravel()

    top = bmap.map.flatten()
    bottom = np.zeros_like(top)
    width = depth = 1

    cmap = cm.get_cmap()
    bar_plt = ax.bar3d(x, y, bottom, width, depth, top, shade=True, color=cmap(top / np.max(top)))

    # xline, yline, zline = path_pts[:, 0], path_pts[:, 1], path_pts[:, 2]
    # ax1.plot3D(xline, yline, zline, 'red')
    if path_pts is not None:
        edges = np.array([(path_pts[i], path_pts[i + 1]) for i in range(len(path_pts) - 1)])
        for edge in edges:
            xline, yline, zline = edge[:, 0], edge[:, 1], edge[:, 2]
            ax.plot3D(xline, yline, zline, 'red')
    if title is not None:
        ax.set_title(title)
    if len(path) > 0:
        path_data = np.array(path)
        plt.plot(path_data[:, 1], path_data[:, 0], path_data[:, 2])
    if len(visited) > 0:
        visited_data = np.array(path)
        plt.scatter(visited_data[:, 1], visited_data[:, 0], visited_data[:, 2])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    # plot source and target
    if src is not None and target is not None:
        print("scatter")
        data = np.array([src, target])
        ax.scatter(data[:, 1], data[:, 0], data[:, 2], marker="o", c="b")

    # plt.close()
    return f


def vis_with_plotly(z_data: np.ndarray, src: Coor = None, target: Coor = None, path_pts: List[Coor] = [],
                    title: str = "Visualization", visited: Union[List[Coor], Set[Coor]] = [], marker_size: float = 1,
                    marker_opacity: float = 0.3, z_axis_range_upper_bound: float = None,
                    background_injection_fn: Callable = None):
    # path_df = pd.DataFrame(np.array(path_pts), columns=['row', 'col', 'height'])
    # path_plot = px.line_3d(path_df, x="col", y="row", z="height")
    fig = go.Figure()
    fig.add_surface(z=z_data, showscale=False)
    if background_injection_fn is not None:
        background_injection_fn(fig)
    if src:
        src_data = np.array([src])
        fig.add_scatter3d(x=src_data[:, 1], y=src_data[:, 0], z=src_data[:, 2], mode='markers',
                          marker=dict(
                              size=20,
                              color="lightgreen",
                              # color=z,  # set color to an array/list of desired values
                              colorscale='Viridis',  # choose a colorscale
                              opacity=0.8
                          ), name="Source")
    if target:
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
    z_max = np.max(z_data)
    fig.update_layout(scene_aspectmode='manual',
                      scene_aspectratio=dict(x=z_data.shape[1] / z_max, y=z_data.shape[0] / z_max, z=1))

    if len(visited):
        visited_data = np.array(list(visited))
        fig.add_scatter3d(x=visited_data[:, 1], y=visited_data[:, 0], z=visited_data[:, 2], mode='markers',
                          marker=dict(
                              size=marker_size,
                              color='#ffffff',  # set color to an array/list of desired values
                              colorscale='Viridis',  # choose a colorscale
                              opacity=marker_opacity
                          ))
    return fig
