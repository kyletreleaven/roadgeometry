import matplotlib.pyplot as plt
import networkx as nx

from setiptah.roadgeometry.planar import PlanarRoadnet


def draw_planar_roadnet(rn: PlanarRoadnet, ax=None, **kwargs):
    if ax is None:
        ax = plt.gca()

    # draw the skeleton (undirected)
    skeleton = nx.DiGraph()
    for edge in rn.edges():
        u, v = rn.endpoints(edge)
        skeleton.add_edge(u, v, oneway=rn.is_oneway(edge))

    pos = rn.pos
    nx.draw_networkx_nodes(skeleton, rn.pos, ax=ax)

    arrowstyles = [
        ("-|>" if rn.is_oneway(edge) else "-")
        for edge in rn.edges()
    ]

    nx.draw_networkx_edges(skeleton, pos=pos, ax=ax,
                           arrowstyle=arrowstyles,
                           arrowsize=30,
                           **kwargs
                           )

    road_labels = {
        rn.endpoints(road): road + (" (directed)" if rn.is_oneway(road) else "") + '\n'
        # the endline is to raise the label
        for road in rn.edges()
    }
    nx.draw_networkx_edge_labels(skeleton, pos=pos, ax=ax, edge_labels=road_labels)

    return ax
