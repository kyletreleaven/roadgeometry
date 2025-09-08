import networkx as nx

from setiptah.roadgeometry.planar import PlanarRoadnet
from setiptah.roadgeometry.protocol import *


def create_multigraph(roadnet: Roadnet):
    g = nx.MultiDiGraph()
    g.add_nodes_from(roadnet.nodes())
    for road in roadnet.edges():
        i, j = roadnet.endpoints(road)
        g.add_edge(i, j, road, length=roadnet.length(road), oneway=roadnet.is_oneway(road))
    return g


def multigraph_to_planar(
        roadmap: nx.MultiDiGraph, pos: dict[tuple[TVert, TVert], tuple[float, float]],
        oneway_attr: bool = "oneway",
) -> PlanarRoadnet[TRoad, TVert]:
    planar = PlanarRoadnet()
    for u in roadmap.nodes():
        planar.add_node(u, pos[u])
    for u, v, road, data in roadmap.edges(keys=True, data=True):
        assert road not in planar.edges()
        planar.add_edge(road, u, v, oneway=data.get(oneway_attr, False))
    return planar
