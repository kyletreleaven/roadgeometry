"""Supporting conversion between MultiDiGraph.

"""
from dataclasses import dataclass
from typing import Union

import networkx as nx

from setiptah.roadgeometry.graphs import RoadNetwork
from setiptah.roadgeometry.protocol import Roadnet


@dataclass(frozen=True)
class GraphSchema:
    edge_attr: str = None
    length_attr: str = "length"
    oneway_attr: str = "oneway"
    oneway_default: bool = False

    def to_networkx(self, roadnet: Roadnet) -> nx.MultiDiGraph:
        g = nx.MultiDiGraph()

        g.add_nodes_from(roadnet.nodes())

        for road in roadnet.edges():
            i, j = roadnet.endpoints(road)

            attrs = {
                self.length_attr: roadnet.length(road),
                self.oneway_attr: roadnet.is_oneway(road),
            }
            if self.edge_attr is not None:
                attrs[self.edge_attr] = road

            g.add_edge(i, j, road, **attrs)

        return g

    def from_networkx(self, graph: Union[nx.DiGraph, nx.MultiDiGraph]) -> RoadNetwork:
        rn = RoadNetwork()
        self.dump_networkx(graph, rn)
        return rn

    def dump_networkx(self, graph: Union[nx.DiGraph, nx.MultiDiGraph], roadnet):

        for i in graph.nodes():
            roadnet.add_node(i)

        if isinstance(graph, nx.MultiDiGraph):
            edges = graph.edges(keys=True, data=True)
        elif isinstance(graph, nx.DiGraph):
            edges = (
                (i, j, k, data)
                for k, (i, j, data) in enumerate(graph.edges(data=True))
            )
        else:
            raise ValueError("graph should be either DiGraph or MultiDiGraph")

        for i, j, road, data in edges:
            if self.edge_attr is not None:
                road = data[self.edge_attr]
            roadnet.add_edge(
                road, i, j,
                data[self.length_attr],
                oneway=data.get(self.oneway_attr, self.oneway_default)
            )


def to_networkx(roadnet: Roadnet, **kwargs) -> nx.MultiDiGraph:
    return GraphSchema(**kwargs).to_networkx(roadnet)


def from_networkx(graph: Union[nx.DiGraph, nx.MultiDiGraph], **kwargs) -> RoadNetwork:
    return GraphSchema(**kwargs).from_networkx(graph)


def dump_networkx(graph: Union[nx.DiGraph, nx.MultiDiGraph], roadnet: RoadNetwork, **kwargs):
    return GraphSchema(**kwargs).dump_networkx(graph, roadnet)
