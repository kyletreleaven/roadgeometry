from dataclasses import dataclass
from typing import TypeVar, Generic, Optional

import numpy as np

from setiptah.basic_graph.dijkstra import RoadnetMetric
from setiptah.basic_graph.graphs import RoadNetwork, TV, TE

TVert = TypeVar("TV")
TRoad = TypeVar("TE")


@dataclass(frozen=True)
class RoadInfo(Generic[TVert]):
    left: TVert
    right: TVert
    oneway: bool = False


class PlanarRoadnet(RoadNetwork[TRoad, TVert]):

    def __init__(self):
        super().__init__()
        self.pos = {}

    def add_node(self, i: TV, pos: Optional[tuple[float, float]] = None):
        if pos is None:
            assert i in self.pos
        else:
            super().add_node(i)
            self.pos[i] = pos

    def add_edge(self, e: TE, i: TV, j: TV, *, oneway: bool = False):
        nodes = self.nodes()
        assert i in nodes and j in nodes
        super().add_edge(e, i, j, length=None, oneway=oneway)  # length managed elsewhere

    def remove_node(self, i: TV):
        super().remove_node(i)
        del self.pos[i]

    def length(self, road: TRoad) -> float:
        i, j = self.endpoints(road)
        xi, yi = self.pos[i]
        xj, yj = self.pos[j]
        return np.linalg.norm([xj - xi, yj - yi], 2)

    def position(self, p: RoadNetwork[TRoad, TVert].Point):
        assert RoadnetMetric(self).is_valid_point(p)
        road, x = p
        pi, pj = [np.array(self.pos[i]) for i in self.endpoints(road)]
        return pi + x * (pj - pi)
