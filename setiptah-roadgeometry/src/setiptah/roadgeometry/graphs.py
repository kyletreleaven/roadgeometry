from collections.abc import Collection, Set
from dataclasses import dataclass
from typing import TypeVar, Generic

from . import protocol
from .protocol import Roadnet

T = TypeVar("T")
TV = TypeVar("TV")
TE = TypeVar("TE")


@dataclass(frozen=True)
class RoadInfo(Generic[TV]):
    length: float
    left: TV
    right: TV
    oneway: bool


@dataclass(frozen=True)
class SetReadOnlyView(Set):
    base: Set

    def __contains__(self, x):
        return x in self.base

    def __iter__(self):
        return iter(self.base)

    def __len__(self):
        return len(self.base)

    def __repr__(self):  # pragma: no cover
        return f"ReadOnlySet({self.base!r})"

    def __eq__(self, other):
        return self.base == other


class RoadNetwork(protocol.Roadnet[TV, TE]):
    """A simple hashmap-based road network (metric graph) data structure."""

    def __init__(self):
        self._edges: dict[TE, RoadInfo[TV]] = {}  # edge -> endpoints (directed)
        self._out: dict[TV, TE] = {}  # vertex -> out edges
        self._in: dict[TV, TE] = {}  # vertex -> in edges

    def add_node(self, i: TV):
        if i not in self._out:
            self._out[i] = set()
            self._in[i] = set()

    def add_edge(self, e: TE, i: TV, j: TV, length: float, *, oneway: bool = False):
        assert e not in self._edges

        self.add_node(i)
        self.add_node(j)

        self._edges[e] = RoadInfo(length, i, j, oneway)
        self._out[i].add(e)
        self._in[j].add(e)

    def remove_edge(self, e: TE):
        assert e in self._edges

        road_info = self._edges.pop(e)
        self._out[road_info.left].remove(e)
        self._in[road_info.right].remove(e)

    def remove_node(self, i: TV):
        outs = [e for e in self._out[i]]
        for e in outs:
            self.remove_edge(e)
        ins = [e for e in self._in[i]]
        for e in ins:
            self.remove_edge(e)

        del self._out[i]
        del self._in[i]

    def nodes(self) -> Collection[TV]:
        return self._out.keys()

    def has_node(self, i: TV) -> bool:
        return i in self._out

    def edges(self) -> Collection[TE]:
        return self._edges.keys()

    def out_edges(self, u: TV) -> Collection[TE]:
        return SetReadOnlyView(self._out[u])

    def in_edges(self, u: TV) -> Collection[TE]:
        return SetReadOnlyView(self._in[u])

    def has_edge(self, e: TE) -> bool:
        return e in self._edges

    def endpoints(self, e: TE) -> tuple[TV, TV]:
        edge = self._edges[e]
        return edge.left, edge.right

    def length(self, edge: TE) -> float:
        return self._edges[edge].length

    def is_oneway(self, edge: TE) -> bool:
        return self._edges[edge].oneway


@dataclass(frozen=True)
class IntRoadnet(Roadnet[int, int]):
    """A network on road and vertex indices (integers).

    """
    roads: tuple[RoadInfo[int], ...]
    n_vertices: int

    @classmethod
    def normalize(cls, rn: Roadnet, seq_maps: bool = True):

        vert_map = {u: k for k, u in enumerate(rn.nodes())}

        road_map = {}
        roads: list[RoadInfo] = []
        for r, road_ in enumerate(rn.edges()):
            road_map[road_] = r

            i_, j_ = rn.endpoints(road_)
            length = rn.length(road_)
            oneway = rn.is_oneway(road_)

            road_info = RoadInfo(length, vert_map[i_], vert_map[j_], oneway)
            roads.append(road_info)

        inst = cls(tuple(roads), len(vert_map))

        if seq_maps:
            road_map = int_map_to_seq(road_map)
            vert_map = int_map_to_seq(vert_map)

        return inst, road_map, vert_map

    def nodes(self) -> Collection[TV]:
        return range(self.n_vertices)

    @property
    def n_roads(self) -> int:
        return len(self.roads)

    def edges(self) -> Collection[TE]:
        return range(self.n_roads)

    def is_valid(self):
        if self.n_vertices < 0:
            return False

        nodes = self.nodes()

        def road_is_valid(road: RoadInfo):
            return (
                road.length > 0.
                and road.left in nodes
                and road.right in nodes
            )

        if not all(road_is_valid(road) for road in self.roads):
            return False

        return True

    def length(self, road: int) -> float:
        return self.roads[road].length

    def endpoints(self, road: int) -> tuple[int, int]:
        road_info = self.roads[road]
        return road_info.left, road_info.right

    def is_oneway(self, road: TE) -> bool:
        return self.roads[road].oneway


def int_map_to_seq(dict_: dict[T, int]) -> list[T]:
    result = [None] * len(dict_)
    for i, k in dict_.items():
        result[k] = i
    return result
