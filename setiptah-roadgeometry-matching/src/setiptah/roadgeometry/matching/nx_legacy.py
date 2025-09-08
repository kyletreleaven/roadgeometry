import bintrees
import networkx as nx

from setiptah.roadgeometry.graphs import RoadNetwork
from .bm import *


@dataclass
class MultiDiGraphRoadnet(RoadNetwork[TVert, TRoad]):
    graph: nx.MultiDiGraph
    length_attr: str = "length"
    oneway_attr: str = "oneway"

    def __post_init__(self):
        super().__init__()

        for i in self.graph.nodes:
            self.add_node(i)

        for i, j, road, data in self.graph.edges(keys=True, data=True):
            self.add_edge(road, i, j, data[self.length_attr], oneway=data.get(self.oneway_attr, False))


def ROADSBIPARTITEMATCH( P, Q, roadnet_graph: nx.MultiDiGraph, **kwargs ) :
    return optimal_roadnet_matching(
        P, Q, MultiDiGraphRoadnet(roadnet_graph), **kwargs
    )


OrderedPoints = bintrees.RBTree
"""

A collection data structure for bipartite points on a road.
The keys are numeric coordinates (e.g., float), with
a `TwoQueues`---a pair of "queues" (AKA lists)---containing points of the two types,
respectively, at given coordinate.

"""


def SEGMENTS(P, Q, roadnet: nx.MultiDiGraph) -> dict[TRoad, OrderedPoints]:
    # TODO: Return RBTree inner here?
    return compute_segments(P, Q, MultiDiGraphRoadnet(roadnet))


def WRITEOBJECTIVES(P, Q, roadnet_graph: nx.MultiDiGraph):
    return write_objectives(P, Q, MultiDiGraphRoadnet(roadnet_graph))


def write_objectives(P, Q, roadnet: Roadnet):
    segment_dict = compute_segments(P, Q, roadnet)
    surplus_dict = dict()
    objective_dict = dict()

    for road, segment in segment_dict.items():
        match = PREMATCH(segment)

        surplus_dict[road] = SURPLUS(segment)

        measure = MEASURE(segment, roadnet.length(road))
        objective_dict[road] = OBJECTIVE(measure)

    return objective_dict


def compute_segments(P, Q, roadnet: Roadnet[TRoad, TVert]) -> dict[TRoad, OrderedPoints]:
    """
    returns:
    a dictionary whose keys are coordinates and whose values are local (P,Q) index queues
    """
    segments = dict()
    for road in roadnet.edges():
        ensure_road(road, segments)  # these, and only these, roads are allowed

    for i, p in enumerate(P):
        r, y = p
        tree = segments[r]  # crash by design if r not in segments
        queues = ensure_key(y, tree)
        queues.supply.append(i)

    for j, q in enumerate(Q):
        r, y = q
        tree = segments[r]
        queues = ensure_key(y, tree)
        queues.demand.append(j)

    return segments


def CHECKFLOW(
        flow: dict[TRoad, float],
        roadnet: nx.MultiDiGraph,
        surplus: dict[TVert, float]
) -> dict[TVert, float]:
    return check_flow(flow, MultiDiGraphRoadnet(roadnet), surplus)


def MATCHCOSTS(matching: tuple[int, int], P, Q, roadnet: nx.MultiDiGraph):
    metric = RoadnetMetric(MultiDiGraphRoadnet(roadnet))
    inst = MatchingInstance(P, Q, metric)
    return [
        inst.match_cost(match)
        for match in matching
    ]


def ROADMATCHCOST( match, P, Q, roadnet ) :
    costs = MATCHCOSTS( match, P, Q, roadnet )
    return sum( costs )


def ensure_road( road, data ) :
    curr = data.setdefault( road )
    if curr is None : data[road] = bintrees.RBTree()
    return data[road]
