import networkx as nx
import pytest

import setiptah.roadgeometry.legacy.roadmap_basic as ROAD
from setiptah.roadgeometry.dijkstra import RoadnetMetric, RoadSegment
from setiptah.roadgeometry.formats import to_networkx
from setiptah.roadgeometry.graphs import RoadNetwork
from setiptah.roadgeometry.matching.nx_legacy import MultiDiGraphRoadnet
from setiptah.roadgeometry.matching.util.mygraph import *
from setiptah.roadgeometry.protocol import Roadnet


@pytest.fixture
def square_roadnet():

    roadnet = nx.MultiDiGraph()
    if True:
        roadnet.add_edge(0, 1, 'N', length=1.)
    else:
        # TODO: Do this in a meaningful way.
        # to test one-way roads capabilities
        roadnet.add_edge(0, 1, 'N', length=1., oneway=True)

    roadnet.add_edge(1, 2, 'E', length=1.)
    roadnet.add_edge(2, 3, 'S', length=1.)
    roadnet.add_edge(3, 0, 'W', length=1.)

    if True:
        roadnet.add_edge(0, 4, 'dangler', length=1.)

    return MultiDiGraphRoadnet(roadnet)


def test_metric(square_roadnet):
    roadnet = square_roadnet
    roadnet_graph = roadnet.graph

    # assert False, roadnet.nodes()
    # assert False, roadnet.node_data

    metric = RoadnetMetric(roadnet)
    # assert False, list(metric._out_edges(2))

    assert metric.graph_shortest_path_length(2, 4) == 3

    # TODO: Spy test to assert one call only.
    metric.graph_shortest_path_length(2, 0)

    # assert False, metric._upstream
    # assert False, metric.graph_shortest_path(2, 4)
    assert metric.distance(("E", 1.), ("dangler", .5)) == 2.5


def test_embeddings(square_roadnet):
    metric = RoadnetMetric(square_roadnet)
    assert set(metric.embeddings(0)) == {("dangler", 0.), ("N", 0.), ("W", 1.)}


def test_distances():

    rn = RoadNetwork()

    A, B = "AB"
    rn.add_edge(A, 0, 1, 10.)
    rn.add_edge(B, 2, 3, 10., oneway=True)

    metric = RoadnetMetric(rn)

    uA = A, 0.
    vA = A, 10.
    midA = A, 5.

    assert metric.distance_on_road(uA, vA, A) == 10.
    assert metric.distance_on_road(vA, uA, A) == 10.
    assert metric.distance_on_road(uA, midA, A) == 5.

    uB = B, 0.
    vB = B, 10.
    midB = B, 5.

    assert metric.distance_on_road(uB, vB, B) == 10.
    assert metric.distance_on_road(vB, uB, B) == np.inf
    assert metric.distance_on_road(uB, midB, B) == 5.
    assert metric.distance_on_road(midB, vB, B) == 5.
    assert metric.distance_on_road(midB, uB, B) == np.inf

    assert metric.distance_on_road(midA, midB, A) == np.inf

    assert metric.distance_node_to_point(rn.endpoints(A)[0], uA) == 0
    assert metric.distance_node_to_point(rn.endpoints(A)[1], uA) == 10.
    assert metric.distance_node_to_point(rn.endpoints(B)[0], uB) == 0
    assert metric.distance_node_to_point(rn.endpoints(B)[0], vB) == 10.
    assert metric.distance_node_to_point(rn.endpoints(B)[1], uB) == np.inf


def test_shortest_path():
    rn = RoadNetwork()
    rn.add_edge("R", 0, 1, length=10.)

    metric = RoadnetMetric(rn)

    p = ("R", 1)
    q = ("R", 9)

    assert metric.shortest_path(p, q) == [RoadSegment("R", 1, 9)]


@pytest.fixture
def example_net2():
    rn = RoadNetwork()

    rn.add_edge("N", 0, 1, 1., oneway=True)
    rn.add_edge("E", 1, 2, 1.)
    rn.add_edge("S", 2, 3, 1.)
    rn.add_edge("W", 3, 0, 2.)
    rn.add_edge("W_", 0, 3, 1., oneway=True)

    rn.add_edge("D", 0, 4, 1.)

    rn.add_edge("U", 5, 2, 1., oneway=True)

    return rn


def test_unreachable_node(example_net2):
    rn = example_net2

    metric = RoadnetMetric(rn)

    assert min(
        metric.graph_shortest_path_length(u, 5)
        for u in rn.nodes()
        if u != 5
    ) == np.inf


def test_metric_node_distances(example_net2):
    rn = example_net2

    # For metric distance
    metric = RoadnetMetric(rn)

    # For legacy distance
    mg = to_networkx(rn)

    def embed(u):
        for p in metric.embeddings(u):
            return ROAD.RoadAddress(*p)

    def legacy_distance(u, v):
        return ROAD.distance(mg, embed(u), embed(v), "length")

    # For networkx-based distance
    spg = shortest_path_graph(rn)

    def networkx_distance(u, v):
        try:
            return nx.shortest_path_length(spg, u, v, weight="length")
        except nx.exception.NetworkXNoPath:
            return np.inf

    for u in rn.nodes():
        for v in rn.nodes():
            distances = {
                metric.graph_shortest_path_length(u, v),
                legacy_distance(u, v),
                networkx_distance(u, v),
            }
            d, = distances  # i.e., they should all be the same!


def shortest_path_graph(rn: Roadnet):
    g = nx.MultiDiGraph()
    g.add_nodes_from(rn.nodes())

    for e in rn.edges():
        i, j = rn.endpoints(e)
        L = rn.length(e)

        e_ = e, "+"
        g.add_edge(i, j, key=e_, length=L)

        if not rn.is_oneway(e):
            e_ = e, "-"
            g.add_edge(j, i, key=e_, length=L)

    return g


class TestRoadnetMetric:

    def test_shortest_path_on_road(self, square_roadnet):
        metric = RoadnetMetric(square_roadnet)
        assert metric.shortest_path_on_road(("E", .1), ("E", .5), "E") == [RoadSegment("E", .1, .5)]

    def test_shortest_path_node_to_point(self, square_roadnet):
        metric = RoadnetMetric(square_roadnet)
        path = metric.shortest_path_node_to_point(4, ("E", .5))
        assert path == [
            RoadSegment(road='dangler', start=1.0, end=0.0),
            RoadSegment(road='N', start=0.0, end=1.0),
            RoadSegment(road='E', start=0.0, end=0.5)
        ]

    def test_shortest_path(self, square_roadnet):
        metric = RoadnetMetric(square_roadnet)

        path = metric.shortest_path(("N", .5), ("E", .5))
        assert path == [
            RoadSegment(road='N', start=0.5, end=1.0), RoadSegment(road='E', start=0.0, end=0.5)
        ]

    def test_shortest_path_infeasible(self):
        rn = RoadNetwork()
        rn.add_edge("A", 0, 1, 1.)
        rn.add_edge("B", 2, 3, 1.)
        metric = RoadnetMetric(rn)

        assert metric.shortest_path(("A", .5), ("B", .5)) is None
        assert metric.distance(("A", .5), ("B", .5)) == np.inf

        assert metric.shortest_path_node_to_point(0, ("B", .5)) is None

    def test_embedding_error(self):
        rn = RoadNetwork()
        rn.add_edge("A", 0, 1, 1)
        rn.add_edge("B", 1, 2, 1)
        metric = RoadnetMetric(rn)
        with pytest.raises(ValueError):
            metric.embedding(0, "B")
