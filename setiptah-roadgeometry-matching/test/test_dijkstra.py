import itertools

import numpy as np
import networkx as nx

from setiptah.basic_graph.mygraph import mygraph
from setiptah.basic_graph.dijkstra import *
from setiptah.roadbm.nx_legacy import MultiDiGraphRoadnet
from setiptah.basic_graph.graphs import RoadNetwork

import setiptah.roadgeometry.roadmap_basic as ROAD

import pytest


def test_dijkstra():
    g = mygraph()
    g.add_edge('a', 0, 1)
    g.add_edge('b', 0, 2)
    g.add_edge('c', 2, 3)
    g.add_edge('d', 3, 0)

    c = {'a': 10., 'b': 5., 'c': 1., 'd': .5}
    f = {'a': 3., 'b': 1.37}

    g = mygraph()
    c = {}
    edgegen = itertools.count()

    nxg = nx.DiGraph()

    X = [(i, np.random.rand(2)) for i in range(10)]
    for (i, x), (j, y) in itertools.product(X, X):
        cost = np.linalg.norm(y - x)
        if cost < .3:
            e = next(edgegen)
            g.add_edge(e, i, j)
            c[e] = cost

            nxg.add_edge(i, j, weight=cost)

    s = 0
    d, pred = Dijkstra(g, c, s)

    # Compare to networkx solution
    nxd = nx.single_source_dijkstra_path_length(nxg, s)

    assert all(
        d.get(u) == nxd.get(u)
        for u in set(d) | set(nxd)
    )


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


def test_metric_node_distances():
    rn = RoadNetwork()

    rn.add_edge("N", 0, 1, 1., oneway=True)
    rn.add_edge("E", 1, 2, 1.)
    rn.add_edge("S", 2, 3, 1.)
    rn.add_edge("W", 3, 0, 2.)
    rn.add_edge("W_", 0, 3, 1., oneway=True)

    rn.add_edge("D", 0, 4, 1.)

    rn.add_edge("U", 5, 2, 1., oneway=True)

    g = mygraph()
    cost = {}

    for u in rn.nodes():
        g.add_node(u)

    for e in rn.edges():
        i, j = rn.endpoints(e)
        L = rn.length(e)

        e_ = e, "+"
        g.add_edge(e_, i, j)
        cost[e_] = L

        if not rn.is_oneway(e):
            e_ = e, "-"
            g.add_edge(e_, j, i)
            cost[e_] = L

    dref = {
        u: Dijkstra(g, cost, u)[0]
        for u in rn.nodes()
    }

    metric = RoadnetMetric(rn)
    for u in rn.nodes():
        metric._populate_dijkstra(u)

    assert min(
        metric.graph_shortest_path_length(u, 5)
        for u in rn.nodes()
        if u != 5
    ) == np.inf

    # compare to old distance metric
    mg = create_multigraph(rn)

    def embed(u):
        for p in metric.embeddings(u):
            return ROAD.RoadAddress(*p)

    dref_ = {
        u: {
            v: ROAD.distance(mg, embed(u), embed(v), "length")
            for v in rn.nodes()
        }
        for u in rn.nodes()
    }

    from setiptah.roadgeometry.astar_basic import astar_path_length

    distances = [
        metric.graph_shortest_path_length(0, 2),
        dref[0][2],
        # TODO: Debug!
        # dref_[0][2],
        # TODO: How do we integrate with astar in match cost?
        # astar_path_length(mg, 0, 2, None, "length")
    ]
    assert len(set(distances)) == 1, distances

    # assert False, dref
    diff = {
        (u, v)
        for u in rn.nodes()
        for v in rn.nodes()
        if metric._distance[u].get(v, None) != dref_[u][v]
    }
    # assert not diff, diff
    if True:
        assert metric._distance == dref
    else:
        for u in rn.nodes():
            assert metric._distance[u] == dref[u], u


def create_multigraph(rn: Roadnet):
    g = nx.MultiDiGraph()
    g.add_nodes_from(rn.nodes())
    for r in rn.edges():
        i, j = rn.endpoints(r)
        length, oneway = rn.length(r), rn.is_oneway(r)
        g.add_edge(i, j, r, length=length, oneway=oneway)
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
