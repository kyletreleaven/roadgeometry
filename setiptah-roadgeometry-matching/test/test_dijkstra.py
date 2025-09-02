import itertools

import numpy as np
import networkx as nx

from setiptah.basic_graph.mygraph import mygraph
from setiptah.basic_graph.dijkstra import *
from setiptah.roadbm import MultiDiGraphRoadnet
from setiptah.basic_graph.graphs import RoadNetwork

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
