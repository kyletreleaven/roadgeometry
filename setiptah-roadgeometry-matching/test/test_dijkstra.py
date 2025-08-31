import itertools

import numpy as np
import networkx as nx

from setiptah.basic_graph.mygraph import mygraph
from setiptah.basic_graph.dijkstra import *
from setiptah.roadbm import MultiDiGraphRoadnet


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


def test_metric():

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

    roadnet, roadnet_graph = MultiDiGraphRoadnet(roadnet), roadnet
    # assert False, roadnet.nodes()
    # assert False, roadnet.node_data

    metric = RoadnetMetric(roadnet)
    # assert False, list(metric._out_edges(2))

    assert metric.graph_shortest_path_length(2, 4) == 3

    # TODO: Spy test to assert one call only.
    metric.graph_shortest_path_length(2, 0)

    # assert False, metric._upstream
    assert False, metric.graph_shortest_path(2, 4)
