"""Testing codes for drawing matching "trails" on roadnets."""
import random

import bintrees
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import pytest

from setiptah.roadgeometry.dijkstra import RoadnetMetric
from setiptah.roadgeometry.graphs import RoadNetwork
from setiptah.roadgeometry.matching.draw.matchvis import (
    SHOWMATCH, matching_to_flow, create_path_network, VertexNode, PointNode, show_match
)
from setiptah.roadgeometry.planar import PlanarRoadnet


@pytest.mark.parametrize("show_match_impl", [SHOWMATCH, show_match])
def test_showmatch(show_match_impl):
    """Test the match plotting logic and matplotlib+networkx apis."""

    interchanges = [
        (.14, .59), (.48, .6), (.4, .53), (.57, .43),
        # (.36,.27),
        (.37, .34),
        (.58, .23),
        (.11, .39), (.22, .15),
        (.12, .25),
    ]
    interchanges = [np.array(p) for p in interchanges]

    """ construct roads from Delaunay adjacencies """
    import setiptah.roadgeometry.generation as mapgen

    roadmap = mapgen.DelaunayRoadMap(interchanges)

    """ ...and build positions dictionary """
    pos = {k: point for k, point in enumerate(interchanges)}

    """ now, obtain two sets of points """
    M = 100

    import setiptah.roadgeometry.probability as roadprob

    uniform = roadprob.UniformDist(roadmap)
    unpack = lambda addr: (addr.road, addr.coord)

    SS = [unpack(uniform.sample()) for i in range(M)]
    TT = [unpack(uniform.sample()) for i in range(M)]

    # a random matching is fine, we are not testing the algorithm
    order = list(range(M))
    random.shuffle(order)
    random_match = list(zip(range(M), order))

    # test the plotting api - not the resulting image
    plt.figure()
    # network + points + match trails
    show_match_impl(random_match, SS, TT, roadmap, pos=pos, edge_color='k', alpha=1.)

    plt.gca().set_aspect('equal')
    # plt.close("all")


@pytest.fixture
def square_roadnet():
    rn = PlanarRoadnet()

    rn.add_node(0, (0, 0))
    rn.add_node(1, (1, 0))
    rn.add_node(2, (1, 1))
    rn.add_node(3, (0, 1))

    rn.add_edge("S", 0, 1)
    rn.add_edge("E", 2, 1)
    rn.add_edge("N", 2, 3)
    rn.add_edge("W", 0, 3)

    return rn


@pytest.mark.parametrize("p, q, flow_expected", [
    (("N", .75), ("S", .25), dict(zip("SENW", [1, 0, 0, -1]))),
    (("N", .25), ("S", .75), dict(zip("SENW", [0, 1, -1, 0]))),
    (("E", .25), ("W", .75), dict(zip("SENW", [0, -1, 1, 0]))),
    (("E", .75), ("W", .25), dict(zip("SENW", [-1, 0, 0, 1]))),
    (("N", .25), ("N", .75), dict(zip("SENW", [0] * 4))),
    (("N", .75), ("N", .25), dict(zip("SENW", [0] * 4))),
])
def test_matching_to_flow(
        p, q,
        flow_expected,
        square_roadnet
):
    roadnet = square_roadnet
    metric = RoadnetMetric(roadnet)

    S, T = [p], [q]

    matching = [(0, 0)]

    flow = matching_to_flow(matching, S, T, roadnet)
    assert flow == flow_expected


def test_create_path_graph():

    rn = RoadNetwork()
    rn.add_edge("A", 0, 1, 10.)

    p = ("A", 3.)
    mid = PointNode(p)
    rn_ = create_path_network([p], rn)

    road, = rn_.out_edges(VertexNode(0))
    _, v = rn_.endpoints(road)
    assert v == mid
    assert rn_.length(road) == 3.

    road, = rn_.in_edges(VertexNode(1))
    u, _ = rn_.endpoints(road)
    assert u == mid
    assert rn_.length(road) == 7.

    # TODO: Include an edge with zero points but covered by a shortest path.
