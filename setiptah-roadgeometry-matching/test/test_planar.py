from setiptah.basic_graph.dijkstra import RoadnetMetric, RoadSegment
from setiptah.basic_graph.planar import PlanarRoadnet

import pytest

@pytest.fixture
def square_roadnet():
    rn = PlanarRoadnet()

    rn.add_node(0, (0, 0))
    rn.add_node(1, (1, 0))
    rn.add_node(2, (1, 1))
    rn.add_node(3, (0, 1))

    rn.add_edge("S", 0, 1, oneway=True)
    rn.add_edge("E", 1, 2)
    rn.add_edge("N", 2, 3)
    rn.add_edge("W", 3, 0)

    return rn


def test_square(square_roadnet):

    metric = RoadnetMetric(square_roadnet)

    p = ("S", .5)
    q = ("E", .5)

    assert metric.shortest_path(p, q) == [
        RoadSegment(road='S', start=0.5, end=1.), RoadSegment(road='E', start=0.0, end=0.5)
    ]
    assert metric.shortest_path(q, p) == [
        RoadSegment(road='E', start=0.5, end=1.),
        RoadSegment(road='N', start=0.0, end=1.),
        RoadSegment(road='W', start=0.0, end=1.),
        RoadSegment(road='S', start=0.0, end=0.5),
    ]


def test_lerp(square_roadnet):
    ptup = tuple(square_roadnet.position(("E", .5)))
    assert ptup == (1., .5)
