import networkx as nx

from setiptah.roadgeometry.roadmap_paths import *
from setiptah.roadgeometry.roadmap_basic import RoadAddress

def test_minpath():

    g = nx.MultiDiGraph()

    g.add_edge(0, 1, "R", length=10.)

    p = RoadAddress("R", 1)
    q = RoadAddress("R", 9)

    assert minpath(p, q, g) == [RoadSegment("R", 1, 9)]
