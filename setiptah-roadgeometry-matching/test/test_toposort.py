import networkx as nx

import setiptah.roadgeometry.matching.util

from setiptah.roadgeometry.matching.util.mygraph import mygraph
from setiptah.roadgeometry.matching.util.toposort import toposort


def test_toposort():

    g = mygraph()
    g.add_edge( 'a', 0, 1 )
    g.add_edge( 'b', 0, 2 )
    g.add_edge( 'e', 1, 3 )
    g.add_edge( 'c', 2, 3 )
    g.add_edge( 'd', 3, 4 )

    order = toposort( g )
    poses = {i: k for k, i in enumerate(order)}

    assert all(
        poses[u] <= poses[v]
        for u, v in g.E.values()
    )
