import random
import numpy as np
import bintrees

import networkx as nx

import matplotlib.pyplot as plt
from setiptah.roadgeometry.matching.draw.matchvis import SHOWMATCH


def test_showmatch():
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
    SHOWMATCH(random_match, SS, TT, roadmap, pos=pos, edge_color='k', alpha=1.)

    plt.gca().set_aspect('equal')
    plt.close("all")
