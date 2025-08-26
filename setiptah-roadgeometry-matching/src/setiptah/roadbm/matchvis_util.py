"""Interval Graph is a very useful utility, e.g., for visualization
"""
import networkx as nx

import setiptah.roadgeometry.roadmap_basic as ROAD
#from setiptah.roadgeometry.roadmap_basic import RoadAddress, get_road_data

from setiptah.roadbm import bm as roadbm

VERTEX = 'v'
POINT_IN_S = 'S'
POINT_IN_T = 'T'


def INTERVAL_GRAPH( match, S, T, roadmap, pos, length_attr='length' ) :
    # start a "path graph" --- damn, has to be undirected...
    digraph = nx.DiGraph()
    skeleton = nx.Graph()

    # sort points onto segments
    segments = roadbm.SEGMENTS( S, T, roadmap)

    for u, v, road, data in roadmap.edges_iter( keys=True, data=True ) :
        # store coordinate of the r^+ endpoint for later use
        length = data.get( length_attr, 1 )

        # enumerates the points on segment in a specific order
        def traverse() :
            yield 0., VERTEX, u     # location, type, label
            for y, queue in segments[road].iter_items() :
                for s in queue.P : yield y, POINT_IN_S, s
                for t in queue.Q : yield y, POINT_IN_T, t
            yield length, VERTEX, v

        # bigram enumeration and edge insertion
        ITER = traverse()
        next = ITER.next()
        for y2, type2, label2 in ITER:
            y1, type1, label1 = next

            # insert edge into score graph *and* skeleton graph
            digraph.add_edge( (type1,label1), (type2,label2), score=0 )
            skeleton.add_edge( (type1,label1), (type2,label2), length=y2-y1 )

            next = y2, type2, label2

    # for each match in the matching
    for i, j in match :
        # find shortest path on the *skeleton* graph, i.e., ignoring direction
        path = nx.shortest_path( skeleton, (POINT_IN_S,i), (POINT_IN_T,j),
                                 weight='length' )

        # direct unit score along shortest path
        for ii, jj in zip( path[:-1], path[1:] ) :
            # path traverses edge in the forward direction
            if digraph.has_edge( ii, jj ) :
                data = digraph.get_edge_data( ii, jj )
                data['score'] += 1

            # otherwise, path traverses edge in the backward direction
            elif digraph.has_edge( jj, ii ) :
                data = digraph.get_edge_data( jj, ii )
                data['score'] -= 1

                # then, reverse edge if it has negative score (only need to check if minus)
                score = data['score']
                if score < 0 :
                    digraph.remove_edge( jj, ii )
                    digraph.add_edge( ii, jj, score = -score )

            else:
                raise Exception('edge not found')

    def vertpos(u) : return pos[u]
    def pos_from_S(u) : return position( S[u], roadmap, pos )
    def pos_from_T(u) : return position( T[u], roadmap, pos )
    switch = { VERTEX : vertpos,
              POINT_IN_S : pos_from_S,
              POINT_IN_T : pos_from_T }

    other_pos = {}
    for uu in digraph.nodes_iter() :
        typeu, labelu = uu
        other_pos[uu] = switch[typeu]( labelu )

    return digraph, other_pos


def position(address, roadmap, pos, length_attr='length'):
    """
    get the Euclidean position of a street address,
    given roadmap and dictionary of vertex positions
    """
    road, coord = address
    coord = float(coord)

    u, v, key = ROAD.obtain_edge(roadmap, road)
    assert key == road
    data = ROAD.get_road_data(road, roadmap)
    width = data.get( length_attr, 1 )

    #ROAD.get_edge_data( )
    x = pos[u]
    vec = pos[v] - x
    return x + vec * coord / width

