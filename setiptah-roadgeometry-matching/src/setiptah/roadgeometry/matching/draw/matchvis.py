from typing import Union

import matplotlib.pyplot as plt
import networkx as nx

from setiptah.roadgeometry.draw import draw_planar_roadnet
from setiptah.roadgeometry.legacy.conversion import multigraph_to_planar
from setiptah.roadgeometry.matching.nx_legacy import compute_segments
from setiptah.roadgeometry.protocol import Roadnet
from .matchvis_util import position, VERTEX, POINT_IN_S, POINT_IN_T
from .. import BiPartite

""" CONSTANTS """

""" labels for three kinds of graph nodes """

ZNODES = 1
ZLABELS = 2
ZEDGES = 3
ZTRAILS = 4
ZPOINTS = 5


def pointsToXY( points ) :
    """ split a list of (x,y) coordinates into X and Y; usually for plotting """
    X = [ x for x,y in points ]
    Y = [ y for x,y in points ]
    return X, Y


def SHOWTRAILS( S, T, assist, roadmap, pos, ax=None, **kwargs):
    """
    visualize a matching on a roadmap:
    """
    roadnet = multigraph_to_planar(roadmap, pos)
    
    # draw the roadmap
    if ax is None : ax = plt.gca()
    options = { 'edge_color' : 'g', 'alpha' : .15 }     # lightly, though...
    options.update( kwargs )                            # but let overrides

    draw_planar_roadnet(roadnet, ax=ax, **options)

    ax.set_aspect('equal')  # i just really like equal aspect...

    interval_graph = flow_to_interval_graph(assist, S, T, roadnet)

    SHOW_THICKNESS_GRAPH(interval_graph, S, T, roadmap, pos, ax)


def flow_to_interval_graph(flow, S, T, roadnet: Roadnet):

    """ The hard part is getting the edges with proper thickness """
    # sort points onto segments
    segments = compute_segments(S, T, roadnet)

    # initialize a path graph
    graph = nx.Graph()

    for road in roadnet.edges():
        u, v = roadnet.endpoints(road)
        it = iterate_segment(segments[road], u, v, roadnet.length(road))
        prev = next(it)
        z = flow[road]  # start road assistance +0
        for y2, type2, label2 in it:
            y1, type1, label1 = prev
            graph.add_edge((type1, label1), (type2, label2), weight=y2 - y1, score=abs(z))
            if type2 == POINT_IN_S:
                z += 1
            elif type2 == POINT_IN_T:
                z -= 1

            prev = y2, type2, label2

    return graph


def iterate_segment(segment, u, v, length: float):
    yield 0., VERTEX, u
    for y, q in segment.iter_items():
        q: BiPartite[list]
        for s in q.supply:
            yield y, POINT_IN_S, s
        for t in q.demand:
            yield y, POINT_IN_T, t
    yield length, VERTEX, v


def SHOWMATCH(match, S, T, roadmap, pos, ax=None, **kwargs):
    """
    visualize a matching on a roadmap:
    imagine depositing one uniform trail of ink,
    for each match in the matching,
    on the shortest path between the endpoints of the match;
    segments of the network more often covered will obtain more ink
    """
    roadnet = multigraph_to_planar(roadmap, pos)

    # draw the roadmap
    if ax is None : ax = plt.gca()
    options = { 'edge_color' : 'g', 'alpha' : .15 }     # lightly, though...
    options.update( kwargs )                            # but let overrides

    draw_planar_roadnet(roadnet, ax=ax, **options)

    ax.set_aspect('equal')  # i just like equal aspect...

    interval_graph = matching_to_interval_graph(match, S, T, roadnet)

    SHOW_THICKNESS_GRAPH(interval_graph, S, T, roadmap, pos, ax)


def matching_to_interval_graph(matching, S, T, roadnet):
    """ The hard part is getting the edges with proper thickness """
    # sort points onto segments
    segments = compute_segments(S, T, roadnet)

    # make a path graph
    graph = nx.Graph()

    for road in roadnet.edges():
        u, v = roadnet.endpoints(road)
        it = iterate_segment(segments[road], u, v, roadnet.length(road))
        prev = next(it)
        for y2, type2, label2 in it:
            y1, type1, label1 = prev
            graph.add_edge((type1, label1), (type2, label2), weight=y2 - y1, score=0)
            prev = y2, type2, label2

    # add unit weight to shortest paths
    for i, j in matching:
        # TODO: Pretty sure the oneway bug is here.
        path = nx.shortest_path(graph, (POINT_IN_S, i), (POINT_IN_T, j), weight='weight')

        for ii, jj in zip(path[:-1], path[1:]):
            data = graph.get_edge_data(ii, jj)
            data['score'] += 1

    return graph


def SHOW_THICKNESS_GRAPH( graph, S, T, roadmap, pos, ax ) :            

    # position utilities
    def vertpos(u) : return pos[u]
    def pos_from_S(u) : return position(S[u], roadmap, pos)
    def pos_from_T(u) : return position(T[u], roadmap, pos)
    switch = {VERTEX: vertpos,
              POINT_IN_S: pos_from_S,
              POINT_IN_T: pos_from_T}
    
    other_pos = {}
    for uu in graph.nodes() :
        typeu, labelu = uu
        other_pos[uu] = switch[typeu]( labelu )
    
    if False :
        # figure out how to do this?
        colors = [ data['score'] for _,__,data in graph.edges( data=True ) ]
        nx.draw_networkx_edges( graph, pos=other_pos, edge_color=colors )
        plt.colorbar()  #?
        #nx.draw(G,pos,node_color='#A0CBE2',edge_color=colors,width=4,edge_cmap=plt.cm.Blues,with_labels=False)
    else :
        # plot edges in graph with variable thickness? or some other visual cue
        for uu, vv, data in graph.edges( data=True ) :
            score = data['score']
            if score <= 0 : continue    # would just waste effort
            
            posu = other_pos[uu]
            posv = other_pos[vv]
            
            xu, yu = posu
            xv, yv = posv
            options = { 'color' : 'k',
                       'alpha' : .6,
                       'linewidth' : score
                        }
            ax.plot( [xu,xv], [yu,yv], solid_capstyle='butt',
                     # butt style prevents awkward overlap of segments
                     zorder=ZTRAILS,
                     **options )
        
    # plot the points on top, so visible; this isn't working
    # show S points in red
    positions = [position(addr, roadmap, pos) for addr in S]
    options = {
               #'marker' : 'x',
               's' : 80
               }
    X, Y = pointsToXY( positions )
    ax.scatter( X, Y, color='r', zorder=ZPOINTS, marker='x', **options )
    # show T points in blue
    positions = [position(addr, roadmap, pos).tolist() for addr in T]
    X, Y = pointsToXY( positions )
    ax.scatter( X, Y, color='b', zorder=ZPOINTS, marker='$\\circ$', **options )
