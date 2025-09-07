import networkx as nx

from .matchvis_util import position, VERTEX, POINT_IN_S, POINT_IN_T
from setiptah.roadgeometry.matching.nx_legacy import SEGMENTS

""" my dependencies """

import matplotlib.pyplot as plt


""" CONSTANTS """

""" labels for three kinds of graph nodes """

ZNODES = 1
ZLABELS = 2
ZEDGES = 3
ZTRAILS = 4
ZPOINTS = 5




""" convenience functions """


def pointsToXY( points ) :
    """ split a list of (x,y) coordinates into X and Y; usually for plotting """
    X = [ x for x,y in points ]
    Y = [ y for x,y in points ]
    return X, Y


def drawRoadmap( roadmap: nx.DiGraph, pos, ax=None, **kwargs ) :
    if ax is None : ax = plt.gca()
    
    # draw the skeleton (undirected)
    skeleton = roadmap.to_undirected()
    nx.draw_networkx_nodes( skeleton, pos, ax=ax
                            # , zorder=ZNODES
                            )
    style = [
        ("solid" if data.get("oneway", False) else "dotted")
        for _, __, data in roadmap.edges(data=True)
    ]
    nx.draw_networkx_edges( skeleton, pos=pos, ax=ax,
                            # zorder=ZEDGES,
                            **kwargs,
                            # style=style,
                            )
    
    road_labels = {
        (u,v) : road + (" (directed)" if data.get("oneway", False) else "") + '\n'     # the endline is to raise the label
        for u, v, road, data in roadmap.edges(keys=True, data=True)
    }
    nx.draw_networkx_edge_labels( skeleton, pos=pos, ax=ax, 
                                  edge_labels=road_labels,
                                  # zorder=ZLABELS
                                  )
    
    return ax


def SHOWTRAILS( S, T, assist, roadmap, pos, length_attr='length',
                ax=None, **kwargs ) :
    """
    visualize a matching on a roadmap:
    """
    
    # draw the roadmap
    if ax is None : ax = plt.gca()
    options = { 'edge_color' : 'g', 'alpha' : .15 }     # lightly, though...
    options.update( kwargs )                            # but let overrides
    drawRoadmap( roadmap, pos, ax=ax, **options )
    ax.set_aspect('equal')  # i just really like equal aspect...
    
    """ The hard part is getting the edges with proper thickness """
    # sort points onto segments
    segments = SEGMENTS( S, T, roadmap )

    # initialize a path graph
    graph = nx.Graph()
    
    for u, v, road, data in roadmap.edges( keys=True, data=True ) :
        width = data.get( length_attr, 1 )
        
        def traverse() :
            yield 0., VERTEX, u     # location, type, label
            for y, queue in segments[road].iter_items() :
                for s in queue.supply:
                    yield y, POINT_IN_S, s
                for t in queue.demand:
                    yield y, POINT_IN_T, t
            yield width, VERTEX, v
            
        ITER = traverse()
        prev = next(ITER)
        z = assist[road]    # start road assistance +0
        for y2, type2, label2 in ITER :
            y1, type1, label1 = prev
            graph.add_edge( (type1,label1), (type2,label2), weight=y2-y1, score=abs(z) )
            if type2 == POINT_IN_S:
                z += 1
            elif type2 == POINT_IN_T:
                z -= 1 
            
            prev = y2, type2, label2
            
                        
    SHOW_THICKNESS_GRAPH( graph, S, T, roadmap, pos, ax )


def SHOWMATCH( match, S, T, roadmap, pos, length_attr='length', ax=None,
               **kwargs ) :
    """
    visualize a matching on a roadmap:
    imagine depositing one uniform trail of ink,
    for each match in the matching,
    on the shortest path between the endpoints of the match;
    segments of the network more often covered will obtain more ink
    """
    
    # draw the roadmap
    if ax is None : ax = plt.gca()
    options = { 'edge_color' : 'g', 'alpha' : .15 }     # lightly, though...
    options.update( kwargs )                            # but let overrides
    drawRoadmap( roadmap, pos, ax=ax, **options )
    ax.set_aspect('equal')  # i just like equal aspect...
    
    """ The hard part is getting the edges with proper thickness """
    # sort points onto segments
    segments = SEGMENTS( S, T, roadmap )
    
    # make a path graph
    graph = nx.Graph()
    
    for u, v, road, data in roadmap.edges( keys=True, data=True ) :
        width = data.get( length_attr, 1 )
        
        def traverse() :
            yield 0., VERTEX, u     # location, type, label
            for y, queue in segments[road].iter_items() :
                for s in queue.supply:
                    yield y, POINT_IN_S, s
                for t in queue.demand:
                    yield y, POINT_IN_T, t
            yield width, VERTEX, v
            
        ITER = traverse()
        prev = next(ITER)
        for y2, type2, label2 in ITER :
            y1, type1, label1 = prev
            graph.add_edge( (type1,label1), (type2,label2), weight=y2-y1, score=0 )
            prev = y2, type2, label2
            
    # add unit weight to shortest paths
    for i, j in match :
        path = nx.shortest_path(graph, (POINT_IN_S, i), (POINT_IN_T, j),
                                weight='weight')
        
        for ii, jj in zip( path[:-1], path[1:] ) :
            data = graph.get_edge_data( ii, jj )
            data['score'] += 1

    SHOW_THICKNESS_GRAPH( graph, S, T, roadmap, pos, ax )


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
