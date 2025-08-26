# -*- coding: utf-8 -*-
"""
Shortest paths and path lengths ( on "MixedMultiGraph" ) using A* ("A star") algorithm.
Modified (slight generalization) by Kyle Treleaven

"""

#    Copyright (C) 2004-2011 by
#    Aric Hagberg <hagberg@lanl.gov>
#    Dan Schult <dschult@colgate.edu>
#    Pieter Swart <swart@lanl.gov>
#    All rights reserved.
#    BSD license.

import itertools
from heapq import heappush, heappop
from networkx import NetworkXError
import networkx as nx

__author__ = "\n".join(["Salim Fadhley <salimfadhley@gmail.com>",
                        "Matteo Dell'Amico <matteodellamico@gmail.com>"])
#
__author__ = "\n".join( [ __author__, "Kyle Treleaven <ktreleav@mit.edu>" ] )

__all__ = [ 'astar_path', 'path_length', 'astar_path_length']







""" accepts a multi-digraph """

def astar_path( digraph, source, target, heuristic=None, weight='weight'):
    """Return a list of nodes in a shortest path between source and target
    using the A* ("A-star") algorithm.

    There may be more than one shortest path.  This returns only one.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node for path

    target : node
       Ending node for path

    heuristic : function
       A function to evaluate the estimate of the distance
       from the a node to the target.  The function takes
       two nodes arguments and must return a number.

    weight: string, optional (default='weight')
       Edge data key corresponding to the edge weight.

    Raises
    ------
    NetworkXNoPath
        If no path exists between source and target.

    Examples
    --------
    >>> G=nx.path_graph(5)
    >>> print(nx.astar_path(G,0,4))
    [0, 1, 2, 3, 4]
    >>> G=nx.grid_graph(dim=[3,3])  # nodes are two-tuples (x,y)
    >>> def dist(a, b):
    ...    (x1, y1) = a
    ...    (x2, y2) = b
    ...    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    >>> print(nx.astar_path(G,(0,0),(2,2),dist))
    [(0, 0), (0, 1), (1, 1), (1, 2), (2, 2)]


    See Also
    --------
    shortest_path, dijkstra_path
    """
    if heuristic is None:
        # The default heuristic is h=0 - same as Dijkstra's algorithm
        def heuristic(u, v): return 0
        
    # The queue stores priority, node, cost to reach, and parent.
    # Uses Python heapq to keep in priority order.
    # Add each node's hash to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guarenteed unique for all nodes in the graph.
    queue = [ ( 0, hash(source), source, 0, None ) ]
    
    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}
    
    while queue:
        # Pop the smallest item from queue.
        _, __, curnode, dist, predec = heappop( queue )
        
        if curnode == target :
            path = [ curnode ]
            
            parent = predec
            #node = curnode
            while parent is not None :
                edge, node = parent
                path.append( edge )
                path.append( node )
                #
                parent = explored[ node ] 
                
            path.reverse()
            return path
        
        if curnode in explored:
            continue
        explored[curnode] = predec
        
        # need to go through edges in order of length, in case multi-edges
        iters = [ digraph.out_edges_iter, digraph.in_edges_iter ]
        iters = [ iter( curnode, keys=True, data=True ) for iter in iters ]
        iter = itertools.chain( *iters )
        EDGES = [ ( data.get(weight,1), i,j,key,data ) for i,j,key,data in iter ]
        EDGES = sorted( EDGES )
        for edgelen, i,j,key, data in EDGES :
            edge = i,j,key
            
            if i is not curnode :
                if data.get( 'oneway', False ) :
                    #print 'cannot go that way'
                    continue
                
                i,j = j,i   # flip direction
                
            if j in explored : continue
            
            ncost = dist + data.get( weight, 1 )
            if j in enqueued :
                qcost, h = enqueued[j]
                if qcost <= ncost : continue
            else :
                h = heuristic(j, target)
            enqueued[j] = ncost, h
            
            entry = ( ncost + h, hash(j), j, ncost, ( edge, curnode ) )
            heappush( queue, entry )
            #heappush(queue, (ncost + h, hash(neighbor), neighbor, ncost, curnode))

    raise nx.NetworkXNoPath("Node %s not reachable from %s" % (source, target) )



def path_length( digraph, path, weight='weight' ) :
    path_data = [ digraph.get_edge_data( *edge ) for edge in path[1::2] ]
    path_weight = [ data.get( weight, 1 ) for data in path_data ]
    total_length = sum( path_weight )
    return total_length





def astar_path_length( digraph, source, target, heuristic=None, weight='weight'):
    """Return the length of the shortest path between source and target using
    the A* ("A-star") algorithm.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node for path

    target : node
       Ending node for path

    heuristic : function
       A function to evaluate the estimate of the distance
       from the a node to the target.  The function takes
       two nodes arguments and must return a number.

    Raises
    ------
    NetworkXNoPath
        If no path exists between source and target.

    See Also
    --------
    astar_path

    """
    path = astar_path( digraph, source, target, heuristic, weight )
    return path_length( digraph, path, weight )





""" convenience utilities """
def add_verbose_path( graph, *seq ) :
    i_seq   = seq[0::2]
    key_seq = seq[1::2]
    j_seq   = seq[2::2]
    edges = zip( i_seq, j_seq, key_seq )
    for i,j,key in edges :
        graph.add_edge( i,j,key )




if __name__ == '__main__' :
    digraph = nx.MultiDiGraph()
    
    add_verbose_path( digraph, 0, 'S', 1, 'W', 2, 'N', 3 )
    digraph.add_edge( 3, 0, 'E', oneway=True )
    
    path = astar_path( digraph, 0, 3 )
    
    
    
    
    