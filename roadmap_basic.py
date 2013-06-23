

import itertools
import random
import numpy as np

import networkx as nx       # it's time for better control

import astar_basic

__author__ = "Kyle Treleaven <ktreleav@mit.edu>"
__all__ = [ 'RoadAddress', 'distance' ]

"""
Notes on the Roadmap format:
These utilities expect a NetworkX MultiDiGraph,
the edge keys are interpreted as "road names",
and they should all be unique, even among edges between different nodes;
(also, they should be distinct from the node labels).
All roads are two-way by default (the direction of edges is for the coordinate system only);
However, one-way roads are handled, and should be indicated with a key:value pair 'oneway':True .
"""



""" convenience utils """

def roadify( digraph, node, weight='weight' ) :
    """
    used to obtain a road address for a particular node...
    by hook, or by crook
    """
    iters = [ digraph.out_edges_iter, digraph.in_edges_iter ]
    iters = [ iter( node, keys=True, data=True ) for iter in iters ]
    iter = itertools.chain( *iters )
    
    try :
        i,j,road, data = iter.next()
    except :
        return None
    
    if i is node :
        return RoadAddress(road,0.)
    elif j is node :
        return RoadAddress(road, data.get(weight,1) )
    else :
        raise Exception('networkx should not let this happen')


def obtain_edge( digraph, road, data_flag=False ) :
    """
    Input: digraph, a multi-digraph; road, a key
    Output: if road is the key of some edge in digraph,
        then returns edge, [data if data_flag]
    """
    
    def result( edge ) :
        if data_flag :
            data = digraph.get_edge_data( *edge )
            return edge, data
        else :
            return edge
    
    store = digraph.graph
    edge = store.get( road, None )
    if edge is not None and digraph.has_edge( *edge ) : return result( edge )
    
    # else, find and cache
    for i,j,key in digraph.edges_iter( keys=True ) :
        if key == road :
            edge = i,j,key
            store[road] = edge
            return result( edge )


def check_point( digraph, point, weight='weight' ) :
    """ answers whether a RoadAddress is contained by the Roadmap digraph """
    res = obtain_edge( digraph, point.road, data_flag=True )
    if res == None : return False
    edge, data = res
    return point.coord >= 0. and point.coord <= data.get( weight, 1 )


def connectivity_graph( roadnet, length_in='length', length_out='length' ) :
    """
    gives a digraph with a single shortest-path length edge
    between nodes that are locally connected by roadnet
    """
    digraph = nx.DiGraph()
    for uu, vv, road, road_data in roadnet.edges( data=True, keys=True ) :
        road_len    = road_data.get( length_in, 1 )
        oneway      = road_data.get( 'oneway', False )
        options     = [ (uu,vv) ]
        if not oneway : options.append( (vv,uu) )
        
        for u, v in options :
            if digraph.has_edge(u, v) :
                edge_data = digraph.get_edge_data( u, v )
                if road_len < edge_data[ length_out ] :
                    edge_data['road'] = road
                    edge_data[length_out] = road_len
            else :
                edge_data = { 'road' : road, length_out : road_len }
                digraph.add_edge( u, v, edge_data )
                
    return digraph





"""
Class defs
"""

class RoadAddress(object) :
    def __init__(self, road, coord ) :
        self.road = road
        self.coord = coord
        
    def __repr__(self) :
        return '(%s,%s)' % ( repr( self.road ), repr( self.coord ) )
    
    



""" Distance functions """

def distance( digraph, p, q, weight='weight' ) :
    """ returns the shortest-path distance between two points on a Roadmap """
    road        = p.road
    edge, data  = obtain_edge( digraph, road, True )
    i,j,key     = edge
    roadlen     = data.get( weight, 1 )
    
    nodes = [ i, j ]
    points = [ RoadAddress(road,0.), RoadAddress(road,roadlen) ]

    dist = distance_on_road( digraph, road, p, q, weight=weight )
    options = [ dist ]
    
    for u, qq in zip( nodes, points ) :
        try :
            dist = distance_on_road( digraph, road, p, qq, weight=weight )
            dist += distance_node_to_point( digraph, u, q, weight=weight )
        except :
            print p, q
            raise Exception()
        options.append( dist )
        
    return min( options )
        

def distance_on_road( digraph, road, p, q, weight='weight' ) :
    """
    subroutine and useful utility:
    returns the distance from p to q on road [actually, its interior!],
    if the direction of travel is admissible;
    if not, or if p and q are not co-'road'-al, returns infinity 
    """ 
    edge, data  = obtain_edge( digraph, road, True )
    i,j,key     = edge
    
    if not p.road == road or not check_point( digraph, p, weight ) : return np.inf
    if not q.road == road or not check_point( digraph, q, weight ) : return np.inf
    if data.get( 'oneway', False ) and q.coord < p.coord : return np.inf
    return np.abs( q.coord - p.coord )

def distance_node_to_point( digraph, u, q, weight='weight' ) :
    """
    subroutine and useful utility:
    returns the shortest-path distance from a node u to a point q, on digraph    
    """
    assert check_point( digraph, q, weight )
    road        = q.road
    edge, data  = obtain_edge( digraph, road, True )
    i,j,key     = edge
    roadlen = data.get( weight, 1 )
    
    nodes = [ i, j ]
    points = [ RoadAddress(road,0.), RoadAddress(road,roadlen) ]
    
    options = [ np.inf ]
    for v, p in zip( nodes, points ) :
        try :
            dist = astar_basic.astar_path_length( digraph, u, v, None, weight=weight )
        except :
            continue
        
        dist += distance_on_road( digraph, road, p, q, weight=weight )
        options.append( dist )
    
    return min( options )




if __name__ == '__main__' :
    import random
        
    def randomaddress( roadnet, length='length' ) :
        _,__,road,data = random.choice( roadnet.edges( keys=True, data=True ) )
        roadlen = data.get(length,1)
        y = roadlen * np.random.rand()
        return RoadAddress(road,y)
    
    
