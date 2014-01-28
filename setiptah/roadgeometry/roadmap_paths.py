

import itertools
import random
import numpy as np

import networkx as nx       # it's time for better control
#import bintrees

import astar_basic
from roadmap_basic import RoadAddress
from roadmap_basic import check_point, obtain_edge


__author__ = "Kyle Treleaven <ktreleav@mit.edu>"
#__all__ = [ 'RoadAddress', 'distance' ]

"""
Notes on the Roadmap format:
These utilities expect a NetworkX MultiDiGraph,
the edge keys are interpreted as "road names",
and they should all be unique, even among edges between different nodes;
(also, they should be distinct from the node labels).
All roads are two-way by default (the direction of edges is for the coordinate system only);
However, one-way roads are handled, and should be indicated with a key:value pair 'oneway':True .
"""


"""
Class defs --- for paths
"""

class RoadSegment(object) :
    def __init__(self, road, first, second ) :
        self.road = road
        self.first = first
        self.second = second
        
    def __repr__(self) :
        return '(%s,[%s,%s])' % ( repr(self.road), repr(self.first), repr(self.second) )
    
    
def check_segment( segment, roadmap, length_attr='length' ) :
    # a rare case where "static variable" is useful;
    # to prevent "costly" re-allocation of RoadAddress structure
    if not hasattr(check_segment, 'first' ) :
        check_segment.first = RoadAddress(None,None)
    if not hasattr(check_segment, 'second' ) :
        check_segment.second = RoadAddress(None,None)
        
    road = segment.road
    check_segment.first.road = road
    check_segment.first.coord = segment.first
    check_segment.second.road = road
    check_segment.second.coord = segment.second
    
    try :
        assert check_point( roadmap, check_segment.first, length_attr )
        assert check_point( roadmap, check_segment.second, length_attr )
    except AssertionError :
        return False
    
    return True


""" Simple Path Operations """

def pathReversed( path ) :
    res = []
    for segment in path :
        res.insert(0, RoadSegment( segment.road, segment.second, segment.first ) )
    return res

def pathLength( path ) :
    if len( path ) <= 0 :
        return np.inf
    else :
        segLengths = [ np.abs( seg.first - seg.second ) for seg in path ]
        return np.sum( segLengths )

def pathEvaluate( path, x ) :
    if x < 0. : raise Exception('cannot interpret negative distance')
    segLengths = [ np.abs( seg.first - seg.second ) for seg in path ]
    A = 0.
    for i, y in enumerate( segLengths ) :
        B = A + y
        if x <= B : break
        A = B
    if x > B : raise Exception('query distance longer than path')      # only happens when B == path length
    
    segment = path[i]
    dx = x - A
    if segment.first < segment.second :
        return RoadAddress( segment.road, segment.first + dx )
    else :
        return RoadAddress( segment.road, segment.first - dx )
    
def pathFromAStar( astar_path, roadmap, length_attr ) :
    path = []
    
    nodewalk = astar_path[::2]
    edgewalk = astar_path[1::2]
    for i,j, edgedata in zip( nodewalk[:-1], nodewalk[1:], edgewalk ) :
        _,__,road = edgedata
        edge, data  = obtain_edge( roadmap, road, True )
        u,v,_       = edge
        roadlen     = data.get( length_attr, 1 )
        
        if i == u and j == v :  # forward jump, yay!
            segment = RoadSegment( road, 0., roadlen )
        elif i == v and j == u : # backward jump, yay!
            segment = RoadSegment( road, roadlen, 0. )
        else :
            raise Exception('invalid hop on roadmap')
        
        path.append( segment )
    
    return path


""" Shortest Path Functions """

def minpath( p, q, roadmap, length_attr='length' ) :
    """ returns one of the min length paths between two points on a Roadmap """
    # some convenient static variables
    if not hasattr( minpath, 'STATICS' ) :
        minpath.STATICS = True
        minpath.first = RoadAddress(None,None)
        minpath.second = RoadAddress(None,None)
        minpath.points = [ minpath.first, minpath.second ]
    
    # convenient non-static variables
    road        = p.road
    edge, data  = obtain_edge( roadmap, road, True )
    i,j,key     = edge
    roadlen     = data.get( length_attr, 1 )
    
    # we will be choosing best path among options
    options = []
    
    # one possible type of path is a segment on a single road
    path = minpath_on_road( p, q, road, roadmap, length_attr )
    options.append( ( pathLength(path), path ) )
    
    # two more possible types of paths are p->u->q, for u either of endpoints of road of p
    minpath.first.init(road,0.)
    minpath.second.init(road,roadlen)
    for u, qq in zip( (i,j), minpath.points ) :
        try :
            first = minpath_on_road( p, qq, road, roadmap, length_attr )
            second = minpath_node_to_point( u, q, roadmap, length_attr )
        except None :
            print p, q
            raise Exception()
        path = first + second
        options.append( ( pathLength(path), path ) )
        
    # return path having the shortest length --- might be []
    return min( options )[1]
        

def minpath_on_road( p, q, road, roadmap, length_attr='length' ) :
    """
    subroutine and useful utility:
    returns the path from p to q on road, if the direction of travel is admissible;
    if not, or if p and q are not co-'road'-al, returns empty path, implying non-existence 
    """ 
    edge, data  = obtain_edge( roadmap, road, True )
    i,j,key     = edge
    
    path = []
    # regular logic, NOT error catching
    try :
        assert p.road == road and check_point( roadmap, p, length_attr )
        assert q.road == road and check_point( roadmap, q, length_attr )
        assert p.coord < q.coord or not data.get( 'oneway', False )
        path.append( RoadSegment( road, p.coord, q.coord ) )
        
    except AssertionError :
        pass
    
    return path


def minpath_node_to_point( u, q, roadmap, length_attr='length' ) :
    """
    subroutine and useful utility:
    returns the shortest-path distance from a node u to a point q, on digraph    
    """
    
    # some convenient static variables
    scope = minpath_node_to_point
    if not hasattr( scope, 'STATICS' ) :
        scope.STATICS = True
        scope.first = RoadAddress(None,None)
        scope.second = RoadAddress(None,None)
        scope.points = [ scope.first, scope.second ]
    
    # convenient non-static variables
    assert check_point( roadmap, q, length_attr )
    road        = q.road
    edge, data  = obtain_edge( roadmap, road, True )
    i,j,key     = edge
    roadlen = data.get( length_attr, 1 )
    
    # pre-load options with nopath; in case there are no feasible paths
    options = [ (np.inf, [] ) ]
    
    # possible paths are u->v->q, for v in endpoints of road of q
    scope.first.init(road,0.)
    scope.second.init(road,roadlen)
    for v, pp in zip( (i,j), scope.points ) :
        try :
            astarPath = astar_basic.astar_path( roadmap, u, v, None, weight=length_attr )
            first = pathFromAStar( astarPath, roadmap, length_attr )
        except nx.NetworkXNoPath :
            continue
        second = minpath_on_road( pp, q, road, roadmap, length_attr )
        path = first + second
        options.append( (pathLength(path), path ) )
        
    return min( options )[1]



""" Compound Path Operations """

def pathExtend( path, nextAddress, roadmap, length_attr='length' ) :
    segment = path[-1]
    road = segment.road
    q = RoadAddress(road,segment.second)
    second = minpath( q, nextAddress, roadmap, length_attr )
    # TODO: combine contiguous, co-directional segments;
    # TODO: also, zero-measure segments
    return path + second


""" Path Planning """

class RoadTrajectory :
    def __init__(self, path ) :
        self.path = path
        
    def __call__(self, progress ) :
        return pathEvaluate( self.path, progress )
    
    
class RoadmapPlanner :
    def __init__(self, roadmap ) :
        self.roadmap = roadmap
        
    def __call__(self, orig, dest, length_attr='length' ) :
        path = minpath( orig, dest, self.roadmap, length_attr )
        return pathLength(path), RoadTrajectory(path)




if __name__ == '__main__' :
    from setiptah.roadgeometry.roadmap_basic import distance
    
    import setiptah.roadgeometry.probability as roadprob
    
    if True :
        roadmap = roadprob.sampleroadnet()
        p = roadprob.sampleaddress( roadmap )
        q = roadprob.sampleaddress( roadmap )
    
    # give me a summary of the road network
    for u,v,road, data in roadmap.edges_iter( keys=True, data=True ) :
        print '%s: %s -> %s ; length=%f' % ( road, repr(u), repr(v), data.get('length',1) )
        
    print '...going from %s to %s' % ( repr(p), repr(q) )
    frwd = minpath( p, q, roadmap )
    frwdL = pathLength( frwd )
    frwdLRef = distance( roadmap, p, q, 'length' )
    print frwd
    print 'FRWD LENGTH: %f; survey says: %f' % ( frwdL, frwdLRef )
    
    
    bkwd = minpath( q, p, roadmap )
    bkwdL = pathLength( bkwd )
    bkwdLRef = distance( roadmap, q, p, 'length' )
    
    print 'BKWD LENGTH: %f; survey says: %f' % ( bkwdL, bkwdLRef )
    
    if np.abs( frwdL - frwdLRef ) > 10**-10 or np.abs( bkwdL - bkwdLRef ) > 10**-10 :
        print 'ALERT!!! NOT AGREE WITH REF'
    else :
        print 'cool as a cucumber'
        
        
        
        
