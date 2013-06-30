
# python standard
import heapq


# science common
import numpy as np
import networkx as nx

# for a Red-Black tree, efficient search, self-balancing, etc..
import bintrees

# dev
import roadmap_basic as ROAD


class node :
    def __init__(self) :
        self.type = None    # can be 'node' or 'addr'
        self.vertex
        self.addr


class PointSet :
    def __init__(self) :
        """ _points will contain one RBTree for each road that it knows about """
        self._points = {}
        
    def count(self) :
        # can I implement len?
        return sum([ len( tree ) for tree in self._points ]) 
        
        
    def get_roadtree(self, road ) :
        if road in self._points :
            res = self._points[ road ]
        else :
            res = bintrees.RBTree()     # create a RedBlack tree
            res[ -np.inf ] = None       # insert symbolic left endpoint
            res[ np.inf ] = None        # insert symbolic right endpoint
            self._points[ road ] = res
            
        return res
            
    def insert(self, addr ) :
        assert isinstance( addr, ROAD.RoadAddress )
        roadtree = self.get_roadtree( addr.road )
        roadtree[ addr.coord ] = addr
        
    def remove(self, addr ) :
        assert isinstance( addr, ROAD.RoadAddress )
        assert addr.road in self._points
        roadtree = self.get_roadtree( addr.road )
        assert addr.coord in roadtree
        del roadtree[ addr.coord ]
        
        
    def find_nearest(self, addr, roadnet, length='length' ) :
        assert isinstance( addr, ROAD.RoadAddress )
        
        # convenience utilities
        def roadinfo( road ) :
            edge, data  = ROAD.obtain_edge( roadnet, road, True )
            u, v, key   = edge
            roadlen     = data.get( length, 1 )
            #
            return u, v, roadlen
        
        # PREPARE for best first search [with branch and bound]
        OPEN = []       # priority queue of points to be checked, sorted according to min distance;
                        # the min of an open node can be updated if a shorter path is found;
                        # in this case, just insert with the new priority...
                        # when it reaches the old-priority copy, the node will be in CLOSED already
        CLOSED = set()  # nodes already checked, which should not be opened again
        
        # populate initial conditions of the search
        options = [] # options contains tuples of: ( target symbol, target address )
        
        # 1. find the interval containing addr
        roadtree = self.get_roadtree( addr.road )
        _, floor_addr = roadtree.floor_item( addr.coord )
        _, ceil_addr = roadtree.ceiling_item( addr.coord )
        
        # obtain relevant road data for addr.road
        u, v, roadlen = roadinfo( addr.road, roadnet )
        # look left
        if floor_addr is None :
            lbp = ROAD.RoadAddress( road, 0. )
            options.append( ( u, lbp ) )
        elif isinstance( floor_addr, ROAD.RoadAddress ) :
            options.append( ( floor_addr, floor_addr ) )
        else :
            raise 'invalid node type'
        # look right
        if ceil_addr is None :
            rbp = ROAD.RoadAddress( road, roadlen )
            options.append( ( v, rbp ) )
        elif isinstance( floor_addr, ROAD.RoadAddress ) :
            options.append( ( ceil_addr, ceil_addr ) )
        else :
            raise 'invalid node type'
        # add all options to OPEN 
        for ( targ, targ_addr ) in options :
            traveltime = ROAD.distance( roadnet, addr, targ_addr )
            heapq.heappush( ( traveltime, targ ) )
        
        # do best first search, with branch and bound
        res = None
        while len( OPEN ) > 0 :
            traveltime, curr_targ = heapq.heappop( OPEN )
            
            # if it's a point, then we're done!
            if isinstance( curr_targ, ROAD.RoadAddress ) :
                res = curr_addr
                break
            
            # if it's closed already, ignore
            if curr_targ in CLOSED : continue
                
            # otherwise, we need to obtain all neighbors
            # do some shit...
        
        return curr_addr
        
        
