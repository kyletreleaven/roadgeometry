
# python standard
import heapq


# science common
import numpy as np
import networkx as nx

# for a Red-Black tree, efficient search, self-balancing, etc..
import bintrees

# dev
import roadmap_basic as ROAD


def my_isaddress( addr ) :
    # an annoying quick fix for ROAD being two different modules in two different places... arg!
    try :
        r,x = addr.road, addr.coord
        return True
    except :
        return False
        
    
    
# convenience utilities
def roadinfo( road, roadnet, length='length' ) :
    edge, data  = ROAD.obtain_edge( roadnet, road, True )
    u, v, key   = edge
    roadlen     = data.get( length, 1 )
    #
    return u, v, roadlen




class PointSet :
    def __init__(self) :
        """ _points will contain one RBTree for each road that it knows about """
        self._points = {}
        
    def __len__(self) :
        # todo: I think this over-counts by two per road
        return sum([ len( tree ) - 2 for tree in self._points.values() ])
        
        
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
        assert my_isaddress( addr )
        roadtree = self.get_roadtree( addr.road )
        roadtree[ addr.coord ] = addr
        
    def remove(self, addr ) :
        assert my_isaddress( addr )
        assert addr.road in self._points
        roadtree = self.get_roadtree( addr.road )
        assert addr.coord in roadtree
        del roadtree[ addr.coord ]
        
        
    def find_nearest(self, addr, roadnet, length='length' ) :
        assert my_isaddress( addr )
        
        # PREPARE for best first search [with branch and bound]
        OPEN = []       # priority queue of points to be checked, sorted according to min distance;
                        # the min of an open node can be updated if a shorter path is found;
                        # in this case, just insert with the new priority...
                        # when it reaches the old-priority copy, the node will be in CLOSED already
        CLOSED = set()  # nodes already checked, which should not be opened again
        
        # populate initial conditions for the search
        INIT = self._expand_address( addr, roadnet, length )
        for targ, time in INIT :
            heapq.heappush( OPEN, ( time, targ ) )
        
        # do best first search, with "branch and bound"
        res = None
        while len( OPEN ) > 0 :
            traveltime, curr_targ = heapq.heappop( OPEN )
            
            # if it's a point, then we're done!
            if my_isaddress( curr_targ ) :
                res = curr_targ
                break
            
            # if it's closed already, ignore;
            if curr_targ in CLOSED : continue
            # if not, then close it...
            CLOSED.add( curr_targ )
            
            # then, we need to expand
            CANDS = self._expand_node( curr_targ, roadnet, length )
            for targ, extra_time in CANDS :
                total_time = traveltime + extra_time
                heapq.heappush( OPEN, ( total_time, targ ) )
        
        return res
        
        
    def _expand_address(self, addr, roadnet, length='length' ) :
        # only ever done at the beginning!!
        options = []
        
        # find the interval containing addr
        roadtree = self.get_roadtree( addr.road )
        _, floor_addr = roadtree.floor_item( addr.coord )
        _, ceil_addr = roadtree.ceiling_item( addr.coord )
        
        # obtain relevant road data for addr.road
        u, v, roadlen = roadinfo( addr.road, roadnet )
        # look left
        if floor_addr is None :
            left_boundary = ROAD.RoadAddress( addr.road, 0. )
            options.append( ( u, left_boundary ) )
        elif my_isaddress( floor_addr ) :
            options.append( ( floor_addr, floor_addr ) )
        else :
            raise 'invalid node type'
        # look right
        if ceil_addr is None :
            right_boundary = ROAD.RoadAddress( addr.road, roadlen )
            options.append( ( v, right_boundary ) )
        elif my_isaddress( ceil_addr ) :
            options.append( ( ceil_addr, ceil_addr ) )
        else :
            raise 'invalid node type'
        
        res = []
        for ( targ, targ_addr ) in options :
            traveltime = ROAD.distance( roadnet, addr, targ_addr, length )
            res.append( ( targ, traveltime ) )
        return res
        
        
    def _expand_node(self, node, roadnet, length='length' ) :
        res = []
        
        # expand out-going edges
        for _, __, road, road_data in roadnet.out_edges_iter( node, keys=True, data=True ) :
            addr = ROAD.RoadAddress( road, 0. )
            subres = self._expand_address( addr, roadnet, length )
            res.extend( subres )
            
        # expand in-coming edges
        for _, __, road, road_data in roadnet.in_edges_iter( node, keys=True, data=True ) :
            roadlen = road_data.get( length, 1 )
            addr = ROAD.RoadAddress( road, roadlen )
            subres = self._expand_address( addr, roadnet, length )
            res.extend( subres )
            
        return res




if __name__ == '__main__' :
    import roadgeometry.probability as roadprob
    
    roadnet = roadprob.sampleroadnet()
    
    n = 100
    points = [ roadprob.sampleaddress( roadnet ) for i in range(n) ]
    
    pset = PointSet()
    
    for p in points :
        pset.insert( p )
        
        
    def find_nearest( addr ) :
        dist_to = lambda q : ROAD.distance( roadnet, addr, q, 'length' )
        trips = [ ( dist_to(q), q ) for q in points ]
        return min( trips )[1]
        
    def sidebyside( addr ) :
        by_pset = pset.find_nearest( addr, roadnet )
        by_naive = find_nearest( addr )
        return by_pset, by_naive
    
    samples = 100
    testpoints = [ roadprob.sampleaddress( roadnet ) for i in range(samples) ]
    answers = [ sidebyside( q ) for q in testpoints ]
    error = [ ROAD.distance( roadnet, p, q, 'length' ) for p, q in answers ]








