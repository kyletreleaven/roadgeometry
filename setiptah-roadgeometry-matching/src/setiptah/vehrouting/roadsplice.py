
""" migrate this to vehicle routing at some point """

import itertools

import numpy as np

import networkx as nx
import bintrees


#import setiptah.roadgeometry.astar_basic as ASTAR
import setiptah.roadgeometry.roadmap_basic as ROAD
import setiptah.roadgeometry.roadmap_paths as roadpaths

import setiptah.roadbm as roadbm
import setiptah.vehrouting.routeinspection as rinspect




""" utility """
class token : pass

def get_road_data( road, roadnet ) :
    for _,__,key, data in roadnet.edges_iter( keys=True, data=True ) :
        if key == road : return data

class traversal :
    def __init__(self, road, forward ) :
        """ forward is a boolean flag, i.e., forward_not_backward """
        self.road = road
        self.forward = forward
        
    def __repr__(self) :
        dir = 'forward'
        if not self.forward : dir = 'reverse'
        return '(%s,%s)' % ( self.road, dir )

def quantity( arr ) :
    num = 0
    for k, tree in arr.iteritems() :
        Z = [ len(q) for y,q in tree.iter_items() ]
        num += sum( Z )
    return num




""" ALGORITHM BEGINS """

class RoadMapFHK :
    def __init__(self, roadmap ) :
        """
        assumes roadmap has all bidirectional roads
        otherwise, another subtour connecting heuristic should be used
        """
        self.roadmap = roadmap
        
        altwalk = rinspect.ROUTEINSPECTION( self.roadmap )
        self.cover = ALTWALK_TO_COVER( altwalk, self.roadmap )
        
        
    def __call__(self, arcs, getTail, getHead ) :
        """ length attribute *must* be 'length' """
        return RoadMapSPLICE( arcs, getTail, getHead, self.roadmap, self.cover )
    
    def distance(self, p, q ) :
        return ROAD.distance( self.roadmap, p, q, weight='length' )
    
    def kSPLICE(self, arcs, agentLocs, getTail, getHead ) :
        tour = self( arcs, getTail, getHead )       # using __call__
        
        assign = SPLITANDASSIGN( tour, agentLocs, arcs, getTail, getHead, self.distance )
        
        return assign

#def SPLITTOUR( tour, N, arcs, getTail, getHead, distance, start=None )
#def ASSIGNFRAGS( seqs, agents, arcs, getTail, getHead, distance )
#def SPLITANDASSIGN( tour, agents, arcs, getTail, getHead, distance, start=None )




def ALTWALK_TO_COVER( altwalk, roadmap ) :
    """ altwalk is of form (node,edge,node,edge,...,node) """
    cover = []
    
    links = zip( altwalk[:-2:2], altwalk[1:-1:2], altwalk[2::2] )
    for u, road, v in links :
        # is forward?
        frwd = roadmap.get_edge_data(u,v)
        if frwd is not None and road in frwd :
            cover.append( traversal(road, forward=True ) )
            continue
        
        bkwd = roadmap.get_edge_data(v,u)
        if bkwd is not None and road in bkwd :
            cover.append( traversal(road, forward=False ) )
            continue
        
        raise Exception('do not see that road between those endpoints')
    
    return cover




def RoadMapSPLICE( arcs, getTail, getHead, roadmap, cover ) :
    """
    tour should be build from traversals, and should cover the network
    """
    QQ = [ getTail(a) for a in arcs ]
    PP = [ getHead(a) for a in arcs ]
    
    match = roadbm.ROADSBIPARTITEMATCH( PP, QQ, roadmap ) 
    order = ORDERPOINTS( QQ, cover )
    
    res = []
    M = dict( match )
    while len( order ) > 0 :
        i = order[0]
        #print 'start at %d', i
        while i in M :
            res.append( i )
            #print i
            
            j = M[i]
            del M[i]
            order.remove( i )
            i = j
            
    if True :
        # return a cyclic chain
        res = { i : j for i, j in zip( res, res[1:] + res[:1] ) }
        
    return res




""" for sorting points according to a covering tour """

def ARRANGEMENT( points ) :
    def ensure_road( road, data ) :
        curr = data.setdefault( road )
        if curr is None : data[road] = bintrees.RBTree()
        return data[road]
    
    def ensure_key( key, tree ) :
        curr = tree.set_default( key )
        if curr is None : tree[key] = []
        return tree[key]
    
    TREES = dict()
    for idx, (road,y) in enumerate( points ) :
        tree = ensure_road( road, TREES )
        Q = ensure_key( y, tree )
        Q.append( idx )
        
    return TREES


def COLLECT( arrangement, tour ) :
    order = []
    
    arr = arrangement.copy()
    #print quantity( arr ) 
    
    for t in tour :
        if t.road not in arr : continue
        
        tree = arr[ t.road ]
        if t.forward :
            items = tree.iter_items()
        else :
            items = tree.iter_items( reverse=True )
        items = [ i for i in items ]        # because it *wasn't* an iterator
            
        for key, Q in items :
            order.extend( Q )
            tree.remove(key)
            
    return order


def ORDERPOINTS( points, tour ) :
    arr = ARRANGEMENT( points )
    order = COLLECT( arr, tour )
    return order








""" MATCH TESTING Utilities """

def CYCLEFACTOR( match ) :
    res = []
    
    M = dict(match)
    while len( M ) > 0 :
        curr = []
        i = M.iterkeys().next()
        while i in M :
            curr.append( i )
            j = M[i]
            del M[i]
            i = j
            
        res.append( curr )
    
    return res



if False :
    def CYCLECOST( cycle, demands, roadnet, length_attr='length' ) :
        addr = lambda p : ROAD.RoadAddress( *p )
        
        carry = [ ROAD.distance( roadnet, addr(dem.pick), addr(dem.delv), length_attr ) for dem in demands ]
        
        edges = zip( cycle, cycle[1:] + cycle[:1] )
        #print edges
        
        edges = [ ( demands[i], demands[j] ) for i,j in edges ] 
        empty = [ ROAD.distance( roadnet, addr(dem1.delv), addr(dem2.pick), length_attr ) for dem1,dem2 in edges ]
        return sum( carry ) + sum( empty )
    
    
    def MATCHCOST( match, demands, roadnet ) :
        cycles = CYCLEFACTOR( match )
        costs = [ CYCLECOST( cycle, demands, roadnet ) for cycle in cycles ]
        return sum( costs )
    





""" use route inspection instead... """

def DOUBLETOUR( roadnet ) :
    eulerian = nx.MultiDiGraph()
    for u,v, road in roadnet.edges_iter( keys=True ) :
        eulerian.add_edge( u,v, label=traversal( road, True ) )
        eulerian.add_edge( v,u, label=traversal( road, False ) )
        
    tour = []
    walk = [ u for u in nx.eulerian_circuit( eulerian ) ]
    for u, v in walk :
        edges = eulerian.edge[u][v]
        key = edges.keys()[0]       # get key of the some (e.g., first) edge from u, v
        data = eulerian.get_edge_data( u, v, key )
        tour.append( data.get('label') )
        eulerian.remove_edge( u, v, key )
        
    return tour








""" convenient sampling utility for the unit test below, might as well be a package-export, though """

# TODO: All below superceded by vehrouting.stackercrane2 ?

#from setiptah.vehrouting.stackercrane2 import SPLITTOUR, ASSIGNFRAGS, SPLITANDASSIGN
from setiptah.vehrouting.stackercrane2 import SPLITANDASSIGN



if False :
    def SPLITTOUR( tour, N, demands, roadmap ) :
        """
        split the tour (seq. of indices) of demands (list of demands) on roadmap
        into k pieces
        """
        tours = []
        tourlen = CYCLECOST( tour, demands, roadmap )
        
        fragtarget = float(tourlen) / N
        EDGES = zip( tour[-1:] + tour[:-1], tour )      # cycle; first "prev" is last
        x, y = [ ROAD.RoadAddress(None,None) for i in range(2) ]    # storage
        
        for k in range(N) :
            frag = []
            fraglen = 0.
            while fraglen < fragtarget :
                if not len( EDGES ) > 0 : break
                i, j = EDGES.pop(0)
                prev, curr = demands[i], demands[j]
                p, q, r = prev.delv, curr.pick, curr.delv
                # fetch
                x.init(*p) ; y.init(*q)
                exten = roadpaths.minpath( x, y, roadmap )
                y.init(*r)
                exten = roadpaths.pathExtend( exten, y, roadmap )
                extenlen = roadpaths.pathLength( exten )
                
                frag.append(j)
                fraglen += roadpaths.pathLength( exten )
                #print exten, extenlen, fraglen, fragtarget
                
            tours.append( frag )
        return tours
    
    
    def ASSIGNFRAGS( tours, agentLocs, demands, roadmap ) :
        """
        assign the fragments to agents in accordance with a minimum matching
        from each agent (its position) to some origin on the tour
        """
        # compute distance to tour (some pickup) from each agent
        graph = nx.Graph()
        x = ROAD.RoadAddress(None,None)
        
        for agent, agentLoc in agentLocs.iteritems() :
            t = token()
            t.agent = agent
            def cost( demidx ) :
                x.init( *demands[demidx].pick )
                return ROAD.distance( roadmap, agentLoc, x, 'length' )
            
            for touridx, tour in enumerate( tours ) :
                if len( tour ) > 0 :
                    options = [ ( cost(demidx), k ) for k, demidx in enumerate(tour) ]
                else :
                    options = [ ( 0., 0 ) ]     # empty tour
                    
                copt, kopt = min( options )
                
                # negative weights for MIN weight
                graph.add_edge( t, touridx, weight = -copt, start_at = kopt )
                
        # get optimal matching
        MATCH = nx.matching.max_weight_matching( graph, maxcardinality=True )
        
        assign = {}
        for touridx, tour in enumerate( tours ) :
            u = MATCH[touridx]
            k = graph.get_edge_data( u, touridx ).get( 'start_at' )
            agent = u.agent
            order = tour[k:] + tour[:k]
            assign[agent] = order
            
        return assign
    
    
    def SPLITANDASSIGN( tour, agentLocs, demands, roadmap ) :
        frags = SPLITTOUR( tour, len( agentLocs ), demands, roadmap )
        assign = ASSIGNFRAGS( frags, agentLocs, demands, roadmap )
        return assign













if __name__ == '__main__' :
    import matplotlib.pyplot as plt
    plt.close('all')
    
    roadnet = nx.MultiDiGraph()
    roadnet.add_edge( 0,1, 'N', length=1. )
    roadnet.add_edge( 1,2, 'E', length=1. )
    roadnet.add_edge( 2,3, 'S', length=1. )
    roadnet.add_edge( 3,0, 'W', length=1. )
    
    #tour = DOUBLETOUR( roadnet )
    #print tour
    
    from setiptah.roadgeometry.probability import UniformDist
    sampler = UniformDist( roadnet )
    
    NUMPOINT = 1000
    ZZ = np.linspace(-NUMPOINT,NUMPOINT,1000)
    #
    PP = [ sampler.sample() for i in xrange(NUMPOINT) ]
    QQ = [ sampler.sample() for i in xrange(NUMPOINT) ]
    
    class demand :
        def __init__(self, o, d ) :
            self.origin = o
            self.destination = d
            
        @classmethod
        def getTail(cls, dem ) :
            return dem.origin
        
        @classmethod
        def getHead(cls, dem ) :
            return dem.destination
    
    DEMANDS = [ demand(p,q) for p,q in zip( PP, QQ ) ]
    
    fhk = RoadMapFHK( roadnet )
    print fhk.cover
    
    tour = fhk( DEMANDS, demand.getTail, demand.getHead )
    #tour = ROADSSPLICE( DEMANDS, roadnet )
    
    
    from setiptah.vehrouting.stackercrane2 import WALKCOST
    tourlen = WALKCOST( tour, DEMANDS, demand.getTail, demand.getHead, fhk.distance )
    
    #cost = CYCLECOST( tour, DEMANDS, roadnet )
    
    N = 3
    agentLocs = { 'agent%d' % i : ROAD.RoadAddress( *sampler.sample() ) for i in range(N) }
    
    assign_star = fhk.kSPLICE( DEMANDS, agentLocs, demand.getTail, demand.getHead )
    
    assign = SPLITANDASSIGN( tour, agentLocs, DEMANDS, demand.getTail, demand.getHead, fhk.distance )
    print assign == assign_star
    
    costs = { i : WALKCOST( walk, DEMANDS, demand.getTail, demand.getHead, fhk.distance )
             for i, walk in assign_star.iteritems() }
    
    
    
    
    #order = ORDERPOINTS( PP, tour )
    #point_tour = [ PP[i] for i in order ]
    #print point_tour
    
    
    
    
    #match = ROADSBIPARTITEMATCH( PP, QQ, roadnet )
    #costs = MATCHCOSTS( match, PP, QQ, roadnet )
    #cost = ROADMATCHCOST( match, PP, QQ, roadnet )
    
    
            