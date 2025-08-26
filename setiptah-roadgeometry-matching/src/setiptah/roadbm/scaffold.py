"""
I have reason to believe that the scaffold problem on road networks is much harder
than the scaffold problem on a line; like formally hard.
Therefore, I am halting development on this branch.
"""

import itertools

import numpy as np

import bintrees




class TwoQueues() :
    def __init__(self) :
        self.P = []
        self.Q = []
        
    def __repr__(self) :
        return '<P:%s,Q:%s>' % ( repr(self.P), repr(self.Q) )

def ensure_key( key, tree ) :
    curr = tree.set_default( key )
    if curr is None : tree[key] = TwoQueues()
    return tree[key]

class terminal :    # simple node type for TRAVERSE
    def __init__(self, q ) :
        self.q = q
        
    def __repr__(self) :
        return repr( self.q )


def SEGMENT( S, T ) :
    """ sorted points into queues hanging from an indexed segment """
    segment = bintrees.RBTree()
    
    for i, s in enumerate( S ) :
        q = ensure_key( s, segment )
        q.P.append( i )
        
    for j, t in enumerate( T ) :
        q = ensure_key( t, segment )
        q.Q.append( j )
        
    return segment


def MEASURE( segment, length ) :
    measure = dict()
    
    posts = [ 0. ] + [ y for y,q in segment.iter_items() ] + [ length ]
    intervals = zip( posts[:-1], posts[1:] )
    
    deltas = [0] + [ len(q.P)-len(q.Q) for y,q in segment.iter_items() ]
    F = np.cumsum( deltas )
    
    for (a,b), f in zip( intervals, F ) :
        measure.setdefault( f, 0. )
        measure[f] += b - a
        
    return measure


def EDGES( segment ) :      # very similar routine, used to build the walk graph
    edges = dict()
    
    posts = [ '-' ] + [ q for y,q in segment.iter_items() ] + [ '+' ]
    posts = [ terminal(q) for q in posts ]
    intervals = zip( posts[:-1], posts[1:] )
    
    deltas = [0] + [ len(q.P)-len(q.Q) for y,q in segment.iter_items() ]
    F = np.cumsum( deltas )
    
    for I, f in zip( intervals, F ) :
        edges.setdefault( f, [] )
        edges[f].append( I )
        
    return edges


def LOOSENCOST( segment, length ) :
    # make empty endpoints
    left = TwoQueues()
    right = TwoQueues()
    
    # logic borrowed from above
    posts = [ (0.,left) ] + [ i for i in segment.iter_items() ] + [ (length,right) ]
    intervals = zip( posts[:-1], posts[1:] )
    deltas = [0] + [ len(q.P)-len(q.Q) for y,q in segment.iter_items() ]
    F = np.cumsum( deltas )
    
    SEEN = dict()       # nest: level.id.cost_so_far
    for (L,R), f in zip( intervals, F ) :
        Ly,Lq = L
        Ry,Rq = R
        width = Ry - Ly
        
        level = SEEN.setdefault( f, dict() )
        for i in Lq.P : level[i] = 0.
            
        # enumerate past points
        for ff, level in SEEN.iteritems() :
            for i in level :
                if ff <= f :
                    level[i] -= width
                else :
                    level[i] += width
                    
    # SEEN is a nested dictionary of:
    #    levels
    #        points
    #            the "cost" (generally negative) of choosing to remove point from level 
    return SEEN





if __name__ == '__main__' :
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    plt.close('all')
    
    LENGTH = 10.
    NS = 100
    NT = 50
    
    S = [ LENGTH * np.random.rand() for k in range(NS) ]
    T = [ LENGTH * np.random.rand() for k in range(NT) ]
    S = sorted( S )
    T = sorted( T )
    
    # just for sanity check
    X = []
    X.extend([ (s,1) for s in S ])
    X.extend([ (t,-1) for t in T ])
    X = sorted( X )
    
        
    """ get nearest neighbors of S """
    TN = [ (t,j) for j,t in enumerate(T) ]
    NN = dict()
    NNDIST = dict()
    for i, s in enumerate( S ) :
        nndist = lambda (t,j) : np.abs( t - s )
        tn, jn = min( TN, key=nndist )
        NN[i] = jn
        NNDIST[i] = np.abs( tn - s )
        
        
    """ begin analysis """
    segment = SEGMENT( S, T )
    
    measure = MEASURE( segment, LENGTH )
    edges = EDGES( segment )
    
    loose = LOOSENCOST( segment, LENGTH )       # this looks decent from inspection
    BEST = dict()
    BESTCOST = dict()
    for f, level in loose.iteritems() :
        if len( level ) <= 0 : continue
        options = [ ( c+NNDIST[i], i ) for i,c in level.iteritems() ]
        (cc,ii) = min( options )
        BEST[f] = ii
        BESTCOST[f] = cc

    # cumulative sum
    CUMBEST = dict()
    total = 0.
    for f, c in BESTCOST.iteritems() :
        total += c
        CUMBEST[f] = total
        
    # two-d plot...
    E = CUMBEST.keys()
    n = len(E)
    Z = [ -f for f in E ]
    
    EE, ZZ = np.meshgrid(E,Z)
    
    SURF = -50. * np.ones( (n,n) )
    for j,z in enumerate(Z) :
        for i,e in enumerate(E) :
            f = e - z
            if f not in CUMBEST : continue
            SURF[i,j] = CUMBEST[f] - CUMBEST[-z]
            
    def display() :
        if False :
            plt.figure()
            plt.scatter( S, np.zeros(NS), marker='x' )
            plt.scatter( T, np.zeros(NT), marker='o' )

        if True :
            fig = plt.figure()
            ax = fig.add_subplot(111,projection='3d')
            ax.plot_wireframe( EE, ZZ, SURF )
        
        if True :
            fig = plt.figure()
            plt.plot(BESTCOST.keys(), BESTCOST.values() )
    
    
    if False :
        """ sort the points into the "levels" they achieve """
        levels = bintrees.RBTree()
        h = 0
        for k, q in segment.iter_items() :
            h += len( q.P ) - len( q.Q )
            supp = levels.set_default( h, [] )
            supp.append( q )
            
            
        """ compute the minimum "cost" of loosening an excess forward match at each level """
        deltaf = dict()
        for k in range(NS-NT) :
            f = k+1     # index from 1
            
        
        
        
