
import numpy as np
import networkx as nx

import bintrees

import setiptah.roadbm as roadbm
import setiptah.roadgeometry.roadmap_basic as ROAD

import scipy.signal as sig






def BIPARTITEMATCH_ROADS_CONGESTED( S, T, roadmap, congestion_dict ) :
    """
    as vanilla bipartite matching on roadmaps, except
    congestion_dict associates with each road an integer function describing the 
    cost per match, per unit length, associated with any interval supporting n matches 
    """
    MATCH = []
    
    segment_dict = roadbm.SEGMENTS( S, T, roadmap )
    surplus_dict = dict()
    measure_dict = dict()
    
    for road, segment in segment_dict.iteritems() :
        match = roadbm.PREMATCH( segment )
        MATCH.extend( match )
        
        surplus_dict[road] = roadbm.SURPLUS( segment )
        
        roadlen = roadbm.get_road_data( road, roadmap ).get( 'length', 1 )
        measure = roadbm.MEASURE( segment, roadlen )
        measure_dict[road] = measure
        
    N = len(S) - len(MATCH)     # should be a better way...?
    assist = SOLVER( roadmap, surplus_dict, measure_dict, congestion_dict )
    
    if True :        # activate for debug
        imbalance = roadbm.CHECKFLOW( assist, roadmap, surplus_dict )
    else :
        imbalance = []
        
    try :
        assert len( imbalance ) <= 0
    except Exception as ex :
        ex.imbal = imbalance
        raise ex
 
    return assist



 
    topograph = roadbm.TOPOGRAPH( segment_dict, assist, roadmap )
 
    return topograph
 
    # will need a more informative TRAVERSE method    
    try :
        match = roadbm.TRAVERSE( topograph )
    except Exception as ex :
        ex.assist = assist
        ex.topograph = topograph
        raise ex
    
    MATCH.extend( match )
    return MATCH










def SOLVER( roadnet, surplus, measure_dict, congestion_dict ) :
    from setiptah.nxopt.cvxcostflow import MinConvexCostFlow
    from setiptah.basic_graph.mygraph import mygraph
    
    # a rather crucial measure of the problem's complexity;
    # see bm.SOLVER for relevant commentary
    U = sum([ len(m) - 1 for m in measure_dict.values() ]) 

    # instantiate cvxcostflow components    
    network = mygraph()
    capacity = {}
    supply = { i : 0. for i in roadnet.nodes() }
    cost = {}   # functions
    #
    oneway_offset = {}  # to process one-way roads
    
    for i,j, road, data in roadnet.edges_iter( keys=True, data=True ) :
        supply[j] += surplus[road]
        measure = measure_dict[road]
        rho = congestion_dict[road]
                
        fobj = CONGESTION_OBJECTIVE( measure, rho, U )  # U, here, restricts domain
        
        # edge construction
        if data.get( 'oneway', False ) :
            # if one-way road
            
            # record minimum allowable flow on road
            zmin = -measure.min_key()   # i.e., z + min key of measure >= 0 
            oneway_offset[road] = zmin
            # create a 'bias point'
            supply[i] -= zmin
            supply[j] += zmin
            
            # shift and record the cost function on only a forward edge
            fobj_offset = roadbm.offsetWrapper( fobj, zmin )
            network.add_edge( road, i, j )
            cost[ road ] = fobj_offset
            
        else :
            # if bi-directional road... instantiate pair of edges
            #cc = roadbm.costWrapper( cost_data )
            n_fobj = roadbm.negativeWrapper( fobj )     # won't have to worry about the C(0) offset
            
            network.add_edge( (road,+1), i, j )
            cost[ (road,+1) ] = fobj
            #
            network.add_edge( (road,-1), j, i )
            cost[ (road,-1) ] = n_fobj

    f = MinConvexCostFlow( network, {}, supply, cost, U )   # U, here, determines phase count
    
    flow = {}
    for i, j, road in roadnet.edges_iter( keys=True ) :
        if road in oneway_offset :
            flow[road] = f[road] + oneway_offset[road]
        else :
            flow[road] = f[(road,+1)] - f[(road,-1)]
            
        flow[road] = int( flow[road] )
    
    return flow










def CONGESTION_OBJECTIVE( measure, rho, U, efficient=True ) :
    obj_data = CONGESTION_OBJECTIVE_DATA( measure, rho, U, efficient )
    return RBTreeLinterp( obj_data )



def CONGESTION_OBJECTIVE_DATA( measure, rho, U, efficient=True ) :
        
    """ prepare convolution """
    a = measure.min_key()
    b = measure.max_key()
    
    # serialize
    if True :
        W = [ w for k, w in measure.items() ]
        W.reverse()
        
    else :
        W = np.zeros(b-a + 1 )
        for k, w in measure.items() :
            W[-k] = w   # reverse here, and account for the circular shift!
    
    """
        need C(z) for z \in [-N,N] ??? is this true? drat!
        so, W_{n-z} = 0 for all n < a - N, and for all n > b + N
    """
    # relevant range
    A, B = a - U, b + U
    L = B-A + 1
    
    alpha = {}
    for n in xrange(A,B+1) :
        alpha[n] = abs(n) * rho(n)

    # prepare sequence form 
    if False :
        # w/ circular shift?
        alpha_seq = np.zeros(L)
        for n in xrange(A,B+1) : alpha_seq[n] = alpha[n]
        
        #for n in xrange(B+1) : alpha_seq[n] = alpha[n]
        #for n in xrange(A,0) :
            #alpha_seq[ B+1 + n-A ] = alpha[n]
            
    else :
        alpha_seq = [ alpha[k] for k in xrange(A,B+1) ]
        
    
    # perform convolution    
    res = bintrees.RBTree()     # not just any map!
    Z = xrange(-U,U+1)
    
    if not efficient :
                
        # long-hand convolution, for verification
        C_manual = {}
        for z in Z :
            C_manual[z] = 0.
            for k, w in measure.items() :
                n = k + z
                C_manual[z] += alpha[n] * w
    
        res.update( C_manual )
        
    else :
        # efficient, FFT method
        C_fft = sig.fftconvolve( W, alpha_seq )
        C_fix = C_fft[b-A-U:b-A+U+1]
        
        res.update( zip( Z, C_fix ) )
        
    # actually... wrap this first
    return res




class RBTreeLinterp :
    """
    linear interpolation wrapper:
    callable returns linear interpolation between floor_key and ceil_key;
    thus, valid for any real value between minkey and maxkey.  
    """ 
    def __init__(self, rbtree ) :
        assert isinstance(rbtree, bintrees.RBTree )
        self.tree = rbtree
        
    def __call__(self, z ) :
        z1, f1 = self.tree.floor_item(z)
        z2, f2 = self.tree.ceiling_item(z)
        
        if z2 > z1 :
            m = float( f2 - f1 ) / ( z2 - z1 )
            return f1 + m * ( z - z1 )
        else :
            return f1




if __name__== '__main__' :
    
    import matplotlib.pyplot as plt
    plt.close('all')

    """ parameters """
    SAMPLE_RANDOMLY = True
    
    if False :
        # circle
        roadmap = nx.MultiDiGraph()
        roadmap.add_edge(0,0,'A', length=1. )
        
        
    elif False :
        # Delaunay roadmap
        
        if False :
            # random
            verts = [ 10*np.random.rand(2) for i in xrange(5) ]
        else :
            # ring
            polar = lambda t : np.array([ np.cos(t), np.sin(t) ])
            theta = np.linspace(0,2*np.pi,7+1)[:-1]
            verts = [ 5. * polar(t) for t in theta ]
            
        import setiptah.roadgeometry.generation as roadgen
        roadmap = roadgen.DelaunayRoadMap(verts)
        
        pos = { k : p for k, p in enumerate(verts) }
    
    elif True :
        stage_width = 1.
        via_buffer = 1.
        traverse = 10.
        band_height = .3
        
        NUMPOINTS = 10
        BANDWIDTH = 20

        # construct instance        
        roadmap = nx.MultiDiGraph()
                        
        # establish notable x positions
        X = np.cumsum([ 0, stage_width, via_buffer, traverse, via_buffer, stage_width ])

        # draw staging area
        roadmap.add_edge(0,1,'start', length=1. )
        roadmap.add_edge(4,5,'end', length=1. )
        # ...and position the nodes
        pos = {}
        for k in [ 0, 1, 4, 5 ] : pos[k] = np.array( ( X[k], 0 ) )

        # place a number of bands
        for k in xrange(BANDWIDTH) :
            for k in xrange(BANDWIDTH) :
                u, v = [ fmt % k for fmt in [ 'L%d', 'R%d' ] ]
                pos[u] = np.array( (X[2],-k * band_height) )
                pos[v] = np.array( (X[3],-k * band_height) )
                
                r1, r2, r3 = [ fmt % k for fmt in [ 'LCONN%d', 'BAND%d', 'RCONN%d' ] ]
                roadmap.add_edge(1,u, r1 )
                roadmap.add_edge(u,v, r2 )
                roadmap.add_edge(v,4, r3 )
                
        # give length annotations
        for u,v, road, data in roadmap.edges_iter( keys=True, data=True ) :
            #p = np.array( pos[u] )
            #q = np.array( pos[v] )
            data['length'] = np.linalg.norm( pos[v] - pos[u] )

        # place the points
        Y = np.linspace(0,1,NUMPOINTS+2)[1:-1]
        S = [ ROAD.RoadAddress('start',y) for y in Y ]
        T = [ ROAD.RoadAddress('end',y) for y in Y ]
        SAMPLE_RANDOMLY = False



    
    """ congestion function """
    #rho = lambda x : np.power( x, 2. )          # square law, why not!
    p = .07
    rho = lambda x : np.power( abs(x), p )                      # linear congestion?


    # separate instance?

    if SAMPLE_RANDOMLY :
        NUMPOINTS = 10
        
        
        import setiptah.roadgeometry.probability as roadprob
        sampler = roadprob.UniformDist(roadmap)
        
        S = [ sampler.sample() for i in xrange(NUMPOINTS) ]
        T = [ sampler.sample() for i in xrange(NUMPOINTS) ]
    
    # algorithm
    # stolen form ROADSBIPARTITEMATCH    
    
    rho_dict = { road : rho for i,j,road in roadmap.edges_iter(keys=True) }
    
    
    assist = BIPARTITEMATCH_ROADS_CONGESTED( S, T, roadmap, rho_dict )
    assist_nocongestion = roadbm.ROADSBIPARTITEMATCH( S, T, roadmap, assist_only=True )
    
    import setiptah.roadbm.matchvis as matchvis
    
    plt.figure()
    plt.title('Congestion Optimal')
    matchvis.SHOWTRAILS( S, T, assist, roadmap, pos )
    
    plt.figure()
    plt.title('Pure Path-length Optimal')
    matchvis.SHOWTRAILS( S, T, assist_nocongestion, roadmap, pos )


    if False :    
        segment_dict = roadbm.SEGMENTS( S, T, roadmap )
        measure_dict = {}
        
        for road, segment in segment_dict.iteritems() :
            match = roadbm.PREMATCH( segment )
            MATCH.extend( match )
            
            #surplus_dict[road] = SURPLUS( segment )
            
            roadlen = ROAD.get_road_data( road, roadmap ).get( 'length', 1 )
            measure = roadbm.MEASURE( segment, roadlen )
            measure_dict[road] = measure
            #objective_dict[road] = OBJECTIVE( measure )
            #objective_dict[road] = objective
        """ measure_dict now contains sequence of W_n """
    
    
        C_man = CONGESTION_OBJECTIVE_DATA( measure, rho, False )
        C_fft = CONGESTION_OBJECTIVE_DATA( measure, rho, True )
        
        C = CONGESTION_OBJECTIVE( measure, rho )
        
    
    
    
    
    
    
    
    
    
    
    
    


    