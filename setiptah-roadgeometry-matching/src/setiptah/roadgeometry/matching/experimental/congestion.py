import bintrees
import networkx as nx
import numpy as np
import scipy.signal as sig

import setiptah.roadgeometry.matching as roadbm
from setiptah.roadgeometry.matching.nx_legacy import MultiDiGraphRoadnet, CHECKFLOW


def BIPARTITEMATCH_ROADS_CONGESTED( S, T, roadmap, congestion_dict ) :
    """
    as vanilla bipartite matching on roadmaps, except
    congestion_dict associates with each road an integer function describing the 
    cost per match, per unit length, associated with any interval supporting n matches 
    """
    MATCH = []

    roadnet = MultiDiGraphRoadnet(roadmap)

    segment_dict = roadbm.compute_segments2(S, T, roadnet)
    surplus_dict = dict()
    measure_dict = dict()
    
    for road, segment in segment_dict.items() :
        match = roadbm.PREMATCH( segment )
        MATCH.extend( match )
        
        surplus_dict[road] = roadbm.SURPLUS( segment )

        roadlen = roadnet.length(road) or 1
        measure = roadbm.MEASURE( segment, roadlen )
        measure_dict[road] = measure

    # assert False, (surplus_dict, measure_dict)

    N = len(S) - len(MATCH)     # should be a better way...?
    assist = SOLVER( roadmap, surplus_dict, measure_dict, congestion_dict )
    
    if True :        # activate for debug
        imbalance = CHECKFLOW( assist, roadmap, surplus_dict )
    else :
        imbalance = []
        
    try :
        assert len( imbalance ) <= 0
    except Exception as ex :
        ex.imbal = imbalance
        raise ex
 
    return assist


def SOLVER(roadnet: nx.MultiDiGraph, surplus, measure_dict, congestion_dict):
    from setiptah.roadgeometry.matching.nxopt.cvxcostflow import MinConvexCostFlow
    from setiptah.roadgeometry.matching.util.mygraph import mygraph
    
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
    
    for i,j, road, data in roadnet.edges( keys=True, data=True ) :
        supply[j] += surplus[road]
        measure = measure_dict[road]
        rho = congestion_dict[road]
                
        fobj = CONGESTION_OBJECTIVE( measure, rho, U )  # U, here, restricts domain
        
        # edge construction
        if data.get( 'oneway', False ) :
            # if one-way road
            
            # record minimum allowable flow on road
            zmin = -measure.min_index   # i.e., z + min index of measure >= 0
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
    for i, j, road in roadnet.edges( keys=True ) :
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
    a = measure.min_index
    b = measure.max_index
    
    # serialize
    if True :
        W = [ w for k, w in measure.items() ]
        W.reverse()
        
    else :
        W = np.zeros(b-a + 1 )
        for k, w in measure.items() :
            W[-k] = w   # reverse here, and account for the circular shift!
    
    r"""
        need C(z) for z \in [-N,N] ??? is this true? drat!
        so, W_{n-z} = 0 for all n < a - N, and for all n > b + N
    """
    # relevant range
    A, B = a - U, b + U
    L = B-A + 1
    
    alpha = {}
    for n in range(A,B+1) :
        alpha[n] = abs(n) * rho(n)

    # prepare sequence form 
    if False :
        # w/ circular shift?
        alpha_seq = np.zeros(L)
        for n in range(A,B+1) : alpha_seq[n] = alpha[n]
    else :
        alpha_seq = [ alpha[k] for k in range(A,B+1) ]
        
    
    # perform convolution    
    res = bintrees.RBTree()     # not just any map!
    Z = range(-U,U+1)
    
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
