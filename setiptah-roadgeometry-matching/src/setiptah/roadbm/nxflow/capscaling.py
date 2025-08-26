
#from bpmatch_roads.bm_roadnet import costWrapper

from setiptah.basic_graph.mygraph import mygraph
from setiptah.nxopt.cvxcostflow import MinConvexCostFlow

import setiptah.roadbm.bm as roadbm


class negativeWrapper :
    def __init__(self, func ) :
        self.func = func
        
    def __call__(self, z ) :
        return self.func( -z )

class offsetWrapper :
    def __init__(self, func, shift ) :
        self.func = func
        self.shift = shift
        
    def __call__(self, z ) :
        return self.func( z + self.shift )



def SOLVER( roadnet, surplus, objectives ) :
    network = mygraph()
    capacity = {}
    supply = { i : 0. for i in roadnet.nodes() }
    cost = {}   # functions
    #
    oneway_offset = {}  # for one-way roads
    
    for i,j, road, data in roadnet.edges_iter( keys=True, data=True ) :
        supply[j] += surplus[road]
        cost_data = objectives[road]
        
        # edge construction
        if data.get( 'oneway', False ) :
            # if one-way road
            
            # record minimum allowable flow on road
            zmin = cost_data.max_key()
            oneway_offset[road] = zmin
            supply[i] -= zmin
            supply[j] += zmin
            
            # shift and record the cost function on a forward edge only
            cc = roadbm.costWrapper( cost_data )
            cc_offset = offsetWrapper( cc, zmin )
            network.add_edge( road, i, j )
            cost[ road ] = cc_offset
            
        else :
            # if bi-directional road... currently, instantiate two edges
            cc = roadbm.costWrapper( cost_data )
            ncc = negativeWrapper( cc )     # won't have to worry about the C(0) offset
            
            network.add_edge( (road,+1), i, j )
            cost[ (road,+1) ] = cc
            #
            network.add_edge( (road,-1), j, i )
            cost[ (road,-1) ] = ncc
            
    # we need to compute the size U of the first cvxcost algorithm phase
    #U = sum([ len( obj_dict ) for obj_dict in objectives.values() ])
    # below is almost certainly just as good a bound, but I'm a scaredy-cat
    U = sum([ len( obj_dict ) - 2 for obj_dict in objectives.values() ])
    
    f = MinConvexCostFlow( network, {}, supply, cost, U )
    
    flow = {}
    for i, j, road in roadnet.edges_iter( keys=True ) :
        if road in oneway_offset :
            flow[road] = f[road] + oneway_offset[road]
        else :
            flow[road] = f[(road,+1)] - f[(road,-1)]
            
        flow[road] = int( flow[road] )
    
    #print flow
    return flow





