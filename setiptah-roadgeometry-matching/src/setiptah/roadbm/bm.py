from typing import Dict, TypeVar, Tuple, Generic, Protocol, Iterable
from collections import defaultdict
from dataclasses import dataclass

import numpy as np
import bintrees  # Migrate to `sortedcontainers`?

import networkx as nx   # TODO: Migrate it out?

""" my dependencies """
import setiptah.roadgeometry.roadmap_basic as ROAD
"""

TODO: Migrate away?

Here only used to compute individual match costs, and total matching cost (which we can compute directly from the flow).  

"""

TRoad = TypeVar("TRoad")
TVert = TypeVar("TVert")


class Roadnet(Protocol[TRoad, TVert]):

    def edges(self) -> Iterable[TRoad]:
        """Iterate the edges (roads) in the graph."""

    def nodes(self) -> Iterable[TVert]:
        """Iterate the nodes in the graph.

        TODO: Needed?

        """

    def length(self, road: TRoad) -> float:
        """Get the length of a road in the network."""

    def endpoints(self, road: TRoad) -> Tuple[TVert, TVert]:
        """Get the endpoints of the road."""

    def is_oneway(self, road: TRoad) -> bool:
        """Get whether the road is one-way."""


@dataclass(frozen=True)
class _EdgeData(Generic[TRoad, TVert]):
    edge: TRoad
    length: float
    i: TVert
    j: TVert
    oneway: bool


class MultiDiGraphRoadnet(Roadnet[TRoad, TVert]):

    def __init__(self, graph: nx.MultiDiGraph):
        self.graph = graph

        self.data = {
            road: _EdgeData(
                road, data["length"], i, j, data.get("oneway", False)
            )
            for i, j, road, data in self.graph.edges(keys=True, data=True)
        }

    def edges(self):
        return self.data.keys()

    def nodes(self) -> Iterable[TVert]:
        return self.graph.nodes()

    def length(self, road) -> float:
        return self.data[road].length

    def endpoints(self, road: TRoad) -> Tuple[TVert, TVert]:
        data = self.data[road]
        return data.i, data.j

    def is_oneway(self, road: TRoad) -> bool:
        return self.data[road].oneway


def ROADSBIPARTITEMATCH( P, Q, roadnet_graph: nx.MultiDiGraph, **kwargs ) :
    return optimal_roadnet_matching(
        P, Q, MultiDiGraphRoadnet(roadnet_graph), **kwargs
    )


def optimal_roadnet_matching(P, Q, roadnet: Roadnet, **kwargs):
    MATCH = []

    segment_dict = compute_segments( P, Q, roadnet )
    surplus_dict = dict()
    measure_dict = dict()
    
    for road, segment in segment_dict.items() :
        match = PREMATCH( segment )
        MATCH.extend( match )
        
        surplus_dict[road] = SURPLUS( segment )

        road_len = roadnet.length(road)
        measure = MEASURE( segment, road_len )
        measure_dict[road] = measure

    assist = compute_optimal_flow(roadnet, surplus_dict, measure_dict)

    # TODO: Is this needed, e.g., to check feasibility?
    imbalance = check_flow(assist, roadnet, surplus_dict)
    # Previously, this was active.
    # imbalance = []
        
    try :
        assert len( imbalance ) <= 0
    except Exception as ex :
        ex.imbal = imbalance
        raise ex

    if kwargs.get('assist_only', False ):
        return assist

    topograph = create_topograph(segment_dict, assist, roadnet)
    
    try :
        match = TRAVERSE( topograph )
    except Exception as ex :
        ex.assist = assist
        ex.topograph = topograph
        raise ex
    
    MATCH.extend( match )
    return MATCH


""" ALGORITHM SUB-ROUTINES """



""" Phase I: Transcription """


def WRITEOBJECTIVES(P, Q, roadnet_graph: nx.MultiDiGraph):
    return write_objectives(P, Q, MultiDiGraphRoadnet(roadnet_graph))


def write_objectives(P, Q, roadnet: Roadnet):
    segment_dict = compute_segments(P, Q, roadnet)
    surplus_dict = dict()
    objective_dict = dict()
    
    for road, segment in segment_dict.items():
        match = PREMATCH( segment )

        surplus_dict[road] = SURPLUS( segment )

        measure = MEASURE( segment, roadnet.length(road))
        objective_dict[road] = OBJECTIVE( measure )

    return objective_dict


OrderedPoints = bintrees.RBTree
"""

A collection data structure for bipartite points on a road.
The keys are numeric coordinates (e.g., float), with
a `TwoQueues`---a pair of "queues" (AKA lists)---containing points of the two types,
respectively, at given coordinate.

"""


def SEGMENTS(P, Q, roadnet: nx.MultiDiGraph) -> Dict[TRoad, OrderedPoints]:
    return compute_segments(P, Q, MultiDiGraphRoadnet(roadnet))


def compute_segments(P, Q, roadnet: Roadnet[TRoad, TVert]) -> OrderedPoints:
    """
    returns:
    a dictionary whose keys are coordinates and whose values are local (P,Q) index queues
    """
    segments = dict()
    for road in roadnet.edges():
        ensure_road(road, segments)  # these, and only these, roads are allowed

    for i, p in enumerate(P):
        r, y = p
        tree = segments[r]  # crash by design if r not in segments
        queues = ensure_key(y, tree)
        queues.P.append(i)

    for j, q in enumerate(Q):
        r, y = q
        tree = segments[r]
        queues = ensure_key(y, tree)
        queues.Q.append(j)

    return segments


def ONESEGMENT( S, T ) :
    roadnet = nx.MultiDiGraph()
    roadnet.add_edge(0,1, 'line' )
    
    SS = ( ('line',s) for s in S )
    TT = ( ('line',t) for t in T )
    
    segments = SEGMENTS( SS, TT, roadnet )
    return segments['line']

    
def PREMATCH( segment ) :
    match = []
    for y, q in segment.iter_items() :
        annih = min( len( q.P ), len( q.Q ) )
        for k in range( annih ) :
            i = q.P.pop(0)
            j = q.Q.pop(0)
            match.append( (i,j) )
            
    return match


def SURPLUS(segment: OrderedPoints):
    deltas = [ len( q.P ) - len( q.Q ) for y,q in segment.iter_items() ]
    return sum( deltas )


def MEASURE( segment, length, rbound=None ) :
    if rbound is not None :
        lbound = length
    else :
        lbound = 0.
        rbound = length
        
    # bintree instead of dict so that it is enumerated in sorted order
    measure = bintrees.RBTree()
    
    posts = [ lbound ] + [ y for y,q in segment.iter_items() ] + [ rbound ]
    intervals = zip( posts[:-1], posts[1:] )
    
    deltas = [0] + [ len(q.P)-len(q.Q) for y,q in segment.iter_items() ]
    F = np.cumsum( deltas )
    
    for (a,b), f in zip( intervals, F ) :
        measure.setdefault( f, 0. )
        measure[f] += b - a
        
    return measure









""" Phase II: Transformation/Solution/Verification """

from setiptah.basic_graph.mygraph import mygraph
from setiptah.nxopt.cvxcostflow import MinConvexCostFlow



class costWrapper :
    """
    wrap an RBTree arrangement of LineData()s to obtain a piece-wise linear callable function 
    """
    def __init__(self, lines ) :
        self.lines = lines
        
    def __call__(self, z ) :
        """
        this is an O(log n) query function (although, probably an O(1) expected hash map), 
        can be reduced to O(1) by random access after floor operation """
        _, line = self.lines.floor_item( z )
        return line( z )

class negativeWrapper :
    """ a simple callable wrapper to create f(-x) from f(x) """
    def __init__(self, func ) :
        self.func = func
        
    def __call__(self, z ) :
        return self.func( -z )

class offsetWrapper :
    """ a simple callable wrapper to create f(x+x0) from f(x) """
    def __init__(self, func, shift ) :
        self.func = func
        self.shift = shift
        
    def __call__(self, z ) :
        return self.func( z + self.shift )







def OBJECTIVE( measure ) :
    """
    produces the objective LineData()s RBTree arrangement
    given the dictionary of interval measures;
    N levels => N+1 LineData()s (verify?)
    """
    def sweep( x ) :
        Xminus = np.cumsum( x )
        total = Xminus[-1]
        Xplus = total - Xminus
        X = Xplus - Xminus
        return X
    
    # prepare constants kappa and alpha
    PREALPHA = np.array( [ 0. ] + [ w for f,w in measure.items() ] )
    ALPHA = sweep( PREALPHA )
    
    PREKAPPA = np.array( [ 0. ] + [ f*w for f,w in measure.items() ] )
    KAPPA = sweep( PREKAPPA )
    
    Cz = bintrees.RBTree()
    ff = [ f for f in measure ] + [ np.inf ]        # should be in order
    for f, alpha, kappa in zip( ff, ALPHA, KAPPA ) :
        Cz.insert( -f, LineData( alpha, kappa ) )
        
    return Cz


def OBJECTIVE_FUNC( measure ) :
    """ produces the convex objective function assoc. with a set of interval measures """
    return costWrapper( OBJECTIVE(measure) )


def SOLVER( roadnet, surplus, measure_dict ) :
    return compute_optimal_flow(MultiDiGraphRoadnet(roadnet), surplus, measure_dict)


def compute_optimal_flow(
        roadnet: Roadnet[TRoad, TVert],
        surplus: Dict[TVert, float],
        measure_dict: bintrees.RBTree,  # float -> float
) -> Dict[TRoad, float]:
    network = mygraph()
    capacity = {}  # TODO: Was this for something?
    supply = { i : 0. for i in roadnet.nodes() }
    cost = {}   # functions
    #
    oneway_offset = {}  # for one-way roads

    for road in roadnet.edges():
        i, j = roadnet.endpoints(road)

        supply[j] += surplus[road]
        measure = measure_dict[road]
        
        fobj = OBJECTIVE_FUNC( measure )
        
        # edge construction
        if roadnet.is_oneway(road):
            # if one-way road
            
            # record minimum allowable flow on road
            zmin = -measure.min_key()   # i.e., z + min key of measure >= 0 
            oneway_offset[road] = zmin
            # create a 'bias point'
            supply[i] -= zmin
            supply[j] += zmin
            
            # shift and record the cost function on only a forward edge
            fobj_offset = offsetWrapper( fobj, zmin )
            network.add_edge( road, i, j )
            cost[ road ] = fobj_offset
            
        else :
            # if bi-directional road... instantiate pair of edges
            #cc = roadbm.costWrapper( cost_data )
            n_fobj = negativeWrapper( fobj )     # won't have to worry about the C(0) offset
            
            network.add_edge( (road,+1), i, j )
            cost[ (road,+1) ] = fobj
            #
            network.add_edge( (road,-1), j, i )
            cost[ (road,-1) ] = n_fobj

    """
    compute the width U of the first cvxcost algorithm phase;
    a bound on the optimal flow on any edge; 
    Logic: there cannot be more flow on a given road in the graph
    than there are total intervals between levels in the network
    (Proof Sketch):
    1. U <= M ;
    2. (Prove...) Given any matching instance which
        induces a measure network w/ U' total intervals between levels,
        a new matching instance realizing the same measure network can be constructed
        on just U' points in each set
    """
    # safe-ish...
    #U = sum([ len(m) + 1 for m in measure_dict.values() ])
    # below is almost certainly just as good a bound, but I'm a scaredy-cat
    U = sum([ len(m) - 1 for m in measure_dict.values() ])
    
    f = MinConvexCostFlow( network, {}, supply, cost, U )
    
    flow = {}
    for road in roadnet.edges():
        if road in oneway_offset :
            flow[road] = f[road] + oneway_offset[road]
        else :
            flow[road] = f[(road,+1)] - f[(road,-1)]
            
        flow[road] = int( flow[road] )
    
    return flow


def CHECKFLOW(
        flow: Dict[TRoad, float],
        roadnet: nx.MultiDiGraph,
        surplus: Dict[TVert, float]
) -> Dict[TVert, float]:
    return check_flow(flow, MultiDiGraphRoadnet(roadnet), surplus)


def check_flow(
        flow: Dict[TRoad, float],
        roadnet: Roadnet[TRoad, TVert],
        surplus: Dict[TVert, float]
) -> Dict[TVert, float]:
    balance = {u: 0. for u in roadnet.nodes()}
    for road in roadnet.edges():
        i, j = roadnet.endpoints(road)
        balance[i] -= flow.get(road, 0.)
        balance[j] += flow.get(road, 0.) + surplus.get(road, 0.)

    return {k: v for k, v in balance.items() if v != 0.}


""" Phase III: Matching Construction """


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


def TOPOGRAPH(
        segment_dict, assist: Dict[TRoad, float], roadnet: nx.MultiDiGraph
) -> nx.DiGraph:
    return create_topograph(
        segment_dict, assist, MultiDiGraphRoadnet(roadnet)
    )


def create_topograph(
        segment_dict, assist: Dict[TRoad, float], roadnet: Roadnet
) -> nx.DiGraph:
    topograph = nx.DiGraph()

    special = dict()
    for u in roadnet.nodes():
        # data = TwoQueues()
        node = terminal(None)
        special[u] = node

    for road in roadnet.edges():
        u, v = roadnet.endpoints(road)

        segment = segment_dict[road]
        z = assist[road]

        edges = EDGES(segment)
        for f, intervals in edges.items():
            h = f + z
            for (ll, rr) in intervals:
                if ll.q == '-': ll = special[u]
                if rr.q == '+': rr = special[v]

                if h > 0:
                    topograph.add_edge(ll, rr, weight=h)
                if h < 0:
                    topograph.add_edge(rr, ll, weight=-h)

    return topograph


def CHECKTOPO( topograph ) :
    def balance( u ) :
        # starting balance
        q = u.q
        if q is None :
            b = 0
        else :
            b = len( q.P ) - len( q.Q )
            
        # plus input
        for e in topograph.in_edges( u ) :
            b += topograph.get_edge_data( *e ).get('weight')
            
        # minus output
        for e in topograph.out_edges( u ) :
            b -= topograph.get_edge_data( *e ).get('weight')
            
        return b
            
    return [ u for u in topograph.nodes() if balance(u) != 0 ]


def TRAVERSE( topograph ) :
    match = []
    nodes_ord = nx.topological_sort( topograph )

    LISTS = defaultdict(list)

    for u in nodes_ord:
        L = LISTS[u]
        LISTS.pop(u)  # not needed anymore

        queue = u.q
        if queue is not None :
            # collect points from S
            L.extend( queue.P )
            
            # dispatch points in T
            for j in queue.Q :
                i = L.pop(0)
                match.append( (i,j) )
        
        for _,v, data in topograph.out_edges( u, data=True ) :
            w = data.get('weight')
            prefix, L = L[:w], L[w:]
            LISTS[v].extend( prefix )
            
    return match











""" Misc. Algorithm Utilities """

def ensure_road( road, data ) :
    curr = data.setdefault( road )
    if curr is None : data[road] = bintrees.RBTree()
    return data[road]

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

class LineData :
    def __init__(self, m,b) :
        self.slope = m
        self.offset = b
        
    def __call__(self, x ) :
        return self.slope * x + self.offset
    
    def __repr__(self) :
        return '<%f z + %f>' % ( self.slope, self.offset )


class terminal :    # simple node type for TRAVERSE
    def __init__(self, q ) :
        self.q = q

    def __repr__(self):
        return f"terminal({self.q})"


def INTERVALS( segment ) :      # very similar routine, used to build the walk graph
    res = dict()
    
    posts = [ '-' ] + [ y for y,q in segment.iter_items() ] + [ '+' ]
    intervals = zip( posts[:-1], posts[1:] )
    
    deltas = [0] + [ len(q.P)-len(q.Q) for y,q in segment.iter_items() ]
    F = np.cumsum( deltas )
    
    for I, f in zip( intervals, F ) :
        res.setdefault( f, [] )
        res[f].append( I )
        
    return res


def MATCHCOSTS(matching: Tuple[int, int], P, Q, roadnet: nx.MultiDiGraph):
    costs = []
    for i, j in matching:
        p = ROAD.RoadAddress( *P[i] )
        q = ROAD.RoadAddress( *Q[j] )
        d = ROAD.distance( roadnet, p, q, 'length' )
        costs.append( d )
    return costs


def ROADMATCHCOST( match, P, Q, roadnet ) :
    costs = MATCHCOSTS( match, P, Q, roadnet )
    return sum( costs )


def flow_cost_per_road(flow: Dict[TRoad, float], obj_dict):
    return {
        road: obj_dict[road](x)
        for road, x in flow.items()
    }
