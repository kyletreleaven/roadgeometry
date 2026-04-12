import itertools
import logging
import math
from collections.abc import Mapping
from typing import Callable, Protocol

import numpy as np

from setiptah.roadgeometry.matching.util.mygraph import mygraph, Dijkstra
from setiptah.roadgeometry.protocol import TRoad

try:
    from setiptah.roadgeometry.matching._cpp import dijkstra as cpp_dijkstra
except ImportError:
    cpp_dijkstra = None

PHASE_ERROR = 10**-6        # TODO: Find a way to eliminate this.


class DijkstraFn(Protocol):
    """General mygraph-level Dijkstra interface."""
    def __call__(self, graph: mygraph, cost: dict, source) -> tuple[dict, dict]: ...


class FlatIntDijkstra:
    """Wraps a flat int-array dijkstra to satisfy DijkstraFn.

    When called directly, normalizes on each call. FragileMCCF detects this
    wrapper via isinstance, unwraps ._fn, and handles normalization itself
    using a pre-computed node mapping (stable across the solve).

    TODO: long-term this becomes part of a Solver bundle that also covers graph
    evolution, since Dijkstra and graph evolver naturally co-vary (both general
    or both flat-int).
    """

    def __init__(self, fn):
        self._fn = fn

    def __call__(self, graph: mygraph, cost: dict, source) -> tuple[dict, dict]:
        nodes = list(graph.nodes())
        node_to_int = {n: i for i, n in enumerate(nodes)}
        out_edges_arr, endpoints_arr, cost_arr, edges = _normalize_graph(graph, cost, node_to_int)
        dist_arr, up_arr = self._fn(out_edges_arr, endpoints_arr, cost_arr, node_to_int[source])
        return _denormalize_dijkstra(dist_arr, up_arr, nodes, edges, source)

    @classmethod
    def node_mapping(cls, network: mygraph) -> tuple[dict, list]:
        """Pre-compute (node_to_int, int_to_node) for a stable node set."""
        nodes = list(network.nodes())
        return {n: i for i, n in enumerate(nodes)}, nodes

    @property
    def fn(self):
        return self._fn


def py_dijkstra(graph: mygraph, cost: dict, source) -> tuple[dict, dict]:
    return Dijkstra(graph, cost, source)


_default_dijkstra: DijkstraFn = FlatIntDijkstra(cpp_dijkstra) if cpp_dijkstra is not None else py_dijkstra

LOG = logging.getLogger(__name__)


class line :
    def __init__(self, slope ) :
        self.m = slope
    def __call__(self, x ) :
        return self.m * x


""" Utility Algorithms """

"""
each algorithm can either populate an empty data structure, or can incrementally 
update one
"""
    
    
def ResidualGraph( rgraph: mygraph, flow, capacity, Delta, network, edge=None ) :
    if edge is None :
        iter = network.edges()
    else :
        iter = [ edge ]
        
    for e in iter :
        x = flow.get( e, 0. )
        u = capacity.get( e, np.inf )
        assert x >= 0. and x <= u
        
        i,j = network.endpoints(e)
        
        ee = (e,+1)
        if rgraph.has_edge( ee ) : rgraph.remove_edge( ee )
        if x + Delta <= u : rgraph.add_edge( ee, i, j )
        
        ee = (e,-1)
        if rgraph.has_edge( ee ) : rgraph.remove_edge( ee )
        if x >= Delta : rgraph.add_edge( ee, j, i )



def LinearizeCost( lincost, cost, flow, Delta, network, edge=None ) :
    if edge is None :
        iter = network.edges()
    else :
        iter = [ edge ]
    
    for e in iter :
        x = flow.get(e, 0. )
        cc = cost.get( e, line(0.) )
        for dir in [ +1, -1 ] :
            lincost[(e,dir)] = float( cc( x + dir * Delta ) - cc(x) ) / Delta
            
            
def ReducedCost( rcost, lincost, potential, network, edge=None ) :
    if edge is None :
        iter = network.edges()
    else :
        iter = [ edge ]
        
    # if node is not in potential, it is assumed to have zero potential
    for e in iter :
        i,j = network.endpoints(e)
        #rcost[(e,+1)] = lincost.get( (e,+1), 0. ) + potential.get(j,0.) - potential.get(i,0.)
        #rcost[(e,-1)] = lincost.get( (e,-1), 0. ) + potential.get(i,0.) - potential.get(j,0.)
        rcost[(e,+1)] = lincost[(e,+1)] + potential.get(j,0.) - potential.get(i,0.)
        rcost[(e,-1)] = lincost[(e,-1)] + potential.get(i,0.) - potential.get(j,0.)
            
            
            
def Excess( excess, flow, graph, supply, edge=None ) :
    if edge is None :
        iter = graph.nodes()
    else :
        iter = graph.endpoints( edge )
        
    for i in iter :
        excess[i] = supply.get(i, 0. )
        
        for e in graph.W[i] :   # edges in
            excess[i] += flow.get(e, 0. )
        for e in graph.V[i] :
            excess[i] -= flow.get(e, 0. )
            
            

def _normalize_graph(graph, cost, node_to_int):
    """Build flat int-indexed arrays from a mygraph + cost dict for C++ Dijkstra.

    Returns (out_edges, endpoints, cost_arr, edges) where `edges` is the list
    of edge ids in the order used for indexing (needed for denormalization).
    """
    edges = list(graph.edges())
    n = len(node_to_int)
    out_edges = [[] for _ in range(n)]
    endpoints = [None] * len(edges)
    cost_arr = [0.0] * len(edges)
    for ei, e in enumerate(edges):
        tail, head = graph.endpoints(e)
        out_edges[node_to_int[tail]].append(ei)
        endpoints[ei] = (node_to_int[tail], node_to_int[head])
        cost_arr[ei] = cost[e]
    return out_edges, endpoints, cost_arr, edges


def _denormalize_dijkstra(dist_arr, up_arr, int_to_node, edges, source):
    """Convert C++ Dijkstra output back to {node: dist} and {node: edge} dicts."""
    inf = float('inf')
    dist = {int_to_node[i]: d for i, d in enumerate(dist_arr) if d < inf}
    upstream = {int_to_node[i]: edges[e] for i, e in enumerate(up_arr) if e >= 0}
    upstream[source] = None
    return dist, upstream


def _check_upstream(dist, upstream, source, graph, cost):
    """Dijkstra correctness condition: each node's upstream path to source
    must accumulate exactly dist[node] in total cost."""
    assert dist[source] == 0.0, f"source dist {dist[source]} != 0"
    assert upstream[source] is None, f"source upstream {upstream[source]} != None"
    for node, d in dist.items():
        acc, j, visited = 0.0, node, set()
        while j != source:
            assert j not in visited, f"upstream cycle detected at {j}"
            visited.add(j)
            e = upstream[j]
            acc += cost[e]
            tail, _ = graph.endpoints(e)
            j = tail
        assert abs(acc - d) < 1e-9, f"node {node}: upstream path cost {acc} != dist {d}"
    return True


""" Convex Cost Flow Algorithm """

class ALGGLOBAL :
    REGULAR = ':'
    AUGMENTING = 'AUG'

def MinConvexCostFlow( network, capacity, supply, cost, U, epsilon=None, *, dijkstra: DijkstraFn = _default_dijkstra ) :
    """
    network is a mygraph() --- supports non-negative flow on digraph edges
    capacity is a dict() : road -> real, non-neg flow capacity
    supply is a dict() : vertex -> real vertex supply; assumed conservative supply, i.e., sums to 0
    cost is a dict() : road -> convex cost function assoc. w/ road
    #
    U is the width of the first phase of the capacity-scaling algorithm
    epsilon is final phase width: eps=1 (default) yields integer optimal solution
    """

    # create a *robust* instance, to ensure strong connectivity of *any* Delta-residual graph
    network_aug, capacity_rename, cost_aug = MCCFRobustInstance( network, capacity, supply, cost, U )

    # run the "fragile" implementation
    flow = FragileMCCF( network_aug, capacity_rename, supply, cost_aug, U, epsilon, dijkstra=dijkstra )

    # prepare output --- perhaps do some feasibility checking in the future
    res = { e : x for (type,e), x in flow.items() if type == ALGGLOBAL.REGULAR }
    return res


def MCCFRobustInstance( network, capacity, supply, cost, U ) :
    """
    this wrapper transforms any convex cost flow instance into an equivalent one for which
    every Delta-residual graph is strongly connected
    """
    network_aug = mygraph()
    capacity_rename = {}
    cost_aug = {}
    
    for e in network.edges() :
        i,j = network.endpoints(e)
        newedge = (ALGGLOBAL.REGULAR,e)
        
        network_aug.add_edge( newedge, i, j )
        if e in capacity : capacity_rename[ newedge ] = capacity[e]
        if e in cost : cost_aug[ newedge ] = cost[e]
        
    # add a directed cycle, with prohibitive cost
    CBOUND = sum([ c(U) for c in cost.values() ])
    # since costs are convex, a feasible flow cannot have cost greater than CBOUND
    prohibit = line(CBOUND)
    
    NODES = list(network.nodes())
    edgegen = itertools.count()
    for i,j in zip( NODES, NODES[1:] + NODES[:1] ) :
        frwd = (ALGGLOBAL.AUGMENTING, next(edgegen) )
        network_aug.add_edge( frwd, i, j )
        cost_aug[frwd] = prohibit
        
    return network_aug, capacity_rename, cost_aug
    
    
    
    
def FragileMCCF( network, capacity_in, supply, cost, U, epsilon=None, *, dijkstra: DijkstraFn = _default_dijkstra ) :
    """
    network is a mygraph (above)
    capacity is a dictionary from E -> real capacities
    supply is a dictionary from E -> real supplies
    cost is a dictionary from E -> lambda functions of convex cost edge costs

    1. Assumes supply is conservative (sum to zero).
    2. Assumes every Delta-residual graph is strongly connected,
    i.e., there exists a path with inf capacity b/w any two nodes;
    """
    if epsilon is None : epsilon = 1
    
    # initialize algorithm data
    rgraph = mygraph()
    lincost = {}
    redcost = {}
    excess = {}
    
    """ ALGORITHM """
    # computing U from supplies was wrong, it must be passed in now
    #U = sum([ b for b in supply.values() if b > 0. ])
    #print 'total supply: %f' % U
    
    # trimming infinite capacities to U allows negative initial slopes 
    # the initial flow may not be Delta-optimal at the beginning of Stage One,
    # but achieves Delta-optimality by the end, by saturating any negative cost edges.
    # most treatments fail to consider negative initial slope, which is totally possible... 
    capacity = {}
    for e in network.edges() :
        capacity[e] = min( U, capacity_in.get( e, np.inf ) )
        
    try :
        temp = math.floor( math.log(U,2) )
    except Exception as ex :
        ex.U = U
        raise ex
        
    Delta = 2.**temp
    LOG.debug('Delta: %d' % Delta)

    flow = { e : 0. for e in network.edges() }
    Excess( excess, flow, network, supply )

    # Pre-compute node normalization for FlatIntDijkstra (node set is stable across the solve).
    if isinstance(dijkstra, FlatIntDijkstra):
        _node_to_int, _int_to_node = FlatIntDijkstra.node_mapping(network)

    potential = { i : 0. for i in network.nodes() }
        
    while Delta >= epsilon :
        LOG.debug('\nnew phase: Delta=%f' % Delta)
        
        # Delta is fresh, so we need to [re-] linearize the costs and compute residual graph 
        LinearizeCost( lincost, cost, flow, Delta, network )
        ReducedCost( redcost, lincost, potential, network )
        ResidualGraph( rgraph, flow, capacity, Delta, network )
        #
        cert = { re : c for (re,c) in redcost.items() if re in rgraph.edges() }
        LOG.debug('reduced costs on res. graph, phase init: %s' % repr( cert ))
        
        """ Stage 1. """
        # for every arc (i,j) in the residual network G(x)
        for resedge in list(rgraph.edges()):
            e,dir = resedge
            # theory says, we only need to do this at most once per edge...
            # wouldn't want to question theory
            # ... keep an eye out for a flip-flop; in theory, shouldn't happen
            if redcost[resedge] < 0. :
                LOG.debug('correcting negative red. cost on resedge %s: %f' % ( resedge, redcost[resedge] ))
                
                # no augment, just saturate!
                flow[e] += dir * Delta
                Excess( excess, flow, network, supply, edge=e )
                #print 'flow correction: %s' % repr( flow )
                
                LinearizeCost( lincost, cost, flow, Delta, network, edge=e )
                ResidualGraph( rgraph, flow, capacity, Delta, network, edge=e )
                ReducedCost( redcost, lincost, potential, network, edge=e )
                
        # at end of each stage, verify the optimality certificate (should be empty every time)
        CERT = { re : c for (re,c) in redcost.items() if re in rgraph.edges() and c < 0. }
        LOG.debug('certificate, end stage ONE: %s' % repr( CERT ))
        #if len( CERT ) > 0 : print "STAGE ONE CERTIFICATE CORRUPT!"
        # am considering removing this assertion, but leaving the stage two one
        # could be running into problems where the functional form is defined beyond saturation bounds
        RELAXCERT = { re : c for (re,c) in redcost.items() if re in rgraph.edges() and c < -PHASE_ERROR }
        if len( RELAXCERT ) > 0 : 
            LOG.debug(RELAXCERT)
            LOG.debug("STAGE ONE CERTIFICATE CORRUPT!")
        LOG.debug(RELAXCERT)
        assert len( RELAXCERT ) <= 0
        
                
        """ Stage 2. """
        # while there are imbalanced nodes
        while True :
            LOG.debug('flow: %s' % repr( flow ))
            #excess = Excess( flow, network, supply )        # last function that needs to be increment-ized
            #print 'excess: %s' % repr(excess)
            
            SS = [ i for i,ex in excess.items() if ex >= Delta ]
            TT = [ i for i,ex in excess.items() if ex <= -Delta ]
            LOG.debug('surplus nodes: %s' % repr( SS ))
            LOG.debug('deficit nodes: %s' % repr( TT ))
            if len( SS ) <= 0 or len( TT ) <= 0 : break
            
            s = SS[0] ; t = TT[0]
            LOG.debug('shall augment %s to %s' % ( repr(s), repr(t) ))
            
            #print 'potentials: %s' % repr( potential )
            cert = { re : c for (re,c) in redcost.items() if re in rgraph.edges() }
            #print 'reduced costs on res. graph, for shortest paths: %s' % repr( cert )
            
            if isinstance(dijkstra, FlatIntDijkstra):
                _out_edges, _endpoints, _cost_arr, _redges = _normalize_graph(rgraph, redcost, _node_to_int)
                _dist_arr, _up_arr = dijkstra.fn(_out_edges, _endpoints, _cost_arr, _node_to_int[s])
                dist, upstream = _denormalize_dijkstra(_dist_arr, _up_arr, _int_to_node, _redges, s)
            else:
                dist, upstream = dijkstra(rgraph, redcost, s)
            #print 'Dijkstra shortest path distances: %s' % repr( dist )
            #print 'Dijkstra upstreams: %s' % repr( upstream )
            
            # find shortest path w.r.t. reduced costs (just follow ancestry links to the root)
            try :
                PATH = [] ; j = t
                while j != s :      # previously was "is"... that created problems non-deterministically
                    e = upstream[j]
                    i,_ = rgraph.endpoints(e)
                    PATH.insert( 0, e )
                    j = i
                    
            except Exception as e :
                e.rgraph = rgraph
                e.redcost = redcost
                e.s = s
                
                e.path_so_far = PATH
                e.j = j
                
                raise e
                    
            LOG.debug('using path: %s' % repr( PATH ))
            
            # augment Delta flow along the path P
            for e,dir in PATH :
                flow[e] += dir * Delta
                Excess( excess, flow, network, supply, edge=e )
                
                LinearizeCost( lincost, cost, flow, Delta, network, edge=e )    # all edges
                ResidualGraph( rgraph, flow, capacity, Delta, network, edge=e )
                
            # update the potentials; 
            # by connectivity, should touch *every* node
            for i in network.nodes() : potential[i] -= dist[i]
            
            # re-compute the reduced costs... everywhere? (all the potentials have changed)
            ReducedCost( redcost, lincost, potential, network )
            
            
        # at end of each stage, verify the optimality certificate (should be empty every time)
        CERT = { re : c for (re,c) in redcost.items() if re in rgraph.edges() and c < 0. }
        LOG.debug('certificate, end stage TWO: %s' % repr( CERT ))
        RELAXCERT = { re : c for (re,c) in redcost.items() if re in rgraph.edges() and c < -PHASE_ERROR }
        if len( RELAXCERT ) > 0 :
            LOG.debug(RELAXCERT)
            LOG.debug("STAGE TWO CERTIFICATE CORRUPT!")
        assert len( RELAXCERT ) <= 0
                    
        # end the phase
        if Delta <= epsilon : break
        Delta = Delta / 2

    return flow


# ---------------------------------------------------------------------------
# C++ backend for FragileMCCF
# ---------------------------------------------------------------------------

try:
    from setiptah.roadgeometry.matching._cpp import (
        fragile_mccf as _cpp_fragile_mccf,
        PiecewiseLinear as CppPiecewiseLinear,
    )
except ImportError:
    _cpp_fragile_mccf = None
    CppPiecewiseLinear = None

from .pwl import PWL as _PWL


# A cost function accepted by the C++ binding: a Python PWL,
# a CppPiecewiseLinear, or any Python callable float -> float.
CostEntry = _PWL | Callable[[float], float]


def _to_cpp_cost(fn: CostEntry) -> "CppPiecewiseLinear | Callable[[float], float]":
    """Normalise a cost entry for the C++ binding.

    - PWL                → CppPiecewiseLinear  (no Python callback at eval time)
    - CppPiecewiseLinear → pass through  (already the fast path)
    - any callable       → pass through  (wrapped as std::function in C++)
    """
    if isinstance(fn, _PWL):
        return CppPiecewiseLinear(fn.segments)
    return fn


def cpp_fragile_mccf(
    network: mygraph,
    capacity_in,
    supply: Mapping[TRoad, float],
    cost: Mapping[TRoad, CostEntry],
    U: float,
    epsilon: float = 1.0,
) -> dict[TRoad, float]:
    """Drop-in replacement for FragileMCCF backed by the C++ implementation.

    Normalizes `network` to int-indexed arrays, delegates to the C++ solver,
    then restores original edge keys.

    Same preconditions as FragileMCCF (conservative supply, strong connectivity).
    Raises ImportError if the C++ extension is not available.
    """
    if _cpp_fragile_mccf is None:
        raise ImportError("C++ extension not available; build setiptah-roadgeometry-matching-cpp")

    nodes = list(network.nodes())
    edges = list(network.edges())
    node_to_int = {n: i for i, n in enumerate(nodes)}

    out_edges_arr = [[] for _ in range(len(nodes))]
    endpoints_arr = []
    for ei, e in enumerate(edges):
        u, v = network.endpoints(e)
        out_edges_arr[node_to_int[u]].append(ei)
        endpoints_arr.append((node_to_int[u], node_to_int[v]))

    supply_arr = [supply.get(n, 0.0) for n in nodes]
    cost_arr   = [_to_cpp_cost(cost[e]) for e in edges]

    flow_arr = _cpp_fragile_mccf(
        out_edges_arr, endpoints_arr, supply_arr, cost_arr, U, epsilon
    )

    return {e: flow_arr[ei] for ei, e in enumerate(edges)}


def CppMinConvexCostFlow(network, capacity, supply, cost, U, epsilon=None):
    """MinConvexCostFlow using the C++ fragile_mccf solver.

    Wraps the input in MCCFRobustInstance (same as MinConvexCostFlow) then
    delegates the inner solve to cpp_fragile_mccf.
    """
    if epsilon is None:
        epsilon = 1

    network_aug, capacity_rename, cost_aug = MCCFRobustInstance(network, capacity, supply, cost, U)
    flow = cpp_fragile_mccf(network_aug, capacity_rename, supply, cost_aug, U, epsilon)
    return {e: x for (type_, e), x in flow.items() if type_ == ALGGLOBAL.REGULAR}
