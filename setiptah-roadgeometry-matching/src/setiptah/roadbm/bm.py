"""Efficient bipartite matching on road networks (metric graphs).

"""
import dataclasses
from collections import defaultdict, deque
from dataclasses import dataclass
from functools import cached_property
from typing import NamedTuple

import bintrees  # Migrate to `sortedcontainers`?
import networkx as nx  # TODO: Migrate it out?
import numpy as np

from setiptah.basic_graph.graphs import RoadInfo
from setiptah.basic_graph.graphs import RoadNetwork
from setiptah.basic_graph.mygraph import mygraph
from setiptah.basic_graph.protocol import *
from setiptah.nxopt.cvxcostflow import MinConvexCostFlow
from ..basic_graph.dijkstra import RoadnetMetric

T = TypeVar("T")


@dataclass(frozen=True)
class _EdgeData(Generic[TRoad, TVert]):
    edge: TRoad
    length: float
    i: TVert
    j: TVert
    oneway: bool


@dataclass(frozen=True)
class _NodeData(Generic[TRoad, TVert]):
    out_edges: set[TRoad]
    in_edges: set[TRoad]


@dataclass
class MultiDiGraphRoadnet(RoadNetwork[TVert, TRoad]):
    graph: nx.MultiDiGraph

    def __post_init__(self):
        super().__init__()

        for i in self.graph.nodes:
            self.add_node(i)

        for i, j, road, data in self.graph.edges(keys=True, data=True):
            self.add_edge(road, i, j, data["length"], oneway=data.get("oneway", False))


@dataclass(frozen=True)
class StructRoadnet(Roadnet[int, int]):
    """A network on road and vertex indices (integers).

    TODO: Rename to do with "int array".

    """
    roads: tuple[RoadInfo[int], ...]
    n_vertices: int

    @classmethod
    def normalize(cls, rn: Roadnet, seq_maps: bool = True):

        vert_map = {u: k for k, u in enumerate(rn.nodes())}

        road_map = {}
        roads: list[RoadInfo] = []
        for r, road_ in enumerate(rn.edges()):
            road_map[road_] = r

            i_, j_ = rn.endpoints(road_)
            length = rn.length(road_)
            oneway = rn.is_oneway(road_)

            road_info = RoadInfo(length, vert_map[i_], vert_map[j_], oneway)
            roads.append(road_info)

        inst = cls(tuple(roads), len(vert_map))

        if seq_maps:
            road_map = int_map_to_seq(road_map)
            vert_map = int_map_to_seq(vert_map)

        return inst, road_map, vert_map

    def create_multigraph(self):
        g = nx.MultiDiGraph()
        g.add_nodes_from(self.nodes())
        for k, road_info in enumerate(self.roads):
            g.add_edge(road_info.left, road_info.right, k, length=road_info.length, oneway=road_info.oneway)
        return g

    def nodes(self) -> Collection[TVert]:
        return range(self.n_vertices)

    @property
    def n_roads(self) -> int:
        return len(self.roads)

    def edges(self) -> Collection[TRoad]:
        return range(self.n_roads)

    def is_valid(self):
        if self.n_vertices < 0:
            return False

        nodes = self.nodes()

        def road_is_valid(road: RoadInfo):
            return (
                road.length > 0.
                and road.left in nodes
                and road.right in nodes
            )

        if not all(road_is_valid(road) for road in self.roads):
            return False

        return True

    def length(self, road: int) -> float:
        return self.roads[road].length

    def endpoints(self, road: int) -> tuple[int, int]:
        road_info = self.roads[road]
        return road_info.left, road_info.right

    def is_oneway(self, road: TRoad) -> bool:
        return self.roads[road].oneway


def int_map_to_seq(dict_: dict[T, int]) -> list[T]:
    result = [None] * len(dict_)
    for i, k in dict_.items():
        result[k] = i
    return result


class PointInfo(NamedTuple):
    road_index: int
    coordinate: float


@dataclass(frozen=True)
class StructRoadnetMatchingInstance:
    P: tuple[PointInfo, ...]
    Q: tuple[PointInfo, ...]
    roadnet: StructRoadnet

    @classmethod
    def normalize(cls, P_, Q_, roadnet_, seq_maps: bool = True):
        roadnet, road_map, vert_map  = StructRoadnet.normalize(roadnet_, False)

        def make_points(points_) -> tuple[PointInfo, ...]:
            return tuple(
                PointInfo(road_map[r_], x)
                for r_, x in points_
            )

        P, Q = make_points(P_), make_points(Q_)
        inst = cls(P, Q, roadnet)

        if seq_maps:
            road_map = int_map_to_seq(road_map)
            vert_map = int_map_to_seq(vert_map)

        return inst, road_map, vert_map

    @cached_property
    def n_points(self) -> int:
        n, = len({len(self.P), len(self.Q)})
        return n

    def is_valid(self):
        if not self.roadnet.is_valid():
            return False

        def point_is_valid(point: PointInfo):
            try:
                road_length = self.roadnet.length(point.road_index)
            except IndexError:
                return False

            return 0 <= point.coordinate <= road_length

        if not all(point_is_valid(p) for p in (*self.P, *self.Q)):
            return False

        return True


def ROADSBIPARTITEMATCH( P, Q, roadnet_graph: nx.MultiDiGraph, **kwargs ) :
    return optimal_roadnet_matching(
        P, Q, MultiDiGraphRoadnet(roadnet_graph), **kwargs
    )


def optimal_roadnet_matching(P, Q, roadnet: Roadnet, **kwargs):
    matching, cost = optimal_roadnet_matching2(P, Q, roadnet, **kwargs)
    return matching


def optimal_roadnet_matching2(P, Q, roadnet: Roadnet, **kwargs):
    MATCH = []

    segment_dict = compute_segments2( P, Q, roadnet )
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
        match, cost = TRAVERSE2(topograph)
    except Exception as ex :
        ex.assist = assist
        ex.topograph = topograph
        raise ex
    
    MATCH.extend( match )
    return MATCH, cost


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


def SEGMENTS(P, Q, roadnet: nx.MultiDiGraph) -> dict[TRoad, OrderedPoints]:
    # TODO: Return RBTree inner here?
    return compute_segments(P, Q, MultiDiGraphRoadnet(roadnet))


def compute_segments(P, Q, roadnet: Roadnet[TRoad, TVert]) -> dict[TRoad, OrderedPoints]:
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


def compute_segments2(P, Q, roadnet: Roadnet[TRoad, TVert]) -> dict[TRoad, OrderedPoints]:
    """

    returns:
    a dictionary whose keys are coordinates and whose values are local (P,Q) index queues

    """
    tree = bintrees.RBTree()

    for i, p in enumerate(P):
        key = tuple(p); _r, _y = key
        queues = ensure_key(key, tree)
        queues.P.append(i)

    for j, q in enumerate(Q):
        key = tuple(q); _r, _y = key
        queues = ensure_key(key, tree)
        queues.Q.append(j)

    segments = {}
    prev_road, segment = None, None
    for key, qs in tree.iter_items():
        road, y = key
        if segment is None or road != prev_road:
            assert road not in segments
            assert road in roadnet.edges()

            segments[road] = segment = []
            # temporarily
            # segments[road] = segment = bintrees.RBTree()

            prev_road = road

        segment.append((y, qs))
        # segment[y] = qs

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
    for y, q in segment:
        annih = min( len( q.P ), len( q.Q ) )
        for k in range( annih ) :
            i = q.P.pop(0)
            j = q.Q.pop(0)
            match.append( (i,j) )
            
    return match


def SURPLUS(segment: OrderedPoints):
    deltas = [len( q.P ) - len( q.Q ) for y,q in segment]
    return sum( deltas )


def MEASURE(segment: "Segment", length: float, rbound=None):
    if rbound is not None :
        lbound = length
    else :
        lbound = 0.
        rbound = length
        
    # bintree instead of dict so that it is enumerated in sorted order
    # TODO: No, replace with a double-ended vector.
    measure = bintrees.RBTree()
    posts, deltas = [lbound], [0]
    for y, q in segment:
        posts.append(y)
        deltas.append(len(q.P) - len(q.Q))
    posts.append(rbound)

    intervals = zip( posts[:-1], posts[1:] )
    F = np.cumsum( deltas )
    
    for (a,b), f in zip( intervals, F ) :
        measure.setdefault( f, 0. )
        measure[f] += b - a
        
    return measure









""" Phase II: Transformation/Solution/Verification """


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
        surplus: dict[TVert, float],
        measure_dict: bintrees.RBTree,  # float -> float
) -> dict[TRoad, float]:
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
        flow: dict[TRoad, float],
        roadnet: nx.MultiDiGraph,
        surplus: dict[TVert, float]
) -> dict[TVert, float]:
    return check_flow(flow, MultiDiGraphRoadnet(roadnet), surplus)


def check_flow(
        flow: dict[TRoad, float],
        roadnet: Roadnet[TRoad, TVert],
        surplus: dict[TVert, float]
) -> dict[TVert, float]:
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
        segment_dict: dict[TRoad, OrderedPoints], assist: dict[TRoad, float], roadnet: nx.MultiDiGraph
) -> nx.DiGraph:
    return create_topograph(
        segment_dict, assist, MultiDiGraphRoadnet(roadnet)
    )


Segment = list[tuple[float, "TwoQueues"]]

def create_topograph(
        segment_dict: dict[TRoad, Segment], assist: dict[TRoad, float], roadnet: Roadnet
) -> nx.DiGraph:

    topograph = nx.DiGraph()

    def add_edge(u, v, h, l):
        if h > 0:
            topograph.add_edge(u, v, weight=h, length=l)
        if h < 0:
            topograph.add_edge(v, u, weight=-h, length=l)

    special = dict()
    for u in roadnet.nodes():
        special[u] = terminal(None)

    for road in roadnet.edges():
        u, v = roadnet.endpoints(road)
        segment = segment_dict[road]

        h = assist[road]
        prev_node, prev_y = special[u], 0.
        for curr_y, qs in segment:
            curr_node = terminal(qs)
            add_edge(prev_node, curr_node, h, curr_y - prev_y)
            h += len(qs.P) - len(qs.Q)
            prev_node, prev_y = curr_node, curr_y
        add_edge(prev_node, special[v], h, roadnet.length(road) - prev_y)

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


def TRAVERSE(topograph: nx.DiGraph):
    matching, cost = TRAVERSE2(topograph)
    return matching


def TRAVERSE2(topograph: nx.DiGraph):
    matching, cost = [], 0.

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
                matching.append((i, j))

        for _,v, data in topograph.out_edges( u, data=True ) :
            w = data.get('weight')

            prefix, L = L[:w], L[w:]  # TODO: Replace with range queue.

            LISTS[v].extend( prefix )

            l = data["length"]
            cost += w * l
            
    return matching, cost











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


@dataclass(frozen=True)
class MatchingInstance:
    P: tuple[Roadnet.Point, ...]
    Q: tuple[Roadnet.Point, ...]
    roadnet_metric: RoadnetMetric

    def match_cost(self, match: tuple[int, int]) -> float:
        i, j = match
        return self.roadnet_metric.distance(self.P[i], self.Q[j])


def MATCHCOSTS(matching: tuple[int, int], P, Q, roadnet: nx.MultiDiGraph):
    metric = RoadnetMetric(MultiDiGraphRoadnet(roadnet))
    inst = MatchingInstance(P, Q, metric)
    return [
        inst.match_cost(match)
        for match in matching
    ]


def ROADMATCHCOST( match, P, Q, roadnet ) :
    costs = MATCHCOSTS( match, P, Q, roadnet )
    return sum( costs )


def flow_cost_per_road(flow: dict[TRoad, float], obj_dict):
    return {
        road: obj_dict[road](x)
        for road, x in flow.items()
    }


@dataclass
class RoadPointSeq(Generic[TRoad]):
    road: TRoad
    start: int
    end: int

    def __post_init__(self):
        assert self.start <= self.end

    @classmethod
    def empty(cls, road: TRoad):
        return cls(road, 0, 0)

    def __len__(self):
        return self.end - self.start

    def grow(self, n: int = 1):
        self.end += n

    def take(self, n: int):
        assert n <= len(self)
        result = dataclasses.replace(self, end=self.start + n)
        self.start += n
        return result


def add_points(point_seq: deque[RoadPointSeq[TRoad]], road: TRoad, n: int):
    if len(point_seq) > 0:
        last = point_seq[-1]
        if last.road == road:
            last.grow(n)
            return

    point_seq.append(RoadPointSeq(road, 0, n))


def take_points(point_seq: deque[RoadPointSeq[TRoad]], n: int):
    assert n <= sum(len(r) for r in point_seq)

    out = deque()
    while n > 0:
        front = point_seq[0]
        n_ = min(n, len(front))
        out.append(front.take(n_))
        if len(front) <= 0:
            point_seq.popleft()
        n -= n_

    return out
