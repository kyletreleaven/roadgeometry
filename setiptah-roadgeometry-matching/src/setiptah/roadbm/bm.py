"""Efficient bipartite matching on road networks (metric graphs).

"""
import dataclasses
from collections import defaultdict, deque
from dataclasses import dataclass
from functools import cached_property
from typing import NamedTuple, Optional, Callable, Sized

import bintrees  # Migrate to `sortedcontainers`?
import networkx as nx  # TODO: Migrate it out?
import numpy as np

from setiptah.basic_graph.graphs import RoadInfo
from setiptah.basic_graph.mygraph import mygraph
from setiptah.basic_graph.protocol import *
from setiptah.nxopt.cvxcostflow import MinConvexCostFlow
from ..basic_graph.dijkstra import RoadnetMetric

T = TypeVar("T")
Factory = Callable[[], T]


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


def optimal_roadnet_matching(P, Q, roadnet: Roadnet, **kwargs):
    """

    TODO: Do we need this?

    """
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

    # TODO: Create unit test to detect infeasibility...
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

Segment = list[tuple[float, "BiPartite[list[int]]"]]


@dataclass(frozen=True)
class IndexRange:
    start: int
    end: int

    def __post_init__(self):
        assert self.start <= self.end

    def __len__(self):
        return self.end - self.start

    def on_road(self, road: TRoad, reverse: bool = False):
        return RoadPointSeq(road, self.start, self.end, reverse=reverse)


@dataclass(frozen=True)
class MySegment:
    points: "BiPartite[list[int]]"
    """List of integers (point indices) per side."""

    events: list[tuple[float, "BiPartite[IndexRange]"]]
    """List of events.
    
    TODO: Is it better to separate these?
    
    """

    @classmethod
    def create(cls):
        return cls(BiPartite.create_with(list), [])


def compute_segments3(P, Q, roadnet: Roadnet[TRoad, TVert]) -> dict[TRoad, MySegment]:
    segments = {}

    tree = sort_points(P, Q)

    prev_road, segment = None, None
    for key, qs in tree.iter_items():
        road, y = key
        if segment is None or road != prev_road:
            assert road not in segments
            assert road in roadnet.edges()

            segments[road] = segment = MySegment.create()

            prev_road = road

        ns = segment.points.map(len)
        ii = BiPartite(
            IndexRange(ns.supply, ns.supply + len(qs.supply)),
            IndexRange(ns.demand, ns.demand + len(qs.demand)),
        )
        segment.events.append((y, ii))

        segment.points.supply.extend(qs.supply)
        segment.points.demand.extend(qs.demand)

    return segments


def create_point_map(segment_dict: dict[TRoad, Segment]) -> dict[TRoad, MySegment]:
    out = {}
    for road, seg_in in segment_dict.items():
        out[road] = seg_out = MySegment.create()
        for y, qs in seg_in:
            ns = seg_out.points.map(len)
            ii = BiPartite(
                IndexRange(ns.supply, ns.supply + len(qs.supply)),
                IndexRange(ns.demand, ns.demand + len(qs.demand)),
            )
            seg_out.events.append((y, ii))
            seg_out.points.supply.extend(qs.supply)
            seg_out.points.demand.extend(qs.demand)

    return out


def compute_segments2(P, Q, roadnet: Roadnet[TRoad, TVert]) -> dict[TRoad, Segment]:
    """

    Returns:
        a dictionary of segment data structures for each road

    """
    tree = sort_points(P, Q)

    segments = {}
    prev_road, segment = None, None
    for key, qs in tree.iter_items():
        road, y = key
        if segment is None or road != prev_road:
            assert road not in segments
            assert road in roadnet.edges()

            segments[road] = segment = []

            prev_road = road

        segment.append((y, qs))

    return segments

    
def PREMATCH(segment: Segment) -> list[tuple[int, int]]:
    match = []
    for y, q in segment:
        annih = min( len( q.supply ), len( q.demand ) )
        for k in range( annih ) :
            # TODO: Shoot, do these need to be deques?
            i = q.supply.pop(0)
            j = q.demand.pop(0)
            match.append( (i,j) )
            
    return match


def SURPLUS(segment: "BiPartite[Sized]") -> int:
    deltas = [len( q.supply ) - len( q.demand ) for y,q in segment]
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
        deltas.append(len(q.supply) - len(q.demand))
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


def compute_optimal_flow(
        roadnet: Roadnet[TRoad, TVert],
        surplus: dict[TVert, float],
        measure_dict: bintrees.RBTree,  # float -> float
) -> dict[TRoad, float]:
    network = mygraph()
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


def create_topograph2(
        segment_dict: dict[TRoad, MySegment], assist: dict[TRoad, float], roadnet: Roadnet
) -> nx.DiGraph:

    topograph = nx.DiGraph()

    def add_edge(u, v, h, l, r):
        if h > 0:
            topograph.add_edge(u, v, weight=h, length=l, road=r, reverse=False)
        if h < 0:
            topograph.add_edge(v, u, weight=-h, length=l, road=r, reverse=True)

    special = dict()
    for u in roadnet.nodes():
        special[u] = terminal(None)

    for road in roadnet.edges():
        u, v = roadnet.endpoints(road)

        # TODO: Copy to prevent consumption?
        events = segment_dict[road].events

        h = assist[road]
        prev_node, prev_y = special[u], 0.
        for curr_y, contents in events:
            curr_node = terminal(contents)
            add_edge(prev_node, curr_node, h, curr_y - prev_y, road)
            h += len(contents.supply) - len(contents.demand)
            prev_node, prev_y = curr_node, curr_y
        add_edge(prev_node, special[v], h, roadnet.length(road) - prev_y, road)

    return topograph


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
            h += len(qs.supply) - len(qs.demand)
            prev_node, prev_y = curr_node, curr_y
        add_edge(prev_node, special[v], h, roadnet.length(road) - prev_y)

    return topograph


def CHECKTOPO( topograph ) :
    """

    TODO: Good to test me.

    """
    def balance( u ) :
        # starting balance
        q = u.q
        if q is None :
            b = 0
        else :
            b = len( q.supply ) - len( q.demand )
            
        # plus input
        for e in topograph.in_edges( u ) :
            b += topograph.get_edge_data( *e ).get('weight')
            
        # minus output
        for e in topograph.out_edges( u ) :
            b -= topograph.get_edge_data( *e ).get('weight')
            
        return b
            
    return [ u for u in topograph.nodes() if balance(u) != 0 ]


def TRAVERSE2(topograph: nx.DiGraph):
    """

    This version of graph traversal passes lists around,
    and so does _not_ have O(1) push/pop.

    It is, however, conceptually much simpler.

    """
    matching, cost = [], 0.

    nodes_ord = nx.topological_sort(topograph)

    LISTS = defaultdict(list)

    for u in nodes_ord:
        L = LISTS[u]
        LISTS.pop(u)  # not needed anymore

        queue = u.q
        if queue is not None:
            # collect points from S
            L.extend(queue.supply)

            # dispatch points in T
            for j in queue.demand:
                i = L.pop(0)
                matching.append((i, j))

        for _, v, data in topograph.out_edges(u, data=True):
            w = data.get('weight')

            prefix, L = L[:w], L[w:]

            LISTS[v].extend(prefix)

            l = data["length"]
            cost += w * l

    return matching, cost


def TRAVERSE3(topograph: nx.DiGraph):
    """

    This version of graph traversal passes around "index range queues".

    An index range queue can represent certain collections of points
    (i.e., a collection of either supply or demand points
    having contiguous indices on a given road)
    in O(1) space for O(1) push/pop operations per range in the queue.

    """
    matching, cost = [], 0.

    nodes_ord = nx.topological_sort( topograph )

    node_imports: dict[TVert, PointSeqQ[TRoad]] = defaultdict(deque)

    for u in nodes_ord:
        urq = node_imports[u]
        ulocal: Optional[BiPartite[IndexRange]] = u.q

        for _, v, data in topograph.out_edges(u, data=True):
            road, reverse = data["road"], data["reverse"]
            l, w = data["length"], data["weight"]

            cost += w * l  # count our chickens

            vrq = node_imports[v]
            vlocal: Optional[BiPartite[IndexRange]] = v.q

            # Send order: local RQ, local supply
            # Target order: dest demand, dest RQ

            # Promote local supply as needed. There _must_ be `w` available for edge.
            w_ = num_points(urq)
            if w_ < w:
                promote, rem = ulocal.supply.on_road(road, reverse=reverse).split(w - w_)
                append_range(urq, promote)
                ulocal.supply = IndexRange(rem.start, rem.end)

            sending = take_points(urq, w)

            # Distribute...

            # Match with demand at destination first.
            n_local_demand = (len(vlocal.demand) if vlocal is not None else 0)
            w_ = min(n_local_demand, w)
            for _ in range(w_):
                p = pop_point(sending)
                q_, rem = vlocal.demand.on_road(road, reverse=reverse).split(1)
                matching.append((p, q_.as_point()))
                vlocal.demand = IndexRange(rem.start, rem.end)

            # Remainder to destination RQ.
            extend_points(vrq, sending)

    # Finally, match local demand: from RQ first, then local supply.
    demand, ulocal.demand = ulocal.demand, None
    while len(urq) > 0:
        p = pop_point(urq)
        q_, rem = demand.on_road(road, reverse=reverse).split(1)
        demand = IndexRange(rem.start, rem.end)
        matching.append((p, q_.as_point()))

    supply, ulocal.supply = ulocal.supply, None
    while len(supply) > 0:
        p_, rem = supply.on_road(road, reverse=reverse).split(1)
        supply = IndexRange(rem.start, rem.end)

        q_, rem = demand.on_road(road, reverse=reverse).split(1)
        demand = IndexRange(rem.start, rem.end)

        matching.append((p_.as_point(), q_.as_point()))

    return matching, cost


@dataclass(frozen=True)
class RoadPointSeq(Generic[TRoad]):
    road: TRoad
    start: int
    end: int
    reverse: bool = False

    def __post_init__(self):
        assert self.start <= self.end

    def __len__(self):
        return self.end - self.start

    def as_point(self):
        assert len(self) == 1
        return self.road, self.start

    def split(self, n: int) -> tuple["RoadPointSeq", "RoadPointSeq"]:
        assert n <= len(self)

        if self.reverse:
            rem = dataclasses.replace(self, end=self.end - n)
            taken = dataclasses.replace(self, start=self.end - n)
        else:
            taken = dataclasses.replace(self, end=self.start + n)
            rem = dataclasses.replace(self, start=self.start + n)

        return taken, rem

    def can_cat(self, other: "RoadPointSeq[TRoad]") -> bool:
        if other.road != self.road:
            return False

        if self.reverse:
            return other.reverse and other.end == self.start
        else:
            return not other.reverse and other.start == self.end

    def cat(self, other):
        assert self.can_cat(other)
        if self.reverse:
            return dataclasses.replace(self, start=other.start)
        else:
            return dataclasses.replace(self, end=other.end)


PointSeqQ = deque[RoadPointSeq[TRoad]]


def num_points(rq: PointSeqQ):
    return sum(len(r) for r in rq)


def extend_points(ps: PointSeqQ[TRoad], qs: PointSeqQ[TRoad]):
    for r in qs:
        append_range(ps, r)


def append_range(point_seq: PointSeqQ[TRoad], points: RoadPointSeq[TRoad]):
    if len(point_seq) > 0 and point_seq[-1].can_cat(points):
        point_seq[-1] = point_seq[-1].cat(points)
    else:
        point_seq.append(points)


def take_points(point_seq: PointSeqQ[TRoad], n: int) -> PointSeqQ[TRoad]:
    assert n <= sum(len(r) for r in point_seq)

    out = deque()
    while n > 0:
        front = point_seq.popleft()
        n_ = min(n, len(front))
        taken, rem = front.split(n_)
        out.append(taken)
        if len(rem) > 0:
            point_seq.appendleft(rem)  # TODO: Optimize?
        n -= n_

    return out


def pop_point(point_seq: PointSeqQ[TRoad]) -> tuple[TRoad, int]:
    pseq, = take_points(point_seq, 1)
    return pseq.as_point()









""" Misc. Algorithm Utilities """


@dataclass
class BiPartite(Generic[T]):
    supply: T
    demand: T

    def __repr__(self) :
        return f"<S:{self.supply},D:{self.demand}>"

    @classmethod
    def create_with(cls, factory: Factory[T]):
        return cls(factory(), factory())

    @classmethod
    def factory(cls, factory: Factory[T]) -> Factory["BiPartite[T]"]:

        def fn():
            return cls.create_with(factory)

        return fn

    def map(self, fn):
        return self.__class__(fn(self.supply), fn(self.demand))


def sort_points(P, Q):
    tree = bintrees.RBTree()

    for i, p in enumerate(P):
        key = tuple(p); _r, _y = key
        queues = ensure_key(key, tree)
        queues.supply.append(i)

    for j, q in enumerate(Q):
        key = tuple(q); _r, _y = key
        queues = ensure_key(key, tree)
        queues.demand.append(j)

    return tree


def ensure_key( key, tree ) :
    curr = tree.set_default( key )
    if curr is None : tree[key] = BiPartite.create_with(list)
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


@dataclass(frozen=True)
class MatchingInstance:
    P: tuple[Roadnet.Point, ...]
    Q: tuple[Roadnet.Point, ...]
    roadnet_metric: RoadnetMetric

    def match_cost(self, match: tuple[int, int]) -> float:
        i, j = match
        return self.roadnet_metric.distance(self.P[i], self.Q[j])


def flow_cost_per_road(flow: dict[TRoad, float], obj_dict):
    return {
        road: obj_dict[road](x)
        for road, x in flow.items()
    }
