"""Efficient bipartite matching on road networks (metric graphs).

"""
import dataclasses
import enum
from collections import defaultdict, deque
from dataclasses import dataclass
from enum import auto
from functools import cached_property
from typing import NamedTuple, Optional, Callable, Iterable

import bintrees  # Migrate to `sortedcontainers`?
import networkx as nx  # TODO: Migrate it out?
import numpy as np

from setiptah.roadgeometry.dijkstra import RoadnetMetric
from setiptah.roadgeometry.graphs import IntRoadnet, int_map_to_seq
from setiptah.roadgeometry.matching.nxopt.cvxcostflow import MinConvexCostFlow
from setiptah.roadgeometry.matching.protocol import FlowSolver

# Module-level default solver. Replace this to globally swap in a different
# implementation (e.g. the C++ backend) without changing call sites.
default_flow_solver: FlowSolver = MinConvexCostFlow
from setiptah.roadgeometry.matching.range_queues import *
from setiptah.roadgeometry.matching.util import inner_class
from setiptah.roadgeometry.matching.util.mygraph import mygraph
from setiptah.roadgeometry.protocol import *

T, U = TypeVar("T"), TypeVar("U")
Factory = Callable[[], T]


class PointInfo(NamedTuple, Generic[TRoad]):
    road_index: TRoad
    coordinate: float


@dataclass(frozen=True)
class RoadnetMatchingInstance(Generic[TRoad, TVert]):
    P: tuple[PointInfo[TRoad], ...]
    Q: tuple[PointInfo[TRoad], ...]
    roadnet: Roadnet[TRoad, TVert]

    @classmethod
    def normalize(cls, P_, Q_, roadnet_, seq_maps: bool = True):
        roadnet, road_map, vert_map  = IntRoadnet.normalize(roadnet_, False)

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


BasicPoint = tuple[TRoad, float]


class MatchingResult(enum.Enum):
    FLOW = auto()
    MATCHING = auto()
    COST = auto()


@dataclass(frozen=True)
class RoadnetMatchingProblem(Generic[TRoad, TVert]):
    P: tuple[BasicPoint[TRoad], ...]
    Q: tuple[BasicPoint[TRoad], ...]
    roadnet: Roadnet[TRoad, TVert]
    _: dataclasses.KW_ONLY
    use_ranges: bool = False
    flow_solver: FlowSolver = dataclasses.field(default_factory=lambda: default_flow_solver)

    def compute_optimal(self, result: MatchingResult):
        out, = self.compute_optimal_results(result)
        return out

    def compute_optimal_results(self, *required: MatchingResult):
        alg = self.Algorithm()
        results = alg.run(*required)
        return tuple(results[req] for req in required)

    @inner_class
    class Algorithm:

        @property
        def instance(self) -> "RoadnetMatchingProblem":
            return self.__outer__

        def __init__(self):
            self.results = {}

            self.matching = []

        def run(self, *results: MatchingResult) -> dict:
            required = set(results)

            for step in [
                self._compute_optimal_flow,
                self._compute_matching,
            ]:
                step()
                if required.issubset(self.results):
                    break

            return self.results

        def _compute_optimal_flow(self):
            roadnet = self.instance.roadnet

            self.segment_dict = segment_dict = compute_segments2(self.instance.P, self.instance.Q, roadnet)

            surplus_dict = dict()
            measure_dict = dict()

            self.matching = matching = []
            for road, segment in segment_dict.items():
                matching_ = PREMATCH(segment)
                matching.extend(matching_)

                surplus_dict[road] = SURPLUS(segment)

                road_len = roadnet.length(road)
                measure = MEASURE(segment, road_len)
                measure_dict[road] = measure

            self.flow = flow = compute_optimal_flow(
                roadnet, surplus_dict, measure_dict,
                flow_solver=self.instance.flow_solver,
            )

            # TODO: Create unit test to detect infeasibility...
            imbalance = check_flow(flow, roadnet, surplus_dict)
            try:
                assert len(imbalance) <= 0
            except Exception as ex:
                ex.imbal = imbalance
                raise ex

            self.results[MatchingResult.FLOW] = flow

        def _compute_matching(self):
            assist, segment_dict, roadnet = self.flow, self.segment_dict, self.instance.roadnet

            if self.instance.use_ranges:
                matching, cost = compute_matching_for_acyclic_flow_with_ranges(
                    assist, segment_dict, roadnet
                )

            else:
                matching, cost = compute_matching_for_acyclic_flow(assist, segment_dict, roadnet)

            self.matching.extend(matching)
            self.results[MatchingResult.MATCHING] = self.matching
            self.results[MatchingResult.COST] = cost


""" ALGORITHM SUB-ROUTINES """


""" Phase I: Transcription """


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

    def map(self, fn: Callable[[T], U]) -> "BiPartite[U]":
        return self.__class__(fn(self.supply), fn(self.demand))


Segment = Iterable[tuple[float, BiPartite[list[int]]]]


def compute_segments2(P, Q, roadnet: Roadnet[TRoad, TVert]) -> dict[TRoad, Segment]:
    """

    Returns:
        a dictionary of segment data structures for each road

    """
    tree = sort_points(P, Q)

    segments = {road: deque() for road in roadnet.edges()}

    prev_road, segment = None, None
    for key, qs in tree.iter_items():
        road, y = key
        if segment is None or road != prev_road:
            assert road in roadnet.edges(), (road, roadnet.edges())
            segment = segments[road]
            prev_road = road
        segment.append((y, qs))

    return segments


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


def PREMATCH(segment: Segment) -> list[tuple[int, int]]:
    match = []
    for y, q in segment:
        annih = min(len(q.supply), len(q.demand))
        for k in range(annih):
            # This is why we start with deque
            i = q.supply.pop(0)
            j = q.demand.pop(0)
            match.append((i, j))

    return match


def SURPLUS(segment: Segment) -> int:
    deltas = [len(q.supply) - len(q.demand) for y, q in segment]
    return sum(deltas)


def MEASURE(segment: Segment, length: float, rbound=None):
    if rbound is not None:
        lbound = length
    else:
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

    intervals = zip(posts[:-1], posts[1:])
    F = np.cumsum(deltas)

    for (a, b), f in zip(intervals, F):
        measure.setdefault(f, 0.)
        measure[f] += b - a

    return measure


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


def compile_index_ranges(segment_dict: dict[TRoad, Segment]) -> dict[TRoad, MySegment]:
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


""" Phase II: Transformation/Solution/Verification """


def compute_optimal_flow(
        roadnet: Roadnet[TRoad, TVert],
        surplus: dict[TVert, float],
        measure_dict: bintrees.RBTree,  # float -> float
        flow_solver: FlowSolver = None,
) -> dict[TRoad, float]:
    if flow_solver is None:
        flow_solver = default_flow_solver
    network = mygraph()
    supply = {i: 0. for i in roadnet.nodes()}
    cost = {}  # functions
    #
    oneway_offset = {}  # for one-way roads

    for road in roadnet.edges():
        i, j = roadnet.endpoints(road)

        supply[j] += surplus[road]
        measure = measure_dict[road]

        fobj = OBJECTIVE_FUNC(measure)

        # edge construction
        if roadnet.is_oneway(road):
            # if one-way road

            # record minimum allowable flow on road
            zmin = -measure.min_key()  # i.e., z + min key of measure >= 0
            oneway_offset[road] = zmin
            # create a 'bias point'
            supply[i] -= zmin
            supply[j] += zmin

            # shift and record the cost function on only a forward edge
            fobj_offset = offsetWrapper(fobj, zmin)
            network.add_edge(road, i, j)
            cost[road] = fobj_offset

        else:
            # if bi-directional road... instantiate pair of edges
            # cc = roadbm.costWrapper( cost_data )
            n_fobj = negativeWrapper(fobj)  # won't have to worry about the C(0) offset

            network.add_edge((road, +1), i, j)
            cost[(road, +1)] = fobj
            #
            network.add_edge((road, -1), j, i)
            cost[(road, -1)] = n_fobj

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
    # should be fairly tight. the +1 at the end is to accommodate an empty matching.
    U = sum(len(m) - 1 for m in measure_dict.values()) + 1

    f = flow_solver(network, {}, supply, cost, U)

    flow = {}
    for road in roadnet.edges():
        if road in oneway_offset:
            flow[road] = f[road] + oneway_offset[road]
        else:
            flow[road] = f[(road, +1)] - f[(road, -1)]

        flow[road] = int(flow[road])

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


def OBJECTIVE_FUNC(measure):
    """ produces the convex objective function assoc. with a set of interval measures """
    return costWrapper(OBJECTIVE(measure))


def OBJECTIVE(measure):
    """
    produces the objective LineData()s RBTree arrangement
    given the dictionary of interval measures;
    N levels => N+1 LineData()s (verify?)
    """

    def sweep(x):
        Xminus = np.cumsum(x)
        total = Xminus[-1]
        Xplus = total - Xminus
        X = Xplus - Xminus
        return X

    # prepare constants kappa and alpha
    PREALPHA = np.array([0.] + [w for f, w in measure.items()])
    ALPHA = sweep(PREALPHA)

    PREKAPPA = np.array([0.] + [f * w for f, w in measure.items()])
    KAPPA = sweep(PREKAPPA)

    Cz = bintrees.RBTree()
    ff = [f for f in measure] + [np.inf]  # should be in order
    for f, alpha, kappa in zip(ff, ALPHA, KAPPA):
        Cz.insert(-f, LineData(alpha, kappa))

    return Cz


@dataclass(frozen=True, slots=True)
class LineData:
    slope: float
    offset: float

    def __call__(self, x: float):
        return self.slope * x + self.offset

    def __repr__(self):
        return f"<{self.slope} z + {self.offset}>"


def write_objectives(P, Q, roadnet: Roadnet):
    segment_dict = compute_segments2(P, Q, roadnet)
    surplus_dict = dict()
    objective_dict = dict()

    for road, segment in segment_dict.items():
        _match = PREMATCH(segment)

        surplus_dict[road] = SURPLUS(segment)

        measure = MEASURE(segment, roadnet.length(road))
        objective_dict[road] = OBJECTIVE(measure)

    return objective_dict


def compute_roadnet_objective_fns(P, Q, roadnet: Roadnet[TRoad, TVert]) -> dict[TRoad, "classWrapper"]:
    return {
        road: costWrapper(lines)
        for road, lines in write_objectives(P, Q, roadnet).items()
    }


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
        special[u] = terminal(None, ref=u)

    for road in roadnet.edges():
        u, v = roadnet.endpoints(road)
        segment = segment_dict[road]

        h = assist[road]
        prev_node, prev_y = special[u], 0.
        for curr_y, qs in segment:
            curr_node = terminal(qs, ref=road)
            add_edge(prev_node, curr_node, h, curr_y - prev_y)
            h += len(qs.supply) - len(qs.demand)
            prev_node, prev_y = curr_node, curr_y
        add_edge(prev_node, special[v], h, roadnet.length(road) - prev_y)

    return topograph


def CHECKTOPO( topograph ) :
    """

    TODO: Good to test me?

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


def compute_matching_for_acyclic_flow(
        flow: dict[TRoad, int],
        segment_dict: dict[TRoad, Segment],
        roadnet: Roadnet[TRoad, TVert],
):
    """

    For _any_ acylclic flow, not _just_ an optimal one.

    """
    topograph = create_topograph(segment_dict, flow, roadnet)
    return TRAVERSE2(topograph)


def compute_matching_for_acyclic_flow_with_ranges(
        flow: dict[TRoad, int],
        segment_dict: dict[TRoad, Segment],
        roadnet: Roadnet[TRoad, TVert],
):
    ranges_segment_dict: dict[TRoad, MySegment] = compile_index_ranges(segment_dict)
    topograph = create_topograph2(ranges_segment_dict, flow, roadnet)

    match_, cost = TRAVERSE3(topograph)
    # assert False, (ranges_segment_dict, match_)

    # TODO: Better to tuck this into a data structure somewhere so it is done by TRAVERSE3.
    match = [
        (
            ranges_segment_dict[road1].points.supply[i],
            ranges_segment_dict[road2].points.demand[j],
        )
        for (road1, i), (road2, j) in match_
    ]

    return match, cost


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

            # If L was a deque, it is no longer.
            # Also, the split can take O(n) time.
            # TODO: Enter traversing with range queues...
            prefix, L = L[:w], L[w:]

            LISTS[v].extend(prefix)

            l = data["length"]
            cost += w * l

    return matching, cost


class terminal :    # simple node type for TRAVERSE
    def __init__(self, q, ref=None):
        self.q   = q
        self.ref = ref

    def __repr__(self):
        if self.ref is not None:
            return f"terminal({self.q}, ref={self.ref!r})"
        return f"terminal({self.q})"


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



""" Misc. Algorithm Utilities """


@dataclass(frozen=True)
class MatchingInstance:
    """

    TODO: Fix confusing naming.

    """
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


def matching_cost(matching: tuple[int, int], P, Q, roadnet: Roadnet):
    return sum(match_costs(matching, P, Q, roadnet))


def match_costs(matching: tuple[int, int], P, Q, roadnet: Roadnet) -> list[float]:
    metric = RoadnetMetric(roadnet)
    inst = MatchingInstance(P, Q, metric)
    return [
        inst.match_cost(match)
        for match in matching
    ]
