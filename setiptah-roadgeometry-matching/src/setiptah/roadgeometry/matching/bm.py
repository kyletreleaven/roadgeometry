"""Efficient bipartite matching on road networks (metric graphs).

"""
import dataclasses
import enum
import math
from collections import defaultdict, deque
from dataclasses import dataclass
from enum import auto
from functools import cached_property
from typing import NamedTuple, Optional, Callable, Iterable, Iterator, Protocol

import bintrees  # Migrate to `sortedcontainers`?
import networkx as nx  # TODO: Migrate it out?
import numpy as np

from setiptah.roadgeometry.dijkstra import RoadnetMetric
from setiptah.roadgeometry.graphs import IntRoadnet, int_map_to_seq
from setiptah.roadgeometry.matching.nxopt.cvxcostflow import MinConvexCostFlow, CppRobustMinConvexCostFlow

try:
    from setiptah.roadgeometry.matching._cpp import compute_optimal_flow as _cpp_compute_optimal_flow
except ImportError:
    _cpp_compute_optimal_flow = None
from setiptah.roadgeometry.matching.util.double_ended_vector import DoubleEndedVector
from setiptah.roadgeometry.matching.nxopt.pwl import PWL, IntPWL, negate as pwl_negate, shift as pwl_shift
from setiptah.roadgeometry.matching.protocol import FlowSolver
try:
    from setiptah.roadgeometry.matching import _cpp
except ImportError:
    _cpp = None

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


Segment = Iterable[tuple[float, BiPartite[deque[int]]]]


class SegmentSorter(Protocol[TRoad, TVert]):
    """Callable that segments supply/demand points by road."""
    def __call__(
        self,
        P: tuple[BasicPoint[TRoad], ...],
        Q: tuple[BasicPoint[TRoad], ...],
        roadnet: Roadnet[TRoad, TVert],
    ) -> dict[TRoad, Segment]: ...


# Module-level defaults. Replace to globally swap implementations.
def default_flow_solver(network, capacity, supply, cost, U, epsilon=None):
    try:
        return CppRobustMinConvexCostFlow(network, capacity, supply, cost, U, epsilon)
    except ImportError:
        return MinConvexCostFlow(network, capacity, supply, cost, U)

def default_compute_segments(P, Q, roadnet):
    if _cpp is not None:
        return _cpp.sort_and_segment(P, Q, list(roadnet.edges()))
    return compute_segments2(P, Q, roadnet)


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
    compute_segments: SegmentSorter = dataclasses.field(default_factory=lambda: default_compute_segments)

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

            self.segment_dict = segment_dict = self.instance.compute_segments(
                self.instance.P,
                self.instance.Q,
                roadnet
            )

            self.matching = matching = []
            for road, segment in segment_dict.items():
                matching_ = PREMATCH(segment)
                matching.extend(matching_)

            self.flow = flow = flow_from_segments(
                segment_dict, roadnet,
                flow_solver=self.instance.flow_solver,
            )

            if __debug__:
                surplus_dict = {road: SURPLUS(seg) for road, seg in segment_dict.items()}
                imbalance = check_flow(flow, roadnet, surplus_dict)
                assert len(imbalance) <= 0, imbalance

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




class Measure(Protocol):
    """Read interface required of a road measure by OBJECTIVE and compute_optimal_flow."""

    def __len__(self) -> int:
        """Number of allocated flow levels."""
        ...

    def __iter__(self) -> Iterator[int]:
        """Yield flow levels in ascending order."""
        ...

    def items(self) -> Iterator[tuple[int, float]]:
        """Yield (flow_level, interval_length) pairs in ascending flow_level order."""
        ...

    @property
    def min_index(self) -> int:
        """The minimum flow level with nonzero measure."""
        ...


_SUPPLY, _DEMAND = 0, 1


def sort_points2(P, Q):
    """Return points from P and Q sorted by (road, y, side, index).

    Each entry is (road, y, side, index) where side=_SUPPLY(0) or _DEMAND(1).
    Lexicographic sort gives supply before demand at the same position, and
    ascending indices within each side.
    """
    points = []
    points.extend((r, y, _SUPPLY, i) for i, p in enumerate(P) for r, y in (tuple(p),))
    points.extend((r, y, _DEMAND, j) for j, q in enumerate(Q) for r, y in (tuple(q),))
    points.sort()
    return points


def compute_segments2(P, Q, roadnet: Roadnet[TRoad, TVert]) -> dict[TRoad, Segment]:
    """

    Returns:
        a dictionary of segment data structures for each road

    """
    points = sort_points2(P, Q)

    segments = {road: [] for road in roadnet.edges()}

    # Linear groupby pass: group consecutive entries at the same (road, y).
    i = 0
    while i < len(points):
        road, y = points[i][0], points[i][1]
        assert road in roadnet.edges(), (road, roadnet.edges())
        qs = BiPartite.create_with(deque)
        while i < len(points) and points[i][0] == road and points[i][1] == y:
            _, _, side, idx = points[i]
            if side == _SUPPLY:
                qs.supply.append(idx)
            else:
                qs.demand.append(idx)
            i += 1
        segments[road].append((y, qs))

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
    if curr is None : tree[key] = BiPartite.create_with(deque)
    return tree[key]


def PREMATCH(segment: Segment) -> list[tuple[int, int]]:
    match = []
    for y, q in segment:
        annih = min(len(q.supply), len(q.demand))
        for k in range(annih):
            i = q.supply.popleft()
            j = q.demand.popleft()
            match.append((i, j))

    return match


def SURPLUS(segment: Segment) -> int:
    deltas = [len(q.supply) - len(q.demand) for y, q in segment]
    return sum(deltas)


def MEASURE(segment: Segment, length: float, rbound: float | None = None) -> DoubleEndedVector:
    if rbound is not None:
        lbound, rbound = length, rbound
    else:
        lbound, rbound = 0., length
    measure = DoubleEndedVector()
    f = 0
    prev_y = lbound
    for y, q in segment:
        measure[f] += y - prev_y
        f += len(q.supply) - len(q.demand)
        prev_y = y
    measure[f] += rbound - prev_y
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


def _pwl_from_objective(lines: bintrees.RBTree) -> PWL:
    """Convert an OBJECTIVE RBTree to a PWL.

    The RBTree keys are the left boundaries of each segment directly (breakpoints
    in flow space).  They are already in ascending order.  The first key is -inf.
    """
    segments = [(-math.inf if i == 0 else float(k), line.slope, line.offset)
                for i, (k, line) in enumerate(lines.items())]
    return PWL(segments)


def flow_from_segments(
        segments: dict,
        roadnet: 'Roadnet[TRoad, TVert]',
        flow_solver: 'FlowSolver' = None,
) -> dict:
    """Compute optimal flow from pre-matched segments.

    Uses the C++ implementation when available (flow_solver is ignored in that
    case).  Falls back to the Python path, which derives surplus and measure
    from segments and delegates to flow_solver.
    """
    if _cpp_compute_optimal_flow is not None and flow_solver is None:
        endpoints = {road: roadnet.endpoints(road) for road in segments}
        lengths   = {road: roadnet.length(road)    for road in segments}
        is_oneway = {road: roadnet.is_oneway(road) for road in segments}
        return _cpp_compute_optimal_flow(segments, endpoints, lengths, is_oneway)

    surplus_dict = {road: SURPLUS(seg) for road, seg in segments.items()}
    measure_dict = {road: MEASURE(seg, roadnet.length(road)) for road, seg in segments.items()}
    return compute_optimal_flow(roadnet, surplus_dict, measure_dict, flow_solver=flow_solver)


def compute_optimal_flow(
        roadnet: Roadnet[TRoad, TVert],
        surplus: dict[TVert, float],
        measure_dict: dict[TRoad, Measure],
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
            zmin = -measure.min_index  # i.e., z + min index of measure >= 0
            oneway_offset[road] = zmin
            # create a 'bias point'
            supply[i] -= zmin
            supply[j] += zmin

            network.add_edge(road, i, j)
            cost[road] = pwl_shift(fobj, zmin)

        else:
            # if bi-directional road... instantiate pair of edges
            network.add_edge((road, +1), i, j)
            cost[(road, +1)] = fobj
            #
            network.add_edge((road, -1), j, i)
            cost[(road, -1)] = pwl_negate(fobj)

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
    produces the objective LineData()s arrangement given the interval measures.
    N levels => N+1 segments.

    If measure is a DoubleEndedVector (integer flow levels), returns an IntPWL
    with O(1) evaluation.  Otherwise returns a bintrees.RBTree for the general case.
    TODO: always return a callable — fold the RBTree path into costWrapper or similar,
    eliminating the _pwl_from_objective helper used in tests.
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

    if isinstance(measure, DoubleEndedVector):
        ff = list(measure)  # ascending integer flow levels
        assert ff, "measure must not be empty"
        max_f = ff[-1]
        offset = -(max_f + 1)
        slopes     = list(ALPHA[::-1])
        intercepts = list(KAPPA[::-1])

        return _IntPWLWithLines(offset, slopes, intercepts)

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


class _IntPWLWithLines(IntPWL):
    """IntPWL subclass exposing a .lines shim for legacy call sites.

    .lines returns self so that obj_fn.lines.keys() and
    obj_fn.lines.floor_item() work the same way they do on a costWrapper
    wrapping an RBTree.
    """

    @property
    def lines(self):
        return self

    def keys(self):
        return range(self.offset, self.offset + self.size)

    def floor_item(self, z):
        i = max(0, min(self.size - 1, int(math.floor(z)) - self.offset))
        return self.offset + i, LineData(self.slopes[i], self.intercepts[i])


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
    wrap an RBTree arrangement of LineData()s to obtain a piece-wise linear callable function.

    If called with something that is already callable (e.g. IntPWL), returns it unchanged —
    no wrapper is constructed.
    """
    def __new__(cls, lines):
        if not isinstance(lines, bintrees.RBTree):
            if not callable(lines):
                raise TypeError(f"costWrapper requires an RBTree or callable, got {type(lines)}")
            return lines
        return super().__new__(cls)

    def __init__(self, lines ) :
        if not isinstance(lines, bintrees.RBTree):
            return
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

        if prev_node is special[u]:
            # No points on this road. Without an intermediate node the edge would
            # go directly special[u] → special[v], which collides in the DiGraph
            # with any other pointless road sharing the same endpoint pair.
            # Inject a road-specific stub node to keep each road's edges distinct.
            prev_node = terminal(BiPartite.create_with(list), ref=road)
            add_edge(special[u], prev_node, h, 0.)

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
