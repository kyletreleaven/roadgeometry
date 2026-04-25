import networkx as nx

import setiptah.roadgeometry.probability as roadprob
from setiptah.roadgeometry.formats import to_networkx, from_networkx
from setiptah.roadgeometry.matching.nx_legacy import *
from setiptah.roadgeometry.matching.bm import compute_segments2, default_compute_segments
from setiptah.roadgeometry.matching.nxopt.cvxcostflow import MinConvexCostFlow, CppMinConvexCostFlow, CppRobustMinConvexCostFlow

import pytest


def random_instance():

    roadnet = nx.MultiDiGraph()
    if True:
        roadnet.add_edge(0, 1, 'N', length=1.)
    else:
        # TODO: Do this in a meaningful way.
        # to test one-way roads capabilities
        roadnet.add_edge(0, 1, 'N', length=1., oneway=True)

    roadnet.add_edge(1, 2, 'E', length=1.)
    roadnet.add_edge(2, 3, 'S', length=1.)
    roadnet.add_edge(3, 0, 'W', length=1.)

    if True:
        roadnet.add_edge(0, 4, 'dangler', length=1.)

    roadnet_frfr = MultiDiGraphRoadnet(roadnet)
    sampler = roadprob.UniformDist(roadnet)

    NUMPOINT = 50
    #
    PP = [sampler.sample() for i in range(NUMPOINT)]
    QQ = [sampler.sample() for i in range(NUMPOINT)]

    return RoadnetMatchingProblem(PP, QQ, roadnet_frfr)


@pytest.mark.parametrize("instance_factory", [
    random_instance
])
def test_roadnet_matching(instance_factory):

    inst: RoadnetMatchingProblem = instance_factory()

    PP, QQ, roadnet = inst.P, inst.Q, inst.roadnet

    segment_dict = compute_segments2(PP, QQ, roadnet)
    # assert False, segment_dict

    # surplus
    surplus_dict = {
        road: SURPLUS(seg)
        for road, seg in segment_dict.items()
    }
    assert sum(surplus_dict.values()) == 0

    measure_dict = {
        road: MEASURE(seg, roadnet.length(road))
        for road, seg in segment_dict.items()
    }

    flow = compute_optimal_flow(roadnet, surplus_dict, measure_dict)

    # Good to check!
    imbalance = check_flow(flow, roadnet, surplus_dict)
    assert len(imbalance) <= 0

    topograph = create_topograph(segment_dict, flow, roadnet)
    # assert False, topograph.edges(data=True)

    matching_ref, cost_ref = TRAVERSE2(topograph)
    # assert False, match_ref

    # assert False, match_
    cost_ref_shortest_paths = matching_cost(matching_ref, PP, QQ, roadnet)

    # cost computed during matching construction
    matching, cost_ctd = RoadnetMatchingProblem(
        PP, QQ, roadnet
    ).compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)
    assert len(matching) == len(PP) == len(QQ)

    # sum shortest path lengths,
    cost_shortest_paths = matching_cost(matching, PP, QQ, roadnet)

    # objective fn cost of flow,
    obj_fn_dict = {
        road: OBJECTIVE_FUNC(measure)
        for road, measure in measure_dict.items()
    }

    costs_obj = flow_cost_per_road(flow, obj_fn_dict)
    cost_obj = sum(costs_obj.values())

    # limit spread
    costs = [
        cost_ref,
        cost_ref_shortest_paths,
        cost_ctd,
        cost_shortest_paths,
        cost_obj
    ]

    assert within_tolerance(costs), costs

    # TODO: Test vs. a third-party implementation on distance matrix; e.g.,
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.linear_sum_assignment.html
    # TODO: Add a fixed instance with a known optimal cost — e.g. a simple path graph with
    # hand-placed points whose matching and cost can be computed by inspection. This would
    # catch regressions in PREMATCH/OBJECTIVE/compute_optimal_flow that random instances may miss.


def test_index_range_equivalence():

    roadgraph = nx.MultiDiGraph()

    roadgraph.add_edge(0, 1, 'N', length=1., oneway=True)
    roadgraph.add_edge(1, 2, 'E', length=1.)
    roadgraph.add_edge(2, 3, 'S', length=1.)
    roadgraph.add_edge(3, 0, 'W', length=1.)
    roadgraph.add_edge(0, 4, 'dangler', length=1.)

    sampler = roadprob.UniformDist(roadgraph)

    NUMPOINT = 50
    PP = [sampler.sample() for i in range(NUMPOINT)]
    QQ = [sampler.sample() for i in range(NUMPOINT)]

    roadnet = from_networkx(roadgraph)

    matching, cost = RoadnetMatchingProblem(
        PP, QQ, roadnet, use_ranges=False
    ).compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)
    matching_from_ranges = RoadnetMatchingProblem(
        PP, QQ, roadnet, use_ranges=True
    ).compute_optimal(MatchingResult.MATCHING)
    # assert False, matching_from_ranges

    cost_shortest_path = ROADMATCHCOST(matching, PP, QQ, roadgraph)
    cost_from_ranges = ROADMATCHCOST(matching_from_ranges, PP, QQ, roadgraph)

    assert within_tolerance([cost, cost_shortest_path, cost_from_ranges])


@pytest.mark.parametrize("compute_segments,flow_solver", [
    (compute_segments2,        MinConvexCostFlow),
    (default_compute_segments, CppMinConvexCostFlow),
    (default_compute_segments, CppRobustMinConvexCostFlow),
], ids=["py", "cpp", "cpp_robust"])
def test_roadnet_matching_int(compute_segments, flow_solver):

    roadnet = nx.MultiDiGraph()
    if True:
        roadnet.add_edge(0, 1, 'N', length=1.)
    else:
        # TODO: Do this in a meaningful way.
        # to test one-way roads capabilities
        roadnet.add_edge(0, 1, 'N', length=1., oneway=True)

    roadnet.add_edge(1, 2, 'E', length=1.)
    roadnet.add_edge(2, 3, 'S', length=1.)
    roadnet.add_edge(3, 0, 'W', length=1.)

    if True:
        roadnet.add_edge(0, 4, 'dangler', length=1.)

    sampler = roadprob.UniformDist(roadnet)

    NUMPOINT = 50
    PP_ = [sampler.sample() for i in range(NUMPOINT)]
    QQ_ = [sampler.sample() for i in range(NUMPOINT)]

    def make_problem(P, Q, rn):
        return RoadnetMatchingProblem(P, Q, rn, flow_solver=flow_solver, compute_segments=compute_segments)

    roadnet_, roadnet_graph_ = MultiDiGraphRoadnet(roadnet), roadnet
    matching1, cost_ctd1 = make_problem(PP_, QQ_, roadnet_).compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)
    cost_sp1 = ROADMATCHCOST(matching1, PP_, QQ_, roadnet_graph_)

    inst, _roads, __ = RoadnetMatchingInstance.normalize(PP_, QQ_, roadnet_)
    roadnet = inst.roadnet
    assert inst.is_valid()

    PP, QQ = inst.P, inst.Q
    matching2, cost_ctd2 = make_problem(PP, QQ, roadnet).compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)

    roadnet_graph = to_networkx(roadnet)
    cost_sp2 = ROADMATCHCOST(matching2, PP, QQ, roadnet_graph)

    # Compare their costs!
    all_costs = [cost_ctd1, cost_sp1, cost_ctd2, cost_sp2]
    assert within_tolerance(all_costs), all_costs


def within_tolerance(costs):
    costs_ = sorted(costs)
    return abs(costs_[-1] - costs_[0]) < 1e-10


def test_match_empty():
    rn = RoadNetwork()
    rn.add_edge("A", 0, 1, 10)
    match, cost = RoadnetMatchingProblem([], [], rn).compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)
    assert match == []
    assert cost == 0.


class TestBiPartite:

    def test_construction(self):

        bp = BiPartite.create_with(list)
        bp.supply.extend(range(10))
        assert bp.supply == list(range(10))
        assert bp.demand == []

    def test_factory(self):

        f = BiPartite.factory(int)
        bp = f()
        bp.supply += 4
        assert bp.supply == 4
        assert bp.demand == 0


def test_create_point_map():

    roadnet = nx.MultiDiGraph()
    roadnet.add_edge(0, 1, 'N', length=1.)
    roadnet.add_edge(1, 2, 'E', length=1.)
    roadnet.add_edge(2, 3, 'S', length=1.)
    roadnet.add_edge(3, 0, 'W', length=1.)
    roadnet.add_edge(0, 4, 'dangler', length=1.)

    roadnet_frfr = MultiDiGraphRoadnet(roadnet)
    sampler = roadprob.UniformDist(roadnet)

    NUMPOINT = 50

    PP = [sampler.sample() for i in range(NUMPOINT)]
    QQ = [sampler.sample() for i in range(NUMPOINT)]

    pm = compute_segments3(PP, QQ, roadnet_frfr)
    segs_ = compute_segments2(PP, QQ, roadnet_frfr)
    pm_ = compile_index_ranges(segs_)

    assert pm_ == pm


def test_segment_pointless_road():

    rn = RoadNetwork()
    rn.add_edge("R", 0, 1, 1.)

    segment_dict = compute_segments2([], [], rn)
    assert "R" in segment_dict


def test_objectives():

    rn = RoadNetwork()
    rn.add_edge("R", 0, 1, 10.)

    objs = write_objectives([("R", 2.)], [("R", 8.)], rn)

    obj_fn = costWrapper(objs["R"])

    xs = -5, -.5, 1
    cs = [obj_fn(x) for x in xs]
    assert cs == [44., 5., 16.]


def _make_measure_dev():
    from collections import deque
    from setiptah.roadgeometry.matching.bm import MEASURE, BiPartite

    def make_q(supply, demand):
        q = BiPartite.create_with(deque)
        q.supply.extend(supply)
        q.demand.extend(demand)
        return q

    segment = [(0.3, make_q([0], [])), (0.7, make_q([], [0]))]
    return MEASURE(segment, 1.0)


def _make_measure_rbtree():
    import bintrees
    return bintrees.RBTree({0: 0.6, 1: 0.4})


@pytest.mark.parametrize("make_measure", [_make_measure_dev, _make_measure_rbtree])
def test_objective(make_measure):
    """Sanity check OBJECTIVE on a known example.

    Road of length 1, one supply at y=0.3, one demand at y=0.7.
    MEASURE gives flow level 0 with total length 0.6, level 1 with length 0.4.

    The resulting cost function has breakpoints at z=-1 and z=0:
      z < -1 : slope=-1,   intercept=-0.4  →  f(-1.5) = 1.1
      -1<=z<0: slope=-0.2, intercept=0.4   →  f(-0.5) = 0.5
      z >= 0 : slope=1,    intercept=0.4   →  f(0.5)  = 0.9
    """
    from setiptah.roadgeometry.matching.bm import OBJECTIVE_FUNC
    from setiptah.roadgeometry.matching.util.double_ended_vector import DoubleEndedVector
    from setiptah.roadgeometry.matching.nxopt.pwl import IntPWL

    measure = make_measure()
    assert dict(measure.items()) == {0: pytest.approx(0.6), 1: pytest.approx(0.4)}

    obj = OBJECTIVE_FUNC(measure)

    if isinstance(measure, DoubleEndedVector):
        assert isinstance(obj, IntPWL)

    assert obj(-1.5) == pytest.approx(1.1)
    assert obj(-0.5) == pytest.approx(0.5)
    assert obj(0.5)  == pytest.approx(0.9)
