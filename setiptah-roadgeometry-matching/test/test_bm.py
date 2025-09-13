import networkx as nx

import setiptah.roadgeometry.probability as roadprob
from setiptah.roadgeometry.formats import to_networkx, from_networkx
from setiptah.roadgeometry.matching.nx_legacy import *


def test_roadnet_matching():

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

    segs = SEGMENTS(PP, QQ, roadnet)
    segs_: dict[TRoad, MySegment] = compute_segments3(PP, QQ, roadnet_frfr)
    # assert False, segs_

    # prematch
    pm = set(
        m
        for road, seg in segs.items()
        for m in PREMATCH(seg.iter_items())
    )
    # assert False, pm

    # surplus
    surplus_dict = {
        road: SURPLUS(seg.iter_items())
        for road, seg in segs.items()
    }
    assert sum(surplus_dict.values()) == 0
    # assert False, surplus_dict

    road_len = {
        road: roadnet_frfr.length(road)
        for road in segs
    }
    # assert False, road_len

    measure_dict = {
        road: MEASURE(seg.iter_items(), road_len[road])
        for road, seg in segs.items()
    }
    # assert False, measure_dict

    assist = compute_optimal_flow(roadnet_frfr, surplus_dict, measure_dict)

    # Good to check!
    imbalance = CHECKFLOW(assist, roadnet, surplus_dict)
    assert len(imbalance) <= 0

    topograph = create_topograph2(segs_, assist, roadnet_frfr)
    # assert False, topograph.edges(data=True)

    nodes = list(nx.topological_sort( topograph ))
    # assert False, (len(nodes), nodes)

    match_ref, cost_ctd_ = TRAVERSE3(topograph)
    # assert False, match_ref
    match_ = [
        (segs_[road1].points.supply[i1], segs_[road2].points.demand[i2])
        for (road1, i1), (road2, i2) in match_ref
    ]
    # assert False, match_
    cost_sp_ = ROADMATCHCOST(match_, PP, QQ, roadnet)

    # Compare:
    # [x] cost computed during matching construction
    match, cost_ctd = RoadnetMatchingProblem(
        PP, QQ, MultiDiGraphRoadnet(roadnet)
    ).compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)
    assert len(match) == NUMPOINT

    assert True, (
        (cost_ctd_, cost_ctd),
        (match, match_),
    )

    # [x] sum shortest path lengths,
    cost_sp = ROADMATCHCOST(match, PP, QQ, roadnet)

    # [x] objective fn cost of flow,
    obj_fn_dict = {
        road: OBJECTIVE_FUNC(measure)
        for road, measure in measure_dict.items()
    }  # write_objectives(PP, QQ, roadnet_frfr)

    costs_obj = flow_cost_per_road(assist, obj_fn_dict)
    cost_obj = sum(costs_obj.values())

    # limit spread
    costs = [cost_ctd, cost_sp, cost_obj, cost_ctd_, cost_sp_]
    costs_ = sorted(costs)

    assert abs(costs_[-1] - costs_[0]) < 1e-10, costs

    # TODO: Test vs. a third-party implementation on distance matrix; e.g.,
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.linear_sum_assignment.html


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

    cost = RoadnetMatchingProblem(PP, QQ, roadnet, use_ranges=False).compute_optimal(MatchingResult.COST)
    cost_using_ranges = RoadnetMatchingProblem(PP, QQ, roadnet, use_ranges=True).compute_optimal(MatchingResult.COST)

    assert cost_using_ranges == cost


def test_roadnet_matching_int():

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

    roadnet_, roadnet_graph_ = MultiDiGraphRoadnet(roadnet), roadnet
    matching1, cost_ctd1 = RoadnetMatchingProblem(PP_, QQ_, roadnet_).compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)
    cost_sp1 = ROADMATCHCOST(matching1, PP_, QQ_, roadnet_graph_)

    inst, _roads, __ = RoadnetMatchingInstance.normalize(PP_, QQ_, roadnet_)
    roadnet = inst.roadnet
    assert inst.is_valid()

    PP, QQ = inst.P, inst.Q
    matching2, cost_ctd2 = RoadnetMatchingProblem(PP, QQ, roadnet).compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)

    roadnet_graph = to_networkx(roadnet)
    cost_sp2 = ROADMATCHCOST(matching2, PP, QQ, roadnet_graph)

    # Compare their costs.

    within_tolerance([cost_ctd1, cost_sp1, cost_ctd2, cost_sp2])
    # assert False, cost_constr


def within_tolerance(costs):
    costs_ = sorted(costs)
    assert abs(costs_[-1] - costs_[0]) < 1e-10, costs


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
