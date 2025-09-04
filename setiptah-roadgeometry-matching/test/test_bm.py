from setiptah.roadbm import *

import setiptah.roadgeometry.probability as roadprob

import networkx as nx


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
    segs_ = compute_segments2(PP, QQ, roadnet_frfr)
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

    topograph = create_topograph(segs_, assist, roadnet_frfr)
    # assert False, topograph.edges(data=True)

    nodes = list(nx.topological_sort( topograph ))
    # assert False, (len(nodes), nodes)

    match = TRAVERSE(topograph)
    # assert False, match

    # Compare:
    # [x] cost computed during matching construction
    match, cost_ctd = optimal_roadnet_matching2(PP, QQ, MultiDiGraphRoadnet(roadnet))
    assert len(match) == NUMPOINT

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
    costs = [cost_ctd, cost_sp, cost_obj]
    costs_ = sorted(costs)

    assert abs(costs_[-1] - costs_[0]) < 1e-10, costs


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
    #
    PP_ = [sampler.sample() for i in range(NUMPOINT)]
    QQ_ = [sampler.sample() for i in range(NUMPOINT)]

    roadnet_, roadnet_graph_ = MultiDiGraphRoadnet(roadnet), roadnet
    matching_ = optimal_roadnet_matching(PP_, QQ_, roadnet_)

    inst, _roads, __ = StructRoadnetMatchingInstance.normalize(PP_, QQ_, roadnet_)
    roadnet = inst.roadnet
    # assert False, roadnet
    assert inst.is_valid()

    PP, QQ = inst.P, inst.Q
    matching, cost_constr = optimal_roadnet_matching2(PP, QQ, roadnet)

    # Compare their costs.
    cost_ = ROADMATCHCOST(matching_, PP_, QQ_, roadnet_graph_)
    roadnet_graph = roadnet.create_multigraph()
    cost = ROADMATCHCOST(matching, PP, QQ, roadnet_graph)

    within_tolerance([cost, cost_, cost_constr])
    # assert False, cost_constr


def within_tolerance(costs):
    costs_ = sorted(costs)
    assert abs(costs_[-1] - costs_[0]) < 1e-10, costs


class TestRoadPointSeq:

    def test_cat(self):

        a = RoadPointSeq("A", 2, 10)
        a, b = a.split(4)
        b, c = b.split(2)

        assert a == RoadPointSeq("A", 2, 6)
        assert b == RoadPointSeq("A", 6, 8)
        assert c == RoadPointSeq("A", 8, 10)

        assert a.can_cat(b)
        assert not a.can_cat(c)
        assert b.can_cat(c)

        assert a.cat(b) == RoadPointSeq("A", 2, 8)
        assert b.cat(c) == RoadPointSeq("A", 6, 10)

        d = RoadPointSeq("B", 0, 7, reverse=True)
        d, e = d.split(3)
        e, f = e.split(2)

        assert d == RoadPointSeq("B", 4, 7, reverse=True)
        assert e == RoadPointSeq("B", 2, 4, reverse=True)
        assert f == RoadPointSeq("B", 0, 2, reverse=True)

        assert d.can_cat(e)
        assert d.cat(e) == RoadPointSeq("B", 2, 7, reverse=True)
        assert not d.can_cat(f)

        assert not RoadPointSeq("A", 0, 4).can_cat(RoadPointSeq("B", 4, 6))

    def test_deque_ops(self):

        q = deque()
        extend_points(q, RoadPointSeq("A", 0, 4))
        extend_points(q, RoadPointSeq("A", 4, 7))
        extend_points(q, RoadPointSeq("A", 8, 10))  # skip one

        extend_points(q, RoadPointSeq("B", 5, 8, reverse=True))
        extend_points(q, RoadPointSeq("B", 3, 5, reverse=True))
        extend_points(q, RoadPointSeq("B", 0, 2, reverse=True))

        assert list(q) == [
            RoadPointSeq("A", 0, 7),
            RoadPointSeq("A", 8, 10),
            RoadPointSeq("B", 3, 8, reverse=True),
            RoadPointSeq("B", 0, 2, reverse=True),
        ]

        q1 = take_points(q, 12)

        assert list(q1) == [
            RoadPointSeq("A", 0, 7),
            RoadPointSeq("A", 8, 10),
            RoadPointSeq("B", 5, 8, reverse=True),
        ]

        assert list(q) == [
            RoadPointSeq("B", 3, 5, reverse=True),
            RoadPointSeq("B", 0, 2, reverse=True),
        ]

        p = pop_point(q)
        assert p == ("B", 4)
