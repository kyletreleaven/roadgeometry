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

    sampler = roadprob.UniformDist(roadnet)

    NUMPOINT = 50
    #
    PP = [sampler.sample() for i in range(NUMPOINT)]
    QQ = [sampler.sample() for i in range(NUMPOINT)]

    segs = SEGMENTS(PP, QQ, roadnet)

    # prematch
    pm = set(
        m
        for road, seg in segs.items()
        for m in PREMATCH(seg)
    )
    # assert False, pm

    # surplus
    surplus_dict = {
        road: SURPLUS(seg)
        for road, seg in segs.items()
    }
    # assert False, surplus_dict

    roadnet_frfr = MultiDiGraphRoadnet(roadnet)

    road_len = {
        road: roadnet_frfr.length(road)
        for road in segs
    }

    measure_dict = {
        road: MEASURE(seg, road_len[road])
        for road, seg in segs.items()
    }
    # assert False, measure_dict

    assist = compute_optimal_flow(roadnet_frfr, surplus_dict, measure_dict)

    # Good to check!
    imbalance = CHECKFLOW(assist, roadnet, surplus_dict)
    assert len(imbalance) <= 0

    topograph = TOPOGRAPH(segs, assist, roadnet)
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
    matching = optimal_roadnet_matching(PP, QQ, roadnet)

    # Compare their costs.
    cost_ = ROADMATCHCOST(matching_, PP_, QQ_, roadnet_graph_)
    roadnet_graph = roadnet.create_multigraph()
    cost = ROADMATCHCOST(matching, PP, QQ, roadnet_graph)
    assert cost == cost_
