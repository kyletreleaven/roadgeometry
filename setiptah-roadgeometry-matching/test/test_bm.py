from setiptah.roadbm.bm import *

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

    match = ROADSBIPARTITEMATCH(PP, QQ, roadnet)
    assert len(match) == NUMPOINT

    # costs = MATCHCOSTS(match, PP, QQ, roadnet)
    cost = ROADMATCHCOST(match, PP, QQ, roadnet)

    obj_fn_dict = {
        road: OBJECTIVE_FUNC(measure)
        for road, measure in measure_dict.items()
    }  # write_objectives(PP, QQ, roadnet_frfr)

    costs_ = flow_cost_per_road(assist, obj_fn_dict)

    assert abs(cost - sum(costs_.values())) < 1e-7


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

    PP, QQ = inst.P, inst.Q  # TODO: Oops? It's not a roadnet, it's an instance...
    matching = optimal_roadnet_matching(PP, QQ, roadnet)

    # Compare their costs.
    cost_ = ROADMATCHCOST(matching_, PP_, QQ_, roadnet_graph_)
    roadnet_graph = roadnet.create_multigraph()
    cost = ROADMATCHCOST(matching, PP, QQ, roadnet_graph)
    assert cost == cost_
