"""Parity tests: C++ and Python Dijkstra implementations must produce valid, equivalent flows."""
from functools import partial

import networkx as nx
import numpy as np
import pytest

from setiptah.roadgeometry.generation import DelaunayRoadnet
from setiptah.roadgeometry.probability import RoadnetUniformDist
from setiptah.roadgeometry.matching.bm import (
    RoadnetMatchingProblem, MatchingResult,
    compute_segments2, compute_optimal_flow, check_flow,
    SURPLUS, MEASURE, OBJECTIVE_FUNC,
    flow_cost_per_road,
)
from setiptah.roadgeometry.matching.nxopt.cvxcostflow import (
    MinConvexCostFlow,
    py_dijkstra,
)

py_flow_solver = partial(MinConvexCostFlow, dijkstra=py_dijkstra)


def random_instance(rng, n_intersections=12, n_points=20):
    points = rng.random((n_intersections, 2))
    roadnet = DelaunayRoadnet(points)
    sampler = RoadnetUniformDist(roadnet)
    PP = [sampler.sample() for _ in range(n_points)]
    QQ = [sampler.sample() for _ in range(n_points)]
    return PP, QQ, roadnet


def flow_is_acyclic(flow, roadnet):
    g = nx.DiGraph()
    for road in roadnet.edges():
        f = flow.get(road, 0)
        if f > 0:
            i, j = roadnet.endpoints(road)
            g.add_edge(i, j)
        elif f < 0:
            i, j = roadnet.endpoints(road)
            g.add_edge(j, i)
    return nx.is_directed_acyclic_graph(g)


def flow_cost(flow, measure_dict):
    obj_dict = {road: OBJECTIVE_FUNC(m) for road, m in measure_dict.items()}
    return sum(flow_cost_per_road(flow, obj_dict).values())


def check_instance(PP, QQ, roadnet, flow_solver):
    segment_dict = compute_segments2(PP, QQ, roadnet)
    surplus_dict = {road: SURPLUS(seg) for road, seg in segment_dict.items()}
    measure_dict = {road: MEASURE(seg, roadnet.length(road)) for road, seg in segment_dict.items()}
    flow = compute_optimal_flow(roadnet, surplus_dict, measure_dict, flow_solver=flow_solver)
    imbalance = check_flow(flow, roadnet, surplus_dict)
    assert len(imbalance) == 0, f"flow not conservative: {imbalance}"
    assert flow_is_acyclic(flow, roadnet), "flow has cycles"
    return flow, measure_dict


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4])
def test_parity(seed):
    rng = np.random.default_rng(seed)
    PP, QQ, roadnet = random_instance(rng)

    flow_cpp, measure_dict = check_instance(PP, QQ, roadnet, flow_solver=None)
    flow_py,  _            = check_instance(PP, QQ, roadnet, flow_solver=py_flow_solver)

    cost_cpp = flow_cost(flow_cpp, measure_dict)
    cost_py  = flow_cost(flow_py,  measure_dict)
    assert abs(cost_cpp - cost_py) < 1e-9, f"cost mismatch: cpp={cost_cpp}, py={cost_py}"
