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
    create_topograph, CHECKTOPO,
)
from setiptah.roadgeometry.matching.nxopt.cvxcostflow import (
    MinConvexCostFlow,
    py_dijkstra,
    CppMinConvexCostFlow,
)
from setiptah.roadgeometry.matching.io import (
    roadnet_to_json, point_set_to_json,
)

py_flow_solver = partial(MinConvexCostFlow, dijkstra=py_dijkstra)

flow_solvers = {
    "py":          py_flow_solver,
    "cpp_dijkstra": None,           # default: uses C++ Dijkstra inside Python FragileMCCF
    "cpp_fragile":  CppMinConvexCostFlow,
}


def random_instance(rng, n_intersections=12, n_points=20):
    points = rng.random((n_intersections, 2))
    roadnet = DelaunayRoadnet(points)
    sampler = RoadnetUniformDist(roadnet)
    PP = [sampler.sample() for _ in range(n_points)]
    QQ = [sampler.sample() for _ in range(n_points)]
    return PP, QQ, roadnet


def flow_is_acyclic(flow, roadnet):
    g = nx.DiGraph()
    for road, f in flow.items():
        if f == 0:
            continue
        i, j = roadnet.endpoints(road)
        g.add_edge(i, j) if f > 0 else g.add_edge(j, i)
    return nx.is_directed_acyclic_graph(g)


def format_flow_cycle(flow, roadnet):
    """Return a cycle in the flow as 'node1 edge1 weight1 node2 edge2 weight2 ...'."""
    g = nx.MultiDiGraph()
    edge_data = {}  # (src, dst, key) -> (road, weight)
    for road, f in flow.items():
        if f == 0:
            continue
        i, j = roadnet.endpoints(road)
        src, dst = (i, j) if f > 0 else (j, i)
        key = g.add_edge(src, dst)
        edge_data[(src, dst, key)] = (road, abs(f))
    try:
        cycle = nx.find_cycle(g)
    except nx.NetworkXNoCycle:
        return "<no cycle found>"
    parts = []
    for src, dst, key in cycle:
        road, w = edge_data[(src, dst, key)]
        parts.extend([str(src), str(road), str(w)])
    return " ".join(parts)


def flow_cost(flow, measure_dict):
    obj_dict = {road: OBJECTIVE_FUNC(m) for road, m in measure_dict.items()}
    return sum(flow_cost_per_road(flow, obj_dict).values())


def check_instance(PP, QQ, roadnet, flow_solver, check_topograph=False):
    segment_dict = compute_segments2(PP, QQ, roadnet)
    surplus_dict = {road: SURPLUS(seg) for road, seg in segment_dict.items()}
    measure_dict = {road: MEASURE(seg, roadnet.length(road)) for road, seg in segment_dict.items()}
    flow = compute_optimal_flow(roadnet, surplus_dict, measure_dict, flow_solver=flow_solver)
    imbalance = check_flow(flow, roadnet, surplus_dict)
    assert len(imbalance) == 0, f"flow not conservative: {imbalance}"
    if check_topograph:
        topo = create_topograph(segment_dict, flow, roadnet)
        assert nx.is_directed_acyclic_graph(topo), (
            f"topograph has cycles: {format_flow_cycle(flow, roadnet)}"
        )
        unbalanced = CHECKTOPO(topo)
        if unbalanced:
            rn_json, edge_to_id = roadnet_to_json(roadnet)
            import json, sys
            doc = {
                "roadnet": rn_json,
                "supply":  point_set_to_json(PP, edge_to_id),
                "demand":  point_set_to_json(QQ, edge_to_id),
                "flow":    {edge_to_id[e]: int(v) for e, v in flow.items()},
            }
            json.dump(doc, sys.stderr, indent=2)
            assert False, f"topograph not conservative at nodes: {unbalanced}"
    return flow, measure_dict


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4])
@pytest.mark.parametrize("mode", ["py", "cpp_dijkstra", "cpp_fragile"])
def test_parity(seed, mode):
    rng = np.random.default_rng(seed)
    PP, QQ, roadnet = random_instance(rng)

    flow_py, measure_dict = check_instance(PP, QQ, roadnet, flow_solver=py_flow_solver, check_topograph=True)
    cost_py = flow_cost(flow_py, measure_dict)

    flow_alt, _ = check_instance(PP, QQ, roadnet, flow_solver=flow_solvers[mode], check_topograph=True)
    cost_alt = flow_cost(flow_alt, measure_dict)
    assert abs(cost_alt - cost_py) < 1e-9, f"[{mode}] cost mismatch: {cost_alt} vs py={cost_py}"
