"""Regression tests for the topograph invariant:
a conservative flow must produce a balanced topograph.
"""
import json
from functools import partial
from pathlib import Path

import networkx as nx
import pytest

from setiptah.roadgeometry.matching.io import roadnet_from_json, point_set_from_json
from setiptah.roadgeometry.matching.bm import (
    compute_segments2, compute_optimal_flow, check_flow,
    SURPLUS, MEASURE, OBJECTIVE_FUNC, flow_cost_per_road,
    create_topograph, CHECKTOPO,
)
from setiptah.roadgeometry.matching.nxopt.cvxcostflow import MinConvexCostFlow, py_dijkstra

py_solver = partial(MinConvexCostFlow, dijkstra=py_dijkstra)

FIXTURES = Path(__file__).parent / "fixtures"


def load_fixture(name):
    with open(FIXTURES / name) as f:
        doc = json.load(f)
    roadnet = roadnet_from_json(doc["roadnet"])
    supply  = point_set_from_json(doc["supply"])
    demand  = point_set_from_json(doc["demand"])
    flow    = {k: v for k, v in doc["flow"].items()}
    return roadnet, supply, demand, flow


@pytest.mark.parametrize("fixture", [
    "topograph_imbalance.json",
])
def test_conservative_flow_gives_balanced_topograph(fixture):
    roadnet, supply, demand, flow = load_fixture(fixture)

    segment_dict = compute_segments2(supply, demand, roadnet)

    for road, seg in segment_dict.items():
        # each segment should have all the roads' points
        supply_on_seg = set(k for _, qs in seg for k in qs.supply)
        demand_on_seg = set(k for _, qs in seg for k in qs.demand)

        supply_on_road = set(k for k, (road_, y) in enumerate(supply) if road_ == road)
        demand_on_road = set(k for k, (road_, y) in enumerate(demand) if road_ == road)

        assert not supply_on_seg.symmetric_difference(supply_on_road)
        assert not demand_on_seg.symmetric_difference(demand_on_road)

    surplus_dict  = {road: SURPLUS(seg) for road, seg in segment_dict.items()}
    measure_dict  = {road: MEASURE(seg, roadnet.length(road)) for road, seg in segment_dict.items()}
    obj_dict      = {road: OBJECTIVE_FUNC(m) for road, m in measure_dict.items()}

    # The fixture flow was produced by the C++ solver; verify it is conservative.
    imbalance = check_flow(flow, roadnet, surplus_dict)
    assert len(imbalance) == 0, f"flow not conservative: {imbalance}"

    # Justify the fixture as a valid optimal solution: its cost must match the
    # Python solver's optimum on the same instance.
    fixture_cost = sum(flow_cost_per_road(flow, obj_dict).values())
    py_flow      = compute_optimal_flow(roadnet, surplus_dict, measure_dict, flow_solver=py_solver)
    py_cost      = sum(flow_cost_per_road(py_flow, obj_dict).values())
    assert abs(fixture_cost - py_cost) < 1e-9, (
        f"fixture flow cost {fixture_cost} != python optimal {py_cost}"
    )

    topo = create_topograph(segment_dict, flow, roadnet)
    assert nx.is_directed_acyclic_graph(topo), (
        f"topograph has cycles: {nx.find_cycle(topo)}"
    )
    unbalanced = CHECKTOPO(topo)
    assert len(unbalanced) == 0, f"topograph not conservative: {unbalanced}"
