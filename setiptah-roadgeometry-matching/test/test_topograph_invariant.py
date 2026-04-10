"""Regression tests for the topograph invariant:
a conservative flow must produce a balanced topograph.
"""
import json
from pathlib import Path

import networkx as nx
import pytest

from setiptah.roadgeometry.matching.io import roadnet_from_json, point_set_from_json
from setiptah.roadgeometry.matching.bm import (
    compute_segments2, compute_optimal_flow, check_flow,
    SURPLUS, MEASURE, create_topograph, CHECKTOPO,
)

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

    surplus_dict = {road: SURPLUS(seg) for road, seg in segment_dict.items()}

    imbalance = check_flow(flow, roadnet, surplus_dict)
    assert len(imbalance) == 0, f"flow not conservative: {imbalance}"

    topo = create_topograph(segment_dict, flow, roadnet)
    assert nx.is_directed_acyclic_graph(topo), (
        f"topograph has cycles: {nx.find_cycle(topo)}"
    )
    unbalanced = CHECKTOPO(topo)
    assert len(unbalanced) == 0, f"topograph not conservative: {unbalanced}"
