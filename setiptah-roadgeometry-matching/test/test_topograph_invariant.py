"""Regression tests for the topograph invariant:
a conservative flow must produce a balanced topograph.
"""
import json
from pathlib import Path

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
    surplus_dict = {road: SURPLUS(seg) for road, seg in segment_dict.items()}

    imbalance = check_flow(flow, roadnet, surplus_dict)
    assert len(imbalance) == 0, f"flow not conservative: {imbalance}"

    topo = create_topograph(segment_dict, flow, roadnet)
    unbalanced = CHECKTOPO(topo)
    assert len(unbalanced) == 0, f"topograph not conservative: {unbalanced}"
