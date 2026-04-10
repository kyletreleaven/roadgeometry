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

    # Sanity: the topograph balance at an interchange node equals the check_flow
    # balance at the corresponding roadnet node (derivable from how create_topograph
    # assigns edge weights).  Since check_flow passed above, every interchange node
    # that appears unbalanced in the topograph must be a bug in create_topograph.
    def road_of_neighbor(neighbor, u, roadnet, is_out_edge):
        """Return the road corresponding to a topo edge incident to special[u]."""
        if neighbor.q is not None:
            return neighbor.ref   # point node: ref is the road
        # interchange neighbor: find road between u and neighbor.ref
        v = neighbor.ref
        tail, head = (u, v) if is_out_edge else (v, u)
        for road in roadnet.edges():
            if roadnet.endpoints(road) == (tail, head):
                return road
        return None  # shouldn't happen

    for node, topo_balance in unbalanced:
        if node.q is None:
            u = node.ref
            lines = [f"\ninterchange node {u!r}: topo balance={topo_balance}"]

            for _, v_node in topo.out_edges(node):
                w = topo.get_edge_data(node, v_node)['weight']
                road = road_of_neighbor(v_node, u, roadnet, is_out_edge=True)
                f = flow.get(road, 0.)
                tail, head = roadnet.endpoints(road)
                if tail == u:
                    expected = f
                else:                          # head == u, edge was reversed (flow < 0)
                    expected = -(f + surplus_dict.get(road, 0.))
                ok = (w == expected)
                lines.append(
                    f"  out-edge road={road} ({tail}->{head})"
                    f"  weight={w}  expected={expected}  flow={f}"
                    f"  surplus={surplus_dict.get(road, 0.)}  {'OK' if ok else '*** MISMATCH ***'}"
                )

            for u_node, _ in topo.in_edges(node):
                w = topo.get_edge_data(u_node, node)['weight']
                road = road_of_neighbor(u_node, u, roadnet, is_out_edge=False)
                f = flow.get(road, 0.)
                tail, head = roadnet.endpoints(road)
                if head == u:
                    expected = f + surplus_dict.get(road, 0.)
                else:                          # tail == u, edge was reversed (flow+surplus < 0)
                    expected = -f
                ok = (w == expected)
                lines.append(
                    f"  in-edge  road={road} ({tail}->{head})"
                    f"  weight={w}  expected={expected}  flow={f}"
                    f"  surplus={surplus_dict.get(road, 0.)}  {'OK' if ok else '*** MISMATCH ***'}"
                )

            print("\n".join(lines))
            local_balance = 0.
            for road in roadnet.edges():
                i, j = roadnet.endpoints(road)
                f = flow.get(road, 0.)
                if i == u:
                    local_balance -= f
                if j == u:
                    local_balance += f + surplus_dict.get(road, 0.)
            assert local_balance == 0., (
                f"roadnet not conservative at node {u}: {local_balance} "
                f"(check_flow should have caught this)"
            )

    assert len(unbalanced) == 0, f"topograph not conservative: {unbalanced}"
