"""Debug: compare fixture vs python flow for topograph_imbalance.json"""
import json
from pathlib import Path
from functools import partial

from setiptah.roadgeometry.matching.io import roadnet_from_json, point_set_from_json
from setiptah.roadgeometry.matching.bm import (
    compute_segments2, compute_optimal_flow, SURPLUS, MEASURE, OBJECTIVE_FUNC,
    flow_cost_per_road,
)
from setiptah.roadgeometry.matching.nxopt.cvxcostflow import MinConvexCostFlow, py_dijkstra

py_solver = partial(MinConvexCostFlow, dijkstra=py_dijkstra)

with open(Path(__file__).parent.parent / 'test/fixtures/topograph_imbalance.json') as f:
    doc = json.load(f)

roadnet = roadnet_from_json(doc['roadnet'])
supply  = point_set_from_json(doc['supply'])
demand  = point_set_from_json(doc['demand'])
fixture_flow = {k: v for k, v in doc['flow'].items()}

segment_dict = compute_segments2(supply, demand, roadnet)
surplus_dict = {road: SURPLUS(seg) for road, seg in segment_dict.items()}
measure_dict = {road: MEASURE(seg, roadnet.length(road)) for road, seg in segment_dict.items()}
obj_dict     = {road: OBJECTIVE_FUNC(m) for road, m in measure_dict.items()}

py_flow = compute_optimal_flow(roadnet, surplus_dict, measure_dict, flow_solver=py_solver)

nz_fixture = {k: v for k, v in fixture_flow.items() if v != 0}
nz_py      = {k: v for k, v in py_flow.items() if v != 0}
print('fixture non-zero:', sorted(nz_fixture.items()))
print('python  non-zero:', sorted(nz_py.items()))

fixture_cost = sum(flow_cost_per_road(fixture_flow, obj_dict).values())
py_cost      = sum(flow_cost_per_road(py_flow, obj_dict).values())
print(f'fixture cost: {fixture_cost}')
print(f'python  cost: {py_cost}')

costs = flow_cost_per_road(py_flow, obj_dict)
print('python per-road costs:', sorted(costs.items()))
