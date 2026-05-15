"""Regression test for cpp compute_optimal_flow using a captured app session.

To generate the fixture:
  1. Start the webapp backend and frontend.
  2. Place at least one supply and one demand pin on the map.
  3. Click the "capture" button in the overlay.
  4. Copy captured_pins.json (written to the backend working directory) here:
       setiptah-roadgeometry-matching/test/captured_pins.json
"""
import pathlib
import pytest

_HERE    = pathlib.Path(__file__).parent
_BACKEND = _HERE / '../../webapp/backend'
_PINS    = _BACKEND / 'captured_pins.json'
_GRAPHML = _BACKEND / 'cambridge.graphml'

_cpp = pytest.importorskip('setiptah.roadgeometry.matching._cpp',
                            reason='cpp extension not built')


@pytest.fixture(scope='module')
def roadnet():
    if not _GRAPHML.exists():
        pytest.skip('cambridge.graphml not present — start the webapp once to generate it')
    import osmnx as ox
    from setiptah.roadgeometry.geopandas import GeoFramesNetwork

    G = ox.load_graphml(_GRAPHML)
    nodes, edges = ox.graph_to_gdfs(G)
    edges['u'] = edges.index.get_level_values('u')
    edges['v'] = edges.index.get_level_values('v')
    return GeoFramesNetwork(
        edges_gdf=edges, nodes_gdf=nodes,
        left_col='u', right_col='v', oneway_col='oneway',
    )


def test_cpp_optimal_flow_matches_python(roadnet):
    if not _PINS.exists():
        pytest.skip('captured_pins.json not present — see module docstring')

    from setiptah.roadgeometry.matching.io import load_pins
    from setiptah.roadgeometry.matching.bm import (
        PREMATCH, MEASURE, OBJECTIVE_FUNC,
        compute_segments2, flow_from_segments,
    )
    from setiptah.roadgeometry.matching.nxopt.cvxcostflow import MinConvexCostFlow

    P, Q = load_pins(_PINS)

    # --- cpp path (under test) ---
    seg_cpp = _cpp.sort_and_segment(P, Q, list(roadnet.edges()))
    for seg in seg_cpp.values():
        PREMATCH(seg)
    endpoints, lengths, is_oneway = roadnet.graph_props(seg_cpp)
    flow_cpp = _cpp.compute_optimal_flow(seg_cpp, endpoints, lengths, is_oneway)

    # --- python path (reference) ---
    seg_py = compute_segments2(P, Q, roadnet)
    for seg in seg_py.values():
        PREMATCH(seg)
    flow_py = flow_from_segments(seg_py, roadnet, flow_solver=MinConvexCostFlow)

    obj = {
        road: OBJECTIVE_FUNC(MEASURE(seg, roadnet.length(road)))
        for road, seg in seg_py.items()
        if seg
    }

    def total_cost(flow):
        return sum(fn(flow.get(road, 0)) for road, fn in obj.items())

    cost_cpp = total_cost(flow_cpp)
    cost_py  = total_cost(flow_py)

    assert abs(cost_cpp - cost_py) < 1e-6, (
        f'cpp cost={cost_cpp}, python cost={cost_py}, diff={abs(cost_cpp - cost_py)}'
    )
