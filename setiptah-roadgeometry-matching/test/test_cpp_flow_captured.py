"""Regression test for cpp compute_optimal_flow using a captured app session.

To generate the fixture:
  1. Start the webapp backend and frontend.
  2. Place at least one supply and one demand pin on the map.
  3. Click the "capture" button in the overlay.
  4. Copy captured_case.zip (written to the backend working directory) here:
       setiptah-roadgeometry-matching/test/captured_case.zip
"""
import pathlib
import pytest

CASE = pathlib.Path(__file__).parent / 'captured_case.zip'

if not CASE.exists():
    pytest.skip('captured_case.zip not present — see module docstring', allow_module_level=True)

_cpp = pytest.importorskip('setiptah.roadgeometry.matching._cpp',
                            reason='cpp extension not built')


def test_cpp_optimal_flow_matches_python():
    from setiptah.roadgeometry.matching.io import load_instance_geo
    from setiptah.roadgeometry.matching.bm import PREMATCH, compute_segments2, flow_from_segments
    from setiptah.roadgeometry.matching.nxopt.cvxcostflow import MinConvexCostFlow

    P, Q, roadnet = load_instance_geo(CASE)

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

    roads = set(flow_py) | set(flow_cpp)
    for r in roads:
        assert flow_py.get(r, 0) == flow_cpp.get(r, 0), (
            f'road {r}: python={flow_py.get(r, 0)}, cpp={flow_cpp.get(r, 0)}'
        )
