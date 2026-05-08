import time

from setiptah.roadgeometry.matching import RoadnetMatchingProblem, MatchingResult
from setiptah.roadgeometry.matching.bm import (
    _cpp, _cpp_compute_optimal_flow,
    compute_segments2, default_compute_segments, default_flow_solver,
)
from setiptah.roadgeometry.matching.geopandas import trails_from_flow

import network
from state import Session


_AVAILABLE: dict[str, list[str]] = {
    'sort_and_segment': (['cpp', 'python'] if _cpp is not None else ['python']),
    'compute_optimal_flow': (['cpp', 'python'] if _cpp_compute_optimal_flow is not None else ['python']),
}

_selection: dict[str, str] = {k: 'python' for k in _AVAILABLE}


def backend_state() -> dict:
    return {k: {'current': _selection[k], 'available': _AVAILABLE[k]} for k in _AVAILABLE}


def set_backend(key: str, value: str) -> None:
    if key not in _AVAILABLE or value not in _AVAILABLE[key]:
        raise ValueError(f"invalid backend: {key}={value!r}")
    _selection[key] = value


def run_matching(session: Session) -> dict:
    supply_pins = [p for p in session.pins if p.kind == 'supply' and p.road is not None]
    demand_pins = [p for p in session.pins if p.kind == 'demand' and p.road is not None]
    n = min(len(supply_pins), len(demand_pins))

    if n == 0:
        return {'pairs': [], 'trails': [], 'timing': {'translate_ms': 0, 'flow_ms': 0, 'trails_build_ms': 0, 'trails_ms': 0}}

    P = [(p.road, p.y) for p in supply_pins[:n]]
    Q = [(p.road, p.y) for p in demand_pins[:n]]

    roadnet = network._roadnet
    from_utm = network._from_utm

    compute_segments = (
        default_compute_segments if _selection['sort_and_segment'] == 'cpp'
        else compute_segments2
    )
    flow_solver = (
        None if _selection['compute_optimal_flow'] == 'cpp'
        else default_flow_solver
    )

    t0 = time.perf_counter()

    matching, flow = RoadnetMatchingProblem(
        P, Q, roadnet,
        compute_segments=compute_segments,
        flow_solver=flow_solver,
    ).compute_optimal_results(MatchingResult.MATCHING, MatchingResult.FLOW)

    t1 = time.perf_counter()

    trails_gdf = trails_from_flow(flow, P, Q, roadnet, compute_segments=compute_segments)

    t2 = time.perf_counter()

    pairs = [
        {'supply_id': supply_pins[i].id, 'demand_id': demand_pins[j].id}
        for i, j in matching
    ]
    trails = []
    for geom in trails_gdf.geometry:
        coords = []
        for x, y in geom.coords:
            lon, lat = from_utm.transform(x, y)
            coords.append((lat, lon))
        trails.append({'coordinates': coords})

    t3 = time.perf_counter()

    return {
        'pairs': pairs,
        'trails': trails,
        'timing': {
            'translate_ms': 0,
            'flow_ms': (t1 - t0) * 1000,
            'trails_build_ms': (t2 - t1) * 1000,
            'trails_ms': (t3 - t2) * 1000,
        },
    }
