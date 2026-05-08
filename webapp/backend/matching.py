import time

from setiptah.roadgeometry.geopandas import GeoFramesNetwork
from setiptah.roadgeometry.matching import RoadnetMatchingProblem, MatchingResult
from setiptah.roadgeometry.matching.bm import (
    _cpp, _cpp_compute_optimal_flow,
    compute_segments2, default_compute_segments, default_flow_solver,
)
from setiptah.roadgeometry.matching.geopandas import create_path_network_with_surplus
from setiptah.roadgeometry.dijkstra import RoadnetMetric, PointNode

import network
from state import Session


_AVAILABLE: dict[str, list[str]] = {
    'sort_and_segment': (['cpp', 'python'] if _cpp is not None else ['python']),
    'compute_optimal_flow': (['cpp', 'python'] if _cpp is not None else ['python']),
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
        return {'pairs': [], 'trails': [], 'timing': {'translate_ms': 0, 'flow_ms': 0, 'path_network_ms': 0, 'trails_ms': 0}}

    P = [(p.road, p.y) for p in supply_pins[:n]]
    Q = [(p.road, p.y) for p in demand_pins[:n]]

    roadnet = network._roadnet
    edges_gdf = network._edges
    from_utm = network._from_utm

    t0 = time.perf_counter()

    if _selection['compute_optimal_flow'] == 'cpp' and _cpp is not None:
        endpoints = {road: roadnet.endpoints(road) for road in roadnet.edges()}
        lengths   = {road: roadnet.length(road)    for road in roadnet.edges()}
        is_oneway = {road: roadnet.is_oneway(road) for road in roadnet.edges()}
        t_translated = time.perf_counter()
        matching, _ = _cpp.compute_matching(list(P), list(Q), endpoints, lengths, is_oneway)
        t1 = time.perf_counter()
        pathnet, segments_gdf = create_path_network_with_surplus(P, Q, roadnet, edges_gdf)
    else:
        t_translated = t0
        compute_segments = (
            default_compute_segments if _selection['sort_and_segment'] == 'cpp'
            else compute_segments2
        )
        matching, flow = RoadnetMatchingProblem(
            P, Q, roadnet,
            compute_segments=compute_segments,
            flow_solver=default_flow_solver,
        ).compute_optimal_results(MatchingResult.MATCHING, MatchingResult.FLOW)
        t1 = time.perf_counter()

        pin_roads = {road for road, _ in P + Q}
        flow_roads = {road for road, f in flow.items() if f != 0}
        relevant = pin_roads | flow_roads
        sub_edges = edges_gdf.loc[sorted(relevant, key=lambda r: edges_gdf.index.get_loc(r))]
        sub_nodes = network._nodes.loc[list(set(sub_edges['u']) | set(sub_edges['v']))]
        sub_roadnet = GeoFramesNetwork(
            edges_gdf=sub_edges, nodes_gdf=sub_nodes,
            left_col='u', right_col='v', oneway_col='oneway',
        )
        pathnet, segments_gdf = create_path_network_with_surplus(P, Q, sub_roadnet, sub_edges)

    path_metric = RoadnetMetric(pathnet)
    t2 = time.perf_counter()

    pairs = []
    trails = []
    for i, j in matching:
        pairs.append({
            'supply_id': supply_pins[i].id,
            'demand_id': demand_pins[j].id,
        })
        path = path_metric.graph_shortest_path(PointNode(P[i]), PointNode(Q[j]))
        coords = []
        if path:
            for geom in segments_gdf.loc[path[1::2]].geometry:
                for x, y in geom.coords:
                    lon, lat = from_utm.transform(x, y)
                    coords.append((lat, lon))
        trails.append({'coordinates': coords})
    t3 = time.perf_counter()

    return {
        'pairs': pairs,
        'trails': trails,
        'timing': {
            'translate_ms': (t_translated - t0) * 1000,
            'flow_ms': (t1 - t_translated) * 1000,
            'path_network_ms': (t2 - t1) * 1000,
            'trails_ms': (t3 - t2) * 1000,
        },
    }
