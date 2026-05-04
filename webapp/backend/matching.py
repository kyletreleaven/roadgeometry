from setiptah.roadgeometry.matching import RoadnetMatchingProblem, MatchingResult
from setiptah.roadgeometry.matching.geopandas import create_path_network_with_surplus
from setiptah.roadgeometry.dijkstra import RoadnetMetric, PointNode

import network
from state import Session


def run_matching(session: Session) -> dict:
    supply_pins = [p for p in session.pins if p.kind == 'supply' and p.road is not None]
    demand_pins = [p for p in session.pins if p.kind == 'demand' and p.road is not None]
    n = min(len(supply_pins), len(demand_pins))

    if n == 0:
        return {'pairs': [], 'trails': []}

    P = [(p.road, p.y) for p in supply_pins[:n]]
    Q = [(p.road, p.y) for p in demand_pins[:n]]

    roadnet = network._roadnet
    edges_gdf = network._edges
    from_utm = network._from_utm

    matching = RoadnetMatchingProblem(P, Q, roadnet).compute_optimal(MatchingResult.MATCHING)

    pathnet, segments_gdf = create_path_network_with_surplus(P, Q, roadnet, edges_gdf)
    path_metric = RoadnetMetric(pathnet)

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

    return {'pairs': pairs, 'trails': trails}
