import bintrees
import pandas as pd
from shapely import Point, LineString

import geopandas as gpd
from setiptah.roadgeometry.dijkstra import VertexNode, PointNode
from setiptah.roadgeometry.geopandas import GeoFramesNetwork
from setiptah.roadgeometry.graphs import RoadNetwork
from setiptah.roadgeometry.matching.bm import compute_segments2
from setiptah.roadgeometry.protocol import Roadnet
from setiptah.roadgeometry.util.shapely import crop_line_string


def create_path_network_with_surplus(P, Q, roadnet: Roadnet, edges: gpd.GeoDataFrame):
    """Create a geometric path network, with surplus, from supply and demand on a geometric roadnet."""

    out = RoadNetwork()
    for u in roadnet.nodes():
        out.add_node(VertexNode(u))

    segment_dict = compute_segments2(P, Q, roadnet)

    records = []

    for road in roadnet.edges():
        u, v, length, oneway = (
            *roadnet.endpoints(road), roadnet.length(road), roadnet.is_oneway(road)
        )
        segment = segment_dict[road]

        def create_edge(prev, curr, F):
            prev_y, u = prev
            curr_y, v = curr

            road_idx = len(records)
            out.add_edge(road_idx, u, v, curr_y - prev_y, oneway=oneway)

            records.append(
                dict(
                    road=road, start=prev_y, end=curr_y,
                    F=F,
                    path=crop_line_string(edges.geometry.loc[road], prev_y, curr_y),
                )
            )

        F = 0
        prev = 0., VertexNode(u)
        for y, bip in segment:
            p = road, y
            curr = y, PointNode(p)
            create_edge(prev, curr, F)
            prev = curr
            F += len(bip.supply) - len(bip.demand)
        create_edge(prev, (length, VertexNode(v)), F)

    return out, gpd.GeoDataFrame(pd.DataFrame.from_records(records), geometry="path", crs=edges.crs)


# TODO: add an overload / alternate signature that accepts a pre-computed segment_dict
# so callers that already ran compute_segments (e.g. during flow computation) can skip the repeat.
def trails_from_flow(flow, P, Q, roadnet: GeoFramesNetwork, compute_segments=compute_segments2) -> gpd.GeoDataFrame:
    """Convert an optimal flow to a GeoDataFrame of trail segments.

    Only roads carrying flow or containing pins are processed.  The caller
    supplies the full roadnet; this function restricts internally.

    Returns a GeoDataFrame (one row per sub-segment with nonzero running flow)
    in the roadnet's CRS.  Apply coordinate transforms afterward.
    """
    pin_roads  = {road for road, _ in [*P, *Q]}
    flow_roads = {road for road, f in flow.items() if f != 0}
    relevant   = pin_roads | flow_roads

    sub_edges = roadnet.edges_gdf.loc[list(relevant)]
    sub_nodes = roadnet.nodes_gdf.loc[list(
        {*sub_edges[roadnet.left_col], *sub_edges[roadnet.right_col]}
    )]
    sub_roadnet = GeoFramesNetwork(
        edges_gdf=sub_edges, nodes_gdf=sub_nodes,
        left_col=roadnet.left_col, right_col=roadnet.right_col,
        oneway_col=roadnet.oneway_col,
    )

    segment_dict = compute_segments(P, Q, sub_roadnet)

    records = []
    for road in relevant:
        seg  = segment_dict.get(road, [])
        geom = sub_edges.geometry.loc[road]
        z    = flow.get(road, 0)
        prev_y = 0.
        for y, pts in seg:
            if z != 0:
                records.append({'geometry': crop_line_string(geom, prev_y, y), 'flow': z})
            z += len(pts.supply) - len(pts.demand)
            prev_y = y
        if z != 0:
            records.append({'geometry': crop_line_string(geom, prev_y, geom.length), 'flow': z})

    return gpd.GeoDataFrame(records, geometry='geometry', crs=roadnet.edges_gdf.crs)
