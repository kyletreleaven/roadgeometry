import bintrees
import pandas as pd
from shapely import Point, LineString

import geopandas as gpd
from setiptah.roadgeometry.dijkstra import VertexNode, PointNode
from setiptah.roadgeometry.graphs import RoadNetwork
from setiptah.roadgeometry.matching import compute_segments2
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
