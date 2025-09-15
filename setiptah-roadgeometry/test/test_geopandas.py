import bintrees
import geopandas as gpd
import pandas as pd
import pytest
from shapely import Point, LineString

from setiptah.roadgeometry.geopandas import GeoFramesRoads, points_to_gdf


def create_single_road_network():

    nodes = gpd.GeoDataFrame(
        {
            "point": [Point(-1, 8), Point(0, 0), Point(10, 0), Point(11, 8)]
        },
        # index=[0, 1],
        geometry="point",
        crs=None,
    )

    topology = pd.DataFrame(
        {
            "u": [0, 1, 2,],
            "v": [1, 2, 3,],
        },
        index=["L", "M", "R"]
    )

    return GeoFramesRoads(topology, "u", "v").straight_lines_on(nodes)


@pytest.mark.parametrize("embed_opts, edge_key", [
    ({}, "edge"),
    (dict(edge_col="road"), "road"),
])
def test_points_to_gdf(embed_opts, edge_key):

    rn = create_single_road_network()

    points = [("L", 4), ("M", 7.5), ("R", 1)]
    points_ = points_to_gdf(points, rn.edges_gdf, **embed_opts)

    for _, row in points_.iterrows():
        assert (row[edge_key], row.coord) in points
        if row[edge_key] == "M":
            assert row.point == Point(7.5, 0)
