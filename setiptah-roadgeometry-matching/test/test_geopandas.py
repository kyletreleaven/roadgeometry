import bintrees
import geopandas as gpd
import numpy as np
import pandas as pd
from shapely import Point, LineString

from setiptah.roadgeometry.geopandas import GeoFramesRoads
from setiptah.roadgeometry.matching.bm import compute_roadnet_objective_fns
from setiptah.roadgeometry.matching.geopandas import create_path_network_with_surplus


def test_create_path_network_with_surplus():

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

    rn = GeoFramesRoads(topology, "u", "v").straight_lines_on(nodes)

    supply = sorted(("M", x) for x in rn.length("M") * np.random.rand(5))
    demand = sorted(("M", x) for x in rn.length("M") * np.random.rand(5))

    pathnet, segment_gdf = create_path_network_with_surplus(supply, demand, rn, rn.edges_gdf)

    obj_fns = compute_roadnet_objective_fns(supply, demand, rn)
    obj_fn = obj_fns["M"]

    road = segment_gdf[segment_gdf.road == "M"]

    def graph_cost(z):
        occupancy = (road.F + z).apply(abs)
        return (road.geometry.length * occupancy).sum()

    cost_vertices = list(obj_fn.lines.keys())[1::]

    assert min(
        abs(graph_cost(z) - obj_fn(z))
        for z in cost_vertices
    ) < 1e-10
