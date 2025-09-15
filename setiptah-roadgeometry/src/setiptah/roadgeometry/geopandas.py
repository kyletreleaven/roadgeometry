from dataclasses import dataclass
from typing import Collection

import bintrees
import pandas as pd
from shapely import Point, LineString

import geopandas as gpd
from setiptah.roadgeometry.protocol import Roadnet, TRoad, TVert


@dataclass(frozen=True)
class GeoFramesNetwork(Roadnet):
    """A road network defined by two GeoDFs, one for edges and one for nodes."""
    edges_gdf: gpd.GeoDataFrame
    nodes_gdf: gpd.GeoDataFrame
    left_col: str
    right_col: str
    oneway_col: str = None
    oneway_default: bool = False

    def edges(self) -> Collection[TRoad]:
        return self.edges_gdf.index

    def out_edges(self, u: TVert) -> Collection[TRoad]:
        return self.edges_gdf[self.edges_gdf[self.left_col] == u].index

    def in_edges(self, u: TVert) -> Collection[TRoad]:
        return self.edges_gdf[self.edges_gdf[self.right_col] == u].index

    def nodes(self) -> Collection[TVert]:
        return self.nodes_gdf.index

    def endpoints(self, road: TRoad) -> tuple[TVert, TVert]:
        return (
            self.edges_gdf[self.left_col].loc[road],
            self.edges_gdf[self.right_col].loc[road],
        )

    def length(self, road: TRoad) -> float:
        return self.edges_gdf.geometry.loc[road].length

    def is_oneway(self, road: TRoad) -> bool:
        assert road in self.edges_gdf.index
        if self.oneway_col is None:
            return self.oneway_default
        return self.edges_gdf[self.oneway_col].loc[road]


@dataclass(frozen=True)
class GeoFramesRoads:
    edges_df: pd.DataFrame
    left_col: str
    right_col: str
    oneway_col: str = None
    oneway_default: bool = False

    def straight_lines_on(self, nodes: gpd.GeoDataFrame) -> GeoFramesNetwork:
        """Construct a GeoFramesNetwork on input nodes by creating straight-line paths."""

        def path(row):
            p1 = nodes.geometry.loc[row[self.left_col]]
            p2 = nodes.geometry.loc[row[self.right_col]]
            return LineString((p1, p2))

        edges_gdf = gpd.GeoDataFrame(
            self.edges_df, geometry=self.edges_df.apply(path, axis=1), crs=nodes.crs
        )

        return GeoFramesNetwork(
            edges_gdf, nodes, self.left_col, self.right_col, self.oneway_col, self.oneway_default
        )



def points_to_gdf(
        points,
        edges_gdf: gpd.GeoDataFrame,
        *,
        edge_col: str = "edge",
        coord_col: str = "coord",
        point_col: str = "point",
):
    """Embed road addresses in the plane using edge interpolation."""
    df = pd.DataFrame.from_records(
        {
            edge_col: e,
            coord_col: x
        }
        for e, x in points
    )

    def get_point(row):
        road = row[edge_col]
        x = row[coord_col]
        return edges_gdf.geometry.loc[road].interpolate(x)

    return gpd.GeoDataFrame(
        df.assign(**{
            point_col: lambda df_: df_.apply(get_point, axis=1)
        }),
        geometry=point_col,
        crs=edges_gdf.crs
    )
