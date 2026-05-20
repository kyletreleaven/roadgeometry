#!/usr/bin/env python3
"""Dump the captured Cambridge matching instance to JSON for the C++ bench.

Run once from the repo root via the bench nox session:
    nox -s bench -- bench/dump_captured.py [output_path]

Output defaults to bench/captured_instance.json.
"""
import json
import pathlib
import sys

_BACKEND = pathlib.Path(__file__).parent.parent / "webapp/backend"
_PINS    = _BACKEND / "captured_pins.json"
_GRAPHML = _BACKEND / "cambridge.graphml"
_OUT     = pathlib.Path(__file__).parent / "captured_instance.json"

out_path = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else _OUT

import osmnx as ox
from setiptah.roadgeometry.geopandas import GeoFramesNetwork
from setiptah.roadgeometry.matching.io import load_pins

print("Loading network...")
G = ox.load_graphml(_GRAPHML)
nodes, edges = ox.graph_to_gdfs(G)
edges['u'] = edges.index.get_level_values('u')
edges['v'] = edges.index.get_level_values('v')
roadnet = GeoFramesNetwork(edges_gdf=edges, nodes_gdf=nodes,
                           left_col='u', right_col='v', oneway_col='oneway')

P, Q = load_pins(_PINS)

# Normalize IDs to contiguous integers (road: 0..R-1, vertex: 0..V-1)
nodes_list = list(roadnet.nodes())
node_idx   = {n: i for i, n in enumerate(nodes_list)}

roads_list = list(roadnet.edges())
road_idx   = {r: i for i, r in enumerate(roads_list)}

endpoints, lengths, is_oneway = roadnet.graph_props(roads_list)

data = {
    "n_vertices":  len(nodes_list),
    "edge_u":      [node_idx[endpoints[r][0]] for r in roads_list],
    "edge_v":      [node_idx[endpoints[r][1]] for r in roads_list],
    "edge_length": [lengths[r]   for r in roads_list],
    "edge_oneway": [bool(is_oneway[r]) for r in roads_list],
    "supply_road": [road_idx[r] for r, _ in P],
    "supply_y":    [y           for _, y in P],
    "demand_road": [road_idx[r] for r, _ in Q],
    "demand_y":    [y           for _, y in Q],
}

with open(out_path, 'w') as f:
    json.dump(data, f)

print(f"Roads: {len(roads_list)}, vertices: {len(nodes_list)}, "
      f"supply: {len(P)}, demand: {len(Q)}")
print(f"Written: {out_path}")
