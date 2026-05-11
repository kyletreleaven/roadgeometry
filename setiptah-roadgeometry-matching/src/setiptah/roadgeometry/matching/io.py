"""Serialization helpers for road network matching data.

JSON helpers (roadnet_to_json / roadnet_from_json / point_set_*) use a
human-readable dict format suitable for small instances and debugging logs.

Numpy helpers (save_case / load_case) write a compact .npz archive where
road and vertex IDs are normalized to contiguous integers.  Use these for
capturing app sessions as self-contained unit-test fixtures.
"""
import json

from setiptah.roadgeometry.graphs import RoadNetwork


def _id_to_json(id_):
    return id_ if isinstance(id_, (str, int)) else f"{id_}"


def _id_from_json(raw):
    if isinstance(raw, int):
        return raw
    try:
        return int(raw)
    except (ValueError, TypeError):
        return raw


def roadnet_to_json(roadnet) -> dict:
    """Serialize a Roadnet to a JSON-compatible dict.

    Returns:
        {
            "edges":   { "e0": [tail, head], ... },
            "lengths": { "e0": 1.23, ... },
        }
    along with an edge_to_id mapping as a second return value, needed
    to serialize point sets and flows consistently.
    """
    edge_list  = list(roadnet.edges())
    edge_to_id = {
        e: str(e) if isinstance(e, (str, int)) else f"e{i}"
        for i, e in enumerate(edge_list)
    }

    edges   = {}
    lengths = {}
    for e in edge_list:
        eid = edge_to_id[e]
        tail, head = roadnet.endpoints(e)
        edges[eid]   = [_id_to_json(tail), _id_to_json(head)]
        lengths[eid] = roadnet.length(e)

    return {"edges": edges, "lengths": lengths}, edge_to_id


def roadnet_from_json(d: dict) -> RoadNetwork:
    """Reconstruct a RoadNetwork from a roadnet_to_json dict."""
    rn      = RoadNetwork()
    lengths = d.get("lengths", {})
    for eid_str, (tail_raw, head_raw) in d["edges"].items():
        eid  = _id_from_json(eid_str)
        tail = _id_from_json(tail_raw)
        head = _id_from_json(head_raw)
        rn.add_edge(eid, tail, head, lengths.get(eid_str, 1.0))
    return rn


def point_set_to_json(points, edge_to_id: dict) -> list:
    """Serialize a list of (edge, offset) pairs using the given edge_to_id mapping."""
    return [[edge_to_id[e], y] for e, y in points]


def point_set_from_json(data: list) -> list:
    """Deserialize a point set to a list of (edge_id, offset) pairs."""
    return [(_id_from_json(eid), y) for eid, y in data]


# ---------------------------------------------------------------------------
# Numpy .npz case format
# ---------------------------------------------------------------------------

def save_instance_npz(path: str, P, Q, roadnet) -> None:
    """Save a matching instance to a compressed numpy archive.

    Road and vertex IDs are normalized to contiguous integers so the file is
    self-contained and loadable without the original network object.
    """
    import numpy as np

    nodes = list(roadnet.nodes())
    node_idx = {n: i for i, n in enumerate(nodes)}

    roads = list(roadnet.edges())
    road_idx = {r: i for i, r in enumerate(roads)}

    edge_u      = np.array([node_idx[roadnet.endpoints(r)[0]] for r in roads], dtype=np.int64)
    edge_v      = np.array([node_idx[roadnet.endpoints(r)[1]] for r in roads], dtype=np.int64)
    edge_length = np.array([roadnet.length(r)    for r in roads], dtype=np.float64)
    edge_oneway = np.array([roadnet.is_oneway(r) for r in roads], dtype=bool)

    np.savez_compressed(
        path,
        supply_road=np.array([road_idx[r] for r, _ in P], dtype=np.int64),
        supply_y   =np.array([y           for _, y in P], dtype=np.float64),
        demand_road=np.array([road_idx[r] for r, _ in Q], dtype=np.int64),
        demand_y   =np.array([y           for _, y in Q], dtype=np.float64),
        edge_u=edge_u,
        edge_v=edge_v,
        edge_length=edge_length,
        edge_oneway=edge_oneway,
        n_vertices=np.array([len(nodes)], dtype=np.int64),
    )


def load_instance_npz(path: str):
    """Load a matching instance from a compressed numpy archive.

    Returns (P, Q, roadnet) where roadnet is an IntRoadnet with roads 0..N-1
    and vertices 0..M-1.
    """
    import numpy as np
    from setiptah.roadgeometry.graphs import IntRoadnet, RoadInfo

    data = np.load(path)
    roads = tuple(
        RoadInfo(float(l), int(u), int(v), bool(ow))
        for l, u, v, ow in zip(
            data['edge_length'], data['edge_u'], data['edge_v'], data['edge_oneway'],
        )
    )
    roadnet = IntRoadnet(roads=roads, n_vertices=int(data['n_vertices'][0]))
    P = list(zip(data['supply_road'].tolist(), data['supply_y'].tolist()))
    Q = list(zip(data['demand_road'].tolist(), data['demand_y'].tolist()))
    return P, Q, roadnet
