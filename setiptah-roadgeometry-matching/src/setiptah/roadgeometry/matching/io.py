"""JSON serialization helpers for road network matching data.

Usage pattern:
    doc = {
        "roadnet": roadnet_to_json(rn),
        "supply":  point_set_to_json(PP, rn),
        "demand":  point_set_to_json(QQ, rn),
        "flow":    {edge_to_id[e]: v for e, v in flow.items()},
    }
    json.dump(doc, fp)

    doc = json.load(fp)
    rn = roadnet_from_json(doc["roadnet"])
    PP = point_set_from_json(doc["supply"])
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
