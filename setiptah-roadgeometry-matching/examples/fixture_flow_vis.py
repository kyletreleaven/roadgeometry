"""Visualize the topograph_imbalance fixture: road network + flow thickness.

Usage:
    python examples/fixture_flow_vis.py
    python examples/fixture_flow_vis.py --save figure.png
"""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import networkx as nx

from setiptah.roadgeometry.matching.io import (
    roadnet_from_json,
    point_set_from_json,
)
from setiptah.roadgeometry.matching.draw.matchvis import show_flow

FIXTURE = Path(__file__).parent.parent / "test" / "fixtures" / "topograph_imbalance.json"


def load_fixture(path):
    with open(path) as f:
        doc = json.load(f)
    roadnet = roadnet_from_json(doc["roadnet"])
    supply  = point_set_from_json(doc["supply"])
    demand  = point_set_from_json(doc["demand"])
    flow    = dict(doc["flow"])
    return roadnet, supply, demand, flow


def roadnet_to_nx(roadnet):
    g = nx.DiGraph()
    for road in roadnet.edges():
        u, v = roadnet.endpoints(road)
        g.add_edge(u, v)
    return g


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", default=str(FIXTURE))
    parser.add_argument("--save", default=None, help="save figure to this path instead of showing")
    args = parser.parse_args()

    roadnet, supply, demand, flow = load_fixture(args.fixture)

    g = roadnet_to_nx(roadnet)
    pos = nx.kamada_kawai_layout(g)

    fig, ax = plt.subplots(figsize=(8, 8))
    show_flow(flow, supply, demand, roadnet, pos, ax=ax)
    ax.set_title("fixture: topograph_imbalance — flow thickness")

    if args.save:
        fig.savefig(args.save, bbox_inches="tight")
        print(f"saved to {args.save}")
    else:
        plt.show()
