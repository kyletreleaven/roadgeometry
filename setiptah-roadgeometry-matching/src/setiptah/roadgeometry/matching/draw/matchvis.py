from collections import defaultdict
from dataclasses import dataclass
from typing import Any

import bintrees
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

from setiptah.roadgeometry.dijkstra import RoadSegment, RoadnetMetric
from setiptah.roadgeometry.draw import draw_planar_roadnet
from setiptah.roadgeometry.formats import from_networkx
from setiptah.roadgeometry.graphs import RoadNetwork
from setiptah.roadgeometry.matching import bm, mygraph
from setiptah.roadgeometry.protocol import Roadnet, Topology
from setiptah.roadgeometry.planar import PlanarRoadnet

""" CONSTANTS """

""" labels for three kinds of graph nodes """

ZNODES = 1
ZLABELS = 2
ZEDGES = 3
ZTRAILS = 4
ZPOINTS = 5


def pointsToXY( points ) :
    """ split a list of (x,y) coordinates into X and Y; usually for plotting """
    X = [ x for x,y in points ]
    Y = [ y for x,y in points ]
    return X, Y


def show_flow(flow, S, T, roadnet: Roadnet, pos, ax=None, **kwargs):
    # draw the roadmap
    if ax is None : ax = plt.gca()
    options = { 'edge_color' : 'g', 'alpha' : .15 }     # lightly, though...
    options.update( kwargs )                            # but let overrides

    draw_planar_roadnet(PlanarRoadnet.embed_topology(roadnet, pos), ax=ax, **options)

    ax.set_aspect('equal')  # i just really like equal aspect...

    intervals, weights = flow_to_interval_graph(flow, S, T, roadnet)
    show_thickness_graph(intervals, weights, S, T, roadnet, pos, ax)


def matching_to_flow(matching, S, T, roadnet):
    flow = {road: 0. for road in roadnet.edges()}

    metric = RoadnetMetric(roadnet)
    for i, j in matching:
        segments: list[RoadSegment] = metric.shortest_path(S[i], T[j])

        assert segments is not None  # don't support infeasible
        if len(segments) < 2:  # single-segment, no flow created.
            continue

        if segment_is_backwards(segments[0]):
            flow[segments[0].road] -= 1
        for seg in segments[1:-1]:
            flow[seg.road] += (-1 if segment_is_backwards(seg) else +1)
        if not segment_is_backwards(segments[-1]):
            flow[segments[-1].road] += 1

    return flow


def segment_is_backwards(segment: RoadSegment):
    return segment.end < segment.start  # if they're equal it's forward w.l.g.


def show_matching(matching, S, T, roadmap, pos, ax=None, **kwargs):
    roadnet_ = from_networkx(roadmap)
    roadnet = PlanarRoadnet.embed_topology(roadnet_, pos)

    # draw the roadmap
    if ax is None : ax = plt.gca()
    options = { 'edge_color' : 'g', 'alpha' : .15 }     # lightly, though...
    options.update( kwargs )                            # but let overrides

    draw_planar_roadnet(roadnet, ax=ax, **options)

    ax.set_aspect('equal')  # i just like equal aspect...

    interval_metric, weights = matching_to_interval_graph(matching, S, T, roadnet)

    show_thickness_graph(interval_metric.roadnet, weights, S, T, roadnet, pos, ax)


def show_thickness_graph(intervals: Topology, weights: dict, S, T, roadnet: Roadnet, pos, ax):

    def singleton(factory):
        return factory()

    @singleton
    class pos_:

        def __getitem__(self, item):
            if isinstance(item, VertexNode):
                return pos[item.vertex]
            elif isinstance(item, PointNode):
                return point_embedding(item.point, roadnet, pos)

    show_trails(intervals, weights, pos_, ax)

    def embedding_fn(p):
        return point_embedding(p, roadnet, pos)

    scatter_points(S, T, embedding_fn, ax)


def show_trails(intervals: Topology, weights, pos, ax):
    # plot edges in graph with variable thickness? or some other visual cue

    for edge in intervals.edges():
        uu, vv = intervals.endpoints(edge)
        score = weights[edge]
        if score <= 0:
            continue  # don't draw

        posu = pos[uu]
        posv = pos[vv]

        xu, yu = posu
        xv, yv = posv
        options = {'color': 'k',
                   'alpha': .6,
                   'linewidth': score
                   }
        ax.plot([xu, xv], [yu, yv], solid_capstyle='butt',
                # butt style prevents awkward overlap of segments
                zorder=ZTRAILS,
                **options)


def scatter_points(S, T, embedding_fn, ax):
    # show S points in red
    positions = [embedding_fn(addr) for addr in S]
    options = {
               #'marker' : 'x',
               's' : 80
               }
    X, Y = pointsToXY( positions )
    ax.scatter( X, Y, color='r', zorder=ZPOINTS, marker='x', **options )
    # show T points in blue
    positions = [embedding_fn(addr) for addr in T]
    X, Y = pointsToXY( positions )
    ax.scatter( X, Y, color='b', zorder=ZPOINTS, marker='$\\circ$', **options )


def point_embedding(p, roadnet: Roadnet, pos):
    road, x = p
    pi, pj = [np.array(pos[i]) for i in roadnet.endpoints(road)]
    return pi + x * (pj - pi) / roadnet.length(road)


@dataclass(frozen=True)
class VertexNode:
    vertex: Any

    def element(self):
        return


@dataclass(frozen=True)
class PointNode:
    point: Any


def create_path_network(points, roadnet: Roadnet):
    out = RoadNetwork()
    for u in roadnet.nodes():
        out.add_node(VertexNode(u))

    # collect all points
    segments = defaultdict(bintrees.RBTree)
    for road, y in points:
        segments[road][y] = None  # just a set really

    def create_edge(u, v, length, oneway):
        road_idx = len(out.edges())
        out.add_edge(road_idx, u, v, length, oneway=oneway)

    for road in roadnet.edges():
        seg = segments[road]
        u, v = roadnet.endpoints(road)
        length, oneway = roadnet.length(road), roadnet.is_oneway(road)

        prev_node, prev_y = VertexNode(u), 0.
        for curr_y, _ in seg.iter_items():
            p = road, curr_y
            curr_node = PointNode(p)
            create_edge(prev_node, curr_node, curr_y - prev_y, oneway)
            prev_node, prev_y = curr_node, curr_y
        create_edge(prev_node, VertexNode(v), length - prev_y, oneway)

    return out


def matching_to_interval_graph(matching, S, T, roadnet: Roadnet):
    """ The hard part is getting the edges with proper thickness """
    # sort points onto segments

    def all_points():
        yield from S
        yield from T

    pathnet = create_path_network(all_points(), roadnet)
    metric = RoadnetMetric(pathnet)

    # add unit weight to shortest paths
    weight = defaultdict(int)

    for i, j in matching:
        p, q = S[i], T[j]
        path = metric.graph_shortest_path(PointNode(p), PointNode(q))
        edges = path[1::2]
        for edge in edges:
            weight[edge] += 1

    return metric, weight


def flow_to_interval_graph(flow, S, T, roadnet: Roadnet) -> tuple[mygraph, dict]:
    out = mygraph()
    weights_out = {}

    def create_edge(u, v, weight):
        road_idx = len(out.edges())
        out.add_edge(road_idx, u, v)
        weights_out[road_idx] = abs(weight)  # This is for drawing, not traversal.

    for u in roadnet.nodes():
        out.add_node(VertexNode(u))

    segments = bm.compute_segments2(S, T, roadnet)

    for road in roadnet.edges():
        seg = segments[road]
        u, v = roadnet.endpoints(road)

        z = flow[road]  # start road assistance +0

        prev_node = VertexNode(u)
        for y, pts in seg:
            p = road, y
            curr_node = PointNode(p)
            create_edge(prev_node, curr_node, z)
            prev_node = curr_node
            z += len(pts.supply) - len(pts.demand)
        create_edge(prev_node, VertexNode(v), z)

    return out, weights_out
