from collections import defaultdict
from dataclasses import dataclass
from typing import Any

import bintrees
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

from setiptah.roadgeometry.dijkstra import RoadSegment, RoadnetMetric
from setiptah.roadgeometry.draw import draw_planar_roadnet
from setiptah.roadgeometry.graphs import RoadNetwork
from setiptah.roadgeometry.legacy.conversion import multigraph_to_planar
from setiptah.roadgeometry.matching.nx_legacy import compute_segments
from setiptah.roadgeometry.protocol import Roadnet
from .matchvis_util import position, VERTEX, POINT_IN_S, POINT_IN_T
from .. import BiPartite

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


def SHOWTRAILS( S, T, assist, roadmap, pos, ax=None, **kwargs):
    """
    visualize a matching on a roadmap:
    """
    roadnet = multigraph_to_planar(roadmap, pos)
    
    # draw the roadmap
    if ax is None : ax = plt.gca()
    options = { 'edge_color' : 'g', 'alpha' : .15 }     # lightly, though...
    options.update( kwargs )                            # but let overrides

    draw_planar_roadnet(roadnet, ax=ax, **options)

    ax.set_aspect('equal')  # i just really like equal aspect...

    interval_graph = flow_to_interval_graph(assist, S, T, roadnet)

    SHOW_THICKNESS_GRAPH(interval_graph, S, T, roadmap, pos, ax)


def flow_to_interval_graph(flow, S, T, roadnet: Roadnet):

    """ The hard part is getting the edges with proper thickness """
    # sort points onto segments
    segments = compute_segments(S, T, roadnet)

    # initialize a path graph
    graph = nx.Graph()

    for road in roadnet.edges():
        u, v = roadnet.endpoints(road)
        it = iterate_segment(segments[road], u, v, roadnet.length(road))
        prev = next(it)
        z = flow[road]  # start road assistance +0
        for y2, type2, label2 in it:
            y1, type1, label1 = prev
            graph.add_edge((type1, label1), (type2, label2), weight=y2 - y1, score=abs(z))
            if type2 == POINT_IN_S:
                z += 1
            elif type2 == POINT_IN_T:
                z -= 1
            prev = y2, type2, label2

    return graph


def iterate_segment(segment, u, v, length: float):
    yield 0., VERTEX, u
    for y, q in segment.iter_items():
        q: BiPartite[list]
        for s in q.supply:
            yield y, POINT_IN_S, s
        for t in q.demand:
            yield y, POINT_IN_T, t
    yield length, VERTEX, v


def matching_to_interval_graph(matching, S, T, roadnet: Roadnet):
    """ The hard part is getting the edges with proper thickness """
    # sort points onto segments
    segments = compute_segments(S, T, roadnet)

    # make a path graph
    graph = nx.Graph()

    for road in roadnet.edges():
        u, v = roadnet.endpoints(road)
        it = iterate_segment(segments[road], u, v, roadnet.length(road))
        prev = next(it)
        for y2, type2, label2 in it:
            y1, type1, label1 = prev
            graph.add_edge((type1, label1), (type2, label2), weight=y2 - y1, score=0)
            prev = y2, type2, label2

    # add unit weight to shortest paths
    for i, j in matching:
        # TODO: Pretty sure the oneway bug is here.
        path = nx.shortest_path(graph, (POINT_IN_S, i), (POINT_IN_T, j), weight='weight')

        for ii, jj in zip(path[:-1], path[1:]):
            data = graph.get_edge_data(ii, jj)
            data['score'] += 1

    return graph


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


def SHOW_THICKNESS_GRAPH( graph, S, T, roadmap, pos, ax ) :            

    # position utilities
    def vertpos(u) : return pos[u]
    def pos_from_S(u) : return position(S[u], roadmap, pos)
    def pos_from_T(u) : return position(T[u], roadmap, pos)
    switch = {VERTEX: vertpos,
              POINT_IN_S: pos_from_S,
              POINT_IN_T: pos_from_T}

    other_pos = {}
    for uu in graph.nodes() :
        typeu, labelu = uu
        other_pos[uu] = switch[typeu]( labelu )

    trailnet = RoadNetwork()
    trail_pos = {}
    trail_weight = {}

    # plot edges in graph with variable thickness? or some other visual cue
    for k, (uu, vv, data) in enumerate(graph.edges(data=True)):
        score = data['score']
        if score <= 0: continue  # would just waste effort

        posu = other_pos[uu]
        posv = other_pos[vv]

        does_it_matter = 1
        trailnet.add_edge(k, uu, vv, does_it_matter)
        trail_weight[k] = score

        trail_pos[uu] = posu
        trail_pos[vv] = posv

    # Draw the trails.
    show_trails(trail_weight, trailnet, trail_pos, ax)

    # plot the points on top, so visible
    scatter_points_with_legacy_embedding(S, T, roadmap, pos, ax)


def show_match(matching, S, T, roadmap, pos, ax=None, **kwargs):
    roadnet = multigraph_to_planar(roadmap, pos)

    # draw the roadmap
    if ax is None : ax = plt.gca()
    options = { 'edge_color' : 'g', 'alpha' : .15 }     # lightly, though...
    options.update( kwargs )                            # but let overrides

    draw_planar_roadnet(roadnet, ax=ax, **options)

    ax.set_aspect('equal')  # i just like equal aspect...

    interval_graph, weights = matching_to_interval_graph2(matching, S, T, roadnet)

    show_thickness_graph(interval_graph, weights, S, T, roadnet, pos, ax)


def show_thickness_graph(interval_metric, weights, S, T, roadnet: Roadnet, pos, ax):

    def singleton(factory):
        return factory()

    @singleton
    class pos_:

        def __getitem__(self, item):
            if isinstance(item, VertexNode):
                return pos[item.vertex]
            elif isinstance(item, PointNode):
                return point_embedding(item.point, roadnet, pos)

    show_trails(weights, interval_metric.roadnet, pos_, ax)

    def embedding_fn(p):
        return point_embedding(p, roadnet, pos)

    scatter_points(S, T, embedding_fn, ax)


def show_trails(weights, roadnet: Roadnet, pos, ax):
    # plot edges in graph with variable thickness? or some other visual cue

    for edge in roadnet.edges():
        uu, vv = roadnet.endpoints(edge)
        score = weights[edge]
        if score <= 0:
            continue  # would just waste effort

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


def scatter_points_with_legacy_embedding(S, T, roadmap, pos, ax):
    embedding_fn = lambda addr: position(addr, roadmap, pos)
    return scatter_points(S, T, embedding_fn, ax)


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


def matching_to_interval_graph2(matching, S, T, roadnet: Roadnet):
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
