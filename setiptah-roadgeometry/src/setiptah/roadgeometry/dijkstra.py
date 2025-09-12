from collections import defaultdict
from dataclasses import dataclass
from functools import cached_property
from typing import Generator, NamedTuple, Optional, Any

import bintrees
import numpy as np

from .graphs import RoadNetwork
from .protocol import *
from .util.priodict import *

SourceVertex = TVert
TargetVertex = TVert

Point = Roadnet[TRoad, TVert].Point


class RoadSegment(NamedTuple, Generic[TRoad]):
    road: TRoad
    start: float
    end: float


RoadPath = Optional[list[RoadSegment]]


@dataclass(frozen=True)
class RoadnetQuery(Generic[TRoad, TVert]):
    roadnet: Roadnet[TRoad, TVert]

    def embeddings(self, u: TVert) -> Generator[Point, None, None]:
        for e in self.roadnet.out_edges(u):
            yield e, 0.
        for e in self.roadnet.in_edges(u):
            yield e, self.roadnet.length(e)

    def embedding(self, u: TVert, road: TRoad) -> Point:
        i, j = self.roadnet.endpoints(road)
        if u == i:
            return road, 0.
        if u == j:
            return road, self.roadnet.length(road)
        raise ValueError(f"Node {u} not incident to road {road}.")

    def _out_edges(self, i: TVert):
        for e in self.roadnet.out_edges(i):
            _, j = self.roadnet.endpoints(e)
            yield e, j
        for e in self.roadnet.in_edges(i):
            if self.roadnet.is_oneway(e):
                continue
            j, _ = self.roadnet.endpoints(e)
            yield e, j

    def is_valid_point(self, p: Point):
        road, _ = p
        return self.is_on_road(p, road)

    def is_on_road(self, p: Point, road: TRoad) -> bool:
        road_, x = p
        return (
            road_ == road
            and 0. <= x <= self.roadnet.length(road)
        )


class RoadnetMetric(RoadnetQuery[TRoad, TVert]):

    @cached_property
    def _distance(self) -> dict[SourceVertex, dict[TargetVertex, float]]:
        return {}

    @cached_property
    def _upstream(self) -> dict[SourceVertex, dict[TargetVertex, TVert]]:
        return {}

    def graph_shortest_path_length(self, source: SourceVertex, target: TargetVertex) -> float:
        self._ensure(source)
        return self._distance[source].get(target, np.inf)

    def graph_shortest_path(self, source: SourceVertex, target: TargetVertex):
        self._ensure(source)
        upstream = self._upstream[source]

        path = [target]
        j = target
        while j in upstream:
            _e, i = tup = upstream[j]
            path.extend(tup)
            j = i
        assert j == source, (j, source)

        path.reverse()
        return path

    def _ensure(self, source: SourceVertex):
        if source not in self._distance:
            self._populate_dijkstra(source)

    def _populate_dijkstra(self, source: SourceVertex):
        d = {}  # only in here if they are seen... duh!!
        upstream = {}

        OPEN = priorityDictionary()
        OPEN[source] = 0.

        while len(OPEN) > 0:
            i = OPEN.smallest()
            d[i] = di = OPEN[i]
            del OPEN[i]

            for e, j in self._out_edges(i):
                if j in d:
                    continue

                dj = di + self.roadnet.length(e)
                if dj < OPEN.get(j, np.inf):
                    OPEN[j] = dj
                    upstream[j] = e, i

        self._distance[source] = d
        self._upstream[source] = upstream

    def distance(self, p: Point, q: Point) -> float:
        """

        returns the shortest-path distance between two points on a Roadmap

        """
        return self._shortest_path(p, q).distance

    def shortest_path(self, p: Point, q: Point) -> RoadPath:
        return self._shortest_path(p, q).path

    @cached_property
    def _shortest_path(self):

        metric = self

        @dataclass
        class _Path:
            p: Point
            q: Point

            @cached_property
            def _result(self):
                p, q, roadnet = self.p, self.q, metric.roadnet

                assert metric.is_valid_point(p)
                road1, start1 = p

                i, j = roadnet.endpoints(road1)
                nodes = [i, j]
                points = [(road1, 0.), (road1, roadnet.length(road1))]

                def place_of(x):
                    yield x

                options = [
                    (
                        metric.distance_on_road(p, p_, road1) + d2,
                        (u, v)
                    )
                    for u, p_ in zip(nodes, points)
                    for d2, v in place_of(metric._shortest_path_node_to_point(u, q)._result)
                ]

                d_off, (u, v) = min(options, key=lambda pair: pair[0])
                d_on, path_on = metric._shortest_path_on_road(p, q, road1)

                if d_off < d_on:
                    return d_off, (u, v)
                else:
                    return d_on, path_on

            @property
            def distance(self):
                d, _ = self._result
                return d

            @property
            def path(self):
                _, path_ = self._result

                if path_ is None:  # infeasible
                    return

                try:
                    u, v = path_
                except ValueError:  # path on one road
                    return path_

                road1, start1 = self.p
                _, end1 = metric.embedding(u, road1)
                road2, end2 = self.q
                _, start2 = metric.embedding(v, road2)

                return [
                    RoadSegment(road1, start1, end1),
                    *metric.graph_path_to_road_path(metric.graph_shortest_path(u, v)),
                    RoadSegment(road2, start2, end2)
                ]

        return _Path

    def graph_path_to_road_path(self, path):
        result = []
        us, roads, vs = path[0::2], path[1::2], path[2::2]
        for u, road, v in zip(us, roads, vs):
            _, start = self.embedding(u, road)
            _, end = self.embedding(v, road)
            result.append(RoadSegment(road, start, end))
        return result

    def distance_on_road(self, p: Point, q: Point, road: TRoad) -> float:
        """

        returns the distance from p to q on road,
        if the direction of travel is admissible;
        if not, or if p and q are not co-'road'-al, returns infinity

        """
        d, _ = self._shortest_path_on_road(p, q, road)
        return d

    def shortest_path_on_road(self, p: Point, q: Point, road: TRoad) -> RoadPath:
        _, path = self._shortest_path_on_road(p, q, road)
        return path

    def _shortest_path_on_road(self, p: Point, q: Point, road: TRoad) -> tuple[float, RoadPath]:
        if not self.is_on_road(p, road) or not self.is_on_road(q, road):
            return np.inf, None

        _, xp = p
        _, xq = q

        if xq < xp and self.roadnet.is_oneway(road):
            return np.inf, None

        return abs(xq - xp), [RoadSegment(road, xp, xq)]

    def distance_node_to_point(self, u: TVert, q: Point) -> float:
        """

        returns the shortest-path distance from a node u to a point q, on digraph

        """
        return self._shortest_path_node_to_point(u, q).distance

    def shortest_path_node_to_point(self, u: TVert, q: Point) -> Optional[RoadPath]:
        return self._shortest_path_node_to_point(u, q).path

    @cached_property
    def _shortest_path_node_to_point(self):

        metric = self

        @dataclass
        class _Path:
            u: TVert
            q: Point

            @cached_property
            def _result(self):
                u, q, roadnet = self.u, self.q, metric.roadnet
                assert metric.is_valid_point(q)
                road, x = q

                i, j = roadnet.endpoints(road)
                nodes = [i, j]
                points = [(road, 0.), (road, roadnet.length(road))]

                options = [
                    (
                        metric.graph_shortest_path_length(u, v) + metric.distance_on_road(q_, q, road),
                        v
                    )
                    for v, q_ in zip(nodes, points)
                ]

                return min(options, key=lambda pair: pair[0])

            @property
            def distance(self):
                d, _ = self._result
                return d

            @property
            def path(self):
                d, v = self._result
                if not d < np.inf:
                    return

                path = metric.graph_path_to_road_path(metric.graph_shortest_path(self.u, v))
                road, end = self.q
                _, start = metric.embedding(v, road)
                path.append(RoadSegment(road, start, end))

                return path

        return _Path


@dataclass(frozen=True)
class VertexNode:
    vertex: Any


@dataclass(frozen=True)
class PointNode:
    point: Any


def create_path_network(points, roadnet: Roadnet):
    """

    A path network has as its vertices the union of the original road network vertices
    and a set of query points. The edges are such that shortest path distances between all vertices
    in the path network agree with those between the same features in the road network metric space.

    """
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


def get_segment(
        u: VertexNode | PointNode,
        v: VertexNode | PointNode,
        roadnet: Roadnet,
) -> RoadSegment:
    """Recover the segment on a road network between adjacent nodes in one of its path networks."""
    road = get_segment_road(u, v, roadnet)
    _, start = get_road_embedding(u, road, roadnet)
    _, end = get_road_embedding(v, road, roadnet)
    return RoadSegment(road, start, end)


def get_segment_road(
        u: VertexNode | PointNode,
        v: VertexNode | PointNode,
        roadnet: Roadnet,
):
    """Determine the road underneath the smallest segment between two nodes on a path network."""
    if isinstance(u, PointNode):
        road, _ = u.point
    elif isinstance(v, PointNode):
        road, _ = v.point
    else:
        roads = (
            road
            for road in roadnet.out_edges(u.vertex)
            if roadnet.endpoints(road)[1] == v.vertex
        )
        road = min(roads, key=roadnet.length)

    return road


def get_road_embedding(u: VertexNode | PointNode, road, roadnet: Roadnet):
    """Resolve a path network node to its corresponding point on the original road network."""
    if isinstance(u, PointNode):
        road_, _ = u.point
        assert road_ == road
        return u.point

    if isinstance(u, VertexNode):
        return RoadnetQuery(roadnet).embedding(u.vertex, road)

    raise ValueError()
