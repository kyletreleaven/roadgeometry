from collections import defaultdict
from dataclasses import dataclass
from functools import cached_property
from numbers import Number
from typing import Dict, Generator

import numpy as np

from .mygraph import mygraph
from .priodict import *
from .protocol import *


def Dijkstra(graph: mygraph, cost: Dict[TRoad, Number], s: TVert):
    """Get the distance from s

    """
    d = {}  # only in here if they are seen... duh!!
    upstream = {s: None}

    OPEN = priorityDictionary()
    OPEN[s] = 0.

    while len(OPEN) > 0:
        i = OPEN.smallest()
        d[i] = OPEN[i]
        del OPEN[i]

        for e in graph.V[i]:
            _, j = graph.endpoints(e)
            if j in d: continue

            dj = d[i] + cost[e]
            if dj < OPEN.get(j, np.inf):
                OPEN[j] = dj
                upstream[j] = e

    return d, upstream


SourceVertex = TVert
TargetVertex = TVert


@dataclass(frozen=True)
class RoadnetMetric(Generic[TRoad, TVert]):
    roadnet: Roadnet[TRoad, TVert]

    Point = Roadnet[TRoad, TVert].Point

    @cached_property
    def _distance(self) -> dict[SourceVertex, dict[TargetVertex, float]]:
        return {}

    @cached_property
    def _upstream(self) -> dict[SourceVertex, dict[TargetVertex, TVert]]:
        return {}

    def _out_edges(self, i: TVert):
        for e in self.roadnet.out_edges(i):
            _, j = self.roadnet.endpoints(e)
            yield e, j
        for e in self.roadnet.in_edges(i):
            if self.roadnet.is_oneway(e):
                continue
            j, _ = self.roadnet.endpoints(e)
            yield e, j

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

    def is_valid_point(self, p: Point):
        road, _ = p
        return self.is_on_road(p, road)

    def is_on_road(self, p: Point, road: TRoad) -> bool:
        road_, x = p
        return (
            road_ == road
            and 0. <= x <= self.roadnet.length(road)
        )

    def distance(self, p: Point, q: Point) -> float:
        """

        returns the shortest-path distance between two points on a Roadmap

        TODO: We'll want _path_ version of this method!

        """
        assert self.is_valid_point(p)
        road, x = p

        i, j = self.roadnet.endpoints(road)
        nodes = [i, j]
        points = [(road, 0.), (road, self.roadnet.length(road))]

        options = [
            self.distance_on_road(p, p_, road) + self.distance_node_to_point(u, q)
            for u, p_ in zip(nodes, points)
        ]

        return min(options)

    def distance_on_road(self, p: Point, q: Point, road: TRoad) -> float:
        """

        returns the distance from p to q on road,
        if the direction of travel is admissible;
        if not, or if p and q are not co-'road'-al, returns infinity

        """
        if not self.is_on_road(p, road) or not self.is_on_road(q, road):
            return np.inf

        _, xp = p
        _, xq = q

        if xq < xp and self.roadnet.is_oneway(road):
            return np.inf

        return abs(xq - xp)

    def distance_node_to_point(self, u: TVert, q: Point) -> float:
        """

        returns the shortest-path distance from a node u to a point q, on digraph

        """
        assert self.is_valid_point(q)
        road, x = q

        i, j = self.roadnet.endpoints(road)
        nodes = [i, j]
        points = [(road, 0.), (road, self.roadnet.length(road))]

        options = [
            self.graph_shortest_path_length(u, v) + self.distance_on_road(q_, q, road)
            for v, q_ in zip(nodes, points)
        ]

        return min(options)
