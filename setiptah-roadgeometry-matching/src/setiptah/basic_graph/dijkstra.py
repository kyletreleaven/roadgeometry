from collections import defaultdict
from dataclasses import dataclass
from functools import cached_property
from numbers import Number
from typing import Dict

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

    @cached_property
    def _distance(self) -> dict[SourceVertex, dict[TargetVertex, float]]:
        return defaultdict(dict)

    @cached_property
    def _upstream(self) -> dict[SourceVertex, dict[TargetVertex, TVert]]:
        return defaultdict(dict)

    def _out_edges(self, i: TVert):
        for e in self.roadnet.out_edges(i):
            _, j = self.roadnet.endpoints(e)
            yield e, j
        for e in self.roadnet.in_edges(i):
            if self.roadnet.is_oneway(e):
                continue
            j, _ = self.roadnet.endpoints(e)
            yield e, j

    def shortest_path_length(self, source: SourceVertex, target: TargetVertex) -> float:
        self._ensure(source, target)
        return self._distance[source][target]

    def _ensure(self, source: SourceVertex, target: TargetVertex):
        if target not in self._distance[source]:
            self._populate_dijkstra(source)

    def _populate_dijkstra(self, source: SourceVertex):
        d = self._distance[source]
        upstream = self._upstream[source]

        OPEN = priorityDictionary()
        OPEN[source] = 0.

        while len(OPEN) > 0:
            i = OPEN.smallest()
            up = upstream.get(i)  # None when i == source

            source_, di = i, 0.
            while True:
                self._distance[source_][i] = di
                if up:
                    self._upstream[source_][i] = up

                if source_ == source:
                    break

                # otherwise move up
                e, i_ = upstream[source_]
                source_, di = i_, di + self.roadnet.length(e)

            del OPEN[i]

            for e, j in self._out_edges(i):
                if j in d:  # we already know shortest path source -> j!
                    continue
                dj = di + self.roadnet.length(e)
                if dj < OPEN.get(j, np.inf):
                    OPEN[j] = dj
                    upstream[j] = e, i

    def _dijkstra(self, source: SourceVertex):
        # Return a copy of what's _already_ in the map.
        return {**self._distance[source]}, {**self._upstream[source]}
