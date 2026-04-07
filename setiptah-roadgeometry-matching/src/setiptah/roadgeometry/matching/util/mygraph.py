from collections.abc import Iterable
from numbers import Number
from typing import Dict, Collection
from typing import TypeVar, Generic

import numpy as np

from setiptah.roadgeometry.protocol import Topology
from setiptah.roadgeometry.util.priodict import priorityDictionary as _PurePriorityDict
try:
    from setiptah.roadgeometry._cpp import PriorityDict as priorityDictionary
except ImportError:
    priorityDictionary = _PurePriorityDict

TV = TypeVar("TV")
TE = TypeVar("TE")


class mygraph(Topology[TV, TE]):
    """A simplified hashmap-based alternative to `networkx.MultiDiGraph`.

    Unlike `MultiDiGraph`, this DS stores both vertices and edges,
    with adjacency lists connecting them.

    Topology only. Attributes of graph objects to be stored separately.

    """

    def __init__(self) :
        self.E: dict[TE, tuple[TV, TV]] = {}    # edge -> endpoints (directed)
        self.V: dict[TV, TE] = {}               # vertex -> out edges
        self.W: dict[TV, TE] = {}               # vertex -> in edges
        
    def __repr__(self) :
        return '(V:%s, E:%s, W:%s)' % ( repr(self.V), repr(self.E), repr(self.W) )

    def add_node(self, i: TV):
        self.V.setdefault( i, set() )
        self.W.setdefault( i, set() )
        
    def add_edge(self, e: TE, i: TV, j: TV):
        assert e not in self.E
        self.add_node(i)
        self.add_node(j)
        
        self.E[e] = (i,j)
        self.V[i].add(e)
        self.W[j].add(e)
        
    def remove_edge(self, e: TE):
        assert e in self.E
        i, j = self.E[e]
        
        self.V[i].remove(e)
        self.W[j].remove(e)
        del self.E[e]
        
    def remove_node(self, i: TV):
        succ = [ e for e in self.V[i] ]
        for e in succ : self.remove_edge(e)
        pred = [ e for e in self.W[i] ]
        for e in pred : self.remove_edge(e)
        
        del self.V[i]
        del self.W[i]
        
    def nodes(self) -> Iterable[TV]:
        return self.V.keys()
    
    def has_node(self, i: TV) -> bool:
        return i in self.V
    
    def edges(self) -> Collection[TE]:
        return self.E.keys()
    
    def has_edge(self, e: TE) -> bool:
        return e in self.E
        
    def endpoints(self, e: TE) -> tuple[TV, TV]:
        return self.E[e]


def Dijkstra(graph: mygraph, cost: Dict[TE, Number], s: TV):
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
