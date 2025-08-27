from typing import Dict, TypeVar
from numbers import Number

import numpy as np

from .priodict import *
from .mygraph import mygraph

TVert = TypeVar("TVert")
TEdge = TypeVar("TEdge")


def Dijkstra(graph: mygraph, cost: Dict[TEdge, Number], s: TVert):
    """Get the distance from s

    """
    d = {}      # only in here if they are seen... duh!!
    upstream = { s : None }
    
    OPEN = priorityDictionary()
    OPEN[s] = 0.
    
    while len( OPEN ) > 0 :
        i = OPEN.smallest()
        d[i] = OPEN[i]
        del OPEN[i]
        
        for e in graph.V[i] :
            _, j = graph.endpoints(e)
            if j in d : continue

            dj = d[i] + cost[e]
            if dj < OPEN.get(j, np.inf):
                OPEN[j] = dj
                upstream[j] = e

    return d, upstream
