import numpy as np

from .mygraph import mygraph
from priodict import *


def Dijkstra( graph, cost, s ) :
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
            
            dj = d[i] + cost.get( e, 0. )   # should this be zero or some default like 1.?
            if dj < OPEN.get(j, np.Inf ) :
                OPEN[j] = dj
                upstream[j] = e
                
    return d, upstream
