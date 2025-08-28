from typing import List, Any, Dict
from .mygraph import mygraph, TV

""" recursive form, why not, obtained from Cormen """
WHITE = 0
GRAY = 1
BLACK = 2


def toposort(graph: mygraph[TV, Any]) -> List[TV]:
    """
    do a DFS traversal of graph, and apply visit at each node
    """
    order = []
    
    color = {}
    for u in graph.nodes() :
        c = color.setdefault( u, WHITE )
        if c == WHITE :
            DFSVISIT( graph, u, color, order )
    return order


def DFSVISIT(graph: mygraph[TV, Any], u: TV, color: Dict[TV, int], order: List[TV]):
    color[u] = GRAY
    for e in graph.V[u] :
        _,v = graph.endpoints(e)
        c = color.setdefault( v, WHITE )
        if c == WHITE :
            DFSVISIT( graph, v, color, order )
    color[u] = BLACK
    order.insert(0, u )
