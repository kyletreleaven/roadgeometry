
import numpy as np

from mygraph import mygraph
#from priodict import *

""" recursive form, why not, obtained from Cormen """
WHITE = 0
GRAY = 1
BLACK = 2


def toposort( graph ) :
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
            
def DFSVISIT( graph, u, color, order ) :
    color[u] = GRAY
    for e in graph.V[u] :
        _,v = graph.endpoints(e)
        c = color.setdefault( v, WHITE )
        if c == WHITE :
            DFSVISIT( graph, v, color, order )
    color[u] = BLACK
    order.insert(0, u )



""" unit test """

if __name__ == '__main__' :
    import itertools
    
    import numpy as np
    import networkx as nx
    
    g = mygraph()
    g.add_edge( 'a', 0, 1 )
    g.add_edge( 'b', 0, 2 )
    g.add_edge( 'e', 1, 3 )
    g.add_edge( 'c', 2, 3 )
    g.add_edge( 'd', 3, 4 )
    
    order = toposort( g )
    
    
    
    
    
    
    
    