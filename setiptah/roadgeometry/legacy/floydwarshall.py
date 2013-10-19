import itertools

import numpy as np
import networkx as nx




def update_distance_tableau( tableau, graph, weight_attr='weight', self_loops=True ) :
    # can handle non- or multi-graph
    if graph.is_multigraph() :
        
        if graph.is_directed() :
            simple_graph = nx.DiGraph()
        else :
            simple_graph = nx.Graph()
            
        simple_graph.add_nodes_from( graph.nodes_iter() )
        for u, v, key, data in graph.edges_iter( keys=True, data=True ) :
            cand_val = data[ weight_attr ]
            prev_val = simple_graph.get_edge_data( u, v, {} ).get( weight_attr, np.inf )
            if cand_val < prev_val :
                simple_graph.get_edge_data( u, v )[ weight_attr ] = cand_val
                
    else :
        simple_graph = graph
        
    # ensure inclusion of nodes
    for u in simple_graph.nodes_iter() : tableau.setdefault( u, {} )
    
    for u, v, data in simple_graph.edges_iter( data=True ) :
        cand_val = data[ weight_attr ]
        if v not in tableau[u] or cand_val < tableau[u][v] :
            tableau[u][v] = cand_val
            if not simple_graph.is_directed() : tableau[v][u] = cand_val
            
    if self_loops :
        for u in tableau :
            tableau[u][u] = 0.
            
            
def functify_tableau( arc_tableau ) :
    def distance( u, v ) :
        row = arc_tableau.setdefault( u, { v : np.inf } )
        dist = row.get( v, np.inf )
        return dist
    return distance


def floyd_warshall( points, distance_function ) :
#def floyd_warshall( arc_tableau ) :
    APSP_length = {}
    APSP_direction = {}
    
    for u in points :
        APSP_length[u] = {}
        APSP_direction[u] = {}
        
        for v in points :
            dist = distance_function( u, v )
            if dist < np.inf :
                APSP_length[u][v] = dist
                APSP_direction[u][v] = v
                
        #if self_loops :
        #    APSP_length[u][u] = 0.
        #    APSP_direction[u][u] = u
            
    for k in APSP_length :
        for u in APSP_length :
            if k == u or k not in APSP_length[u] : continue
            for v in APSP_length[k] :
                options = []
                
                length = APSP_length[u][k] + APSP_length[k][v]
                dir = APSP_direction[u][k]
                options.append( ( length, dir ) )
                
                if v in APSP_length[u] :
                    length = APSP_length[u][v]
                    dir = APSP_direction[u][v]
                    options.append( ( length, dir ) )
                # going toward zero isn't WRONG, except in the sense you 'eventually' have to leave...
                # maybe self-arcs are bad!
                
                
                length, dir = min( options )
                APSP_length[u][v] = length
                APSP_direction[u][v] = dir
                
    return APSP_length, APSP_direction


def floyd_warshall_test( rad=.15 ) :
    import numpy as np
    
    graph = nx.random_geometric_graph( 20, rad )
    
    for u, v, data in graph.edges_iter( data=True ) :
        #u, v = edge
        p = np.array( graph.node[u]['pos'] )
        q = np.array( graph.node[v]['pos'] )
        weight = np.linalg.norm( q - p )
        data['weight'] = weight
    
    tableau = {}
    update_distance_tableau( tableau, graph )
    dist_func = functify_tableau( tableau )
    
    apsp, apsp_dir = floyd_warshall( tableau, dist_func )
    return graph, tableau, apsp, apsp_dir
        
    #return graph, floyd_warshall( tableau )

