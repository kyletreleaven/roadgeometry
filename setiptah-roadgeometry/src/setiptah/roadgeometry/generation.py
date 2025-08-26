
import numpy as np
import networkx as nx




def DelaunayRoadMap( interchanges ) :
    N = len(interchanges)
    
    """ construct roads from Delaunay adjacencies """
    import scipy.spatial as spatial
    tri = spatial.Delaunay( interchanges )
    
    graph = nx.Graph()
    # find the edges in the triangulation
    indices, seq = tri.vertex_neighbor_vertices
    for i in xrange(N) :
        for j in seq[ indices[i]:indices[i+1] ] :
            graph.add_edge(i,j)
    
    """ build the roadmap data structure """
    roadmap = nx.MultiDiGraph()
    for ridx, (u,v) in enumerate( graph.edges() ) :
        x, y = [ tri.points[k] for k in (u,v) ]
        roadmap.add_edge(u,v, 'road %d' % ridx, length=np.linalg.norm(y-x) )
        
    return roadmap


