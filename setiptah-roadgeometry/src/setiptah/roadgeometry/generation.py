import numpy as np
import networkx as nx
import scipy.spatial as spatial


def DelaunayRoadMap( interchanges ) :
    N = len(interchanges)

    """ construct roads from Delaunay adjacencies """
    tri = spatial.Delaunay( interchanges )

    graph = nx.Graph()
    # find the edges in the triangulation
    indices, seq = tri.vertex_neighbor_vertices
    for i in range(N) :
        for j in seq[ indices[i]:indices[i+1] ] :
            graph.add_edge(i,j)

    """ build the roadmap data structure """
    roadmap = nx.MultiDiGraph()
    for ridx, (u,v) in enumerate( graph.edges() ) :
        x, y = [ tri.points[k] for k in (u,v) ]
        roadmap.add_edge(u,v, 'road %d' % ridx, length=np.linalg.norm(y-x) )

    return roadmap


class DelaunayRoadnet:
    """Bidirectional road network from a Delaunay triangulation of 2D points.

    Implements the Roadnet protocol. Each undirected Delaunay edge (i, j) becomes
    two directed roads (i, j) and (j, i) with length equal to the Euclidean distance.
    No one-way roads.
    """

    def __init__(self, points):
        points = np.asarray(points)
        tri = spatial.Delaunay(points)
        N = len(points)

        indices, seq = tri.vertex_neighbor_vertices
        undirected = set()
        for i in range(N):
            for j in seq[indices[i]:indices[i + 1]]:
                undirected.add((min(i, j), max(i, j)))

        self._edges = []
        self._out_edges = {i: [] for i in range(N)}
        self._in_edges  = {i: [] for i in range(N)}
        self._endpoints = {}
        self._lengths   = {}

        for u, v in undirected:
            dist = float(np.linalg.norm(points[v] - points[u]))
            for road in ((u, v), (v, u)):
                tail, head = road
                self._edges.append(road)
                self._out_edges[tail].append(road)
                self._in_edges[head].append(road)
                self._endpoints[road] = (tail, head)
                self._lengths[road] = dist

        self._nodes = list(range(N))

    def nodes(self):
        return self._nodes

    def edges(self):
        return self._edges

    def endpoints(self, road):
        return self._endpoints[road]

    def out_edges(self, u):
        return self._out_edges[u]

    def in_edges(self, u):
        return self._in_edges[u]

    def length(self, road):
        return self._lengths[road]

    def is_oneway(self, road):
        return False
