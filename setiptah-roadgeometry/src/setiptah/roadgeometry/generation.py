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
    """Road network from a Delaunay triangulation of 2D points.

    Implements the Roadnet protocol. Each undirected Delaunay edge is oriented
    canonically as (i, j) with i < j. Flow on a road may be positive (i→j) or
    negative (j→i); no road is one-way.
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
            self._edges.append((u, v))
            self._out_edges[u].append((u, v))
            self._in_edges[v].append((u, v))
            self._endpoints[(u, v)] = (u, v)
            self._lengths[(u, v)] = dist

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
