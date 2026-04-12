# builtin
import random, itertools

# scientific common
import numpy as np
import networkx as nx

# community
# TODO: Upgrade to `sortedcontainers`.
import bintrees     # --- weird warnings?

# dev
from .legacy import roadmap_basic as ROAD


def sampleroadnet( n=10, p=.3, n_oneway=0 ) :
    # based on Erdos Renyi ; n=# of nodes, p=probability any two nodes are linked
    g = nx.erdos_renyi_graph( n, p )
    # ...then just get the biggest connected component
    for c in nx.connected_components(g):
        g = g.subgraph(c)
        break
    else:
        raise StopIteration("No connected component found?")
    
    # create a roadnet with such connectivity and random street lengths
    roadnet = nx.MultiDiGraph()
    def roadmaker() :
        for i in itertools.count():
            yield 'road%d' % i, np.random.exponential()
    road_iter = roadmaker()
    
    for i, (u, v, data) in enumerate(g.edges(data=True)):
        label, length = next(road_iter)
        roadnet.add_edge( u, v, label, length=length )
        
    # add some random one-way roads
    nodes = roadnet.nodes()
    for i in range( n_oneway ) :
        u = random.choice( nodes )
        v = random.choice( nodes )
        label, length = next(road_iter)
        roadnet.add_edge( u, v, label, length=length, oneway=True )
        
    return roadnet


def sample_onroad( road, roadnet, length='length' ) :
    """ samples uniformly from the given road """
    _, road_data = ROAD.obtain_edge( roadnet, road, True )
    roadlen = road_data.get( length, 1 )
    y = roadlen * np.random.rand()
    return ROAD.RoadAddress(road,y)


class WeightedSet:
    """Sampler where elements are chosen according to provided weights.

    Keys are targets, values are non-negative weights; need not sum to 1.

    Exposes `digitize` so callers can share the random draw and look up
    parallel arrays (e.g. lengths) without a per-sample attribute lookup.
    """

    def __init__(self, weight_dict: dict) -> None:
        self.targets = list(weight_dict.keys())
        self._bins = np.cumsum(np.array(list(weight_dict.values()), dtype=float))

    def digitize(self, z: np.ndarray) -> np.ndarray:
        """Map uniform random values in [0, total_weight) to target indices."""
        return np.digitize(z, self._bins)

    def sample(self, size: int = 1):
        """Draw `size` samples.  Returns a single element if size=1, else a list."""
        z = self._bins[-1] * np.random.rand(size)
        indices = self.digitize(z)
        if size == 1:
            return self.targets[indices[0]]
        return [self.targets[i] for i in indices]


class UniformDist :
    """
    class implements a uniform distribution, built using weighted set
    """
    def __init__(self, roadnet=None, length=None ) :
        if roadnet is not None :
            self.set_roadnet( roadnet, length )
        
    def set_roadnet(self, roadnet, length=None):
        if length is None: length = 'length'
        length_dict = {road: data.get(length, 1)
                       for _, __, road, data in roadnet.edges(keys=True, data=True)}

        class _Adapter:
            def edges(self): return length_dict.keys()
            def length(self, road): return length_dict[road]

        self.roadnet = roadnet
        self._inner  = RoadnetUniformDist(_Adapter())

    def sample(self, size: int = 1):
        result = self._inner.sample(size)
        if size == 1:
            return ROAD.RoadAddress(*result)
        return [ROAD.RoadAddress(road, y) for road, y in result]


class RoadnetUniformDist:
    """Uniform distribution over a Roadnet (protocol-compatible alternative to UniformDist).

    Samples a road weighted by length, then a uniform offset along that road.
    Works with any object satisfying the Roadnet protocol.
    """

    def __init__(self, roadnet):
        self.roadnet = roadnet
        weight_dict = {road: roadnet.length(road) for road in roadnet.edges()}
        self.road_sampler = WeightedSet(weight_dict)
        self._lengths = np.array([weight_dict[r] for r in self.road_sampler.targets])

    def sample(self, size: int = 1):
        z = self.road_sampler._bins[-1] * np.random.rand(size)
        indices = self.road_sampler.digitize(z)
        roads   = [self.road_sampler.targets[i] for i in indices]
        lengths = self._lengths[indices]
        y       = lengths * np.random.rand(size)
        if size == 1:
            return (roads[0], float(y[0]))
        return list(zip(roads, y.tolist()))


def sampleaddress(roadnet: nx.MultiDiGraph, length: str = "length") -> ROAD.RoadAddress:
    """
    quick sampling function,, roads are elements chosen with equal probability;
    not in proportion to road length; for that see UniformDist
    """
    roads = [road for _, __, road in roadnet.edges(keys=True)]
    road = random.choice(roads)
    return sample_onroad(road, roadnet, length)
