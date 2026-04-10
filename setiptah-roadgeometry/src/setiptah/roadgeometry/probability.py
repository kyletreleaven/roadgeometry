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


class WeightedSet :
    """
    utility class, instantiates a sampler, where
    elements are chosen from a set according to provided weights
    """
    def __init__(self, weight_dict ) :
        """
        keys are targets, values are weights; needn't sum to 1
        doesn't check for repeats
        """
        targets = weight_dict.keys()
        weights = list(weight_dict.values())
        scores = np.cumsum( np.array(weights) )
        
        self._hiscore = scores[-1]
        self._tree = bintrees.RBTree()
        for target, score in zip( targets, scores ) :
            self._tree.insert( score, target )
            
    def sample(self) :
        z = self._hiscore * np.random.rand()
        _, res = self._tree.ceiling_item( z )
        return res


class UniformDist :
    """
    class implements a uniform distribution, built using weighted set
    """
    def __init__(self, roadnet=None, length=None ) :
        if roadnet is not None :
            self.set_roadnet( roadnet, length )
        
    def set_roadnet(self, roadnet, length=None ) :
        if length is None : length = 'length'
        
        weight_dict = dict()
        for _,__, road, data in roadnet.edges( keys=True, data=True ) :
            weight_dict[road] = data.get( length, 1 )
            
        self.roadnet = roadnet
        self.road_sampler = WeightedSet( weight_dict )

    def sample(self) :
        road = self.road_sampler.sample()
        L = ROAD.get_road_data( road, self.roadnet ).get( 'length', 1 )
        y = L * np.random.rand()
        return ROAD.RoadAddress( road, y )


class RoadnetUniformDist:
    """Uniform distribution over a Roadnet (protocol-compatible alternative to UniformDist).

    Samples a road weighted by length, then a uniform offset along that road.
    Works with any object satisfying the Roadnet protocol.
    """

    def __init__(self, roadnet):
        self.roadnet = roadnet
        weight_dict = {road: roadnet.length(road) for road in roadnet.edges()}
        self.road_sampler = WeightedSet(weight_dict)

    def sample(self):
        road = self.road_sampler.sample()
        y = self.roadnet.length(road) * np.random.rand()
        return (road, y)


def sampleaddress(roadnet: nx.MultiDiGraph, length: str = "length") -> ROAD.RoadAddress:
    """
    quick sampling function,, roads are elements chosen with equal probability;
    not in proportion to road length; for that see UniformDist
    """
    roads = [road for _, __, road in roadnet.edges(keys=True)]
    road = random.choice(roads)
    return sample_onroad(road, roadnet, length)
