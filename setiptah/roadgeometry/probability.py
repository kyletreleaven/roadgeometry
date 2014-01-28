
# built-in
import random, itertools

# scientific common
import numpy as np
import networkx as nx

# community
import bintrees     # --- weird warnings?

# dev
import roadmap_basic as ROAD 


def sampleroadnet( n=10, p=.3, n_oneway=0 ) :
    # based on Erdos Renyi ; n=# of nodes, p=probability any two nodes are linked
    g = nx.erdos_renyi_graph( n, p )
    # ...then just get the biggest connected component
    g = nx.connected_component_subgraphs( g )[0]
    
    # create a roadnet with such connectivity and random street lengths
    roadnet = nx.MultiDiGraph()
    def roadmaker() :
        for i in itertools.count() : yield 'road%d' % i, np.random.exponential()
    road_iter = roadmaker()
    
    for i, ( u,v,data ) in enumerate( g.edges_iter( data=True ) ) :
        label, length = road_iter.next()
        roadnet.add_edge( u, v, label, length=length )
        
    # add some random one-way roads
    nodes = roadnet.nodes()
    for i in range( n_oneway ) :
        u = random.choice( nodes )
        v = random.choice( nodes )
        label, length = road_iter.next()
        roadnet.add_edge( u, v, label, length=length, oneway=True )
        
    return roadnet



""" convenient sampling utility for the unit test below, might as well be a package-export, though """

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
        weights = weight_dict.values()
        scores = np.cumsum( np.array( weights ) )
        
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
        for _,__, road, data in roadnet.edges_iter( keys=True, data=True ) :
            weight_dict[road] = data.get( length, 1 )
            
        self.roadnet = roadnet
        self.road_sampler = WeightedSet( weight_dict )
        
    def sample(self) :
        road = self.road_sampler.sample()
        L = ROAD.get_road_data( road, self.roadnet ).get( 'length', 1 )
        y = L * np.random.rand()
        return (road,y)




def sampleaddress( roadnet, length='length' ) :
    """
    quick sampling function,, roads are elements chosen with equal probability;
    not in proportion to road length; for that see UniformDist
    """
    _,__,road = random.choice( roadnet.edges( keys=True ) )
    return sample_onroad( road, roadnet, length )





