
# built-in
import random, itertools

# scientific common
import numpy as np
import networkx as nx

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







def sampleaddress( roadnet, length='length' ) :
    _,__,road = random.choice( roadnet.edges( keys=True ) )
    return sample_onroad( road, roadnet, length ) 


def sample_onroad( road, roadnet, length='length' ) :
    _, road_data = ROAD.obtain_edge( roadnet, road, True )
    roadlen = road_data.get( length, 1 )
    y = roadlen * np.random.rand()
    return ROAD.RoadAddress(road,y)



def samplepair( roadnet, distr1, distr2=None, length_attr='length' ) :
    """ assume distr sums to one; if not, no guaranteed behavior """
    if distr2 is not None : raise 'not implemented yet'
    
    # choose appropriately, randomly... can probably do better
    x = np.random.rand()
    X = 0.
    for (road1,road2), prob in distr1.iteritems() :
        X += prob
        if X >= x : break
        
    _, data1 = ROAD.obtain_edge( roadnet, road1, data_flag=True )
    _, data2 = ROAD.obtain_edge( roadnet, road2, data_flag=True )
    roadlen1 = data1.get( 'length' )
    roadlen2 = data2.get( 'length' )
    
    x = roadlen1 * np.random.rand()
    y = roadlen2 * np.random.rand()
    p = ROAD.RoadAddress( road1, x )
    q = ROAD.RoadAddress( road2, y )
    
    return p, q



