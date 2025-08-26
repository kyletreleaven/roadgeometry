
import itertools

import numpy as np




""" REGRESSION """

def fit( X, Y, BASIS ) :
    # assumes regression of form \hat y_k = \sum_k \theta_k * f_i(x_k)
    R, C = len(X), len(BASIS)
    F = np.zeros( (R,C))
    for c, f in enumerate( BASIS ) :
        for r, x in enumerate( X ) : F[r,c] = f(x)
        
    A = np.dot( F.T, F )
    b = np.dot( F.T, Y )
    return np.linalg.solve( A, b )




if __name__ == '__main__' :
    import matplotlib.pyplot as plt
    plt.close('all')
    
    import networkx as nx
    #import setiptah.bpmatch.roadmaps as roadbm
    import setiptah.roadbm.bm as roadbm
    
    
    
    roadnet = nx.MultiDiGraph()
    roadnet.add_edge( 0,1, 'N', length=1. )
    roadnet.add_edge( 1,2, 'E', length=1. )
    roadnet.add_edge( 2,3, 'S', length=1. )
    roadnet.add_edge( 3,0, 'W', length=1. )
    
    sampler = roadbm.UniformDist( roadnet )
    
    
    Nmin, Nmax = 10, 10000
    N = [ int(n) for n in np.logspace( np.log10(Nmin), np.log10(Nmax), 10 ) ]
    trials = 5
    
    
    nt = []
    cost = []
    for n, t in itertools.product( N, range(trials) ) :
        PP = [ sampler.sample() for i in xrange(n) ]
        QQ = [ sampler.sample() for i in xrange(n) ]
        
        match = roadbm.ROADSBIPARTITEMATCH( PP, QQ, roadnet )
        #costs = MATCHCOSTS( match, PP, QQ, roadnet )
        c = roadbm.ROADMATCHCOST( match, PP, QQ, roadnet )
        
        nt.append( n )
        cost.append( c )
    
    basis = [ lambda n : n , lambda n : np.power( n, .5 ) ]
    [ alpha, kappa ] = fit( nt, cost, basis )
    
    plt.scatter( nt, cost )
    interp = np.array([ alpha*basis[0](n) + kappa*basis[1](n) for n in N ])
    plt.plot( N, interp, '--' )
    plt.show()




