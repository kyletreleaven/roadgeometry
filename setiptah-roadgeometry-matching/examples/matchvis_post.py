#!/usr/bin/env python

import itertools

import numpy as np
import bintrees

import networkx as nx

""" my dependencies """
#import setiptah.roadgeometry.roadmap_basic as ROAD
#import setiptah.roadgeometry.astar_basic as ASTAR
#import setiptah.roadbm.bm as roadbm

import matplotlib.pyplot as plt
from setiptah.roadbm.matchvis import drawRoadmap, SHOWMATCH



if __name__ == '__main__' :
    plt.close('all')
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument( '--roads', type=int, default=6 )
    parser.add_argument( '--points', type=int, default=100 )
    args = parser.parse_args()
    
    
    """ get N network vertices """
    N = args.roads
    
    def sampledisc() :
        while True :
            p = np.random.rand(2)
            if np.linalg.norm( p ) < 1 : return p
            
    #interchanges = [ sampledisc() for i in xrange(N) ]
    #interchanges = [ np.random.rand(2) for i in xrange(N) ]
    interchanges = [
                    (.14,.59), (.48,.6), (.4,.53), (.57,.43),
                    #(.36,.27),
                    (.37,.34),
                    (.58,.23),
                    (.11,.39),(.22,.15),
                    (.12,.25),
                    ]
    interchanges = [ np.array(p) for p in interchanges ]
    N = len(interchanges)
    
    
    
    
    """ construct roads from Delaunay adjacencies """
    import setiptah.roadgeometry.generation as mapgen
    roadmap = mapgen.DelaunayRoadMap( interchanges )
            
    """ ...and build positions dictionary """
    pos = { k : point for k, point in enumerate( interchanges ) }
    
    
    """ now, obtain two sets of points """
    M = args.points
    
    import setiptah.roadgeometry.probability as roadprob
    uniform = roadprob.UniformDist( roadmap )
    unpack = lambda addr : ( addr.road, addr.coord )
    
    SS = [ unpack( uniform.sample() ) for i in xrange(M) ]
    TT = [ unpack( uniform.sample() ) for i in xrange(M) ]
    
    
    """ obtain a random matching """
    import random
    order = range(M)
    random.shuffle(order)
    bad_match = zip( xrange(M), order )
    
    """ obtain the optimal matching """
    import setiptah.roadbm.bm as roadbm
    opt_match = roadbm.ROADSBIPARTITEMATCH( SS, TT, roadmap )
    
    
    """ Now, do all the plotting! """
    if False :
        # just points
        plt.figure()
        nx.draw_networkx_nodes( graph, pos=pos )
        plt.gca().set_aspect('equal')
        
        # the Delaunay-induced network
        plt.figure()
        drawRoadmap( roadmap, pos )
        plt.gca().set_aspect('equal')
        
        # network + points, no matches
        plt.figure()
        SHOWMATCH( [], SS, TT, roadmap, pos=pos, edge_color='k', alpha=1. )
        
        
    """ try to do an animation """
    from matplotlib.widgets import Slider, Button, RadioButtons
    
    fig = plt.figure()
    mainax = plt.subplot(111)
    plt.subplots_adjust(left=0.2, bottom=0.15)
    
    # gui widgets
    axcolor = 'lightgoldenrodyellow'
    #axfreq = plt.axes([0.25, 0.1, 0.65, 0.03], axisbg=axcolor)
    axmatches = plt.axes([0.25, 0.1, 0.65, 0.03], axisbg=axcolor)
    
    #sfreq = Slider(axfreq, 'Freq', 0.1, 30.0, valinit=15.)
    sliderCount = Slider(axmatches, '# Matches', 0, len( opt_match ), valfmt='%d', valinit=0 )
    
    SHOWALL = 0
    INCREMENTAL = 1
    switch = INCREMENTAL
    figmatch = bad_match
    lims_flag = False       # update should override limits first time only? 
    
    def update( val ) :
        MM = int( sliderCount.val )
        
        submatch = figmatch[:MM]
        if switch == INCREMENTAL :
            #print 'incremental'
            SSS = [ SS[s] for s,t in submatch ]
            TTT = [ TT[t] for s,t in submatch ]
            pltmatch = zip( xrange(MM), xrange(MM) )
            
        elif switch == SHOWALL :
            #print 'showall'
            SSS = SS
            TTT = TT
            pltmatch = submatch
            
        else :
            raise 'which points to show?'
        
        lims = mainax.axis()
        mainax.clear()
        SHOWMATCH( pltmatch, SSS, TTT, roadmap, pos=pos, ax=mainax )
        
        global lims_flag
        if lims_flag :
            mainax.axis( lims )
        else :
            mainax.set_aspect( 'equal' )
            lims_flag = True
        
        plt.draw()
        
    # set the callback
    #sfreq.on_changed(update)
    sliderCount.on_changed(update)
    
    rax = plt.axes([0.025, 0.5, 0.15, 0.15], axisbg=axcolor)
    radio = RadioButtons(rax, ('allpoints','incremental'), active=1)
    def setswitch( label ) :
        global switch
        
        #print label == 'allpoints', label == 'incremental'
        if label == 'allpoints' :
            switch = SHOWALL
        elif label == 'incremental' :
            switch = INCREMENTAL
            
        #print switch
        update( None )
    radio.on_clicked( setswitch )
    

    mrax = plt.axes([0.025, 0.75, 0.15, 0.15], axisbg=axcolor)
    matchrad = RadioButtons(mrax, ('random','optimal'), active=0)
    def setmatch( label ) :
        global figmatch
        
        #print label == 'allpoints', label == 'incremental'
        if label == 'random' :
            figmatch = bad_match
        elif label == 'optimal' :
            figmatch = opt_match
            
        update( None )
    matchrad.on_clicked( setmatch )


    
    # reset, do we need?
    resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
    button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')
    def reset(event):
        #sfreq.reset()
        sliderCount.reset()
        global lims_flag
        lims_flag = False
        #
        update( None )
        
    button.on_clicked(reset)
    
    # make a save button
    saveax = plt.axes([0.6, 0.025, 0.1, 0.04])
    savebutton = Button(saveax, 'Save', color=axcolor, hovercolor='0.975')
    def saveplot(event) :
        extent = mainax.get_window_extent().transformed( fig.dpi_scale_trans.inverted() )
        fig.savefig('figure.svg', bbox_inches=extent )
        # Pad the saved area by 10% in the x-direction and 20% in the y-direction
        #fig.savefig('figure.pdf', bbox_inches=extent.expanded(1.1, 1.2) )
    savebutton.on_clicked(saveplot)
    
    # draw once
    update( None )
    plt.show()
    
    
    
    
    
    
    
    
    
    
