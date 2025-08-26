#!/usr/bin/env python

import itertools

import numpy as np
import bintrees

import networkx as nx

""" my dependencies """
from setiptah.roadbm import matchvis_util

import matplotlib.pyplot as plt


def SANITIZE( I_graph ) :
    # remove nil edges (so sorry)
    for i, j, data in I_graph.edges( data=True ) :
        if data['score'] <= 0 :
            I_graph.remove_edge( i, j )
        
    # remove nodes with no degree
    for i in I_graph.nodes() :
        if I_graph.in_degree(i) <= 0 and I_graph.out_degree(i) <= 0 :
            I_graph.remove_node(i)


def INITIALIZE_BAGS( I_graph ) :
    
    for i, data in I_graph.nodes_iter( data=True ) :
        typei, labeli = i
        
        data.update( S=[], T=[] )
        
        if typei == matchvis_util.VERTEX:
            continue
        
        elif typei == matchvis_util.POINT_IN_S:
            data['S'].append( labeli )
            
        elif typei == matchvis_util.POINT_IN_T:
            data['T'].append( labeli )
            
        else :
            raise Exception('unrecognized node type')


def UPDATE_EDGE( i, j, I_graph, MATCH ) :
    src_data = I_graph.node[i]
    src_S_bag = src_data['S']
    
    edge_data = I_graph.get_edge_data( i, j )
    score = edge_data['score']
    
    dst_data = I_graph.node[j]
    dst_S_bag = dst_data['S']
    dst_T_bag = dst_data['T']
    
    cancel = min( score, len( dst_T_bag ) )
    shift = score - cancel
    
    for k in xrange(cancel) :
        s = src_S_bag.pop(0)
        t = dst_T_bag.pop(0)
        MATCH.append( (s,t) )
    
    temp, src_S_bag[:] = src_S_bag[:shift], src_S_bag[shift:]
    dst_S_bag[:] = temp + dst_S_bag[:]
    
    I_graph.remove_edge( i, j )
    
    if len( src_S_bag ) <= 0 : I_graph.remove_node( i )
    if len( dst_S_bag ) + len( dst_T_bag ) <= 0 : I_graph.remove_node( j )
    #if I_graph.out_degree( i ) <= 0 : I_graph.remove_node( i )


def UPDATE_NODE( i, I_graph, MATCH ) :
    """ node must have *no* in-degree """
    
    src_data = I_graph.node[i]
    src_S_bag = src_data['S']
    
    if I_graph.out_degree( i ) <= 0 :
        I_graph.remove_node( i )
        
    else :
        # just process one edge
        iter = I_graph.out_edges_iter( i, data=True )
        _, j, edge_data = iter.next()
        
        score = edge_data['score']
        
        dst_data = I_graph.node[j]
        dst_S_bag = dst_data['S']
        dst_T_bag = dst_data['T']
        
        for k in xrange(score) :
            s = src_S_bag.pop(0)
            
            if len(dst_T_bag) > 0 :
                t = dst_T_bag.pop(0)
                MATCH.append( (s,t) )
            else :
                dst_S_bag.append( s )
                
        I_graph.remove_edge( i, j )
        
        
        
        
def DISPLAY_STATE( I_graph, pos, active_node=None ) :
    
    # get an axis
    ax = plt.gca()
    
    # populate node labels, and...
    # initialize S and T queues on the interval graph
    interchanges = []
    active = []
    other = []
    
    interchange_labels = {}
    
    # draw nodes
    for i, data in I_graph.nodes_iter( data=True ) :
        typei, labeli = i
        
        if i == active_node :
            active.append( i )
        
        elif typei == matchvis_util.VERTEX:
            interchanges.append( i )
            interchange_labels[i] = labeli
            
        elif typei == matchvis_util.POINT_IN_S:
            #data.update( S = [ labeli ], T = [] )
            other.append( i )
            
        elif typei == matchvis_util.POINT_IN_T:
            #data.update( S = [], T = [ labeli ] )
            other.append( i )
            
        else :
            raise Exception('unrecognized node type')
        
    #nx.draw_networkx_nodes( I_graph, pos=pos, label=node_labels )
    def show_nodes( **kwargs ) :
        nx.draw_networkx_nodes( I_graph, pos=pos, ax=ax, **kwargs )
        
    show_nodes( nodelist=active, node_color='b', node_size=100, label=None )
    show_nodes( nodelist=interchanges, label=None )
    show_nodes( nodelist=other, node_color='k', node_size=50, label=None )
    
    S_bags = {}
    T_bags = {}
    for i, data in I_graph.nodes_iter( data=True ) :
        temp = data['S']
        if len(temp) > 0 : S_bags[i] = temp
        
        temp = data['T']
        if len(temp) > 0 : T_bags[i] = temp
    
    offset = .01
    pos_labels = { i : (x,y+offset) for i, (x,y) in pos.iteritems() }
    def show_labels( labels, **kwargs ) :
        nx.draw_networkx_labels( I_graph, pos=pos_labels, ax=ax, labels=labels, **kwargs )
    show_labels( S_bags, font_color='r' )
    show_labels( T_bags, font_color='b' )
    #nx.draw_networkx_labels( I_graph, pos=pos_labels, labels=SS_bags, font_color='r' )
    #nx.draw_networkx_labels( I_graph, pos=pos_labels, labels=TT_bags, font_color='b' )
    
    # draw edges
    score_map = {}
    for i, j, data in I_graph.edges_iter( data=True ) :
        score = data['score']
        
        if score not in score_map : score_map[score] = []
        score_map[score].append( (i,j) )
        
    #print 'SCORE_MAP', score_map
    
    for score, edges in score_map.iteritems() :
        nx.draw_networkx_edges( I_graph, pos=pos, edgelist=edges, width=score,
                                label=None, ax=ax )
        
        
    # record the matching-so-far on a special node near the bottom!!
    
    
    
    



def texheader() :
    return """
\\documentclass{article}
\\usepackage{tikz}

\\begin{document}
\\begin{tikzpicture}[x=\\linewidth,y=\\linewidth]
"""
    
def texfooter() :
    return """
\\end{tikzpicture}
\\end{document}
"""


def DISPLAY_STATE_TIKZ( I_graph, pos, MATCH=None, active_node=None ) :
    
    mystr = ''
    
    # draw a bounding box
    mystr += '\\draw (0,.1) rectangle (.7,.7) ;\n'
    
    # populate node labels, and...
    # initialize S and T queues on the interval graph
    interchanges = []
    active = []
    other = []
    
    interchange_labels = {}
    
    node_indices = {}
    
    # categorize nodes
    opt = {}
    k = 0
    for i, data in I_graph.nodes_iter( data=True ) :
        typei, labeli = i
        
        # map nodes to coordinates
        node_indices[i] = k
        
        x, y = pos[i]
        opt.update( k=k, x=x, y=y )
        mystr +=  '\\coordinate (coord%(k)d) at (%(x).3f,%(y).3f) ;\n' % opt
        k += 1
        
        if i == active_node :
            active.append( i )
        
        elif typei == matchvis_util.VERTEX:
            interchanges.append( i )
            interchange_labels[i] = labeli
            
        elif typei == matchvis_util.POINT_IN_S:
            #data.update( S = [ labeli ], T = [] )
            other.append( i )
            
        elif typei == matchvis_util.POINT_IN_T:
            #data.update( S = [], T = [ labeli ] )
            other.append( i )
            
        else :
            raise Exception('unrecognized node type')
        
    # draw interchanges
    data = dict( sz=.01 )
    for i in interchanges :
        data.update( k=node_indices[i] )
        mystr += '\\draw (coord%(k)d) circle (%(sz)f) ;\n' % data
        
    # draw active
    data = dict( sz=.005 )
    for i in active :
        data.update( k=node_indices[i] )
        mystr += '\\fill [blue] (coord%(k)d) circle (%(sz)f) ;\n' % data
        
    # draw other
    data = dict( sz=.002 )
    for i in other :
        data.update( k=node_indices[i] )
        mystr += '\\fill (coord%(k)d) circle (%(sz)f) ;\n' % data
        
    # draw bags
    S_bags = {}
    T_bags = {}
    for i, data in I_graph.nodes_iter( data=True ) :
        temp = data['S']
        if len(temp) > 0 : S_bags[i] = temp
        
        temp = data['T']
        if len(temp) > 0 : T_bags[i] = temp
        
    fmt = '\\path (coord%(k)d) node [anchor=south,%(color)s] {\\footnotesize %(label)s} ;\n' 
    data = dict( offset=.001 )
    
    data.update( color='red' )
    for i, bag in S_bags.iteritems() :
        data.update( k=node_indices[i], label=repr(bag) )
        mystr += fmt % data
        
    data.update( color='blue' )
    for i, bag in T_bags.iteritems() :
        data.update( k=node_indices[i], label=repr(bag) )
        mystr += fmt % data
        
        
    # draw edges
    score_map = {}
    for i, j, data in I_graph.edges_iter( data=True ) :
        score = data['score']
        
        if score not in score_map : score_map[score] = []
        score_map[score].append( (i,j) )
    
    fmt = '\\draw [->,line width=%(w)f] (coord%(k1)d) -- (coord%(k2)d) '
    fmt += 'node [midway,below] {\\tiny %(score)d};\n'
    data = {}
    for score, edges in score_map.iteritems() :
        data.update( w= .5 * score, score=score )
        
        for i, j in edges :
            data.update( k1=node_indices[i], k2=node_indices[j] )
            mystr += fmt % data
        
    if MATCH is not None :
        # display the matching-so-far
        match_fmt = '({\\color{red}%d},{\\color{blue}%d}), '
        match_strings = [ match_fmt % (s,t) for s, t in MATCH ]
        
        cat = lambda s1, s2 : s1 + s2
        match_tex = reduce( cat, [ '[' ] + match_strings + [ ']' ] )
        
        mystr += '\\node at (.35,0) {%s} ;\n' % match_tex
    
    return mystr












    
    
        
    # record the matching-so-far on a special node near the bottom!!
    
    
    









if __name__ == '__main__' :
    plt.close('all')
    
    import argparse
    
    
    # example instance data
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
    
    from setiptah.roadgeometry.generation import DelaunayRoadMap
    
    roadmap = DelaunayRoadMap( interchanges )
    """ and build positions dictionary """
    pos = { k : p for k, p in enumerate( interchanges ) }
    
    """ now, obtain two sets of points """
    # M = args.points
    M = 10
    
    import setiptah.roadgeometry.probability as roadprob
    uniform = roadprob.UniformDist( roadmap )
    unpack = lambda addr : ( addr.road, addr.coord )
    
    if False :
        SS = [ unpack( uniform.sample() ) for i in xrange(M) ]
        TT = [ unpack( uniform.sample() ) for i in xrange(M) ]
    
    else :
        SS = [  ('road 0',.15),
                ('road 1',.1), ('road 1',.15),
                ('road 3',.05), ('road 3',.075),
                ('road 8',.05),
                ('road 10',.05),
              ]
        
        TT = [  ('road 6',.1),('road 6',.15),
                ('road 8',.1),
                ('road 10',.1), ('road 10',.15),
                ('road 12',.1),
                ('road 13',.1),
              ]
    
    
    """ obtain the optimal matching """
    import setiptah.roadbm.bm as roadbm
    opt_match = roadbm.ROADSBIPARTITEMATCH( SS, TT, roadmap )
    
    """ obtain an interval graph from the matching """
    I_graph, I_pos = matchvis_util.INTERVAL_GRAPH(opt_match, SS, TT, roadmap, pos)
    # sometimes, there's still a cycle?
    
    
    """ make tikz animation """
    
    def writeslide( k, mystr ) :
        f = open( 'slides/slide%d.tex' % k, 'w' )
        f.write( mystr )
        f.close()
        
    f = open( 'slides/construction_animation.tex', 'w' )
    
    SANITIZE( I_graph )
    INITIALIZE_BAGS( I_graph )
    match = []
    
    slide_fmt = '\\only<%(k)d>{ \\input{slides/slide%(k)d.tex} }\n'
    
    order = nx.topological_sort( I_graph )
    edges = []
    for i in order : edges.extend( I_graph.out_edges_iter(i) )
    k=1
    for i,j in edges :
        mystr = DISPLAY_STATE_TIKZ( I_graph, I_pos, match )
        writeslide( k, mystr )
        
        f.write( slide_fmt % { 'k' : k } )
        
        UPDATE_EDGE( i, j, I_graph, match )
        
        k += 1
        
    mystr = DISPLAY_STATE_TIKZ( I_graph, I_pos, match )
    writeslide( k, mystr )
    
    f.write( slide_fmt % { 'k' : k } )
    f.close()
    
    
    
    
    
    