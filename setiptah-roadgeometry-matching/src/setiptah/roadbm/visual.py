

import itertools

import numpy as np
import bintrees

import networkx as nx

""" my dependencies """
import setiptah.roadgeometry.roadmap_basic as ROAD
import setiptah.roadgeometry.astar_basic as ASTAR

import setiptah.roadbm.bm as roadbm


def texline( x1,y1, x2,y2, style=None ) :
    data = { 'x1' : x1, 'x2' : x2, 'y1' : y1, 'y2' : y2 }
    if style is None :
        data['style'] = ''
    else :
        data['style'] = style
        
    return "\\draw [%(style)s] (%(x1)f,%(y1)f) -- (%(x2)f,%(y2)f) ; \n" % data


def texhline( y, xmin, xmax, style=None ) :
    return texline( xmin, y, xmax, y, style )

def texvline( x, ymin, ymax, style=None ) :
    return texline( x, ymin, x, ymax, style )




def heightFunctionTex( S, T, ymin, ymax, z=None, steps=None ) :
    if z is None :
        zplus = 0
    else :
        zplus = z
        
    #str = "\\begin{tikzpicture}\n"
    str = ''
    
    # compute necessary data
    segment = roadbm.ONESEGMENT( S, T )
    intervals = roadbm.INTERVALS( segment )
    
    # draw the axis
    # horizontal
    #str += "\\draw [->] (%f,0) -- (%f,0) node [right] {$\\coordvar$} ;\n" % ( YMIN, YMAX+.5 )
    str += texhline( 0., ymin, ymax + .5, style='->' )
    #
    str += "\\draw [thick] (%(ymax)f,-.15) -- (%(ymax)f,.15) " % { 'ymax' : ymax }
    str += "node [above right] {$\\roadlen_\\roadvar$} ;\n"
    # vertical
    fmin, fmax = min( intervals ), max( intervals )
    hmin, hmax = min( fmin + zplus, 0 ), max( fmax + zplus, 0 )
    hmin, hmax = np.floor( hmin ), np.ceil( hmax )
    #np.floor( min( intervals ) )
    #fmax = np.ceil( max( intervals ) )
    data = { 'ymin' : ymin, 'fmin' : hmin-.25, 'fmax' : hmax+.25 }
    str += "\\draw [->] (%(ymin)f,%(fmin)f) -- (%(ymin)f,%(fmax)f) " % data
    str += "node [above] {$\\postcumarcs(y) = \\cumarcs(y) + \\numarcs$} ;\n"
    # vertical ticks
    for tick in np.arange(hmin,hmax+1,1) :
        data = { 'tR' : ymin+.1, 'tL' : ymin-.1, 'tick' : tick }
        str += "\\draw (%(tR)f,%(tick)d) -- (%(tL)f,%(tick)d) " % data
        str += "node [left=5] {$%(tick)d$} ;\n" % data
        
    # place the X's and O's
    str += texhline( zplus, ymin, ymax, style='dashed' )
    for y, item in segment.items() :
        phases = [ (item.P, '${\\color{red}\\times}$' ), (item.Q, '${\\color{blue}\\circ}$') ]
        for Y, mark in phases :
            for i in Y : str += "\\draw (%f,%f) node {%s} ;\n" % ( y, zplus, mark )
            
    # place the levels
    arr = bintrees.RBTree()
    for f, ranges in intervals.items() :
        for a,b in ranges :
            lb = a
            if lb == '-' : lb = -np.inf
            arr[lb] = ( (a,b), f )
            
    #print arr
    #print [ v for v in arr.values() ]
    
    iter = arr.values()
    if steps is not None : iter = itertools.islice( iter, steps )
    for (a,b), f in iter :
    #for f, ranges in intervals.items() :
        h = f + zplus
        label = ''
        if a == '-' :
            a = ymin
            label = "node [%s] {$\\numarcs_\\roadvar$}"
            if h >= 0 :       # zr >= 0?
                label = label % "above"
            else :
                label = label % "below"
        if b == '+' :
            b = ymax
            str += "\\draw (%f,%f) node [right] {$\\numarcs_\\roadvar + \\surplus_\\roadvar$} ;\n" % (b,h)
                        
        levelstr = "\\draw [thick] (%(yl)f,%(z)f) -- %(extra)s (%(yr)f,%(z)f) ;\n"
        data = { 'yl' : a, 'yr' : b, 'z' : h, 'extra' : label }
        str += levelstr % data
        # place the shades
        str += "\\path [fill=black,opacity=.2] (%(yl)f,0) rectangle (%(yr)f,%(z)f) ;\n" % data
        
    #str += "\\end{tikzpicture}\n"
    return str




def discreteCostTex( S, T, ymin, ymax, zmin, zmax, zplus=None ) :
    segment = roadbm.ONESEGMENT( S, T )
    meas = roadbm.MEASURE( segment, ymin, ymax )
    obj = roadbm.OBJECTIVE( meas )
    Cf = roadbm.costWrapper( obj )
    
    Z = range(zmin,zmax+1)
    C = [ Cf(z) for z in Z ]
    cmin, cmax = min(C), max(C)
    
    # obtain the 'star'
    ZMIN, ZMAX = -max( meas ), -min( meas )
    CC = [ ( Cf(z), z ) for z in range(ZMIN,ZMAX+1) ]
    COPT, ZOPT = min(CC)
    
    
    #str = "\\begin{tikzpicture}[x=1cm,y=.1cm]\n"
    str = ''
    # draw the axis
    # horizontal
    horz_level = np.floor(cmin)
    data = { 'zmin' : zmin, 'zmax' : zmax, 'horz' : horz_level }
    fmt = "\\draw [->] (%(zmin)d,%(horz)d) -- (%(zmax)f,%(horz)d) node [right] {$\\coordvar$} ;\n" 
    str +=  fmt % data
    # vertical
    str += "\\draw [->] (0,%(cmin)f-1) -- (0,%(cmax)f) " % { 'cmin' : cmin, 'cmax' : cmax }
    str += "node [above] {$\\cost(\\numarcs)$} ;\n"
    # horizontal ticks
    for tick in np.arange(zmin,zmax+1,1) :
        data = { 'tick' : tick, 'horz' : horz_level }
        str += "\\draw (%(tick)d,%(horz)f-.1) -- (%(tick)f,%(horz)f+.1) " % data
        str += "node [below=5] {$%d$} ;\n" % tick
        
    # scatter
    for z, c in zip( Z, C ) :
        if z == ZOPT :
            str += '\\draw [fill] (%f,%f) node {$\\star$} ;\n' % (z,c)       # {$\\circ$}'
        else :
            str += '\\draw [fill] (%f,%f) circle (.05cm) ;\n' % (z,c)       # {$\\circ$}'
        
    if zplus is not None and zmin <= zplus and zplus <= zmax :
        str += '\\draw [] (%f,%f) circle (.2cm) ;\n' % (zplus,Cf(zplus))       # {$\\circ$}'
        
    #str += "\\end{tikzpicture}\n"
    return str
    
    
    
    
    
def costFunctionAnimationTex( S, T, ymin, ymax, z=None ) :
    pass






def costFunctionTex( S, T, ymin, ymax, z=None ) :
    if z is None :
        zplus = 0
    else :
        zplus = z
        
    """ C to tikz """
    segment = roadbm.ONESEGMENT( S, T )
    meas = roadbm.MEASURE( segment, ymin, ymax )
    obj = roadbm.OBJECTIVE( meas )
    Cf = roadbm.costWrapper( obj )
    
    str = "\\begin{tikzpicture}[x=2cm,y=.1cm]\n"
    
    # draw the axis
    ZMIN, ZMAX = -max( meas ), -min( meas )
    WIDTH = ZMAX - ZMIN
    
    C = [ ( Cf(z), z ) for z in range(ZMIN,ZMAX+1) ]
    CMIN, ZOPT = min(C)
    CMAX, _ = max(C)
    COPT = Cf(ZOPT)
    
    ZMIN = int( min( ZMIN, np.floor( zplus ) ) )
    ZMAX = int( max( ZMAX, np.ceil( zplus ) ) )
    
    # horizontal
    horz_level = np.floor(CMIN)
    data = { 'zmin' : ZMIN, 'zmax' : ZMAX, 'horz' : horz_level }
    fmt = "\\draw [->] (%(zmin)d,%(horz)d) -- (%(zmax)f,%(horz)d) node [right] {$\\coordvar$} ;\n" 
    str +=  fmt % data
    # vertical
    str += "\\draw [->] (0,%(cmin)f-1) -- (0,%(cmax)f) " % { 'cmin' : CMIN, 'cmax' : CMAX }
    str += "node [above] {$\\cost(\\numarcs)$} ;\n"
    # horizontal ticks
    for tick in np.arange(ZMIN,ZMAX+1,1) :
        data = { 'tick' : tick, 'horz' : horz_level }
        str += "\\draw (%(tick)d,%(horz)f-.1) -- (%(tick)f,%(horz)f+.1) " % data
        str += "node [below=5] {$%d$} ;\n" % tick
        
    # draw dashed C curve
    def drawpieces( ZZ, style ) :
        str = ''
        for z1,z2 in zip( ZZ[:-1], ZZ[1:] ) :
            c1 = Cf(z1)
            c2 = Cf(z2)
            
            data = { 'z1' : z1, 'z2' : z2, 'c1' : c1, 'c2' : c2, 'style' : style }
            str += "\\draw [%(style)s] (%(z1)f,%(c1)f) -- (%(z2)f,%(c2)f) ;\n" % data
            
        return str
            
    # draw dashed C, all of it
    str += drawpieces( range(ZMIN,ZMAX+1), 'dashed' )
    # draw C so far
    lastz = int( np.floor( zplus ) )
    str += drawpieces( range(ZMIN,lastz+1) + [ zplus ], 'thick' )
    
    # add local vertical line
    str += texvline( zplus, horz_level-1, max( CMAX, Cf(zplus) ) + 1, 'dashed' )
    # add optimal
    str += "\\draw node at (%(z)f,%(C)f) {$\\star$} ;\n" % { 'z' : ZOPT, 'C' : COPT }
    
    str += "\\end{tikzpicture}\n"
    return str
    















if __name__ == '__main__' :
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument( '--z', type=float, default=0 )
    parser.add_argument( '--Hout', type=str, default='Hout.tex' )
    parser.add_argument( '--Cout', type=str, default='Cout.tex' )
    args = parser.parse_args()
    
    
    YMIN = -4.
    YMAX = 4.
    WIDTH = YMAX - YMIN
    if False :
        zr = 0
        X = [ YMIN + WIDTH * np.random.rand() for i in xrange(5) ]
        O = [ YMIN + WIDTH * np.random.rand() for i in xrange(3) ]
    else :
        zr = 5.75
        #X = [ -3.5, 2, 2.75 ]
        #O = [ -3., -2., 1., 2.25, 3. ]
        X = [ -2.5, 2, ]
        O = [ -2., -1., 1., 3. ]
        
        
    str = heightFunctionTex( X, O, YMIN, YMAX, args.z )
    f = open( args.Hout, 'w' )
    f.write( str )
    f.close()
    
    #str = costFunctionTex( X, O, YMIN, YMAX, args.z )
    str = discreteCostTex( X, O, YMIN, YMAX, -4, 4, args.z )
    f = open( args.Cout, 'w' )
    f.write( str )
    f.close()

    




