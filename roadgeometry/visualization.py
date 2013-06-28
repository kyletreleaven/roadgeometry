
import random
import numpy as np

import networkx as nx

# this may cause trouble
from utility import enum, Immutable, slidingpairs, setchoose2

import scipy as sp
import scipy.optimize

import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.patches
import matplotlib.lines

from matplotlib.colors import colorConverter
#from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import axes3d, art3d

import roadmap_basic as ROAD


def fitArc2Segment( pt1, pt2, arclen, below=False ) :
    """ provides center, radius, and theta1->theta2 (in CCW direction!) """
    pt1 = np.array(pt1)
    pt2 = np.array(pt2)
    diff = pt2 - pt1
    dEuc = np.linalg.norm( diff )
    angle = np.arctan2( diff[1], diff[0] )
    
    # d = R Theta
    # dEuc = 2 R sin( Theta/2 )
    # sinc(x) = sin(pi*x) / (pi*x)
    func = lambda theta : np.sinc( theta / ( 2*np.pi ) ) - dEuc/arclen
    theta = sp.optimize.bisection( func, 0., 2*np.pi )
    R = arclen / theta
    
    # before rotation and translation
    x = dEuc/2
    y = R * np.cos( 0.5 * theta )
    if below == True : y = -y       # option to invert
    
    #y = np.sqrt( np.power(R,2.) - np.power(x,2.) )
    #y = np.sqrt( R**2 - x**2 )
    xrot = np.cos(angle) * x - np.sin(angle) * y
    yrot = np.sin(angle) * x + np.cos(angle) * y
    center = pt1 + np.array([ xrot, yrot ])
    vi = pt1 - center
    vj = pt2 - center
    thetai = np.arctan2( vi[1], vi[0] )
    thetaj = np.arctan2( vj[1], vj[0] )
    
    return center, R, thetai, thetaj
    
    

def fitArcThruSpacing( pt1, pt2, ht ) :
    pt1 = np.array(pt1)
    pt2 = np.array(pt2)
    diff = pt2 - pt1
    dEuc = np.linalg.norm(diff)
    # ( R - ht )**2 + ( dEuc/2 )**2 = R**2
    ht_positive = ( ht > 0. )
    ht = np.abs(ht)
    R = ( ht**2 + (dEuc/2)**2 ) / ht / 2
    theta = 2 * np.arcsin( dEuc / R / 2 )
    arclen = R*theta
    #print pt1, pt2, ht
    #print R, theta, d, dEuc
    return fitArc2Segment( pt1, pt2, arclen, below = not ht_positive )



def angleconvert_zerototwopi( theta ) :
    circum = 2 * np.pi
    return theta % circum
    
def angleconvert_minuspitopi( theta ) :
    circum = 2 * np.pi
    return angleconvert_zerototwopi( theta + np.pi ) - np.pi


def angle_interpolate_ccw( t1, t2, x, reverse=False ) :
    if reverse == False :
        delta = angleconvert_zerototwopi( t2 - t1 )
        theta = t1 + x * delta
    else :
        delta = angleconvert_zerototwopi( t1 - t2 )
        theta = t1 - x * delta
    return angleconvert_minuspitopi( theta )

def angle_interpolate_shortest( t1, t2, x ) :
    delta = angleconvert_zerototwopi( t2 - t1 )
    if delta <= np.pi :
        res = angle_interpolate_ccw( t1, t2, x )
    else :
        res = angle_interpolate_ccw( t1, t2, x, reverse=True )
    return res







    
class RoadData(object) :
    def __init__(self) :
        self.shapes = {}
        
    def coord_transform_road_to_plane(self, x ) :
        pass
    
    
class shape_data(object) :
    def __init__(self, type_str ) :
        self.type = type_str
    
    



class PlanarLayout(object) :
    """
    to assist in planar graph visualization, including drawing arcs in the right sizes
    """
    
    """ constructors """
    @classmethod
    def fromroadnet(cls, roadnet, length='length' ) :
        res = PlanarLayout()
        res.nodes_layout = cls.nodes_layout(roadnet, length)
        res.roads_layout = cls.roads_layout(roadnet, res.nodes_layout, length)
        return res
    
    @classmethod
    def nodes_layout(cls, roadnet, length='length' ) :
        """ produce spring layout """
        layout = nx.layout.spring_layout( roadnet, weight=length )
        
        # re-scale... for whatever reason
        ratio = {}
        for edge in roadnet.edges( keys=True ) :
            i,j,road = edge
            dEuc = np.linalg.norm( layout[j] - layout[i] )
            d = roadnet.get_edge_data(i,j,key).get( length, 1 )
            ratio[edge] = d / dEuc
            
        if True :
            minratio = min( ratio.values() )
            maxratio = max( ratio.values() )
            for node in layout :
                layout[node] *= .5 * maxratio
                
        return layout
    
    @classmethod
    def roads_layout(cls, roadnet, nodes_layout, length='length' ) :
        layout = {}
        # do some stuff...
        return layout
    
    
    """ sub-constructors """
    @classmethod
    def roads_between_nodes(cls, u, v, roadnet, layout, length='length' ) :
        pt1 = layout[uu]
        pt2 = layout[vv]
        diff = pt2 - pt1
        dEuc = np.linalg.norm( diff )
        
        road_layout = {}
         
        # get the list of roads between the nodes (both directions)
        roads_utov = roadnet.edge[u][v]
        roads_vtou = roadnet.edge[v][u]
        
        # sort relevant data, by length
        # ( length, road, (left,right) )
        temp1 = [ ( data.get( length, 1 ), road, (u,v) ) for road, data in roads_utov.iteritems() ]
        temp2 = [ ( data.get( length, 1 ), road, (v,u) ) for road, data in roads_vtou.iteritems() ]
        temp = sorted( temp1 + temp2 )
        
        # get a RoadData per road, including shapes, add arrows to oneways; add to road_layout 
        for len_idx, chunk in enumerate( temp ) :
            d, road, dir = chunk
            uu, vv = dir
            
            roadinfo = RoadData()
            """ from a length net, layout, and edge, get the shape of the circular/linear arc representation """
            """
            > dictionary with keys 'dashed' (for edge drawing) and 'solid' (for the geometry)
            > shape has type ( 'arc' or 'segment' ), and relevent details
            """
            if d > dEuc :
                center, R, theta1, theta2 = fitArc2Segment( pt1, pt2, d )
                arc = shape_data('arc')
                arc.center = center
                arc.radius = R
                arc.theta1 = theta1
                arc.theta2 = theta2
                roadinfo.shapes['solid'] = arc
                
            else :        # treat as fragment of a curve (or straight line)
                if len_idx == 0 :
                    seg = shape_data('segment')
                    seg.xdata = [ pt1[0], pt2[0] ]
                    seg.ydata = [ pt1[1], pt2[1] ]
                    roadinfo.shapes['dashed'] = seg
                    
                    mid = ( pt1 + pt2 ) / 2
                    offset = ( d/dEuc/2 ) * ( pt2 - pt1 )
                    pt1_inner, pt2_inner = mid-offset, mid+offset
                    
                    subseg = shape_data('segment')
                    subseg.xdata = [ pt1_inner[0], pt2_inner[0] ]
                    subseg.ydata = [ pt1_inner[1], pt2_inner[1] ]
                    roadinfo.shapes['solid'] = subseg
                    
                else :
                    spacing = (-1)**len_idx * np.ceil( float(len_idx) / 2 )
                    ht = spacing * ( dEuc / 20 )        # heuristic
                    center, R, theta1, theta2 = fitArcThruSpacing( pt1, pt2, ht )
                    if ht < 0. :
                        temp = theta1
                        theta1 = theta2
                        theta2 = temp
                        
                    arc = shape_data('arc')
                    arc.center = center
                    arc.radius = R
                    arc.theta1 = theta1
                    arc.theta2 = theta2
                    roadinfo.shapes['dashed'] = arc
                    
                    dtheta = angleconvert_zerototwopi( theta2 - theta1 )
                    darc = R*dtheta
                    y = d / darc
                    t1 = angle_interpolate_ccw( theta1, theta2, 0.5 - y/2 )
                    t2 = angle_interpolate_ccw( theta1, theta2, 0.5 + y/2 )
                    
                    subarc = shape_data('arc')
                    subarc.center = center
                    subarc.radius = R
                    subarc.theta1 = t1
                    subarc.theta2 = t2
                    roadinfo.shapes['solid'] = subarc
                    
        return shapes
        
        
    
    """ QUERIES """
    def address_planarcoord(self, road_addr ) :
        pass
    
    
    
    
    
    
    
    
    
    
class RoadShape(object) :
    
    @classmethod
    def roads_between_nodes(cls, roadnet, u, v, layout, length='length' ) :
        pass
        
        
    @classmethod
    def edgeshape(cls, roadnet, layout, edge ) :
        pass






    
    
    @classmethod
    def address_coord(cls, addr, roadnet, layout, length='length' ) :
        if not isinstance( addr, ROAD.RoadAddress ) : raise 'not a valid RoadAddress'
        
        edge, road_data = ROAD.obtain_edge( roadnet, addr.road, True )
        u,v,key = edge
        
        pt1 = layout[u] ; pt2 = layout[v]
        dEuc = np.linalg.norm( pt2 - pt1 )
        d = road_data.get( length, 1 )
        
        if dEuc < d :
            """ on arc """
            center, R, t1, t2 = fitArc2Segment( pt1, pt2, d )
            theta = angle_interpolate_ccw( t1, t2, addr.coord )
            coord = center + R * np.array([ np.cos(theta), np.sin(theta) ])
        else :
            """ segment part """
            idx = lnet.edge[i][j].keys().index(key)
            if idx == 0 :
                """ on "shorty" """
                mid = ( pt1 + pt2 ) / 2
                offset = ( d/dEuc / 2 ) * ( pt2 - pt1 )
                pt1_inner, pt2_inner = mid-offset, mid+offset
                coord = x * pt2_inner + ( 1.0 - x ) * pt1_inner
            else :
                """ on one of the spaced arcs """
                spacing = (-1)**idx * np.ceil( float(idx) / 2 )
                ht = ( dEuc / 20 ) * spacing
                center, R, theta1, theta2 = fitArcThruSpacing( pt1, pt2, ht )
                if ht > 0. :
                    dtheta = angleconvert_zerototwopi( theta2 - theta1 )
                    darc = R * dtheta
                    y = d / darc
                    t1 = angle_interpolate_ccw( theta1, theta2, 0.5 - y/2 )
                    t2 = angle_interpolate_ccw( theta1, theta2, 0.5 + y/2 )
                    theta = angle_interpolate_ccw( t1, t2, x )
                else :
                    dtheta = angleconvert_zerototwopi( theta1 - theta2 )
                    darc = R * dtheta
                    y = d / darc
                    t1 = angle_interpolate_ccw( theta1, theta2, 0.5 - y/2, reverse=True )
                    t2 = angle_interpolate_ccw( theta1, theta2, 0.5 + y/2, reverse=True )
                    theta = angle_interpolate_ccw( t1, t2, x, reverse=True )
                coord = center + R * np.array([ np.cos(theta), np.sin(theta) ])
        return coord
    


    class AddressTest(object) :
        def runtest(self) :
            lnet = LengthNetwork.randomgraph( 10, .8, weighted=True )
            layout = LengthNetworkPlanarLayout.fromlengthnet( lnet )
            locs = [ lnet.randomaddress() for i in xrange(1000) ]
            coords = [ LengthNetworkPlanarLayout.address_coord( loc, lnet, layout ) for loc in locs ]
            X = [ c[0] for c in coords ]
            Y = [ c[1] for c in coords ]
            LengthNetworkPlanarLayout.draw_lnet( lnet, layout )
            plt.scatter(X,Y)
        
        
    class EdgeShapeTest(object) :
        def runtest(self) :
            self.lnet = LengthNetwork.randomgraph( 10, .8 )
            self.layout = nx.layout.spring_layout(self.lnet)
            self.edge = random.choice( self.lnet.edges(keys=True) )
            i,j,key = self.edge
            dEuc = np.linalg.norm( self.layout[j] - self.layout[i] )
            d = 2. * dEuc
            self.lnet.get_edge_data(i,j,key)['weight'] = d
            
            
        def testfit(self) :
            pt1 = np.random.rand(2)
            pt2 = np.random.rand(2)
            dEuc = np.linalg.norm( pt2-pt1 )
            d = ( 1. + np.random.rand() ) * dEuc
            center, R, theta1, theta2 = fitArc2Segment( pt1, pt2, d )

            plt.figure()
            xdata = [ pt1[0], pt2[0] ]
            ydata = [ pt1[1], pt2[1] ]
            plt.scatter(xdata,ydata)
            ax = plt.gca()
            circ = mpl.patches.Circle( center, R, linestyle='--' )
            ax.add_patch(circ)
            arc = mpl.patches.Arc( center, R, R, 0., theta2, theta1 )
            ax.add_patch(arc)
            ax.set_aspect('equal')
            
            
        def display(self) :
            plt.figure()
            self.ax = plt.gca()
            i,j,key = self.edge
            
            nx.draw( self.lnet, pos=self.layout, ax=self.ax )
            #pti = self.layout[i]
            #ptj = self.layout[j]
            #X = [ pti[0], ptj[0] ]
            #Y = [ pti[1], ptj[1] ]
            #plt.scatter(X,Y)
            
            P = PlanarLayout.edgeshape( self.lnet, self.layout, self.edge )
            for p in P : self.ax.add_patch(p)
            self.ax.set_aspect('equal')
            
        
        
        
        

    
def mplArt_from_shape( shapes, in3d=False, z=0.0, zdir='z' ) :
    """
    can be made an Arc with:
    matplotlib.patches.Arc(xy, width, height, angle=0.0, theta1=0.0, theta2=360.0, **kwargs)
    for circle width=height=R
    """
    """
    > dictionary with keys 'dashed' (for edge drawing) and 'solid' (for the geometry)
    > shape has type ( 'arc' or 'segment' ), and relevent details
    """
    Art = []
    for style in [ 'solid', 'dashed' ] :
        if not style in shapes : continue
        
        shape = shapes[style]
        if shape.type == 'arc' :
            piece = mpl.patches.Arc( shape.center, 2*shape.radius, 2*shape.radius,
                                   angle = 0.0,
                                   theta1 = 180.0 * shape.theta1 / np.pi,
                                   theta2 = 180.0 * shape.theta2 / np.pi,
                                   linestyle=style, color='b' )
            if in3d : piece = art3d.patch_2d_to_3d( piece, z=z, zdir=zdir )
            
        elif shape.type == 'segment' :
            piece = mpl.lines.Line2D( shape.xdata, shape.ydata, linestyle=style )
            if in3d : piece = art3d.line_2d_to_3d( piece, zs=z, zdir=zdir )
            
        Art.append(piece)
    return Art





def art_convert3d( art, z=0.0, zdir='z' ) :
    """ doesn't effing work """
    if isinstance( art, mpl.lines.Line2D ) :
        new_art = art3d.line_2d_to_3d( art, zs=z, zdir=zdir )
    elif isinstance( art, mpl.patches.Patch ) :
        new_art = art3d.patch_2d_to_3d( art, z=z, zdir=zdir )
    else : raise Exception('not a valid type of art')
    
    return new_art







