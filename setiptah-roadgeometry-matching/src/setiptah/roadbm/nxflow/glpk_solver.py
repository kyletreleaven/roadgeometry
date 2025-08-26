
import os
import tempfile
import subprocess as subp

import itertools


def package_local( filename ) :
    dirname = os.path.dirname( __file__ )
    return os.path.join( dirname, filename )



""" this one is going to write a Mathprog file instead of a cvxpy program """
def SOLVER( roadnet, surplus, objectives ) :
    prog, map = PROGRAM( roadnet, surplus, objectives )
    
    # prepare file space
    data = tempfile.NamedTemporaryFile()
    output = tempfile.NamedTemporaryFile( delete=False )
    output_name = output.name
    output.file.close()
    
    # write data file
    data.file.write( prog )
    data.file.flush()
    
    # prepare command
    cmd = [ 'glpsol', '--math', '--interior' ]
    cmd.extend([ '-m', package_local('pl_nxopt.model') ])
    cmd.extend([ '-d', data.name ])
    cmd.extend([ '-y', output_name ])
    subp.call( cmd )
    
    data.file.close()
    
    output = open( output_name, 'r' )
    ans = output.readlines()
    output.close()
    
    os.remove( output_name )
    
    assist = dict()
    for line in ans :
        road, z = line.split()
        assist[ map[ int(road) ] ] = int(z)
        
    print assist
    return assist
    
    
def PROGRAM( roadnet, surplus, objectives ) :
    """ write a Mathprog data file """
    data_str = "data;\n\n"
    assist = dict()
    
    VERTS = dict()
    for k, u in enumerate( roadnet.nodes_iter() ) :
        VERTS[u] = k
    
    ROADS = dict()
    TOPOLOGY = []
    for k, e in enumerate( roadnet.edges_iter( keys=True ) ) :
        u, v, road = e
        ROADS[road] = k
        assist[k] = road
        
        tup = ( k, VERTS[u], VERTS[v] )
        TOPOLOGY.append( tup )
        
    data_str += "set VERTS := "
    for k in VERTS.values() : data_str += "%d " % k
    data_str += ";\n\n"
    
    data_str += "set ROADS := "
    for k in ROADS.values() : data_str += "%d " % k
    data_str += ";\n\n"
    
    data_str += "set TOPOLOGY := "
    for tup in TOPOLOGY :
        data_str += "(%d,%d,%d) " % tup
    data_str += ";\n\n"
    
    data_str += "param b := "
    for road, b in surplus.iteritems() :
        data_str += "%d %d  " % ( ROADS[road], b )
    data_str += ";\n\n"
    
    LINES = []
    ROWS = []
    slope = dict()
    offset = dict()
    
    line_iter = itertools.count()
    for road, line_data in objectives.iteritems() :
        for f, line in line_data.iter_items() :
            k = line_iter.next()
            LINES.append( k )
            row = ( k, ROADS[road] )
            ROWS.append( row )
            
            slope[k] = line.slope
            offset[k] = line.offset
            
    data_str += "set LINES := "
    for k in LINES : data_str += "%d " % k
    data_str += ";\n\n"
    
    data_str += "set ROWS := "
    for row in ROWS : data_str += "(%d,%d) " % row
    data_str += ";\n\n"
    
    data_str += "param slope := "
    for item in slope.iteritems() :
        data_str += "%d %f  " % item
    data_str += ";\n\n"
    
    data_str += "param offset := "
    for item in offset.iteritems() :
        data_str += "%d %f  " % item
    data_str += ";\n\n"
    
    data_str += "end;\n"
    
    return data_str, assist


