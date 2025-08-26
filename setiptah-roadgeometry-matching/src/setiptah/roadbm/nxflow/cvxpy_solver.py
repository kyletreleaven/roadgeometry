
import cvxpy


def SOLVER( roadnet, surplus, objectives ) :
    prog, assist = PROGRAM( roadnet, surplus, objectives )
    prog.solve()
    
    res = dict()
    for road, a in assist.iteritems() :
        res[road] = int( round( a.value ) )
        
    return res
    

def PROGRAM( roadnet, surplus, objectives ) :
    """ construct the program """
    # optvars
    assist = dict()
    cost = dict()
    DELTA = .00001   # cvxpy isn't quite robust to non-full dimensional optimization
    
    for _,__,road in roadnet.edges_iter( keys=True ) :
        assist[road] = cvxpy.variable( name='z_{%s}' % road )
        cost[road] = cvxpy.variable( name='c_{%s}' % road )
    #print assist
    #print cost
        
    objfunc = sum( cost.values() )
    OBJECTIVE = cvxpy.minimize( objfunc )
    
    CONSTRAINTS = []
    
    # the flow conservation constraints
    for u in roadnet.nodes_iter() :
        INFLOWS = []
        for _,__,road in roadnet.in_edges( u, keys=True ) :
            INFLOWS.append( assist[road] + surplus[road] )
            
        OUTFLOWS = []
        for _,__,road in roadnet.out_edges( u, keys=True ) :
            OUTFLOWS.append( assist[road] )
            
        #conserve_u = cvxpy.eq( sum(OUTFLOWS), sum(INFLOWS) )
        error_u = sum(OUTFLOWS) - sum(INFLOWS)
        conserve_u = cvxpy.leq( cvxpy.abs( error_u ), DELTA )
        
        CONSTRAINTS.append( conserve_u )
        
    # the cost-form constraints
    for road in cost :
        for f, line in objectives[road].iter_items() :
            # is this plus or minus alpha?
            LB = cvxpy.geq( cost[road], line.offset + line.slope * assist[road] )
            CONSTRAINTS.append( LB )
    
    prog = cvxpy.program( OBJECTIVE, CONSTRAINTS )
    return prog, assist


