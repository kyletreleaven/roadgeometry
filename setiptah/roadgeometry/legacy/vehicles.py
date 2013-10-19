
#import roadmap

from mixedmultigraph import MixedMultiGraph
import astar

def triplet_slider( seq ) :
    return zip( seq[::2], seq[1::2], seq[2::2] )




""" System, State and Trajectory Abstractions """
class AbstractSystem(object) :
    def copy(self) :
        raise Exception('not implemented')
    
    def state(self) :
        raise Exception('not implemented')
    
    def trajectory(self) :
        raise Exception('not implemented')
    
    def rewind(self, time ) :
        raise Exception('not implemented')
    
    def reset(self) :
        raise Exception('not implemented')
    
    
    """ abstract system objects """
    class State(object) :
        def __eq__(self, other ) :
            raise Exception('not implemented')
        
    class Trajectory(object) :
        def copy(self) :
            raise Exception('not implemented')
        
        def isempty(self) :
            raise Exception('abstract method')
        
        def length(self) :
            raise Exception('not implemented')
        
        def start(self) :
            raise Exception('not implemented')
        
        def end(self) :
            raise Exception('not implemented')
        
        def __add__(self, other ) :
            # for concatenation of matched trajectories
            raise Exception('not implemented')
        
        def __sub__(self, other ) :
            # can subtract "time"
            raise Exception('not implemented')
        
        def at_time(self, t ) :
            raise Exception('not implemented')
        
        def fragment(self, t1, t2, left_closed=True, right_open=True ) :
            raise Exception('not implemented')






"""
Omni-directional Vehicle Model
Special vehicle model is used to "define" the road network metric space.
"""
class OmniDirectional(AbstractSystem) :
    def __init__( self, roadmap=None, *args ) :
        self.set_roadmap( roadmap )
        self.set_state( *args )
        #
        self.reset()
        
    def copy(self, trajectory=False ) :
        res = OmniDirectional( self.roadmap, self.road, self.x )
        if trajectory :
            res._traj = self._traj
        return res

    def __repr__(self) :
        return 'omnidir at %s' % repr( self.state() )

    """ initialization/configuration """
        
    def set_roadmap(self, roadmap ) :
        if roadmap is None or isinstance( roadmap, MixedMultiGraph ) :
            self.roadmap = roadmap
            self.mmgraph = roadmap      # laziness
            
        else :
            raise Exception('roadmap must be multi-digraph')
        
        return self
        
    def set_state(self, *args ) :
        if len( args ) == 1 :
            if isinstance( args[0], OmniDirectional.State ) :
                state = args[0]
                road = state.road
                x = state.x
            elif isinstance( args[0], tuple ) :
                #print args[0]
                road, x = args[0]
                
        elif len( args ) == 2 :
            road, x = args
            
        else :
            raise Exception()
        
        edge = self.mmgraph.edge( road )
        roadlen = self.mmgraph.get_edge_data( edge ).get('length', 1 )
        assert x >= 0 and x <= roadlen
        self.road = road
        self.x = x
        
        self.reset()
        return self
        
        
    def reset(self) :
        self._traj = OmniDirectional.Trajectory( self.state() )
        return self
    
    """ configuration: query """
    def roadmap(self) :
        return self.roadmap

    def state(self) :
        return OmniDirectional.State( self.road, self.x )
    
    def is_interior(self, state=None ) :
        if state is None : state = self.state()
        road = state.road
        edge = self.mmgraph.edge( road )
        roadlen = self.mmgraph.get_edge_data( edge ).get( 'length', 1 )
        
        return state.x > 0 and state.x < roadlen
    
    def available_jumps(self) :
        # assume there's no point in jumping to the head of an arc
        # what's the "call sign" of a jump? : (road,left/right)?
        road = self.road
        x = self.x
        
        edge = self.mmgraph.edge( road )
        roadlen = self.mmgraph.get_edge_data( edge ).get( 'length', 1 )
        
        nodeset = set()
        if x <= 0. :        nodeset.add( edge.start )
        if x >= roadlen :   nodeset.add( edge.end )
        
        jumps = []
        for node in nodeset :
            for edge in self.mmgraph.out_edges_iter( node ) :
                if edge.id == road : continue
                u, v = edge.endpoints()
                if u == node : jumps.append( (edge.id,'start') )
                if v == node : jumps.append( (edge.id,'end') )
                
        return jumps
    
    def trajectory(self) :
        if len( self._traj.seq ) <= 0 : return None
        return self._traj.copy()
    
    
    
    
    """ SYSTEM CONTROL INTERFACE """
    
    """ LOCAL CONTROL """
    def apply_control(self, u, duration=None, ignore_direction=False ) :
        road = self.road
        x = self.x
        edge = self.mmgraph.edge( road )
        roadlen = self.mmgraph.get_edge_data( edge ).get('length', 1 )
        
        # check input
        if u < 0. :
            if edge.directed and not ignore_direction:
                raise Exception('this road is directed')
            horizon = -self.x / u
        elif u == 0. :
            horizon = np.inf
        elif u > 0. :
            horizon = ( roadlen-self.x ) / u
            
        if duration is None : duration = horizon
        if duration > horizon :
            print duration, horizon
            raise Exception( 'control does not satisfy invariant %f > %f' % ( duration, horizon ) )
            return None
        
        # create the trajectory
        xx = self.x + u * duration
        start_state = self.state()
        self.x = xx
        
        if duration > 0. :
            prim = OmniDirectional.Trajectory.ConstantSpeed( u, duration )
            end_state = self.state()
            self._traj.seq.append( prim )
            self._traj.seq.append( end_state )
            
        return self
    
    
    def jump(self, road, side ) :
        if not (road,side) in self.available_jumps() :
            raise Exception('not an available jump')
        
        start_state = self.state()
        
        end_edge = self.mmgraph.edge( road )
        if side == 'start' :
            node = end_edge.start
            self.road = road
            self.x = 0.
        elif side == 'end' :
            node = end_edge.end
            roadlen = self.mmgraph.get_edge_data( end_edge ).get( 'length', 1 )
            self.road = road
            self.x = roadlen
        else : raise Exception('wat happened?')
        
        prim = OmniDirectional.Trajectory.Jump( (road,side), node )
        end_state = self.state()
        self._traj.seq.append( prim )
        self._traj.seq.append( end_state )
        
        return self
    
    
    def follow(self, traj ) :
        # will "sanity check" the data
        if not isinstance( traj, OmniDirectional.Trajectory ) :
            raise Exception()
        
        assert self.state() == traj.start()
        
        for s1,prim,s2 in triplet_slider( traj.seq ) :
            if isinstance( prim, OmniDirectional.Trajectory.ConstantSpeed ) :
                u = prim.u
                duration = prim.duration()
                self.apply_control( u, duration )
                
            elif isinstance( prim, OmniDirectional.Trajectory.Jump ) :
                road, side = prim.jump
                self.jump( road, side )
                
            else : raise Exception()
        
        return self
    
    
    """ HIGH-LEVEL CONTROL """
    def move_to_on_road(self, state ) :
        road = self.road
        if not state.road == road :
            raise OmniDirectional.CannotComply()
        
        edge = self.mmgraph.edge( road )
        p_coord = self.x
        q_coord = state.x
        
        if q_coord >= p_coord :
            self.apply_control( 1., q_coord - p_coord )
        elif not edge.directed :
            self.apply_control( -1., p_coord - q_coord )
            
        return self
    
    
    def move_to(self, state ) :
        """
        moves to state (if possible), outputs some shortest path trajectory (if such exists) 
        """
        options = []
        
        # it *could* be local
        try :
            traj = self.copy().move_to_on_road( state ).trajectory()
            options.append( ( traj.length(), traj ) )
        except OmniDirectional.CannotComply :
            pass
        
        other_sys = self.copy().set_state( state )
        p_road = self.road
        p_edge = self.mmgraph.edge( p_road )
        q_road = other_sys.road
        q_edge = self.mmgraph.edge( q_road )
        
        # but *probably* not
        if self.is_interior() :
            for u, condition in [ (1.,True), (-1., not p_edge.directed ) ] :
                if condition :
                    sys = self.copy()
                    sys.apply_control( u, None, ignore_direction=True ).move_to( state )
                    traj = sys.trajectory()
                    options.append( ( traj.length(), traj ) )
                
        elif other_sys.is_interior() :
            try :
                for u, condition in [ (-1.,True), (1., not q_edge.directed ) ] :
                    if condition :
                        end_state = other_sys.state()
                        mid_state = other_sys.copy().apply_control( u, None, ignore_direction=True ).state()
                        sys = self.copy()
                        sys.move_to( mid_state ).move_to_on_road( end_state )
                        traj = sys.trajectory()
                        options.append( ( traj.length(), traj ) )
            except Exception :
                print self.state(), mid_state, end_state
                print u, condition, self.available_jumps()
                
        else :
            if self.x <= 0. :
                u = p_edge.start
            else :
                u = p_edge.end
                
            if other_sys.x <= 0. :
                v = q_edge.start
            else :
                v = q_edge.end
                
            path = astar.astar_path( self.mmgraph, u, v )
            #print path
            
            # make a copy of self, and having it follow the "path"
            sys = self.copy() 
            for u, edge, v in triplet_slider( path ) :
                if u == edge.start :
                    side = 'start'
                    control = 1.
                elif u == edge.end :
                    side = 'end'
                    control = -1.
                else : raise Exception()
                
                if not sys.road == edge.id :
                    sys.jump( edge.id, side )
                sys.apply_control( control ) 
                
            if not sys.road == other_sys.road :
                if other_sys.x <= 0. :
                    sys.jump( other_sys.road, 'start' )
                else :
                    sys.jump( other_sys.road, 'end' )
            
            # store the trajectory information
            traj = sys.trajectory()
            options.append( ( traj.length(), traj ) )
            
            
        # selection
        _, traj = min( options )
        #print traj
        self.follow( traj )
        return self
    
    
    
    
    """ System Information """
    class State(AbstractSystem.State) :
        def __init__(self, road, x ) :
            self.road = road
            self.x = x
            
        def __eq__(self, other ) :
            return self.road == other.road and self.x == other.x
        
        def __repr__(self) :
            return '(%s,%f)' % ( repr( self.road ), self.x )
    
    class Trajectory(AbstractSystem.Trajectory) :
        def __init__(self, state=None ) :
            self.seq = []
            if isinstance( state, OmniDirectional.State ) :
                self.seq.append( state )
            
        def __repr__(self) :
            return repr( self.seq )
            
        def copy(self) :
            res = OmniDirectional.Trajectory()
            res.seq = self.seq
            return res
        
        def isnone(self) :
            return len( self._traj ) == 0
            
        def length(self) :
            mylen = sum( prim.duration() for prim in self.seq[1::2] )
            return float( mylen )
        
        def start(self) :
            return self.seq[0]
            #return self.seq[0].start_state
        
        def end(self) :
            return self.seq[-1]
            #return self.seq[-1].end_state
            
        # trajectory operators
        def __add__(self, other ) :
            if not isinstance( other, OmniDirectional.Trajectory ) :
                raise Exception()
                return
            
            res = self.copy()
            res.seq.extend( other.seq[1:] )
            return res
                
        def __sub__(self, other ) :
            raise Exception('not implemented')
        
        def at_time(self, t ) :
            raise Exception('not implemented')
        
        def fragment(self, t1, t2, left_closed=True, right_open=True ) :
            raise Exception('not implemented')
            
        """ construction """
        """ primitive trajectory objects """
        class Primitive(object) :
            def duration(self) :
                return 0.
            
        class OnRoad(Primitive) :
            def duration(self) :
                return self.length
            
        class ConstantSpeed(OnRoad) :
            def __init__(self, u, duration ) :
                self.u = u
                self.length = duration
                #self.start_state = start_state
                #self.end_state = end_state
                
            def __repr__(self) :
                return '%f for %f' % ( self.u, self.length )
                #return '%s to %s' % ( repr(self.start_state), repr(self.end_state) )
        
        class Jump(Primitive) :
            def __init__(self, jump, node ) :
                self.jump = jump
                #self.start_state = start_state
                #self.end_state = end_state
                self.node = node
                
            def __repr__(self) :
                #return '(%s to %s)' % ( repr(self.start_state), repr(self.end_state) )
                return '(%s at %s)' % ( repr( self.jump ), repr(self.node) )
            
    """ exceptions """
    class CannotComply(Exception) : pass
    
    
    
    
    
    
    
if __name__ == '__main__' :
    
    graph = MixedMultiGraph()
    graph.add_path( 0, 'S', 1, 'W', 2, 'N', 3 )
    graph.add_path( 3, 'E', 0, directed=True )
    
    #roadmap = RoadMap.from_graph( graph )
    state1 = OmniDirectional.State( 'N', .25 ) 
    state2 = OmniDirectional.State( 'E', .5 )
    
    gold = OmniDirectional( graph, state1 )
    
    car = gold.copy()
    other = car.copy().move_to( state2 )
    
    temp = car.state()
    car.move_to( other.state() )
    other = other.copy().move_to( temp )    #other.move_to( temp )
    
    #car.move_to( state2 )
    
