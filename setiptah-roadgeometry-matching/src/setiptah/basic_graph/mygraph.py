"""

TODO: Taking a O(log n) hit for dictionary lookups?

"""

class mygraph :
    def __init__(self) :
        self.E = {}
        self.V = {}
        self.W = {}     # partial list
        
    def __repr__(self) :
        return '(V:%s, E:%s, W:%s)' % ( repr(self.V), repr(self.E), repr(self.W) )
    
        
    def add_node(self, i ) :
        self.V.setdefault( i, set() )
        self.W.setdefault( i, set() )
        
    def add_edge(self, e, i, j ) :
        assert e not in self.E
        self.add_node(i)
        self.add_node(j)
        
        self.E[e] = (i,j)
        self.V[i].add(e)
        self.W[j].add(e)
        
    def remove_edge(self, e ) :
        assert e in self.E
        i,j = self.E[e]
        
        self.V[i].remove(e)
        self.W[j].remove(e)
        del self.E[e]
        
    def remove_node(self, i ) :
        succ = [ e for e in self.V[i] ]
        for e in succ : self.remove_edge(e)
        pred = [ e for e in self.W[i] ]
        for e in pred : self.remove_edge(e)
        
        del self.V[i]
        del self.W[i]
        
    def nodes(self) : return self.V.keys()
    
    def has_node(self, i ) : return i in self.V
    
    def edges(self) : return self.E.keys()
    
    def has_edge(self, e ) : return e in self.E
        
    def endpoints(self, e ) : return self.E[e]

        
if __name__ == '__main__' :
    g = mygraph()
    g.add_edge( 'a', 0, 1 )
    g.add_edge( 'b', 0, 2 )
    g.add_edge( 'c', 2, 3 )
    g.add_edge( 'd', 3, 0 )
    
    c = { 'a' : 10., 'b' : 5., 'c' : 1., 'd' : .5 }
    f = { 'a' : 3., 'b' : 1.37 }
