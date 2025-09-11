import bintrees
import networkx as nx
import numpy as np

from setiptah.roadgeometry.matching.nx_legacy import MultiDiGraphRoadnet

""" my dependencies """
from setiptah.roadgeometry.matching import (
    nx_legacy as roadbm,
    RoadnetMatchingProblem,
    MatchingResult,
)
import setiptah.roadgeometry.legacy.roadmap_basic as ROAD

import matplotlib.pyplot as plt

VERTEX = 'v'
POINT_IN_S = 'S'
POINT_IN_T = 'T'


def INTERVAL_GRAPH( match, S, T, roadmap, pos, length_attr='length' ) :
    # start a "path graph" --- damn, has to be undirected...
    digraph = nx.DiGraph()
    skeleton = nx.Graph()

    # sort points onto segments
    segments = roadbm.SEGMENTS( S, T, roadmap)

    for u, v, road, data in roadmap.edges( keys=True, data=True ) :
        # store coordinate of the r^+ endpoint for later use
        length = data.get( length_attr, 1 )

        # enumerates the points on segment in a specific order
        def traverse() :
            yield 0., VERTEX, u     # location, type, label
            for y, queue in segments[road].iter_items() :
                for s in queue.supply: yield y, POINT_IN_S, s
                for t in queue.demand: yield y, POINT_IN_T, t
            yield length, VERTEX, v

        # bigram enumeration and edge insertion
        ITER = traverse()
        prev = next(ITER)
        for y2, type2, label2 in ITER:
            y1, type1, label1 = prev

            # insert edge into score graph *and* skeleton graph
            digraph.add_edge( (type1,label1), (type2,label2), score=0 )
            skeleton.add_edge( (type1,label1), (type2,label2), length=y2-y1 )

            prev = y2, type2, label2

    # for each match in the matching
    for i, j in match :
        # find shortest path on the *skeleton* graph, i.e., ignoring direction
        path = nx.shortest_path( skeleton, (POINT_IN_S,i), (POINT_IN_T,j),
                                 weight='length' )

        # direct unit score along shortest path
        for ii, jj in zip( path[:-1], path[1:] ) :
            # path traverses edge in the forward direction
            if digraph.has_edge( ii, jj ) :
                data = digraph.get_edge_data( ii, jj )
                data['score'] += 1

            # otherwise, path traverses edge in the backward direction
            elif digraph.has_edge( jj, ii ) :
                data = digraph.get_edge_data( jj, ii )
                data['score'] -= 1

                # then, reverse edge if it has negative score (only need to check if minus)
                score = data['score']
                if score < 0 :
                    digraph.remove_edge( jj, ii )
                    digraph.add_edge( ii, jj, score = -score )

            else:
                raise Exception('edge not found')

    def vertpos(u) : return pos[u]
    def pos_from_S(u) : return position( S[u], roadmap, pos )
    def pos_from_T(u) : return position( T[u], roadmap, pos )
    switch = { VERTEX : vertpos,
              POINT_IN_S : pos_from_S,
              POINT_IN_T : pos_from_T }

    other_pos = {}
    for uu in digraph.nodes() :
        typeu, labelu = uu
        other_pos[uu] = switch[typeu]( labelu )

    return digraph, other_pos


def position(address, roadmap, pos, length_attr='length'):
    """
    get the Euclidean position of a street address,
    given roadmap and dictionary of vertex positions
    """
    road, coord = address
    coord = float(coord)

    u, v, key = ROAD.obtain_edge(roadmap, road)
    assert key == road
    data = ROAD.get_road_data(road, roadmap)
    width = data.get( length_attr, 1 )

    #ROAD.get_edge_data( )
    x = pos[u]
    vec = pos[v] - x
    return x + vec * coord / width


def SANITIZE(I_graph):
    # remove nil edges (so sorry)
    edges = list(I_graph.edges(data=True))
    for i, j, data in edges:
        if data['score'] <= 0:
            I_graph.remove_edge(i, j)

    # remove nodes with no degree
    nodes = list(I_graph.nodes())
    for i in nodes:
        if I_graph.in_degree(i) <= 0 and I_graph.out_degree(i) <= 0:
            I_graph.remove_node(i)


def INITIALIZE_BAGS(I_graph):
    for i, data in I_graph.nodes(data=True):
        typei, labeli = i

        data.update(S=[], T=[])

        if typei == VERTEX:
            continue

        elif typei == POINT_IN_S:
            data['S'].append(labeli)

        elif typei == POINT_IN_T:
            data['T'].append(labeli)

        else:
            raise Exception('unrecognized node type')


def UPDATE_EDGE(i, j, I_graph, MATCH):
    src_data = I_graph.nodes[i]
    src_S_bag = src_data['S']

    edge_data = I_graph.get_edge_data(i, j)
    score = edge_data['score']

    dst_data = I_graph.nodes[j]
    dst_S_bag = dst_data['S']
    dst_T_bag = dst_data['T']

    cancel = min(score, len(dst_T_bag))
    shift = score - cancel

    for k in range(cancel):
        s = src_S_bag.pop(0)
        t = dst_T_bag.pop(0)
        MATCH.append((s, t))

    temp, src_S_bag[:] = src_S_bag[:shift], src_S_bag[shift:]
    dst_S_bag[:] = temp + dst_S_bag[:]

    I_graph.remove_edge(i, j)

    if len(src_S_bag) <= 0: I_graph.remove_node(i)
    if len(dst_S_bag) + len(dst_T_bag) <= 0: I_graph.remove_node(j)
    # if I_graph.out_degree( i ) <= 0 : I_graph.remove_node( i )


def UPDATE_NODE(i, I_graph, MATCH):
    """ node must have *no* in-degree """

    src_data = I_graph.node[i]
    src_S_bag = src_data['S']

    if I_graph.out_degree(i) <= 0:
        I_graph.remove_node(i)

    else:
        # just process one edge
        iter = I_graph.out_edges_iter(i, data=True)
        _, j, edge_data = iter.next()

        score = edge_data['score']

        dst_data = I_graph.node[j]
        dst_S_bag = dst_data['S']
        dst_T_bag = dst_data['T']

        for k in xrange(score):
            s = src_S_bag.pop(0)

            if len(dst_T_bag) > 0:
                t = dst_T_bag.pop(0)
                MATCH.append((s, t))
            else:
                dst_S_bag.append(s)

        I_graph.remove_edge(i, j)


def DISPLAY_STATE(I_graph, pos, active_node=None):
    # get an axis
    ax = plt.gca()

    # populate node labels, and...
    # initialize S and T queues on the interval graph
    interchanges = []
    active = []
    other = []

    interchange_labels = {}

    # draw nodes
    for i, data in I_graph.nodes(data=True):
        typei, labeli = i

        if i == active_node:
            active.append(i)

        elif typei == VERTEX:
            interchanges.append(i)
            interchange_labels[i] = labeli

        elif typei == POINT_IN_S:
            # data.update( S = [ labeli ], T = [] )
            other.append(i)

        elif typei == POINT_IN_T:
            # data.update( S = [], T = [ labeli ] )
            other.append(i)

        else:
            raise Exception('unrecognized node type')

    # nx.draw_networkx_nodes( I_graph, pos=pos, label=node_labels )
    def show_nodes(**kwargs):
        nx.draw_networkx_nodes(I_graph, pos=pos, ax=ax, **kwargs)

    show_nodes(nodelist=active, node_color='b', node_size=100, label=None)
    show_nodes(nodelist=interchanges, label=None)
    show_nodes(nodelist=other, node_color='k', node_size=50, label=None)

    S_bags = {}
    T_bags = {}
    for i, data in I_graph.nodes(data=True):
        temp = data['S']
        if len(temp) > 0: S_bags[i] = temp

        temp = data['T']
        if len(temp) > 0: T_bags[i] = temp

    offset = .01
    pos_labels = {i: (x, y + offset) for i, (x, y) in pos.items()}

    def show_labels(labels, **kwargs):
        nx.draw_networkx_labels(I_graph, pos=pos_labels, ax=ax, labels=labels, **kwargs)

    show_labels(S_bags, font_color='r')
    show_labels(T_bags, font_color='b')
    # nx.draw_networkx_labels( I_graph, pos=pos_labels, labels=SS_bags, font_color='r' )
    # nx.draw_networkx_labels( I_graph, pos=pos_labels, labels=TT_bags, font_color='b' )

    # draw edges
    score_map = {}
    for i, j, data in I_graph.edges(data=True):
        score = data['score']

        if score not in score_map: score_map[score] = []
        score_map[score].append((i, j))

    # print 'SCORE_MAP', score_map

    for score, edges in score_map.items():
        nx.draw_networkx_edges(I_graph, pos=pos, edgelist=edges, width=score,
                               label=None, ax=ax)

    # record the matching-so-far on a special node near the bottom!!


def texheader():
    return """
\\documentclass{article}
\\usepackage{tikz}

\\begin{document}
\\begin{tikzpicture}[x=\\linewidth,y=\\linewidth]
"""


def texfooter():
    return """
\\end{tikzpicture}
\\end{document}
"""


def DISPLAY_STATE_TIKZ(I_graph, pos, MATCH=None, active_node=None):
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
    for i, data in I_graph.nodes(data=True):
        typei, labeli = i

        # map nodes to coordinates
        node_indices[i] = k

        x, y = pos[i]
        opt.update(k=k, x=x, y=y)
        mystr += '\\coordinate (coord%(k)d) at (%(x).3f,%(y).3f) ;\n' % opt
        k += 1

        if i == active_node:
            active.append(i)

        elif typei == VERTEX:
            interchanges.append(i)
            interchange_labels[i] = labeli

        elif typei == POINT_IN_S:
            # data.update( S = [ labeli ], T = [] )
            other.append(i)

        elif typei == POINT_IN_T:
            # data.update( S = [], T = [ labeli ] )
            other.append(i)

        else:
            raise Exception('unrecognized node type')

    # draw interchanges
    data = dict(sz=.01)
    for i in interchanges:
        data.update(k=node_indices[i])
        mystr += '\\draw (coord%(k)d) circle (%(sz)f) ;\n' % data

    # draw active
    data = dict(sz=.005)
    for i in active:
        data.update(k=node_indices[i])
        mystr += '\\fill [blue] (coord%(k)d) circle (%(sz)f) ;\n' % data

    # draw other
    data = dict(sz=.002)
    for i in other:
        data.update(k=node_indices[i])
        mystr += '\\fill (coord%(k)d) circle (%(sz)f) ;\n' % data

    # draw bags
    S_bags = {}
    T_bags = {}
    for i, data in I_graph.nodes(data=True):
        temp = data['S']
        if len(temp) > 0: S_bags[i] = temp

        temp = data['T']
        if len(temp) > 0: T_bags[i] = temp

    fmt = '\\path (coord%(k)d) node [anchor=south,%(color)s] {\\footnotesize %(label)s} ;\n'
    data = dict(offset=.001)

    data.update(color='red')
    for i, bag in S_bags.items():
        data.update(k=node_indices[i], label=repr(bag))
        mystr += fmt % data

    data.update(color='blue')
    for i, bag in T_bags.items():
        data.update(k=node_indices[i], label=repr(bag))
        mystr += fmt % data

    # draw edges
    score_map = {}
    for i, j, data in I_graph.edges(data=True):
        score = data['score']

        if score not in score_map: score_map[score] = []
        score_map[score].append((i, j))

    fmt = '\\draw [->,line width=%(w)f] (coord%(k1)d) -- (coord%(k2)d) '
    fmt += 'node [midway,below] {\\tiny %(score)d};\n'
    data = {}
    for score, edges in score_map.items():
        data.update(w=.5 * score, score=score)

        for i, j in edges:
            data.update(k1=node_indices[i], k2=node_indices[j])
            mystr += fmt % data

    if MATCH is not None:
        # record the matching-so-far on a special node near the bottom!!
        match_fmt = '({\\color{red}%d},{\\color{blue}%d}), '
        match_strings = [match_fmt % (s, t) for s, t in MATCH]

        from functools import reduce

        cat = lambda s1, s2: s1 + s2
        match_tex = reduce(cat, ['['] + match_strings + [']'])

        mystr += '\\node at (.35,0) {%s} ;\n' % match_tex

    return mystr


class App:

    def main(self):

        """ make tikz animation """
        from pathlib import Path

        Path("slides").mkdir(exist_ok=True)

        def writeslide(k, mystr):
            with open('slides/slide%d.tex' % k, 'w') as f:
                f.write(mystr)

        slide_fmt = '\\only<%(k)d>{ \\input{slides/slide%(k)d.tex} }\n'

        with open('slides/construction_animation.tex', 'w') as f:
            for k, slide_tex in enumerate(self.iterate_slides(), 1):
                writeslide(k, slide_tex)
                f.write(slide_fmt % {'k': k})

    def iterate_slides(self):

        # example instance data
        interchanges = [
            (.14, .59), (.48, .6), (.4, .53), (.57, .43),
            # (.36,.27),
            (.37, .34),
            (.58, .23),
            (.11, .39), (.22, .15),
            (.12, .25),
        ]
        interchanges = [np.array(p) for p in interchanges]
        N = len(interchanges)

        from setiptah.roadgeometry.generation import DelaunayRoadMap

        roadmap = DelaunayRoadMap(interchanges)
        """ and build positions dictionary """
        pos = {k: p for k, p in enumerate(interchanges)}

        """ now, obtain two sets of points """
        # M = args.points
        M = 10

        if False:
            import setiptah.roadgeometry.probability as roadprob
            uniform = roadprob.UniformDist(roadmap)
            unpack = lambda addr: (addr.road, addr.coord)

            SS = [unpack(uniform.sample()) for i in xrange(M)]
            TT = [unpack(uniform.sample()) for i in xrange(M)]

        else:
            SS = [('road 0', .15),
                  ('road 1', .1), ('road 1', .15),
                  ('road 3', .05), ('road 3', .075),
                  ('road 8', .05),
                  ('road 10', .05),
                  ]

            TT = [('road 6', .1), ('road 6', .15),
                  ('road 8', .1),
                  ('road 10', .1), ('road 10', .15),
                  ('road 12', .1),
                  ('road 13', .1),
                  ]

        """ obtain the optimal matching """
        roadnet = MultiDiGraphRoadnet(roadmap)
        opt_match = RoadnetMatchingProblem(SS, TT, roadnet).compute_optimal(MatchingResult.MATCHING)

        """ obtain an interval graph from the matching """
        I_graph, I_pos = INTERVAL_GRAPH(opt_match, SS, TT, roadmap, pos)

        # Iterate.
        SANITIZE(I_graph)

        INITIALIZE_BAGS(I_graph)
        match = []

        order = nx.topological_sort(I_graph)
        edges = []
        for i in order:
            # edges.extend(I_graph.out_edges(i))
            edges.extend(I_graph.in_edges(i))  # Actually, this might be better.

        for i, j in edges:
            yield DISPLAY_STATE_TIKZ(I_graph, I_pos, match)
            UPDATE_EDGE(i, j, I_graph, match)
        yield DISPLAY_STATE_TIKZ(I_graph, I_pos, match)


if __name__ == "__main__":
    App().main()
