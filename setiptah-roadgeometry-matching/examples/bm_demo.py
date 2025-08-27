import itertools

from setiptah.roadbm.bm import *

import setiptah.roadgeometry.probability as roadprob

import setiptah.roadgeometry.astar_basic as ASTAR

import networkx as nx


def drawCBounds(ZZ, Ctree, ax=None):
    if ax is None:
        plt.figure()
        ax = plt.gca()

    cost = costWrapper(Ctree)
    Cref = np.array([cost(z) for z in ZZ])
    ax.plot(ZZ, Cref, c='b', linewidth=3, alpha=.25)
    for f, (kappa, alpha) in Ctree.iter_items():
        C = kappa + alpha * ZZ
        ax.plot(ZZ, C, c='k', linestyle='--')
    # ax.set_aspect( 'equal' )
    return ax


VISUAL = False
if VISUAL:
    import matplotlib.pyplot as plt

    plt.close('all')

roadnet = nx.MultiDiGraph()
if True:
    roadnet.add_edge(0, 1, 'N', length=1.)
else:
    # to test one-way roads capabilities
    roadnet.add_edge(0, 1, 'N', length=1., oneway=True)
roadnet.add_edge(1, 2, 'E', length=1.)
roadnet.add_edge(2, 3, 'S', length=1.)
roadnet.add_edge(3, 0, 'W', length=1.)

if True:
    roadnet.add_edge(0, 4, 'dangler', length=1.)

sampler = roadprob.UniformDist(roadnet)

NUMPOINT = 50
ZZ = np.linspace(-NUMPOINT, NUMPOINT, 1000)
#
PP = [sampler.sample() for i in range(NUMPOINT)]
QQ = [sampler.sample() for i in range(NUMPOINT)]

z = np.arange(-NUMPOINT, NUMPOINT, .25)
objective_dict = WRITEOBJECTIVES(PP, QQ, roadnet)

if VISUAL:
    for road, Cz in objective_dict.items():
        cost = costWrapper(Cz)
        C = [cost(zz) for zz in z]
        plt.figure()
        plt.plot(z, C, '--', marker='x')

match = ROADSBIPARTITEMATCH(PP, QQ, roadnet)
costs = MATCHCOSTS(match, PP, QQ, roadnet)
cost = ROADMATCHCOST(match, PP, QQ, roadnet)
print(match)
print(costs)
print(cost)

# compare to optimal matching
if True and NUMPOINT <= 50:
    PR = [ROAD.RoadAddress(road, y) for road, y in PP]
    QR = [ROAD.RoadAddress(road, y) for road, y in QQ]


    class pointnode():
        def __init__(self, point, idx):
            self.point = point
            self.idx = idx


    RED = [pointnode(p, i) for i, p in enumerate(PR)]
    BLU = [pointnode(q, j) for j, q in enumerate(QR)]
    graph = nx.Graph()
    match_mat = np.zeros((NUMPOINT, NUMPOINT))
    for (i, r), (j, b) in itertools.product(enumerate(RED), enumerate(BLU)):
        w = ROAD.distance(roadnet, r.point, b.point, 'length')
        graph.add_edge(r, b, weight=-w)
        match_mat[i, j] = w

    match_brute = [
        (r.idx, b.idx)
        for r, b in nx.max_weight_matching(graph, True)
    ]
    # match_brute = [(r.idx, match_dict[r].idx) for r in RED]  # match pruning
    #        matchstats = [ ( r.point, b.point, ROAD.distance( roadnet, r.point, b.point, 'length' ) )
    #                      for r,b in match ]
    costs_brute = MATCHCOSTS(match_brute, PP, QQ, roadnet)
    cost_brute = ROADMATCHCOST(match_brute, PP, QQ, roadnet)
    print(match_brute)
    print(costs_brute)
    print(cost_brute)
    # print 'optimal match has cost: %f' % matchcost

if False:  # validate CTREES
    zmin = -NUMPOINT / 2
    zmax = NUMPOINT / 2
    ZZZ = range(zmin, zmax)
    #
    for road, C in CTREES.items():
        ax = drawCBounds(ZZ, CTREES[road])
        # plt.plot( ZZ, Cz, linestyle='--' )

        width = get_road_data(road, roadnet).get('length', 1)
        Cmatch = [MATCHCOST(P[road], Q[road], width, z) for z in ZZZ]
        plt.scatter(ZZZ, Cmatch, marker='x')

if False:
    # validate C's of the different roads
    for road in CTREES:
        ax = drawCBounds(ZZ, CTREES[road])
        ax.set_title('road=%s' % road)
        zr = assist[road].value
        cost = costWrapper(CTREES[road])
        Cr = cost(zr)
        # ax.axvline( assist[road].value )
        ax.scatter([zr], [Cr], marker='o')
        ax.scatter([zr], [cost[road].value], marker='x')

    dC = dict()
    for road in assist:
        zr = assist[road].value
        f, (kappa, alpha) = CTREES[road].floor_item(-zr)
        dC[road] = alpha

    # compute a matching and verify cost
    matchZ = dict()
    for road, var in assist.items():
        matchZ[road] = int(round(var.value))
    the_match = ROADMATCH(PP, QQ, matchZ, roadnet)
    the_match_cost = ROADMATCHCOST(the_match, roadnet)
    print(f'constructed matching has cost: {the_match_cost}')
