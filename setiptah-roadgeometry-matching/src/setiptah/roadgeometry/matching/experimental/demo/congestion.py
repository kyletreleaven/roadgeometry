import networkx as nx
import matplotlib.pyplot as plt

from setiptah.roadgeometry.matching.experimental.congestion import *
from setiptah.roadgeometry.matching.nx_legacy import *

from setiptah.roadgeometry.legacy import roadmap_basic as ROAD


def congestion_demo():
    """ parameters """
    SAMPLE_RANDOMLY = True

    if False:
        # circle
        roadmap = nx.MultiDiGraph()
        roadmap.add_edge(0, 0, 'A', length=1.)


    elif False:
        # Delaunay roadmap

        if False:
            # random
            verts = [10 * np.random.rand(2) for i in xrange(5)]
        else:
            # ring
            polar = lambda t: np.array([np.cos(t), np.sin(t)])
            theta = np.linspace(0, 2 * np.pi, 7 + 1)[:-1]
            verts = [5. * polar(t) for t in theta]

        import setiptah.roadgeometry.generation as roadgen

        roadmap = roadgen.DelaunayRoadMap(verts)

        pos = {k: p for k, p in enumerate(verts)}

    elif True:
        stage_width = 1.
        via_buffer = 1.
        traverse = 10.
        band_height = .3

        NUMPOINTS = 10
        BANDWIDTH = 20

        # construct instance
        roadmap = nx.MultiDiGraph()

        # establish notable x positions
        X = np.cumsum([0, stage_width, via_buffer, traverse, via_buffer, stage_width])

        # draw staging area
        roadmap.add_edge(0, 1, 'start', length=1.)
        roadmap.add_edge(4, 5, 'end', length=1.)
        # ...and position the nodes
        pos = {}
        for k in [0, 1, 4, 5]: pos[k] = np.array((X[k], 0))

        # place a number of bands
        for k in range(BANDWIDTH):
            for k in range(BANDWIDTH):
                u, v = [fmt % k for fmt in ['L%d', 'R%d']]
                pos[u] = np.array((X[2], -k * band_height))
                pos[v] = np.array((X[3], -k * band_height))

                r1, r2, r3 = [fmt % k for fmt in ['LCONN%d', 'BAND%d', 'RCONN%d']]
                roadmap.add_edge(1, u, r1)
                roadmap.add_edge(u, v, r2)
                roadmap.add_edge(v, 4, r3)

        # give length annotations
        for u, v, road, data in roadmap.edges(keys=True, data=True):
            # p = np.array( pos[u] )
            # q = np.array( pos[v] )
            data['length'] = np.linalg.norm(pos[v] - pos[u])

        # place the points
        Y = np.linspace(0, 1, NUMPOINTS + 2)[1:-1]
        S = [ROAD.RoadAddress('start', y) for y in Y]
        T = [ROAD.RoadAddress('end', y) for y in Y]
        SAMPLE_RANDOMLY = False

    """ congestion function """
    # rho = lambda x : np.power( x, 2. )          # square law, why not!
    p = .07
    rho = lambda x: np.power(abs(x), p)  # linear congestion?

    # separate instance?

    if SAMPLE_RANDOMLY:
        NUMPOINTS = 10

        import setiptah.roadgeometry.probability as roadprob

        sampler = roadprob.UniformDist(roadmap)

        S = [sampler.sample() for i in range(NUMPOINTS)]
        T = [sampler.sample() for i in range(NUMPOINTS)]

    # algorithm
    # stolen form ROADSBIPARTITEMATCH

    rho_dict = {road: rho for i, j, road in roadmap.edges(keys=True)}

    # assert False, (S, T)
    # assert False, roadmap.edges
    # assert False, roadmap.nodes
    assist_nocongestion = RoadnetMatchingProblem(
        S, T, MultiDiGraphRoadnet(roadmap)
    ).compute_optimal(MatchingResult.FLOW)

    assist = BIPARTITEMATCH_ROADS_CONGESTED(S, T, roadmap, rho_dict)

    import setiptah.roadgeometry.matching.draw.matchvis as matchvis
    roadnet = MultiDiGraphRoadnet(roadmap)

    plt.figure()
    plt.title('Congestion Optimal')
    matchvis.show_flow(assist, S, T, roadnet, pos)

    plt.figure()
    plt.title('Pure Path-length Optimal')
    matchvis.show_flow(assist_nocongestion, S, T, roadnet, pos)

    if False:
        segment_dict = roadbm.SEGMENTS(S, T, roadmap)
        measure_dict = {}

        for road, segment in segment_dict.items():
            match = roadbm.PREMATCH(segment)
            MATCH.extend(match)

            # surplus_dict[road] = SURPLUS( segment )

            roadlen = ROAD.get_road_data(road, roadmap).get('length', 1)
            measure = roadbm.MEASURE(segment, roadlen)
            measure_dict[road] = measure
            # objective_dict[road] = OBJECTIVE( measure )
            # objective_dict[road] = objective
        """ measure_dict now contains sequence of W_n """

        C_man = CONGESTION_OBJECTIVE_DATA(measure, rho, False)
        C_fft = CONGESTION_OBJECTIVE_DATA(measure, rho, True)

        C = CONGESTION_OBJECTIVE(measure, rho)


if __name__ == "__main__":
    congestion_demo()
    plt.show()
