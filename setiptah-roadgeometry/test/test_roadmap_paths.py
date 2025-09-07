import networkx as nx

from setiptah.roadgeometry.legacy import roadmap_basic as ROAD
from setiptah.roadgeometry.legacy.roadmap_basic import RoadAddress
from setiptah.roadgeometry.legacy.roadmap_paths import *

LOG = logging.getLogger(__name__)


def test_minpath():

    g = nx.MultiDiGraph()

    g.add_edge(0, 1, "R", length=10.)

    p = RoadAddress("R", 1)
    q = RoadAddress("R", 9)

    assert minpath(p, q, g) == [RoadSegment("R", 1, 9)]


def test_pathlen():
    import setiptah.roadgeometry.probability as roadprob

    roadmap = roadprob.sampleroadnet()

    p = roadprob.sampleaddress(roadmap)
    q = roadprob.sampleaddress(roadmap)

    frwd = minpath(p, q, roadmap)
    frwdL = pathLength(frwd)
    frwdLRef = ROAD.distance(roadmap, p, q, 'length')

    if np.abs(frwdL - frwdLRef) < 10 ** -10:
        pass

    else:

        with open("pathlen-error.txt", "w") as f:

            def writeline(line):
                f.write(f"{line}\n")

            # writeline("from setiptah.basic_graph.graphs import RoadNetwork")
            # writeline("from setiptah.basic_graph.dijkstra import *")
            writeline("import networkx as nx")
            writeline("from setiptah.roadgeometry.roadmap_basic import *")
            writeline("from setiptah.roadgeometry.roadmap_paths import *")
            # writeline("rn = RoadNetwork()")
            writeline("rn = nx.MultiDiGraph()")
            for u in roadmap.nodes():
                writeline(f"rn.add_node({u})")

            for u, v, road, data in roadmap.edges(keys=True, data=True):
                l = data.get("length", 1)
                # writeline(f'rn.add_edge("{road}", {u}, {v}, {l})')
                writeline(f'rn.add_edge({u}, {v}, "{road}", length={l})')

                if False:
                    writeline(
                        '%s: %s -> %s ; length=%f' % (road, repr(u), repr(v), data.get('length', 1))
                    )

            writeline(f'p = RoadAddress("{p.road}", {p.coord})')
            writeline(f'q = RoadAddress("{q.road}", {q.coord})')

        assert False


def test_canned():

    from setiptah.roadgeometry.legacy.roadmap_paths import minpath

    rn = nx.MultiDiGraph()
    rn.add_node(0)
    rn.add_node(2)
    rn.add_node(1)
    rn.add_node(3)
    rn.add_node(5)
    rn.add_node(6)
    rn.add_node(7)
    rn.add_node(8)
    rn.add_node(4)
    rn.add_node(9)
    rn.add_edge(0, 2, "road0", length=0.08690718896367763)
    rn.add_edge(2, 7, "road6", length=1.0090526344825517)
    rn.add_edge(1, 3, "road1", length=0.3581567968145343)
    rn.add_edge(1, 5, "road2", length=3.0405091619281603)
    rn.add_edge(1, 6, "road3", length=0.5038404955090379)
    rn.add_edge(1, 7, "road4", length=1.9846761312373984)
    rn.add_edge(1, 8, "road5", length=0.16952917209609902)
    rn.add_edge(3, 4, "road7", length=0.30725919265278534)
    rn.add_edge(3, 5, "road8", length=1.1650558623014722)
    rn.add_edge(5, 6, "road9", length=0.2031866604566163)
    rn.add_edge(5, 7, "road10", length=1.3770561219375455)
    rn.add_edge(5, 8, "road11", length=0.1971187509079228)
    rn.add_edge(7, 8, "road12", length=0.1948755750344511)
    rn.add_edge(7, 9, "road13", length=0.6278788902822177)

    p = RoadAddress("road7", 0.16554418185170827)
    q = RoadAddress("road4", 0.6748362063998169)

    frwd = minpath(p, q, rn)
    frwdL = pathLength(frwd)
    frwdLRef = ROAD.distance(rn, p, q, 'length')

    assert np.abs(frwdL - frwdLRef) < 10 ** -10

    from setiptah.roadbm.nx_legacy import MultiDiGraphRoadnet
    from setiptah.basic_graph.dijkstra import RoadnetMetric
    metric = RoadnetMetric(MultiDiGraphRoadnet(rn))
    _, (u, v) = metric._shortest_path(p, q)._result

    from setiptah.roadgeometry.legacy.astar_basic import astar_path_length
    astar_path_length(rn, u, v, None, weight="length")
