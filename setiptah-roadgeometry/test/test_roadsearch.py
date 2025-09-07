from setiptah.roadgeometry.dijkstra import RoadnetMetric
from setiptah.roadgeometry.matching.nx_legacy import MultiDiGraphRoadnet
from setiptah.roadgeometry.roadsearch import *
import setiptah.roadgeometry.probability as roadprob


def test_nearest_neighbor_search():
    roadnet = roadprob.sampleroadnet()

    n = 100
    points = [roadprob.sampleaddress(roadnet) for i in range(n)]

    pset = PointSet()

    for p in points:
        pset.insert(p)

    metric = RoadnetMetric(MultiDiGraphRoadnet(roadnet, length_attr="length"))

    def find_nearest(addr):
        dist_to = lambda q: metric.distance(addr, q)
        trips = [(dist_to(q), q) for q in points]
        return min(trips)[1]

    def sidebyside(addr):
        by_pset = pset.find_nearest(addr, roadnet)
        by_naive = find_nearest(addr)
        return by_pset, by_naive

    samples = 50
    testpoints = [roadprob.sampleaddress(roadnet) for i in range(samples)]
    answers = [sidebyside(q) for q in testpoints]


    error = [metric.distance(p, q) for p, q in answers]

    assert sum(error) == 0
