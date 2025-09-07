"""

Notes on the Roadmap format:
These utilities expect a NetworkX MultiDiGraph,
the edge keys are interpreted as "road names",
and they should all be unique, even among edges between different nodes;
(also, they should be distinct from the node labels).
All roads are two-way by default (the direction of edges is for the coordinate system only);
However, one-way roads are handled, and should be indicated with a key:value pair 'oneway':True .

"""
import logging
from dataclasses import dataclass
from typing import Any

import networkx as nx
import numpy as np

from .roadmap_basic import RoadAddress, check_point

LOG = logging.getLogger(__name__)

__author__ = "Kyle Treleaven <ktreleav@gmail.com>"


@dataclass(frozen=True, repr=False)
class RoadSegment(object):
    road: Any
    first: float
    second: float

    def __repr__(self) :
        return '(%s,[%s,%s])' % ( repr(self.road), repr(self.first), repr(self.second) )


def check_segment(
        segment: RoadSegment, roadmap: nx.MultiDiGraph, length_attr: str ='length'
):
    try :
        assert check_point(roadmap, RoadAddress(segment.road, segment.first), length_attr)
        assert check_point(roadmap, RoadAddress(segment.road, segment.second), length_attr)
    except AssertionError :
        return False
    return True


def pathReversed( path ) :
    res = []
    for segment in path :
        res.insert(0, RoadSegment( segment.road, segment.second, segment.first ) )
    return res


def pathLength( path ) :
    if len( path ) <= 0 :
        return np.inf
    else :
        segLengths = [ np.abs( seg.first - seg.second ) for seg in path ]
        return np.sum( segLengths )


def pathEvaluate( path, x ) :
    if x < 0. : raise Exception('cannot interpret negative distance')
    segLengths = [ np.abs( seg.first - seg.second ) for seg in path ]
    A = 0.
    for i, y in enumerate( segLengths ) :
        B = A + y
        if x <= B : break
        A = B
    if x > B :
        LOG.debug(x, B)
        raise Exception('query distance longer than path')      # only happens when B == path length
    
    segment = path[i]
    dx = x - A
    if segment.first < segment.second :
        return RoadAddress( segment.road, segment.first + dx )
    else :
        return RoadAddress( segment.road, segment.first - dx )


def minpath(
        p: RoadAddress, q: RoadAddress, roadmap: nx.MultiDiGraph, length_attr: str = "length"
):
    """ returns one of the min length paths between two points on a Roadmap """

    from setiptah.roadbm.nx_legacy import MultiDiGraphRoadnet
    from setiptah.basic_graph.dijkstra import RoadnetMetric
    metric = RoadnetMetric(MultiDiGraphRoadnet(roadmap, length_attr=length_attr))

    p_ = p.road, p.coord
    q_ = q.road, q.coord

    path_ = metric.shortest_path(p_, q_)
    return [RoadSegment(seg.road, seg.start, seg.end) for seg in path_]


def pathExtend( path, nextAddress, roadmap, length_attr='length' ) :
    segment = path[-1]
    road = segment.road
    q = RoadAddress(road,segment.second)
    second = minpath( q, nextAddress, roadmap, length_attr )
    # TODO: combine contiguous, co-directional segments;
    # TODO: also, zero-measure segments
    return path + second


class RoadTrajectory :
    def __init__(self, path ) :
        self.path = path
        
    def __call__(self, progress ) :
        return pathEvaluate( self.path, progress )
