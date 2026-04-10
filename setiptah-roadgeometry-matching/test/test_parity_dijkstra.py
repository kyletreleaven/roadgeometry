"""Parity tests: C++ and Python Dijkstra implementations must agree."""
from functools import partial

import numpy as np
import pytest

from setiptah.roadgeometry.generation import DelaunayRoadnet
from setiptah.roadgeometry.probability import RoadnetUniformDist
from setiptah.roadgeometry.matching.bm import RoadnetMatchingProblem, MatchingResult
from setiptah.roadgeometry.matching.nxopt.cvxcostflow import (
    MinConvexCostFlow,
    py_dijkstra,
    _default_dijkstra,
)

py_flow_solver = partial(MinConvexCostFlow, dijkstra=py_dijkstra)


def random_matching_problem(rng, n_intersections=12, n_points=20):
    points = rng.random((n_intersections, 2))
    roadnet = DelaunayRoadnet(points)
    sampler = RoadnetUniformDist(roadnet)
    PP = [sampler.sample() for _ in range(n_points)]
    QQ = [sampler.sample() for _ in range(n_points)]
    return PP, QQ, roadnet


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4])
def test_parity(seed):
    rng = np.random.default_rng(seed)
    PP, QQ, roadnet = random_matching_problem(rng)

    cost_cpp = RoadnetMatchingProblem(PP, QQ, roadnet).compute_optimal(MatchingResult.COST)
    cost_py  = RoadnetMatchingProblem(PP, QQ, roadnet, flow_solver=py_flow_solver).compute_optimal(MatchingResult.COST)

    assert abs(cost_cpp - cost_py) < 1e-9, f"cpp={cost_cpp}, py={cost_py}"
