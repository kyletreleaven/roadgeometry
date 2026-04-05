"""Profile the roadnet matching algorithm at various input sizes.

Usage:
    # cProfile overview (default)
    nox -s bench -- bench/profile_matching.py

    # line-level profiling of hot functions (requires line_profiler)
    nox -s bench -- bench/profile_matching.py --line-profile
"""

import argparse
import cProfile
import pstats
import io
import time

import networkx as nx
import numpy as np

import setiptah.roadgeometry.probability as roadprob
from setiptah.roadgeometry.formats import from_networkx
from setiptah.roadgeometry.matching.nx_legacy import (
    MultiDiGraphRoadnet,
    RoadnetMatchingProblem,
    MatchingResult,
)


def make_grid_network(rows: int, cols: int) -> nx.MultiDiGraph:
    """Simple grid road network with unit-length edges."""
    g = nx.MultiDiGraph()
    def node(r, c): return r * cols + c

    for r in range(rows):
        for c in range(cols):
            if c + 1 < cols:
                key = f"h_{r}_{c}"
                g.add_edge(node(r, c), node(r, c + 1), key, length=1.0)
            if r + 1 < rows:
                key = f"v_{r}_{c}"
                g.add_edge(node(r, c), node(r + 1, c), key, length=1.0)

    return g


def make_instance(n: int, rows: int = 10, cols: int = 10):
    """Build a random matching instance with n points per side."""
    roadgraph = make_grid_network(rows, cols)
    roadnet = MultiDiGraphRoadnet(roadgraph)
    sampler = roadprob.UniformDist(roadgraph)
    P = [sampler.sample() for _ in range(n)]
    Q = [sampler.sample() for _ in range(n)]
    return P, Q, roadnet


def run_once(n: int):
    P, Q, roadnet = make_instance(n)
    prob = RoadnetMatchingProblem(P, Q, roadnet)
    prob.compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)


def profile_cprofile(n: int, top_n: int = 30):
    print(f"\n{'='*60}")
    print(f"cProfile: n={n} points per side")
    print(f"{'='*60}")

    # warm up
    run_once(min(n, 20))

    pr = cProfile.Profile()
    pr.enable()
    run_once(n)
    pr.disable()

    buf = io.StringIO()
    ps = pstats.Stats(pr, stream=buf)
    ps.sort_stats("cumulative")
    ps.print_stats(top_n)
    print(buf.getvalue())


def profile_timing(sizes: list[int], repeats: int = 3):
    print(f"\n{'='*60}")
    print(f"Timing across input sizes")
    print(f"{'='*60}")
    print(f"{'n':>6}  {'mean (s)':>10}  {'min (s)':>10}")
    print(f"{'-'*30}")

    for n in sizes:
        times = []
        for _ in range(repeats):
            P, Q, roadnet = make_instance(n)
            prob = RoadnetMatchingProblem(P, Q, roadnet)
            t0 = time.perf_counter()
            prob.compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)
            times.append(time.perf_counter() - t0)
        print(f"{n:>6}  {np.mean(times):>10.4f}  {np.min(times):>10.4f}")


def profile_line(n: int):
    try:
        from line_profiler import LineProfiler
    except ImportError:
        print("line_profiler not installed. Run: pip install line_profiler")
        return

    from setiptah.roadgeometry.matching.bm import (
        compute_optimal_flow,
        compute_segments2,
        TRAVERSE2,
        TRAVERSE3,
        MEASURE,
        PREMATCH,
    )
    from setiptah.roadgeometry.matching.nxopt.cvxcostflow import MinConvexCostFlow

    P, Q, roadnet = make_instance(n)

    lp = LineProfiler()
    lp.add_function(compute_optimal_flow)
    lp.add_function(compute_segments2)
    lp.add_function(TRAVERSE2)
    lp.add_function(TRAVERSE3)
    lp.add_function(MEASURE)
    lp.add_function(PREMATCH)
    lp.add_function(MinConvexCostFlow)

    @lp
    def wrapped():
        prob = RoadnetMatchingProblem(P, Q, roadnet)
        prob.compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)

    print(f"\n{'='*60}")
    print(f"line_profiler: n={n} points per side")
    print(f"{'='*60}")
    wrapped()
    lp.print_stats()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--line-profile", action="store_true",
                        help="Run line_profiler on hot functions (requires line_profiler)")
    parser.add_argument("--n", type=int, default=200,
                        help="Number of points per side for detailed profiling (default: 200)")
    parser.add_argument("--sizes", type=int, nargs="+", default=[50, 100, 200, 500, 1000],
                        help="Input sizes for timing sweep (default: 50 100 200 500 1000)")
    args = parser.parse_args()

    profile_timing(args.sizes)
    profile_cprofile(args.n)

    if args.line_profile:
        profile_line(args.n)
