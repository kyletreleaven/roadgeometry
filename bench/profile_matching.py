"""Profile the roadnet matching algorithm at various input sizes.

Usage:
    # timing sweep across all solvers (default)
    nox -s bench -- bench/profile_matching.py

    # cProfile of a specific solver
    nox -s bench -- bench/profile_matching.py --cprofile --solver cpp_fragile

    # line-level profiling (requires line_profiler)
    nox -s bench -- bench/profile_matching.py --line-profile --solver cpp_fragile
"""

import argparse
import cProfile
import pstats
import io
import time
from functools import partial

import networkx as nx
import numpy as np

import setiptah.roadgeometry.probability as roadprob
from setiptah.roadgeometry.matching.nx_legacy import (
    MultiDiGraphRoadnet,
    RoadnetMatchingProblem,
    MatchingResult,
)
from setiptah.roadgeometry.matching.nxopt.cvxcostflow import (
    MinConvexCostFlow, py_dijkstra, CppMinConvexCostFlow,
)

SOLVERS = {
    "py":           partial(MinConvexCostFlow, dijkstra=py_dijkstra),
    "cpp_dijkstra": None,   # default: C++ Dijkstra inside Python FragileMCCF
    "cpp_fragile":  CppMinConvexCostFlow,
}


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
    P = sampler.sample(n)
    Q = sampler.sample(n)
    return P, Q, roadnet


def run_once(n: int, flow_solver):
    P, Q, roadnet = make_instance(n)
    prob = RoadnetMatchingProblem(P, Q, roadnet, flow_solver=flow_solver)
    prob.compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)


def profile_cprofile(n: int, solver_name: str, top_n: int = 30):
    solver = SOLVERS[solver_name]
    print(f"\n{'='*60}")
    print(f"cProfile: solver={solver_name}, n={n}")
    print(f"{'='*60}")

    run_once(max(n // 4, 1), solver)  # warm-up

    pr = cProfile.Profile()
    pr.enable()
    run_once(n, solver)
    pr.disable()

    buf = io.StringIO()
    ps = pstats.Stats(pr, stream=buf)
    ps.sort_stats("cumulative")
    ps.print_stats(top_n)
    print(buf.getvalue())


def profile_timing(sizes: list[int], repeats: int, solvers: list[str]):
    col_w = 12
    header = f"{'n':>6}  " + "  ".join(f"{s:>{col_w}}" for s in solvers)
    print(f"\n{'='*len(header)}")
    print(f"Timing: {repeats} repeats each")
    print(f"{'='*len(header)}")
    print(header)
    print("-" * len(header))

    for n in sizes:
        row = f"{n:>6}  "
        for solver_name in solvers:
            solver = SOLVERS[solver_name]
            run_once(max(n // 4, 1), solver)  # warm-up
            times = []
            for _ in range(repeats):
                t0 = time.perf_counter()
                run_once(n, solver)
                times.append(time.perf_counter() - t0)
            row += f"  {np.mean(times):>{col_w}.4f}"
        print(row)


def profile_line(n: int, solver_name: str):
    try:
        from line_profiler import LineProfiler
    except ImportError:
        print("line_profiler not installed")
        return

    from setiptah.roadgeometry.matching.nxopt.cvxcostflow import (
        MinConvexCostFlow, FragileMCCF, cpp_fragile_mccf,
    )

    solver = SOLVERS[solver_name]
    P, Q, roadnet = make_instance(n)

    lp = LineProfiler()
    lp.add_function(MinConvexCostFlow)
    lp.add_function(FragileMCCF)
    if solver_name == "cpp_fragile":
        lp.add_function(cpp_fragile_mccf)

    @lp
    def wrapped():
        prob = RoadnetMatchingProblem(P, Q, roadnet, flow_solver=solver)
        prob.compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)

    print(f"\n{'='*60}")
    print(f"line_profiler: solver={solver_name}, n={n}")
    print(f"{'='*60}")
    wrapped()
    lp.print_stats()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cprofile", action="store_true")
    parser.add_argument("--line-profile", action="store_true")
    parser.add_argument("--solver", choices=list(SOLVERS), default="cpp_fragile",
                        help="Solver for cprofile/line-profile modes (default: cpp_fragile)")
    parser.add_argument("--n", type=int, default=100,
                        help="Number of points per side for cprofile/line-profile (default: 100)")
    parser.add_argument("--sizes", type=int, nargs="+", default=[10, 25, 50, 100, 200],
                        help="Point counts for timing sweep (default: 10 25 50 100 200)")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--solvers", nargs="+", choices=list(SOLVERS), default=list(SOLVERS),
                        help="Solvers to include in timing sweep (default: all)")
    args = parser.parse_args()

    if args.cprofile:
        profile_cprofile(args.n, args.solver)
    elif args.line_profile:
        profile_line(args.n, args.solver)
    else:
        profile_timing(args.sizes, args.repeats, args.solvers)
