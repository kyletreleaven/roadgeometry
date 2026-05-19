"""Profile the roadnet matching algorithm.

Usage:
    # timing sweep, random scenario (default)
    nox -s bench -- bench/profile_matching.py

    # timing sweep, captured scenario
    nox -s bench -- bench/profile_matching.py --scenario captured

    # cProfile of a specific solver on the captured scenario
    nox -s bench -- bench/profile_matching.py --cprofile --solver cpp_matching --scenario captured

    # line-level profiling
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
    MinConvexCostFlow, py_dijkstra, CppMinConvexCostFlow, CppRobustMinConvexCostFlow,
)


# ---------------------------------------------------------------------------
# Solvers
# ---------------------------------------------------------------------------

def _run_rmp(flow_solver):
    def run(P, Q, roadnet):
        RoadnetMatchingProblem(P, Q, roadnet, flow_solver=flow_solver).compute_optimal_results(
            MatchingResult.MATCHING, MatchingResult.COST)
    return run


def _run_cpp_matching(solver="sparse"):
    from setiptah.roadgeometry.matching._cpp import compute_matching as _cpp_compute_matching
    def run(P, Q, roadnet):
        roads = list(roadnet.edges())
        endpoints = {r: roadnet.endpoints(r) for r in roads}
        lengths   = {r: roadnet.length(r)    for r in roads}
        is_oneway = {r: roadnet.is_oneway(r) for r in roads}
        _cpp_compute_matching(P, Q, endpoints, lengths, is_oneway, solver=solver)
    return run


SOLVERS = {
    "py":               _run_rmp(partial(MinConvexCostFlow, dijkstra=py_dijkstra)),
    "cpp_dijkstra":     _run_rmp(MinConvexCostFlow),
    "cpp_fragile":      _run_rmp(CppMinConvexCostFlow),
    "cpp_robust":       _run_rmp(CppRobustMinConvexCostFlow),
    "cpp_optimal_flow": _run_rmp(None),
    "cpp_matching":     _run_cpp_matching(solver="sparse"),
    "cpp_matching_dense": _run_cpp_matching(solver="dense"),
}


# ---------------------------------------------------------------------------
# Scenarios  — instance_fn(n) -> (P, Q, roadnet)
# ---------------------------------------------------------------------------

def make_grid_network(rows: int, cols: int) -> nx.MultiDiGraph:
    g = nx.MultiDiGraph()
    def node(r, c): return r * cols + c
    for r in range(rows):
        for c in range(cols):
            if c + 1 < cols:
                g.add_edge(node(r, c), node(r, c + 1), f"h_{r}_{c}", length=1.0)
            if r + 1 < rows:
                g.add_edge(node(r, c), node(r + 1, c), f"v_{r}_{c}", length=1.0)
    return g


def make_random_instance(n: int, rows: int = 10, cols: int = 10):
    roadgraph = make_grid_network(rows, cols)
    roadnet = MultiDiGraphRoadnet(roadgraph)
    sampler = roadprob.UniformDist(roadgraph)
    return sampler.sample(n), sampler.sample(n), roadnet


def load_captured_scenario():
    import pathlib
    _BACKEND = pathlib.Path(__file__).parent.parent / "webapp/backend"
    _PINS    = _BACKEND / "captured_pins.json"
    _GRAPHML = _BACKEND / "cambridge.graphml"

    if not _GRAPHML.exists():
        raise FileNotFoundError(f"cambridge.graphml not found at {_GRAPHML}")
    if not _PINS.exists():
        raise FileNotFoundError(f"captured_pins.json not found at {_PINS}")

    import osmnx as ox
    from setiptah.roadgeometry.geopandas import GeoFramesNetwork
    from setiptah.roadgeometry.matching.io import load_pins

    G = ox.load_graphml(_GRAPHML)
    nodes, edges = ox.graph_to_gdfs(G)
    edges['u'] = edges.index.get_level_values('u')
    edges['v'] = edges.index.get_level_values('v')
    roadnet = GeoFramesNetwork(edges_gdf=edges, nodes_gdf=nodes,
                               left_col='u', right_col='v', oneway_col='oneway')
    P, Q = load_pins(_PINS)
    return P, Q, roadnet


def get_scenario(args):
    """Return (instance_fn, sizes) for the chosen scenario."""
    if args.scenario == "captured":
        print("Loading captured scenario...")
        P, Q, roadnet = load_captured_scenario()
        n = max(len(P), len(Q))
        print(f"  {len(P)} supply, {len(Q)} demand pins, {len(list(roadnet.edges()))} roads")
        return lambda _: (P, Q, roadnet), [n]
    else:
        return make_random_instance, args.sizes


# ---------------------------------------------------------------------------
# Profiling modes
# ---------------------------------------------------------------------------

def profile_timing(instance_fn, sizes, repeats, solvers):
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
            P, Q, roadnet = instance_fn(max(n // 4, 1))
            solver(P, Q, roadnet)  # warm-up
            times = []
            for _ in range(repeats):
                P, Q, roadnet = instance_fn(n)
                t0 = time.perf_counter()
                solver(P, Q, roadnet)
                times.append(time.perf_counter() - t0)
            row += f"  {np.mean(times):>{col_w}.4f}"
        print(row)


def profile_cprofile(instance_fn, n, solvers, top_n=30):
    for solver_name in solvers:
        solver = SOLVERS[solver_name]
        print(f"\n{'='*60}")
        print(f"cProfile: solver={solver_name}, n={n}")
        print(f"{'='*60}")

        P, Q, roadnet = instance_fn(max(n // 4, 1))
        solver(P, Q, roadnet)  # warm-up

        P, Q, roadnet = instance_fn(n)
        pr = cProfile.Profile()
        pr.enable()
        solver(P, Q, roadnet)
        pr.disable()

        buf = io.StringIO()
        ps = pstats.Stats(pr, stream=buf)
        ps.sort_stats("cumulative")
        ps.print_stats(top_n)
        print(buf.getvalue())


def profile_line(instance_fn, n, solver_name):
    try:
        from line_profiler import LineProfiler
    except ImportError:
        print("line_profiler not installed")
        return

    from setiptah.roadgeometry.matching.nxopt.cvxcostflow import (
        MinConvexCostFlow, FragileMCCF, cpp_fragile_mccf,
        CppRobustMinConvexCostFlow, _normalize_for_cpp,
    )
    from setiptah.roadgeometry.matching.bm import (
        RoadnetMatchingProblem, compute_segments2, default_compute_segments,
        compute_optimal_flow as bm_compute_optimal_flow,
    )

    solver = SOLVERS[solver_name]
    P, Q, roadnet = instance_fn(n)

    lp = LineProfiler()
    lp.add_function(MinConvexCostFlow)
    lp.add_function(FragileMCCF)
    lp.add_function(compute_segments2)
    lp.add_function(default_compute_segments)
    lp.add_function(bm_compute_optimal_flow)
    if solver_name == "cpp_fragile":
        lp.add_function(cpp_fragile_mccf)
    if solver_name == "cpp_robust":
        lp.add_function(CppRobustMinConvexCostFlow)
        lp.add_function(_normalize_for_cpp)

    @lp
    def wrapped():
        prob = RoadnetMatchingProblem(P, Q, roadnet, flow_solver=solver)
        prob.compute_optimal_results(MatchingResult.MATCHING, MatchingResult.COST)

    print(f"\n{'='*60}")
    print(f"line_profiler: solver={solver_name}, n={n}")
    print(f"{'='*60}")
    wrapped()
    lp.print_stats()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=["random", "captured"], default="random")
    parser.add_argument("--cprofile", action="store_true")
    parser.add_argument("--line-profile", action="store_true")
    parser.add_argument("--solver", choices=list(SOLVERS), default="cpp_matching",
                        help="Solver for cprofile/line-profile modes (default: cpp_matching)")
    parser.add_argument("--n", type=int, default=100,
                        help="Points per side for random scenario cprofile/line-profile (default: 100)")
    parser.add_argument("--sizes", type=int, nargs="+", default=[10, 25, 50, 100, 200],
                        help="Point counts for random timing sweep (default: 10 25 50 100 200)")
    parser.add_argument("--repeats", type=int, default=None,
                        help="Repeats per solver (default: 1 for captured, 3 for random)")
    parser.add_argument("--solvers", nargs="+", choices=list(SOLVERS), default=list(SOLVERS),
                        help="Solvers to include in timing sweep (default: all)")
    args = parser.parse_args()

    if args.repeats is None:
        args.repeats = 1 if args.scenario == "captured" else 3

    instance_fn, sizes = get_scenario(args)

    if args.cprofile:
        profile_cprofile(instance_fn, args.n, args.solvers)
    elif args.line_profile:
        profile_line(instance_fn, args.n, args.solver)
    else:
        profile_timing(instance_fn, sizes, args.repeats, args.solvers)
