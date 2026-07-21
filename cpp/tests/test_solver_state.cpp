#include "third_party/doctest/doctest.h"

#include <functional>
#include <unordered_map>

#include "roadgeometry/input_graph.hpp"

// Target headers — do not exist yet; building them is the work:
#include "roadgeometry/instance.hpp"       // FlowInstance concept + map_backed_instance
#include "roadgeometry/solver_state.hpp"   // SolverState, CertificateView
#include "roadgeometry/solver.hpp"         // solve(FlowInstance) -> SolverState
#include "roadgeometry/certificate.hpp"    // certifies_optimality(instance, CertificateView)

// ===========================================================================
// NORTH-STAR ACCEPTANCE TEST — drive this to green incrementally.
//
// Concepts
//   EdgeMap<M,E>    map-like (find/end) over edges; used for ub and lb (symmetric bounds).
//   FlowInstance<I> problem data via accessors: network() (InputGraph), cost(e), ub(e),
//                   lb(e), supply(n). map_backed_instance(...) adapts plain maps so simple
//                   callers stay one-liners; power users model it directly (fused/lazy reps).
//
// State (the memo)
//   SolverState  owns {flow, potential}. Exposes:
//                  certificate() -> CertificateView  (const view; cheap, no copy)
//                  residual()    -> ResidualGraph    (const view; derived)
//                  snapshot() / clone()              (explicit owning copies: serialize / fork)
//                {flow, π} is BOTH the resume memo AND the optimality proof — one artifact,
//                one invariant (reduced cost >= 0), so the const-view certificate makes the
//                check as cheap as possible (O(E) scan, nothing reconstructed).
//
// Solver
//   solve(FlowInstance) -> SolverState   (base solver; wraps capacity-scaling today, SSP later)
//
// Certificate checker (the oracle)
//   certifies_optimality(instance, CertificateView) == ""  iff  feasible AND every available
//   unit-residual arc has reduced cost >= 0 under the potentials.
//
// Path to green:
//   1. EdgeMap + CertificateView + certifies_optimality  (test vs hand-built {flow, π}).
//   2. FlowInstance concept + map_backed_instance.
//   3. SolverState + solve() (wrap existing fragile_mccf, return {flow, π}).
//   4. This test compiles and passes; add a brute-force cross-check on tiny instances.
// ===========================================================================

using Graph   = roadgeometry::HashMapGraph<int, int>;
using CostMap = std::unordered_map<int, std::function<double(double)>>;

TEST_CASE("north star: a solved instance yields a self-certifying SolverState") {
    // Single road 0->1, cost c(x)=x, bounds [0,5], move one unit 0->1.
    Graph g;
    g.add_edge(0, 0, 1);

    auto inst = roadgeometry::map_backed_instance(
        g,
        CostMap{ {0, [](double x) { return x; }} },
        /*ub*/     std::unordered_map<int, double>{{0, 5.0}},
        /*lb*/     std::unordered_map<int, double>{{0, 0.0}},
        /*supply*/ std::unordered_map<int, double>{{0, +1.0}, {1, -1.0}});

    roadgeometry::SolverState state = roadgeometry::solve(inst);

    // The memo proves its own optimality, cheaply, via the const-view certificate.
    CHECK(roadgeometry::certifies_optimality(inst, state.certificate()) == "");
    CHECK(state.flow().at(0) == doctest::Approx(1.0));
}
