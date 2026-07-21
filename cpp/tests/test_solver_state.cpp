#include "third_party/doctest/doctest.h"

#include <functional>
#include <unordered_map>

#include "roadgeometry/input_graph.hpp"

#include "roadgeometry/mccf/concepts.hpp"     // mccf::Instance concept       (built)
#include "roadgeometry/mccf/traits.hpp"       // mccf::map_backed_instance    (built)
// Target headers — do not exist yet; building them is the work:
#include "roadgeometry/mccf/state.hpp"        // mccf::State, mccf::OptimalityCertificate
#include "roadgeometry/mccf/solver.hpp"       // mccf::solve(Instance) -> State
#include "roadgeometry/mccf/certificate.hpp"  // mccf::certifies_optimality(Instance, cert)

// ===========================================================================
// NORTH-STAR ACCEPTANCE TEST — drive this to green incrementally.
//
// PROBLEM: min convex-cost flow (MCCF). An instance is a network + convex,
// separable per-edge costs + [lb, ub] bounds + node supplies. Convexity is
// load-bearing: optimality is judged on marginal slopes at the current flow,
// not fixed arc costs. Everything below lives in namespace roadgeometry::mccf,
// so the short names are unambiguous by scope (a sibling matching:: would get
// its own Instance/Certificate). Matches the existing vocabulary: fragile_mccf,
// robust_mccf, MinConvexCostFlow.
//
// Concepts
//   EdgeMap<M,E>     map-like (find/end) over edges; used for ub and lb (symmetric bounds).
//   Instance<I>      problem data via accessors: network() (InputGraph), cost(e), ub(e),
//                    lb(e), supply(n). map_backed_instance(...) adapts plain maps so simple
//                    callers stay one-liners; power users model it directly (fused/lazy reps).
//
// State (the memo)
//   State            owns {flow, potential}. Exposes:
//                      certificate() -> OptimalityCertificate  (const view; cheap, no copy)
//                      residual()    -> ResidualGraph          (const view; derived)
//                      snapshot() / clone()                    (owning copies: serialize / fork)
//                    {flow, π} is BOTH the resume memo AND the optimality proof — one artifact,
//                    one invariant (reduced cost >= 0). The const-view certificate makes the
//                    check as cheap as possible (O(E) scan, nothing reconstructed).
//
// Certificate — specifically the DUAL / optimality certificate (node potentials making all
//   residual reduced costs >= 0), NOT an infeasibility certificate.
//   certifies_optimality(instance, OptimalityCertificate) == ""  iff  feasible AND every
//   available unit-residual arc has reduced cost >= 0 under the potentials.
//
// Solver
//   solve(Instance) -> State   (base solver; wraps capacity-scaling today, SSP later)
//
// Path to green:
//   1. EdgeMap + OptimalityCertificate view + certifies_optimality  (test vs hand-built {flow, π}).
//   2. Instance concept + map_backed_instance.
//   3. State + solve() (wrap existing fragile_mccf, return {flow, π}).
//   4. This test compiles and passes; add a brute-force cross-check on tiny instances.
// ===========================================================================

namespace mccf = roadgeometry::mccf;

using Graph   = roadgeometry::HashMapGraph<int, int>;
using CostMap = std::unordered_map<int, std::function<double(double)>>;

TEST_CASE("north star: a solved MCCF instance yields a self-certifying State") {
    // Single road 0->1, convex cost c(x)=x, bounds [0,5], move one unit 0->1.
    Graph g;
    g.add_edge(0, 0, 1);

    auto inst = mccf::map_backed_instance(
        g,
        CostMap{ {0, [](double x) { return x; }} },
        /*ub*/     std::unordered_map<int, double>{{0, 5.0}},
        /*lb*/     std::unordered_map<int, double>{{0, 0.0}},
        /*supply*/ std::unordered_map<int, double>{{0, +1.0}, {1, -1.0}});

    mccf::State state = mccf::solve(inst);

    // The memo proves its own optimality, cheaply, via the const-view certificate.
    CHECK(mccf::certifies_optimality(inst, state.certificate()) == "");
    CHECK(state.flow().at(0) == doctest::Approx(1.0));
}
