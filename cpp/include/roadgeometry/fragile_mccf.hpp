#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "concepts.hpp"
#include "dijkstra.hpp"
#include "flow_potential.hpp"
#include "hash_utils.hpp"
#include "residual_graph.hpp"
#include "mccf/concepts.hpp"   // mccf::Instance, node_t, edge_t
#include "mccf/traits.hpp"     // mccf::map_backed_instance, has_lower_bounds_v

namespace roadgeometry {

// ---------------------------------------------------------------------------
// BasicCost<C, E> — minimal cost-map concept (find(key)/end()). Used by the
// sparse engine; the Instance-based core below takes costs via Instance::cost(e).
// ---------------------------------------------------------------------------
template <typename C, typename E>
concept BasicCost = requires(const C& c, E key) {
    { c.find(key) };
    { c.end() };
};

// ---------------------------------------------------------------------------
// fragile_mccf_state (Instance-based core)
//
// Capacity-scaling successive shortest-paths algorithm for convex-cost flow.
// Consumes a single mccf::Instance (network + cost + ub/lb + supply); U and
// epsilon stay explicit algorithm parameters.
//
// Preconditions (caller's responsibility — not checked):
//   1. supply is conservative: sum of all supply values == 0.
//   2. Every Delta-residual graph is strongly connected for all Delta in the
//      scaling sequence.  Callers who can't guarantee this use robust_mccf, which
//      wraps the instance in a RobustInstance (adds a Hamiltonian cycle).
//
// Guarantee: returns an epsilon-optimal feasible flow — no augmenting cycle
// of capacity epsilon has negative cost in the residual graph at termination.
//
// Correctness sketch:
//   For any arc e at flow f, convexity of the cost c gives:
//
//     redcost(e,+1) + redcost(e,-1) = [c(f+Δ) + c(f-Δ) - 2c(f)] / Δ ≥ 0
//
//   so at most one direction is negative.  Stage 1 pushes on each arc at most
//   once per pass: after a push on {e,+1}, the code immediately recomputes
//   redcost(e,−1) = −old_redcost(e,+1) ≥ 0, so the opposite direction is
//   never pushed in the same pass without a fresh re-evaluation.  Stage 2
//   augments along a Dijkstra shortest path, which is a simple (acyclic) path:
//   each arc appears at most once, so no arc is augmented twice in one step.
//   The backward arc of any augmented arc enters the residual with
//   redcost = −old_redcost ≥ 0; the Dijkstra triangle inequality ensures all
//   other arcs also remain non-negative.  Dijkstra augmentation subsumes
//   cancellation: the backward arc {e,−1} is simply a residual arc like any
//   other, and routing through it reduces flow on e.
//
// Returns: {flow, potential}. Potentials are computed regardless, so returning
// them is a move, not extra work — they are the optimality certificate and the
// SSP warm-start memo.
// ---------------------------------------------------------------------------
template <mccf::Instance I,
          typename RG = typename residual_graph_traits<typename I::network_type>::type>
FlowPotential<mccf::edge_t<I>, mccf::node_t<I>>
fragile_mccf_state(const I& inst, double U, double epsilon = 1.0)
{
    using G      = typename I::network_type;
    using Node   = typename G::node_type;
    using Edge   = typename G::edge_type;
    using Arc    = typename RG::edge_type;   // std::pair<Edge, int>
    using ArcMap = std::unordered_map<Arc, double, PairHash>;

    constexpr double inf = std::numeric_limits<double>::infinity();

    const G& network = inst.network();

    auto map_get = [](const auto& m, const auto& k, double def = 0.0) -> double {
        auto it = m.find(k);
        return it != m.end() ? it->second : def;
    };

    // Check supply conservation.
    // TODO: iterate supply directly rather than network.nodes() — supply is
    // typically much smaller (only nodes with nonzero supply are keyed).
    {
        double total = 0.0;
        for (const Node& i : network.nodes()) total += inst.supply(i);
        if (std::abs(total) > epsilon)
            throw std::invalid_argument("fragile_mccf: supply is not epsilon-conservative (|sum| > epsilon)");
    }

    // ---- Initialize flow and capacity ------------------------------------

    // Trim infinite capacities to U (allows negative-slope initialization).
    std::unordered_map<Edge, double> capacity;
    for (const Edge& e : network.edges())
        capacity[e] = std::min(U, inst.ub(e));

    std::unordered_map<Edge, double> flow;
    for (const Edge& e : network.edges()) flow[e] = 0.0;

    // ---- Excess (net supply at each node) --------------------------------

    std::unordered_map<Node, double> excess;

    auto recompute_excess_node = [&](const Node& i) {
        double ex = inst.supply(i);
        for (const Edge& e : network.in_edges(i))  ex += flow.at(e);
        for (const Edge& e : network.out_edges(i)) ex -= flow.at(e);
        excess[i] = ex;
    };

    for (const Node& i : network.nodes()) recompute_excess_node(i);

    // ---- Potentials and cost structures ----------------------------------

    std::unordered_map<Node, double> potential;
    for (const Node& i : network.nodes()) potential[i] = 0.0;

    ArcMap lincost, redcost;

    // ---- Residual graph --------------------------------------------------

    RG rgraph;
    for (const Node& i : network.nodes()) rgraph.add_node(i);

    // ---- Per-edge update helpers -----------------------------------------

    auto update_residual_edge = [&](const Edge& e, double D) {
        auto [u, v] = network.endpoints(e);
        double x   = flow.at(e);
        double cap = capacity.at(e);

        Arc fwd{e, +1};
        if (rgraph.has_edge(fwd)) rgraph.remove_edge(fwd);
        if (x + D <= cap)         rgraph.add_edge(fwd, u, v);

        Arc bwd{e, -1};
        if (rgraph.has_edge(bwd)) rgraph.remove_edge(bwd);
        // lb ≡ 0 fast path: skip the lb() call and the subtraction entirely.
        double lb_e = 0.0;
        if constexpr (mccf::has_lower_bounds_v<I>) lb_e = inst.lb(e);
        if (x - lb_e >= D) rgraph.add_edge(bwd, v, u);
    };

    auto linearize_cost_edge = [&](const Edge& e, double D) {
        double x = flow.at(e);
        const auto& c = inst.cost(e);   // retrieve the invocable once, evaluate many
        for (int dir : {+1, -1})
            lincost[Arc{e, dir}] = (c(x + dir * D) - c(x)) / D;
    };

    auto reduce_cost_edge = [&](const Edge& e) {
        auto [u, v] = network.endpoints(e);
        double pu = potential.at(u), pv = potential.at(v);
        redcost[Arc{e, +1}] = lincost.at(Arc{e, +1}) + pv - pu;
        redcost[Arc{e, -1}] = lincost.at(Arc{e, -1}) + pu - pv;
    };

    // Thin wrapper so Dijkstra can subscript a const ArcMap.
    struct ArcCost {
        const ArcMap& m;
        double operator[](const Arc& a) const { return m.at(a); }
    };

    // ---- Capacity-scaling main loop --------------------------------------

    double Delta = std::pow(2.0, std::floor(std::log2(U)));

    while (Delta >= epsilon) {

        // Re-initialize residual graph and costs at the start of each phase.
        for (const Edge& e : network.edges()) {
            linearize_cost_edge(e, Delta);
            update_residual_edge(e, Delta);
            reduce_cost_edge(e);
        }

        // -- Stage 1: saturate every negative reduced-cost residual arc ----
        //
        // Snapshot edges first: the residual graph is modified during iteration.
        std::vector<Arc> res_edges(rgraph.edges().begin(), rgraph.edges().end());
        for (const Arc& arc : res_edges) {
            if (redcost.at(arc) >= 0.0) continue;

            auto [e, dir] = arc;
            flow[e] += dir * Delta;

            auto [u, v] = network.endpoints(e);
            recompute_excess_node(u);
            recompute_excess_node(v);

            // Incremental update (potentials unchanged → only this edge matters).
            linearize_cost_edge(e, Delta);
            update_residual_edge(e, Delta);
            reduce_cost_edge(e);
        }

        // -- Stage 2: augment Delta-flow along shortest paths ---------------
        while (true) {
            // Find a surplus node s and a deficit node t. optional doubles as the
            // "found" flag and needs only copyability — no default-constructible Node.
            std::optional<Node> s, t;
            for (const Node& i : network.nodes()) {
                if (!s && excess.at(i) >=  Delta) s = i;
                if (!t && excess.at(i) <= -Delta) t = i;
                if (s && t) break;
            }
            if (!s || !t) break;

            // Shortest path (w.r.t. reduced costs) from s in the residual graph.
            auto [dist, upstream] = dijkstra(rgraph, ArcCost{redcost}, *s);

            // Trace path from s to t via upstream pointers.
            // Use upstream.count(j) instead of j != s: the source node s is
            // never given an upstream entry, so count(j)==0 terminates
            // correctly even when j and s are different C++ objects for the
            // same logical node (e.g. distinct PyObject* for the same int).
            std::vector<Arc> path;
            {
                Node j = *t;
                while (upstream.count(j)) {
                    Arc arc = upstream.at(j);
                    path.push_back(arc);
                    j = rgraph.endpoints(arc).first;
                }
                std::reverse(path.begin(), path.end());
            }

            // Augment Delta flow along the path.
            for (const Arc& arc : path) {
                auto [e, dir] = arc;
                flow[e] += dir * Delta;

                auto [u, v] = network.endpoints(e);
                recompute_excess_node(u);
                recompute_excess_node(v);

                linearize_cost_edge(e, Delta);
                update_residual_edge(e, Delta);
                // (reduced costs updated below after potentials shift)
            }

            // Update potentials from Dijkstra distances.
            for (const Node& i : network.nodes())
                potential[i] -= map_get(dist, i, 0.0);

            // All potentials changed → recompute all reduced costs.
            for (const Edge& e : network.edges())
                reduce_cost_edge(e);
        }

        if (Delta <= epsilon) break;
        Delta /= 2.0;
    }

    return { std::move(flow), std::move(potential) };
}

// ---------------------------------------------------------------------------
// Unpacked shims — the map-based signatures, for callers who guarantee
// connectivity themselves. They build a map_backed_instance over the plain maps
// and delegate to the Instance core. (robust_mccf has the symmetric pair and
// interposes a RobustInstance before delegating.)
// ---------------------------------------------------------------------------
template <InputGraph G,
          typename Cap,
          typename Cost,
          typename RG = typename residual_graph_traits<G>::type>
FlowPotential<typename G::edge_type, typename G::node_type>
fragile_mccf_state(
    const G& network,
    const Cap&  capacity_in,
    const std::unordered_map<typename G::node_type, double>& supply,
    const Cost& cost,
    double U,
    double epsilon = 1.0,
    // Optional per-edge lower bound (default 0). Only lb <= 0 is supported here:
    // it needs no feasibility pre-flow since the initial flow x = 0 satisfies it.
    const std::unordered_map<typename G::edge_type, double>& lb = {}
)
{
    return fragile_mccf_state<
        mccf::MapBackedInstance<G, Cost, Cap,
            std::unordered_map<typename G::edge_type, double>,
            std::unordered_map<typename G::node_type, double>>,
        RG>(
        mccf::map_backed_instance(network, cost, capacity_in, lb, supply),
        U, epsilon);
}

// Thin wrapper: the map-based signature, returning just the flow.
template <InputGraph G,
          typename Cap,
          typename Cost,
          typename RG = typename residual_graph_traits<G>::type>
std::unordered_map<typename G::edge_type, double>
fragile_mccf(
    const G& network,
    const Cap&  capacity_in,
    const std::unordered_map<typename G::node_type, double>& supply,
    const Cost& cost,
    double U,
    double epsilon = 1.0,
    const std::unordered_map<typename G::edge_type, double>& lb = {}
)
{
    return fragile_mccf_state<G, Cap, Cost, RG>(
        network, capacity_in, supply, cost, U, epsilon, lb).flow;
}

} // namespace roadgeometry
