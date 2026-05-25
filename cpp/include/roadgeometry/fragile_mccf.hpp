#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "concepts.hpp"
#include "dijkstra.hpp"
#include "hash_utils.hpp"
#include "residual_graph.hpp"

namespace roadgeometry {

// ---------------------------------------------------------------------------
// BasicCost<C, E>
//
// Minimal concept for cost maps accepted by fragile_mccf (dense solver).
// find(key) must always return a valid iterator (never end()).
// ---------------------------------------------------------------------------
template <typename C, typename E>
concept BasicCost = requires(const C& c, E key) {
    { c.find(key) };
    { c.end() };
};

// ---------------------------------------------------------------------------
// fragile_mccf
//
// Capacity-scaling successive shortest-paths algorithm for convex-cost flow.
//
// Preconditions (caller's responsibility — not checked):
//   1. supply is conservative: sum of all supply values == 0.
//   2. Every Delta-residual graph is strongly connected for all Delta in the
//      scaling sequence.  (Use a robust-instance wrapper if not guaranteed.)
//
// Template parameters:
//   G    — InputGraph satisfying the InputGraph concept.
//   Cap  — Map-like: Edge → double.  Needs find(Edge).
//   Cost — Map-like: Edge → callable double(double).  Needs find(Edge).
//          Missing entries treated as zero cost.
//   RG   — ResidualGraph type; defaults to residual_graph_traits<G>::type.
//
// Returns: flow map edge → double (same key set as network.edges()).
// ---------------------------------------------------------------------------
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
    double epsilon = 1.0
)
{
    using Node   = typename G::node_type;
    using Edge   = typename G::edge_type;
    using Arc    = typename RG::edge_type;   // std::pair<Edge, int>
    using ArcMap = std::unordered_map<Arc, double, PairHash>;

    constexpr double inf = std::numeric_limits<double>::infinity();

    auto map_get = [](const auto& m, const auto& k, double def = 0.0) -> double {
        auto it = m.find(k);
        return it != m.end() ? it->second : def;
    };

    // Check supply conservation.
    // TODO: iterate supply directly rather than network.nodes() — supply is
    // typically much smaller (only nodes with nonzero supply are keyed).
    {
        double total = 0.0;
        for (const Node& i : network.nodes()) total += map_get(supply, i, 0.0);
        if (std::abs(total) > epsilon)
            throw std::invalid_argument("fragile_mccf: supply is not epsilon-conservative (|sum| > epsilon)");
    }

    // ---- Initialize flow and capacity ------------------------------------

    // Trim infinite capacities to U (allows negative-slope initialization).
    std::unordered_map<Edge, double> capacity;
    for (const Edge& e : network.edges())
        capacity[e] = std::min(U, map_get(capacity_in, e, inf));

    std::unordered_map<Edge, double> flow;
    for (const Edge& e : network.edges()) flow[e] = 0.0;

    // ---- Excess (net supply at each node) --------------------------------

    std::unordered_map<Node, double> excess;

    auto recompute_excess_node = [&](const Node& i) {
        double ex = map_get(supply, i, 0.0);
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
        if (x >= D)               rgraph.add_edge(bwd, v, u);
    };

    auto linearize_cost_edge = [&](const Edge& e, double D) {
        double x  = flow.at(e);
        auto   it = cost.find(e);
        for (int dir : {+1, -1}) {
            lincost[Arc{e, dir}] = (it != cost.end())
                ? (it->second(x + dir * D) - it->second(x)) / D
                : 0.0;
        }
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
            // Find a surplus node s and a deficit node t.
            Node s{}, t{};
            bool found_s = false, found_t = false;
            for (const Node& i : network.nodes()) {
                if (!found_s && excess.at(i) >=  Delta) { s = i; found_s = true; }
                if (!found_t && excess.at(i) <= -Delta) { t = i; found_t = true; }
                if (found_s && found_t) break;
            }
            if (!found_s || !found_t) break;

            // Shortest path (w.r.t. reduced costs) from s in the residual graph.
            auto [dist, upstream] = dijkstra(rgraph, ArcCost{redcost}, s);

            // Trace path from s to t via upstream pointers.
            // Use upstream.count(j) instead of j != s: the source node s is
            // never given an upstream entry, so count(j)==0 terminates
            // correctly even when j and s are different C++ objects for the
            // same logical node (e.g. distinct PyObject* for the same int).
            std::vector<Arc> path;
            {
                Node j = t;
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

    return flow;
}

} // namespace roadgeometry
