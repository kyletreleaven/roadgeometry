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

namespace roadgeometry {

// ===========================================================================
// fragile_mccf_sparse
//
// Sparse/lazy alternative to fragile_mccf.  Same algorithm and preconditions;
// faster on large graphs by avoiding O(|E|) sweeps.  May carry small constant-
// factor overhead on small graphs due to hash-map lookups and generation checks.
//
// -- State component dependency graph ---------------------------------------
//
//   supply   (fixed) ──┐
//   flow              ──┼──► excess
//                       │
//   flow              ──┤
//   Delta             ──┼──► lincost ──┐
//   cost fn  (fixed)  ──┘              │
//                                      ├──► redcost
//   potential         ─────────────────┘
//   topology (fixed)  ─────────────────┘
//
//   flow              ──┐
//   Delta             ──┼──► arc_presence
//   capacity (fixed)  ──┘
//
// -- Invalidation by mutation -----------------------------------------------
//
//   push_flow(e, dir)         excess(u), excess(v)
//                             lincost, arc_presence, redcost  — arcs of e only
//
//   update_potentials(dist)   redcost  — ALL arcs
//
//   change_delta(Δ_new)       lincost, arc_presence, redcost  — ALL arcs
//
// -- Invalidation types -----------------------------------------------------
//
//   change_delta      — global: ALL entries of lincost, arc_presence, redcost
//                       become invalid.  Cannot be selective.
//   update_potentials — batch: all redcost entries for arcs incident to
//                       visited nodes become invalid.  Selective in principle.
//   push_flow         — local: only the 2 arcs of the augmented edge are
//                       affected (plus 2 nodes for excess).  Always O(1).
//
// -- Representation strategies ----------------------------------------------
//
//   on demand    — recompute from dependencies on every access; no storage.
//                  Invalidation events cost O(1) (nothing to update).
//   lazy cached  — compute on first access and store; on invalidation either
//                  (a) mark dirty / remove entry and defer until next access,
//                  or (b) immediately recompute entries already in the cache
//                  (avoids the next miss, still pays nothing for uncached
//                  entries).  Only ever populates entries for edges visited.
//                  Global invalidations handled O(1) via generation counter.
//   cached       — eagerly recompute all cached entries on invalidation;
//                  O(|cache|) ≤ O(|E|) per event, O(1) per query.  Global
//                  invalidation sweeps the entire cache; local is O(1).
//
// -- Representation options and cost per event ------------------------------
//
//   Component     | Strategy      | change_delta  | push_flow     | update_potentials | query
//   --------------|---------------|---------------|---------------|-------------------|----------------------
//   flow          | sparse map    | —             | O(1)          | —                 | O(1)
//   excess        | on demand     | —             | —             | —                 | O(deg)
//                 | sparse cached | —             | O(1)          | —                 | O(1)
//   lincost       | on demand     | O(1)          | O(1)          | —                 | O(cost fn)
//                 | lazy cached   | O(1)          | O(1)          | —                 | O(cost fn) miss, O(1) hit
//                 | cached        | ≤O(|E|)       | O(1)          | —                 | O(1)
//   potential     | sparse map    | —             | —             | O(|visited|)      | O(1)
//   redcost       | on demand     | O(1)          | O(1)          | O(1)              | O(1)
//                 | lazy cached   | O(1)          | O(1)          | O(1)              | O(1) miss, O(1) hit
//                 | cached        | ≤O(|E|)       | O(1)          | ≤O(|E|)           | O(1)
//   arc_presence  | on demand     | O(1)          | O(1)          | —                 | O(1)
//                 | cached        | ≤O(|E|)       | O(1)          | —                 | O(1)
//
// -- Key observations -------------------------------------------------------
//
//   - redcost on demand eliminates the O(|E|) sweep in update_potentials (biggest
//     win; happens every Dijkstra iteration).  Miss cost is just two lookups +
//     a subtraction, so on demand and lazy cached are nearly equivalent for redcost.
//   - lincost lazy cached cuts change_delta from O(|E|) to O(|wavefront|) and only
//     pays O(cost fn) for edges actually visited, not all of |E|.  Deferring
//     recompute to first access (via generation counter) rather than sweeping the
//     cache eagerly at change_delta means later phases pay for fewer recomputes as
//     Dijkstra converges and visits a shrinking frontier.
//   - excess sparse cached is a free O(deg)→O(1) improvement regardless of other
//     choices; initialized from sparse supply (O(n_pins)), then incremental per
//     push_flow.  Only non-zero-excess nodes have entries.
//   - arc_presence on demand eliminates O(|E|) residual rebuild in change_delta.
//   - Irreducible cost: Dijkstra still traverses O(|E|) edges in the worst case.
//
// -- Selected strategies ----------------------------------------------------
//
//   excess        sparse cached   — initialized from supply, ±delta at push_flow
//   lincost       lazy cached     — lincost.clear() at change_delta; eager recompute
//                                   for push_flow (only 2 arcs, always O(cost fn)).
//                                   New edges computed on first access.  Per-query
//                                   cost: O(1) hit or O(cost fn) miss.
//                                   TODO: generation counter would make change_delta
//                                   truly O(1) if clear()'s O(bucket_count) overhead
//                                   becomes measurable.
//   redcost       on demand       — lincost[arc] + potential[v] - potential[u];
//                                   O(1) given cached lincost.  Eliminates O(|E|)
//                                   sweep at every update_potentials.
//   arc_presence  on demand       — flow + Delta + capacity check per arc; O(1).
//                                   Eliminates O(|E|) residual rebuild at change_delta.
// ===========================================================================

template <InputGraph G, typename Cap, typename Cost>
std::unordered_map<typename G::edge_type, double>
fragile_mccf_sparse(
    const G&    network,
    const Cap&  capacity_in,
    const std::unordered_map<typename G::node_type, double>& supply,
    const Cost& cost,
    double U,
    double epsilon = 1.0
)
{
    using Node = typename G::node_type;
    using Edge = typename G::edge_type;
    using Arc  = std::pair<Edge, int>;

    constexpr double inf = std::numeric_limits<double>::infinity();

    // ---- Supply conservation check --------------------------------------

    {
        double total = 0.0;
        for (const auto& [node, val] : supply) total += val;
        if (std::abs(total) > epsilon)
            throw std::invalid_argument(
                "fragile_mccf_sparse: supply is not epsilon-conservative (|sum| > epsilon)");
    }

    // ---- Helpers --------------------------------------------------------

    auto map_get = [](const auto& m, const auto& k, double def = 0.0) -> double {
        auto it = m.find(k);
        return it != m.end() ? it->second : def;
    };

    auto get_capacity = [&](const Edge& e) -> double {
        auto it = capacity_in.find(e);
        return std::min(U, it != capacity_in.end() ? it->second : inf);
    };

    // ---- Primary state --------------------------------------------------

    std::unordered_map<Edge, double> flow;           // sparse, default 0
    std::unordered_map<Node, double> excess(supply); // sparse cached, init from supply
    std::unordered_map<Node, double> potential;      // sparse, default 0

    // ---- Lincost lazy cache (cleared at change_delta) -------------------
    // TODO: std::unordered_map::clear() is O(bucket_count), not O(entries).
    //       If bucket count grows large from prior phases, swapping with a
    //       fresh empty map (lincost = {}) may be faster in practice.

    std::unordered_map<Arc, double, PairHash> lincost;

    // ---- On-demand lincost query ----------------------------------------

    auto get_lincost = [&](const Arc& arc, double Delta) -> double {
        auto it = lincost.find(arc);
        if (it != lincost.end()) return it->second;
        auto [e, dir] = arc;
        double x  = map_get(flow, e, 0.0);
        auto   ci = cost.find(e);
        double val = (ci != cost.end())
            ? (ci->second(x + dir * Delta) - ci->second(x)) / Delta
            : 0.0;
        lincost[arc] = val;
        return val;
    };

    // ---- On-demand redcost query ----------------------------------------

    auto get_redcost = [&](const Arc& arc, double Delta) -> double {
        auto [e, dir] = arc;
        auto [u, v]   = network.endpoints(e);
        double pu = map_get(potential, u, 0.0);
        double pv = map_get(potential, v, 0.0);
        return get_lincost(arc, Delta) + (dir == +1 ? pv - pu : pu - pv);
    };

    // ---- Push flow (excess: incremental; lincost: eager if cached) ------

    auto push_flow = [&](const Arc& arc, double Delta) {
        auto [e, dir] = arc;
        flow[e] += dir * Delta;
        auto [u, v] = network.endpoints(e);
        excess[u] -= dir * Delta;
        excess[v] += dir * Delta;
        for (int d : {+1, -1}) {
            Arc a{e, d};
            auto it = lincost.find(a);
            if (it != lincost.end()) {
                double x  = flow.at(e);
                auto   ci = cost.find(e);
                it->second = (ci != cost.end())
                    ? (ci->second(x + d * Delta) - ci->second(x)) / Delta
                    : 0.0;
            }
        }
    };

    // ---- Sparse residual view (arc_presence on demand) ------------------

    struct SparseResidual {
        using node_type = Node;
        using edge_type = Arc;

        const G&                               network;
        const std::unordered_map<Edge, double>& flow;
        const Cap&                              capacity_in;
        double                                  U, Delta;

        std::vector<Arc> out_edges(const Node& u) const {
            constexpr double inf = std::numeric_limits<double>::infinity();
            std::vector<Arc> arcs;
            for (const Edge& e : network.out_edges(u)) {
                double x   = flow.count(e) ? flow.at(e) : 0.0;
                auto   it  = capacity_in.find(e);
                double cap = std::min(U, it != capacity_in.end() ? it->second : inf);
                if (x + Delta <= cap) arcs.push_back({e, +1});
            }
            for (const Edge& e : network.in_edges(u)) {
                double x = flow.count(e) ? flow.at(e) : 0.0;
                if (x >= Delta) arcs.push_back({e, -1});
            }
            return arcs;
        }

        std::pair<Node, Node> endpoints(const Arc& arc) const {
            auto [e, dir] = arc;
            auto [u, v]   = network.endpoints(e);
            return dir == +1 ? std::make_pair(u, v) : std::make_pair(v, u);
        }
    };

    // ---- Redcost cost wrapper for Dijkstra (caches lincost on miss) -----

    struct RedcostView {
        const G&                                        network;
        std::unordered_map<Arc, double, PairHash>&      lincost;
        const std::unordered_map<Node, double>&         potential;
        const Cost&                                     cost_fn;
        const std::unordered_map<Edge, double>&         flow;
        double                                          Delta;

        double operator[](const Arc& arc) const {
            auto [e, dir] = arc;
            double lc;
            auto it = lincost.find(arc);
            if (it != lincost.end()) {
                lc = it->second;
            } else {
                double x  = flow.count(e) ? flow.at(e) : 0.0;
                auto   ci = cost_fn.find(e);
                lc = (ci != cost_fn.end())
                    ? (ci->second(x + dir * Delta) - ci->second(x)) / Delta
                    : 0.0;
                lincost[arc] = lc;
            }
            auto [u, v] = network.endpoints(e);
            double pu = potential.count(u) ? potential.at(u) : 0.0;
            double pv = potential.count(v) ? potential.at(v) : 0.0;
            return lc + (dir == +1 ? pv - pu : pu - pv);
        }
    };

    // ---- Capacity-scaling main loop -------------------------------------

    double Delta = std::pow(2.0, std::floor(std::log2(U)));

    while (Delta >= epsilon) {
        lincost.clear();  // invalidate all cached lincost entries for new Delta

        // -- Stage 1: saturate every negative reduced-cost residual arc --
        std::vector<Arc> res_edges;
        for (const Edge& e : network.edges()) {
            double x   = map_get(flow, e, 0.0);
            double cap = get_capacity(e);
            if (x + Delta <= cap) res_edges.push_back({e, +1});
            if (x >= Delta)       res_edges.push_back({e, -1});
        }
        for (const Arc& arc : res_edges) {
            if (get_redcost(arc, Delta) >= 0.0) continue;
            push_flow(arc, Delta);
        }

        // -- Stage 2: augment Delta-flow along shortest paths ------------
        while (true) {
            Node s{}, t{};
            bool found_s = false, found_t = false;
            for (const auto& [node, ex] : excess) {
                if (!found_s && ex >=  Delta) { s = node; found_s = true; }
                if (!found_t && ex <= -Delta) { t = node; found_t = true; }
                if (found_s && found_t) break;
            }
            if (!found_s || !found_t) break;

            SparseResidual rgraph{network, flow, capacity_in, U, Delta};
            RedcostView    rcost{network, lincost, potential, cost, flow, Delta};
            auto [dist, upstream] = dijkstra(rgraph, rcost, s);

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

            for (const Arc& arc : path)
                push_flow(arc, Delta);

            for (const auto& [node, d] : dist)
                potential[node] -= d;
        }

        if (Delta <= epsilon) break;
        Delta /= 2.0;
    }

    return flow;
}

} // namespace roadgeometry
