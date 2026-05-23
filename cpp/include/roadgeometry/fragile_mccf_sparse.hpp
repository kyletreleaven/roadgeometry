#pragma once
#include <algorithm>
#include <cassert>
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
// Originally a sparse/lazy alternative to fragile_mccf.  Now being evolved
// into a matching-aware variant that exploits matching problem structure to
// make saturation (Stage 1) efficient:
//
//   - Roads split into "non-empty" (piecewise-linear cost, pins present) and
//     "empty" (cost = length × |f|, no pins).
//   - Flow is always an integer multiple of the current Delta (capacity scaling
//     invariant).  Therefore on any empty road, a change in flow of ±Delta cannot
//     straddle zero (because of halving: all remaining phases sum to at most Delta,
//     so a step already taken cannot be reversed).
//   - Therefore empty road lincost = ±length, computable on demand, never cached;
//     lincost.clear() at change_delta remains unconditional (empty road arcs are
//     simply never in the cache, so clearing doesn't affect them).
//   - Interface: network input must include road lengths; empty roads are
//     identified by absence from the cost map.  cost map fallback to 0.0 should
//     be removed — a missing key means empty road, not zero piecewise-linear cost.
//   - Replace the O(|E|) Stage 1 sweep with a tracked negative_arcs set:
//       Arcs enter the set whenever a redcost check finds them negative —
//       during the per-Delta sweep of non-empty roads at change_delta, and
//       during rechecks triggered by push_flow and update_potentials.
//       Arcs leave only when a recheck finds redcost ≥ 0; saturation alone
//       is not sufficient since the arc may re-enter the residual at smaller Delta.
//       Invalidation events that require rechecks:
//         push_flow         — recheck 2 arcs of the pushed edge
//         update_potentials — recheck arcs incident to visited nodes (already
//                             iterated per-node during Dijkstra; no O(|E|) phase)
//         change_delta      — recheck non-empty road arcs only; empty road arcs
//                             have unchanged redcost (lincost and potentials both
//                             unaffected by halving Delta)
//
// -- Key invariants for negative_arcs ---------------------------------------
//
//   At zero flow and zero potentials (algorithm start), no empty arc can be
//   negative: lincost = length > 0 and potential difference = 0.
//
//   TODO (incremental): by convexity of cost, both forward and backward
//   finite-difference lincosts are non-decreasing in Delta.  Therefore
//   increasing Delta at fixed flow and potentials makes all redcosts weakly
//   larger — increasing Delta cannot introduce new negative arcs.  The
//   incremental algorithm may be able to exploit this when new pins raise Delta.
//
//   Empty arcs with zero flow are never invalidated by any mutation:
//     push_flow         — only arcs of the pushed edge are affected; an arc
//                         with no flow cannot be the pushed arc (residual arc
//                         (e,-1) requires flow[e] ≥ Delta; (e,+1) is pushed only
//                         if it was in the residual, after which flow ≠ 0).
//     update_potentials — Dijkstra optimality guarantees all arcs (including
//                         empty/zero-flow ones) have non-negative redcost after
//                         update; no recheck needed.
//     change_delta      — empty arc lincost at x=0: length·(|±Delta|-0)/Delta =
//                         length > 0; halving Delta leaves lincost unchanged.
//   More precisely, for empty arcs with nonzero flow x, only the direction
//   toward zero (the side with the cusp at |f|=0) has lincost sensitive to
//   Delta; the away-from-zero direction always has lincost = +length regardless
//   of Delta.  So lincost invalidation at change_delta — whether Delta halves
//   or increases when a new batch of pins arrives — is limited to:
//     (a) non-empty arcs (both directions), and
//     (b) the toward-zero direction of empty arcs with nonzero flow.
//   The away-from-zero direction of empty arcs, and all empty/zero-flow arcs,
//   remain valid across batch boundaries and need not be evicted.
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
// -- Pending implementation (matching-aware saturation) ---------------------
//
// Design: MatchingCostMap provides cost for ALL arcs (no missing-key semantic).
//   Empty roads yield a LinearCost{length} value type (struct with operator()
//   returning length*|f|), produced by value with no heap allocation.  A
//   separate is_non_empty(e) predicate distinguishes non-trivial PWL roads from
//   linear ones; it is used to decide whether to cache lincost and to restrict
//   Stage 1 iteration.  make_cbound (in RobustCost) stays unchanged: iterating
//   begin()/end() now naturally covers all arcs and yields the correct total.
//
//   MatchingCostMap API (optimal_flow.hpp):
//     fns      — unordered_map<int, PiecewiseLinear>: non-empty roads only
//     lengths  — vector<double>: road length for every edge id
//     find(e)  — always returns valid entry; empty roads return LinearCost{len}
//     begin()/end() — two-phase iterator: fns first, then empty roads from
//                     lengths; both phases yield value types, zero-cost
//     is_non_empty(e) — fns.count(e) > 0; O(1)
//
//   RobustCost (robust_mccf.hpp):
//     find()   — cycle edges → prohibitive linear fn; regular edges always
//                delegate to inner cost (inner find() always valid now)
//     begin()/end() — regular arcs from inner cost + cycle arcs
//     is_non_empty(RobustEdge) — RegularEdge → inner cost.is_non_empty(r->e);
//                                CycleEdge   → false (linear prohibitive cost)
//     make_cbound — unchanged; natural iteration now covers all arcs
//     length() forwarder — no longer needed (find() always valid)
//
//   fragile_mccf_sparse (this file):
//     MatchingCost concept — replace length(e) with is_non_empty(e) → bool
//     get_lincost / RedcostView — always use find(e)->second(...); use
//                                 is_non_empty(e) to decide whether to cache
//     push_flow — use is_non_empty(e) to guard cache update (replaces assert)
//     Stage 1   — iterate network.edges(), use is_non_empty(e) to skip empties
//
//   [x] FlowInstance.cost: MatchingCostMap replaces edge_cost + edge_lengths
//   [x] build_flow_reduction: EmptyRoadCost flag; empty roads omitted from fns
//   [x] compute_optimal_flow: EmptyRoadCost template param (default !UseSparse)
//   [ ] MatchingCostMap: LinearCost value type; find() always valid; two-phase
//       iterator; is_non_empty()
//   [ ] RobustCost: updated find(), begin()/end(), is_non_empty(); drop length()
//   [ ] MatchingCost concept: replace length(e) with is_non_empty(e)
//   [ ] get_lincost / RedcostView / push_flow: use find() + is_non_empty()
//   [ ] Stage 1: use is_non_empty(e) instead of find(e) == end()
//
//   [ ] negative_arcs tracked set: replace the Stage 1 sweep of non-empty arcs
//       with a maintained set of arcs known to have negative redcost.
//       Like lincost, it is a cached derived quantity and needs invalidation:
//         push_flow         — recheck 2 arcs of pushed edge; remove if redcost ≥ 0
//         update_potentials — recheck arcs incident to visited nodes
//         change_delta      — recheck non-empty road arcs only (empty road
//                             redcosts are unchanged by halving Delta)
//       Stage 1 then drains the set in O(|negative_arcs|) instead of O(|E|).
//   [ ] SparseResidual::out_edges: eliminate per-call vector allocation
//   [ ] run tests (cd setiptah-roadgeometry-matching && nox -s test)
//
// TODO: for an incremental matching algorithm, the persistent state is
//   (flow, potentials, segment arrangement / cost fns) — enough to resume
//   augmentation after adding or removing pins.  excess and lincost are scratch:
//   excess is recomputable from supply + flow; lincost cache is warm-startable
//   but safe to discard.  SparseResidual and RedcostView remain phase-local
//   adapters over that state.
//
//   Segment arrangement: use a single sorted map<(road, y), group> across all
//   roads rather than a per-road map.  Better asymptotics: O(N) space and
//   O(log N) operations vs O(N + R) space for per-road maps (which pay R
//   overhead even when most roads have no pins).  A road's pins are found via
//   upper_bound({road, -inf}) and iterated until the road key changes.  Cost
//   fn on a road is a running integral of surplus along y, so a new pin at y
//   invalidates the suffix from y onward — recompute O(k) groups after the
//   insertion point.
// ===========================================================================

// ---------------------------------------------------------------------------
// MatchingCost<C, E>
//
// Concept for the cost map accepted by fragile_mccf_sparse.  Extends the
// basic find/end interface with:
//   length(key)          — road length for empty-arc lincost (cost = length×|f|)
//   empty_road_cbound(U) — sum of length×U for roads absent from the cost map;
//                          added to the cycle-edge prohibitive slope in RobustCost
//
// A Cost type without missing arcs satisfies this concept trivially as long as
// it provides the two extra methods — length() and empty_road_cbound() are only
// called when cost.find(e) == cost.end(), so they never execute on a full map.
// ---------------------------------------------------------------------------
template <typename C, typename E>
concept MatchingCost = requires(const C& c, E key, double u) {
    { c.find(key) };
    { c.end() };
    { c.length(key) }          -> std::convertible_to<double>;
    { c.empty_road_cbound(u) } -> std::convertible_to<double>;
};

template <InputGraph G, typename Cap, MatchingCost<typename G::edge_type> Cost>
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
        if (ci != cost.end()) {
            double val = (ci->second(x + dir * Delta) - ci->second(x)) / Delta;
            lincost[arc] = val;
            return val;
        }
        // Empty road: cost = length×|f|.  Compute on demand; don't cache —
        // lincost is O(1) and never needs invalidation at change_delta.
        double len = cost.length(e);
        return len * (std::abs(x + dir * Delta) - std::abs(x)) / Delta;
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
        // Refresh cached lincost for both arcs.  Empty-road arcs are never
        // cached (get_lincost skips the insert), so find() will always miss them.
        for (int d : {+1, -1}) {
            Arc a{e, d};
            auto it = lincost.find(a);
            if (it != lincost.end()) {
                double x  = flow.at(e);
                auto   ci = cost.find(e);
                assert(ci != cost.end() && "lincost cache hit for empty-road arc — cost map has missing arcs");
                it->second = (ci->second(x + d * Delta) - ci->second(x)) / Delta;
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

        // TODO: allocates a fresh vector on every Dijkstra node visit; consider
        // passing an output iterator or reusing a caller-provided buffer.
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
                if (ci != cost_fn.end()) {
                    lc = (ci->second(x + dir * Delta) - ci->second(x)) / Delta;
                    lincost[arc] = lc;
                } else {
                    // Empty road: compute on demand; don't cache.
                    double len = cost_fn.length(e);
                    lc = len * (std::abs(x + dir * Delta) - std::abs(x)) / Delta;
                }
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

        // -- Stage 1: saturate negative-redcost residual arcs ----------------
        // Empty road arcs have non-negative redcost at the start of each Delta
        // level (lincost = ±length, unchanged by halving; potentials unchanged
        // → redcost unchanged from end of previous level, which was ≥ 0).
        // cost.find(e) == cost.end() iff road is empty, so skip those.
        std::vector<Arc> res_edges;
        for (const Edge& e : network.edges()) {
            if (cost.find(e) == cost.end()) continue;  // empty road — non-negative by invariant
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
