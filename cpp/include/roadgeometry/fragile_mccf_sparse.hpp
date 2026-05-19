#pragma once
#include <unordered_map>

#include "concepts.hpp"

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
//   lincost       lazy cached     — hybrid invalidation: generation counter for
//                                   change_delta (O(1), defers recompute to first
//                                   access per phase); eager recompute for push_flow
//                                   (only 2 arcs, always O(cost fn)).  New edges
//                                   computed on first access.  Per-query cost: one
//                                   generation check + O(1) hit or O(cost fn) miss.
//   redcost       on demand       — lincost[arc] + potential[v] - potential[u];
//                                   O(1) given cached lincost.  Eliminates O(|E|)
//                                   sweep at every update_potentials.
//   arc_presence  on demand       — flow + Delta + capacity check per arc; O(1).
//                                   Eliminates O(|E|) residual rebuild at change_delta.
// ===========================================================================

template <InputGraph G,
          typename Cap,
          typename Cost>
std::unordered_map<typename G::edge_type, double>
fragile_mccf_sparse(
    const G&    network,
    const Cap&  capacity_in,
    const std::unordered_map<typename G::node_type, double>& supply,
    const Cost& cost,
    double U,
    double epsilon = 1.0
);

} // namespace roadgeometry
