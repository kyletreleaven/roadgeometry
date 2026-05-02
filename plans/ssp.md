# Successive Shortest Paths for Road Matching

## Summary

SSP is an alternative to capacity scaling (`fragile_mccf`) for the road matching
min-cost flow reduction. For this problem it is asymptotically equivalent in the
batch case, and strictly better in the incremental (web app) case.

---

## SSP Algorithm

Maintain a feasible, optimal flow. To add one unit of supply at `s` and demand at `t`:

1. Compute Dijkstra from `s` to `t` in the **residual graph** (forward arcs with
   their costs; backward arcs on edges carrying flow, with negated costs).
2. Reweight potentials: `π(v) += d[v]` (Johnson update — keeps all reduced costs ≥ 0
   for subsequent queries).
3. Augment one unit along the shortest path.

The result is 0-optimal (fully optimal) for the updated supply/demand.

Repeat for each supply/demand pair. After n pairs: optimal flow for the full problem.
Total cost: **O(n × Dijkstra)**.

---

## Comparison with Capacity Scaling

Both algorithms do **O(n) total augmentations** for total flow n (geometric series
argument: Σ n/Δ_k = O(n) over the log-U phases).

Capacity scaling's potential advantage: early phases (large Δ) run Dijkstra on a
**sparse subgraph** — only arcs with residual capacity ≥ Δ. Each augmentation sends
Δ units, amortizing the Dijkstra cost over more flow. This helps when arc capacities
are large and arcs saturate progressively.

For the road matching problem, arc capacities are effectively unit:

- Each supply/demand pin contributes 1 unit at its road position.
- `sort_and_segment` merges pins on the same road segment into a single node with
  count > 1 — but this only happens when multiple pins coincide, which is rare for
  human-placed pins.
- Road arcs carry capacity n (total supply), but since n units are spread across many
  arcs, individual arcs rarely saturate early. The "sparse subgraph" in early phases
  is essentially the full graph.

**Conclusion:** for sparse pin distributions (the typical case), capacity scaling
offers no advantage over SSP. SSP is the simpler and equivalent algorithm.

---

## Incremental Update (Web App)

This is where SSP is strictly better.

When a new supply/demand pair is added to the web app:

1. **Map existing flow onto the new instance**: inject breakpoints where the new
   src/dst positions split their respective road edges. Carry existing flow onto
   the sub-edges. O(1) edge splits.

2. **One Dijkstra** from new `src` to new `dst` in the updated residual graph.

3. **Augment** along the shortest path.

The result is **0-optimal** for the new instance (SSP theorem). No `fragile_mccf`
call needed. Cost per new pair: **O(1 Dijkstra)**.

This is not just a warm start for capacity scaling — it IS the solution.

### Why 0-optimal, not just Delta-optimal

The original flow is 0-optimal (all residual arcs have non-negative reduced cost).
The Dijkstra path consists of non-negative reduced-cost arcs. After augmenting and
updating potentials by `π += d[·]`, reverse arcs of the augmenting path get reduced
cost exactly 0, all other arcs remain ≥ 0. The SSP theorem guarantees this.

---

## Relation to Relevant Subgraph Optimization

The web app already plans to maintain a relevant subgraph (union of shortest paths
between matched pairs) for efficiency. The SSP Dijkstra query serves double duty:

- It finds the augmenting path (SSP step).
- The path itself is the new shortest-path contribution to the relevant subgraph.

Both can be done in a single early-termination Dijkstra: stop as soon as the
wavefront hits any node already in the relevant subgraph (the existing residual
structure handles the rest). The path found simultaneously updates the subgraph
and augments the flow.

---

## Implementation Plan

### Batch (cold start / reset)

Replace `fragile_mccf` with SSP built directly on the road network residual graph:

1. Initialize zero flow, zero potentials.
2. For each supply/demand pair (in any order): Dijkstra + augment + update potentials.
3. After all pairs: optimal flow, ready for traversal.

For n pairs this is identical in complexity to the current algorithm. No change needed
to `traverse` or the rest of the pipeline.

### Incremental (web app, pin added)

1. Split the (at most 2) road edges containing new src/dst; propagate existing flow.
2. Run Dijkstra in the residual graph from src to dst.
3. Augment one unit along the path; update potentials.
4. Incorporate path into relevant subgraph.
5. Re-run `traverse` on the updated topograph to extract the new matching.

### Pin removed

Rebuild from scratch via SSP (n Dijkstra queries). Acceptable for small pin counts.
Future: maintain a "reverse flow" structure for efficient decremental updates.

---

## Shared Infrastructure with Capacity Scaling

SSP and capacity scaling operate on the same data structure and differ only in one
parameter. The core is:

- **Residual graph**: forward arcs `(u,v)` with cost `c(e)` and residual capacity;
  backward arcs `(v,u)` with cost `-c(e)` and residual capacity equal to current flow.
- **Potential vector** `π`: maintained across queries so all reduced costs
  `c(e) + π(u) - π(v) ≥ 0`, enabling plain Dijkstra (no negative arcs).

The three primitive operations:

```
dijkstra(src, capacity_threshold=0)  →  distance vector d[]
augment(path, amount)                →  updates residual capacities
update_potentials(d[])               →  π(v) += d[v]
```

**SSP**: call `dijkstra(src, threshold=0)`, augment 1 unit, update potentials. Repeat.

**Capacity scaling**: call `dijkstra(src, threshold=Δ)` (only traverse arcs with
residual capacity ≥ Δ), augment Δ units, update potentials. Stage 1 (saturate
negative reduced-cost arcs) is just degenerate single-arc augmentations.

The `capacity_threshold` parameter is the only algorithmic difference. A single
implementation of the residual graph + Dijkstra + augment serves both. Capacity
scaling can be layered on top of the SSP infrastructure if ever needed (e.g., if
pin clustering makes unit-capacity assumption break down).

---

## Open Questions

- Does `traverse` need changes to accept an SSP-computed flow, or is it already
  decoupled from how the flow was computed? (It should be — it only consumes
  `segments` and `flow`.)
- Potential representation: the Johnson potentials maintained across SSP steps are
  the same potentials `fragile_mccf` maintains internally. Can we expose/reuse that
  structure?
- For the continuous-space road network (pins at arbitrary arc positions), the
  Dijkstra must use `RoadnetQuery` / the existing continuous-space implementation.
  Backward arcs need to be added to that interface.
