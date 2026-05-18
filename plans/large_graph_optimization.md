# Large-Graph Optimization: Lazy Evaluation of Edgeless Roads

## Problem

The Cambridge driving graph has ~10 000 edges. For a matching instance with
n ≪ 10 000 pins, nearly all edges carry zero flow and contain no pins. The
current pipeline pays O(|E|) work in several places even though the
*useful* subgraph is O(n).

Profiled costs with 2 pins on the Cambridge graph:

| step | time |
|------|------|
| `compute_optimal_flow` (= `sort_and_segment` + BFR + `fragile_mccf`) | ~2 050 ms |
| `create_path_network_with_surplus` (after relevant-subgraph fix) | ~7 ms |
| trail extraction | < 1 ms |

The bottleneck is the flow computation itself, driven by `sort_and_segment`
and `build_flow_reduction` iterating over all edges.

---

## Key Insight: Implicit Structure for Edgeless Roads

An edge that contains no supply or demand pins has a completely determined
(and trivially computable) structure at every stage of the pipeline:

| stage | implicit value | cost to compute |
|-------|----------------|-----------------|
| `sort_and_segment` | empty point list → single segment spanning full edge | O(1) |
| BFR node/arc construction | zero-weight arc of capacity n, cost = edge length | O(1) |
| `fragile_mccf` / Dijkstra | arc exists but is only entered if the wavefront reaches it | lazy |
| `create_path_network_with_surplus` | already fixed to skip non-flow edges | done |

The pipeline can be restructured so that edgeless roads are **never
materialized** in the segment or BFR data structures. They are represented
implicitly and only instantiated on demand when the Dijkstra wavefront
reaches them.

---

## Per-Stage Plan

### `sort_and_segment`

Current: iterates all edges to build a sorted list of breakpoints per edge.

Optimization: build only the edges that appear in the pin set.
Edgeless edges are not placed in the segment dictionary at all.
Any lookup for a missing edge returns the implicit single-segment result.

Estimated speedup: ~linear in |E| → |pins|.

### `build_flow_reduction` (BFR)

Current: creates BFR nodes and arcs for every segment on every edge.

Optimization: for each edge not in the segment dictionary, emit a single
implicit arc (u → v) of cost = edge length, capacity = n, flow = 0.
These arcs are **never written to memory** — instead, the Dijkstra
relaxation accesses them through a virtual adjacency iterator that reads
directly from the edges GeoDataFrame.

The BFR graph thereby has:
- **Dense region**: O(n) segment-nodes for edges that carry pins.
- **Implicit region**: remaining ~|E| road arcs, accessed via the virtual
  iterator only if the wavefront reaches them.

Estimated speedup: BFR construction O(n) instead of O(|E|).

### `fragile_mccf` / Dijkstra

**Implicit residual graph.** The current implementation maintains an explicit
`rgraph` data structure, adding and removing arcs as flow changes. This costs
O(|E|) to initialize and O(1) per arc update, but requires materializing the
full residual upfront. Instead, arc presence and arc cost should both be
computed on-the-fly during Dijkstra relaxation:

- Forward arc `(u,v)`: present iff `flow[e] + Δ ≤ capacity[e]` (always true
  for infinite-capacity matching roads); reduced cost =
  `lincost(e, +1) + π.get(v, 0) - π.get(u, 0)`.
- Backward arc `(v,u)`: present iff `flow[e] ≥ Δ`; reduced cost =
  `lincost(e, -1) + π.get(u, 0) - π.get(v, 0)`.

This eliminates both `rgraph` and the explicit `redcost` map. The sparse
potential vector π (see `ssp.md`) is looked up directly during relaxation;
nodes with implicit potential 0 cost nothing to handle.

**Sparse flow storage.** The flow map should store only nonzero entries; zero
is the implicit default. Roads never on any augmenting path are never touched.
For a sparse pin instance this keeps the flow map at O(n × avg_path_length).

With both changes, Dijkstra works directly over the virtual arc iterator
(edgeless roads) and the implicit residual predicate (roads with flow), with
no separate residual graph object to maintain.

### `create_path_network_with_surplus`

Already fixed in the previous session: only processes edges in
`pin_roads ∪ flow_roads`. No further work needed.

---

## Implementation Sequence

1. ✓ **Downstream sparse compatibility** — rather than modifying `sort_and_segment`
   first, made all consumers handle a sparse segments dict: `flow_from_segments`
   passes full network metadata to C++; `surplus_dict` and `measure_dict` in the
   Python path are sparse (only pinned roads); `compute_optimal_flow` uses
   `_DefaultCostMap` / `WeightedAbs` to provide implicit costs for edgeless roads
   without calling `OBJECTIVE_FUNC`. `build_flow_reduction` (C++) already fell back
   to `empty_seg` for missing segments.

2. **Sparse `sort_and_segment`** — now that downstream is ready, modify
   `sort_and_segment` to only include roads that contain pins. Edgeless roads are
   absent from the dict; consumers infer the implicit single-segment from road length.

3. **Virtual arc iterator for BFR** — replace the full-edge loop in
   `build_flow_reduction` with an iterator that materializes real BFR arcs only for
   pinned edges; edgeless roads are visited lazily by Dijkstra via the virtual
   iterator.

4. **Dijkstra integration** — ensure the Dijkstra relaxation step uses the virtual
   iterator. No change to the priority queue or potential update logic.

5. **Verify correctness** with existing integration tests and web app profiling.
   Target: flow computation < 50 ms for 2–10 pins on Cambridge.

---

## Matching-Specific MCCF Specializations

The road matching problem has known analytical structure on empty roads that
allows specializing the MCCF implementation beyond generic lazy evaluation.

### Cost form

An empty road (no pins) has measure equal to `length` at every flow level, so
its cost function is `length * |z|` — symmetric linear. This means:

- No `PiecewiseLinear` object is needed; cost is represented as a scalar.
- The linearized cost at any Δ and any flow `x ≠ 0` is just `±length`
  (sign determined by direction). No per-phase recomputation.
- At `x = 0` the kink requires care, but for Δ ≥ 1 the linearized slopes
  are still `+length` (forward) and `+length` (backward) — symmetric.

### Capacity

All roads in the matching problem have effectively infinite capacity — there
is no hard per-edge flow limit, only the global bound U. This holds regardless
of whether a road carries pins. In the residual predicate this simplifies to:
- Forward arc: always present (no saturation possible).
- Backward arc: present iff `flow > 0`.

No per-edge capacity value needs to be stored or looked up for any road.

### Implication for linearization

Since the cost is already linear, `linearize_cost_edge` is a no-op for empty
roads at any Δ. This eliminates the dominant term in the per-phase O(|E|)
sweep for sparse instances where most edges are empty.

---

## Relation to SSP

The SSP plan (`ssp.md`) calls for replacing `fragile_mccf` with successive
single-unit Dijkstra augmentations. The lazy evaluation plan is orthogonal:

- In **batch SSP**, each Dijkstra query benefits from the implicit arc
  iterator — the wavefront expands only as far as needed.
- In **incremental SSP** (one Dijkstra per new pin), the wavefront is
  even smaller; the implicit region is barely touched.
- In **capacity scaling** (if retained), the large-Δ early phases use a
  sparse subgraph anyway; lazy eval ensures the dense final phase is also
  bounded by the actual wavefront.

The virtual arc iterator is the single shared primitive that unlocks
O(wavefront) work in all three algorithms.

---

## Note on A* / Pluggable Shortest-Path (In Review)

A natural question is whether A* with a Euclidean heuristic could replace
Dijkstra inside SSP, exploiting the planar structure of road networks.

**Why it does not extend cleanly to the residual graph:**

After the first SSP augmentation, backward arcs appear with reduced cost
**exactly 0** — they were on the previous augmenting path, which was tight,
so the Johnson potential update `π(v) += d[v]` sets their reduced cost to
zero. A path that uses such an arc costs 0 in reduced terms regardless of
its geometric length. The Euclidean heuristic, which lower-bounds physical
road distance, has no way to know this shortcut exists and can overestimate
`d_π(v, t)` arbitrarily — violating admissibility and producing incorrect
(non-optimal) augmentations.

**Could we evolve the heuristic alongside the residual graph?**

An ALT-style approach (A*, Landmarks, Triangle inequality) could maintain
admissibility: precompute distances from/to k landmark nodes, and after each
augmentation re-propagate along the changed arcs. Cost: O(k × path_length)
per SSP step. Admissibility is preserved; the heuristic degrades gracefully
as the residual graph diverges from the road graph. But this adds real
complexity and the benefit is uncertain.

**Natural cutoff for the ALT heuristic:**

Before starting each A* query, compute `h_L(src)` in O(k). If
`h_L(src) / euclidean_dist(src, t) < ε` (e.g. ε = 0.1), the heuristic is
too weak to prune meaningfully — fall back to plain Dijkstra and skip the
landmark maintenance overhead for this step. As backward arcs accumulate
over many augmentations, the residual-graph landmark distances drift from
their road-graph values and this ratio naturally decreases, so the fallback
triggers more often in later SSP steps exactly when the overhead would be
least justified.

**Practical assessment:**

On a planar graph with node density ρ, Dijkstra settles O(ρ d²) nodes (disk
of radius d). A* with a tight heuristic confines the search to a corridor of
width w around the src–dst path, settling O(ρ d w) nodes. The speedup is
O(d/w) — it grows with the distance d between the pin pair, not a constant
factor. For far-apart pins (cross-city), this is a substantial win; for
nearby pins, it degrades toward 1. So A* is most valuable exactly where the
Dijkstra wavefront is most expensive, making the ALT approach worthwhile
despite its maintenance complexity. The O(k) pre-check naturally gates it
off for the easy (nearby-pin) cases where Dijkstra is already fast.

**Where A* does apply cleanly:** the pure road-graph distance oracle (no
residual arcs) used for precomputation or the initial all-pairs distance
matrix. Euclidean heuristic is admissible there and worth using if that
oracle becomes a bottleneck.

---

## Open Questions

- Does `build_flow_reduction` own the arc storage, or does `fragile_mccf`?
  Determines where the virtual iterator is inserted.
- Are there edges in the osmnx graph with `oneway=True` that need asymmetric
  treatment in the implicit arc iterator?
- `sort_and_segment` currently returns a list; should the implicit fallback
  be a `__missing__` dict subclass or an explicit helper function?
