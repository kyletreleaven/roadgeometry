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

**This section describes a matching-specific algorithm, not general SSP.**
In general SSP, costs are fixed and only supply/demand changes. In road
matching, adding a pin also changes the cost function on the road where the
pin lands — a new breakpoint is inserted into the piecewise linear cost at
that position. The incremental algorithm works efficiently because this change
is structurally constrained: local to at most two roads (src and dst), with
known form.

When a new supply/demand pair is added to the web app:

1. **Map existing flow onto the new instance**: inject breakpoints where the new
   src/dst positions split their respective road edges. Carry existing flow onto
   the sub-edges. O(1) edge splits.

2. **Re-linearize the cost on the (at most two) affected road edges.** Since
   Δ = 1 is fixed in the incremental case, re-linearization is just evaluating
   the new marginal cost at the current flow value on those edges — a scalar
   operation per edge. All other edges are unaffected.

3. **One Dijkstra** from new `src` to new `dst` in the updated residual graph.

4. **Augment** along the shortest path.

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
parameter.

**Persistent state** — the residual graph object carries three things:

- **Flow map** `f`: edge → double, stored sparsely (implicit 0). Determines which
  residual arcs exist and their capacity. Together with π, fully defines the
  residual state; both are required to resume correctly from a previous solve.
- **Potential vector** `π`: node → double, stored sparsely (implicit 0). Maintained
  across augmentations so all reduced costs `lincost(e) + π(v) - π(u) ≥ 0`,
  enabling plain Dijkstra. A node enters explicit storage only the first time a
  Dijkstra wavefront reaches it; nodes outside every augmenting path's wavefront
  are never stored. For a sparse pin instance this is O(n × avg_path_length)
  rather than O(V). Reduced costs are computed on-the-fly as
  `lincost(e, dir) + π.get(head, 0) - π.get(tail, 0)`.
- **Current Δ**: drives re-linearization decisions:
  - `last_Δ == new_Δ`: only edges on the augmenting path (or with changed
    underlying cost) need re-linearization.
  - `last_Δ != new_Δ`: full re-linearization sweep required.
  In the incremental case (Δ always 1) re-linearization is always O(affected edges).

**Invariants:**

- **Feasible**: all excesses are zero — the flow satisfies supply/demand at every node.
- **Δ-optimal**: every Δ-residual arc has non-negative reduced cost. At Δ=1 this is full optimality; larger Δ is a weaker condition.
- **Residual valid**: lincosts are current for the present Δ and cost function.

Residual validity is required for Δ-optimality to be well-defined; both are required for `augment_step`. The state is fully solved when feasible and Δ-optimal at Δ=1.

**Operations:**

| action | effects on state |
|---|---|
| add k pins | breaks feasibility; if α-optimal, degrades to (α+k)-optimal; invalidates lincost on ≤2k roads |
| change Δ | invalidates lincost on all edges; Δ-optimality undefined until re-linearized |
| re-linearize (edges S) | restores lincost on S; Δ-optimality violations now detectable |
| saturate negative arcs | requires valid lincosts; restores Δ-optimality |
| find surplus (excess ≥ Δ) | if found: src for next `augment_step`; if none and Δ > 1: halve Δ; if none and Δ = 1: done |
| `augment_step(src, dst)` | requires Δ-optimality and valid lincosts; routes Δ units along shortest path; updates π; re-linearizes path edges; reduces excess at src and deficit at dst by Δ; maintains Δ-optimality and residual validity |

Operations with no precondition (add k pins, change Δ, re-linearize) can be
freely composed in any order before restoring Δ-optimality via saturation. For
efficiency, track the dirty set of roads needing re-linearization: pin additions
contribute ≤2 roads each; a Δ change marks all edges dirty and further tracking
is moot.

`augment_step` combines dijkstra + augment + update π into one atomic operation.
The distance vector `d[]` is consumed immediately by `π += d[]` and does not
persist. **`π` carries across calls; the search tree does not** — the residual
changes along the augmenting path, making the wavefront stale for subsequent
queries even from the same source.

**Termination and feasibility.** The excess at each node — net inflow minus
outflow, accounting for supply — is the same quantity in both the instance and
the residual graph; there is no separate notion. Since pin supplies are integers
and augmentations always move integer Δ units, excesses stay integers throughout.
When find-surplus returns empty at Δ=1, all excesses are less than 1, hence zero.
The flow is fully feasible.

**Halving as a runtime optimizer.** Successive halving (Δ = U, U/2, …, 1) is a
runtime choice, not a correctness requirement. At each scale there are at most
n/Δ augmenting paths (each carries Δ units of the total supply n), so the total
augmentation count across all phases is n/U + n/(U/2) + … + n/1 ≈ 2n —
O(n) by the geometric series. Any strictly decreasing sequence ending at 1 is
correct; halving minimizes the number of phases and keeps the total augmentation
count at 2n.

**SSP**: for each supply/demand pair in turn — add it, ensure Δ=1 and re-linearize
the ≤2 affected roads, then call `augment_step` once to route 1 unit from src to dst.

**Capacity scaling**: add all n pins at once, set Δ to the next power of 2 ≥ n,
re-linearize and saturate; then repeat — `augment_step` until no surplus with
excess ≥ Δ, halve Δ, re-linearize, saturate — until Δ=1 and no surplus remain.

**Variable batch**: the general form. Adding k pins to an α-optimal flow degrades
it to (α+k)-optimal; set Δ to the next power of 2 ≥ (α+k), re-linearize dirty
edges, saturate, and run halving until Δ=1. The next batch can arrive at any
point — mid-halving or mid-augmentation — not just when the flow is fully optimal.
SSP (k=1, each batch added from a fully optimal flow) and capacity scaling (k=n,
added once from zero flow) are both special cases.

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
- **Variable batch cost updates**: when pins arrive mid-loop, the base cost function
  on the ≤2 affected roads changes (new breakpoint inserted into the piecewise-linear
  cost). The data structure for maintaining and updating the base cost function, and
  then re-linearizing at the current Δ, needs design.
