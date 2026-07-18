# Potential-Update Strategy for Shortest-Path Augmentation

## Summary

Any min-cost-flow algorithm that augments along reduced-cost shortest paths maintains node
**potentials** by a Johnson reweighting after each Dijkstra, so that reduced costs stay ≥ 0 for
the next search. This note is about doing that update in **O(|frontier|)** instead of O(V). It
applies to both capacity scaling (`fragile_mccf` Stage 2) and successive shortest paths
([ssp.md](ssp.md) — SSP generalizes capacity scaling), and is independent of connectivity policy
([connectivity.md](connectivity.md)).

The current code leaves the optimization on the table: it runs Dijkstra to completion and updates
**every** node.

---

## The reweighting, and how small it can be

After Dijkstra from `s` gives distances `dist[·]` (from `s`, so `dist[s] = 0`), the classic update
is `potential[v] -= dist[v]` for all `v`, which makes shortest-path-tree arcs tight and keeps every
reduced cost ≥ 0. But with **early termination** at the target `t`, only the **frontier** needs
touching. Distinguish:

- **reached** — the frontier `S = {v : dist[v] ≤ dist[t]}`, i.e. nodes settled *before* `t` pops;
- **reachable** — `R`, the whole component that *would* be settled if Dijkstra ran to completion;
- `S ⊆ R ⊆ V`.

Stop Dijkstra when `t` pops and update only `S`:

```python
# S = {v : dist[v] <= dist[t]}
for v in S:                       # the whole frontier, not just the s->t path
    potential[v] += dist[t] - dist[v]     # nonneg bump; makes the s->t path tight
```

This differs from the code's `-= dist[v]` (full) update only by a global `+dist[t]` (invisible to
reduced costs); on `S` the two agree, and beyond-`t` nodes are simply left alone.

---

## Why the whole frontier — not just the path

The reduced-cost ≥ 0 invariant is *global*, so every settled node must move, not only those on the
`s→t` path. Take a settled off-path node `b` with an arc `b→a` into a path node `a`. Dijkstra
guarantees `redcost(b→a) ≥ dist[a] − dist[b]`, so shifting **both** keeps
`redcost' = redcost − dist[a] + dist[b] ≥ 0`; moving only `a` leaves a negative seam that the next
Dijkstra chokes on.

## Why nothing outside the frontier

Everything outside `S` — reachable-but-unexpanded *and* unreachable — needs **no** update, and its
arcs stay valid untouched. The only at-risk arcs are `S → w` (settled tail `i`, unexpanded head
`w`). When `i` was settled it relaxed `i→w`; `w` being unpopped-before-`t` means its key is
`≥ dist[t]`, so `redcost(i→w) ≥ dist[t] − dist[i]`. The bump lowers that arc by exactly
`dist[t] − dist[i]`, landing at ≥ 0. In words: the reweighting charges each frontier node at most
`dist[t]`, and every unexpanded node is at least `dist[t]` away — the inefficient-path vertices are
safe *because* they are far. So the update is **O(|S|)**.

---

## What the current code does, and the companion fix

`fragile_mccf` Stage 2 (`cvxcostflow.py`) is pessimistic on two independent counts:

1. It runs `dijkstra(rgraph, redcost, s)` **to completion** — no target, no early termination —
   settling everything reachable.
2. It then loops `network.nodes()` with `potential[i] -= dist[i]` (line 412), updating all of them.

Both together are O(V). Neither is required; early termination + the frontier update is O(|S|).

**Why it was never exploited (and isn't a trivial drop-in):** the *naive* early termination is a
bug. If you stop at `t` and reweight settled nodes by their own `dist[v]` while leaving the rest at
old potentials, arcs from unexpanded nodes into settled ones drop by `dist[t]` and can go negative.
Correctness needs the `dist[t] − dist[v]` clamp form above. That subtlety, plus correctness-first
development, is a sufficient explanation for the full-Dijkstra implementation.

**Companion fix — incremental `ReducedCost`.** Right after the potential loop, line 415 recomputes
`ReducedCost` over **all** edges ("all the potentials have changed"). If only `S` moved, only arcs
incident to `S` changed reduced cost, so the recompute can be incremental too. Optimizing the
Dijkstra + potential loop without this leaves an O(E) global pass in place — the two go together,
and this global-vs-incremental pattern is the same one tracked in
[large_graph_optimization.md](large_graph_optimization.md).

**Profiling caveat.** Step 0 profiling (TODO) fingered `ReducedCost` + `LinearizeCost` (~29%
combined), *not* the potential loop, as the hotspot. So the incremental-`ReducedCost` half is the
one that pays; the O(|S|) potential update is correctness-clean but likely minor on its own.

---

## Not applicable when the target is unreachable

The frontier formula assumes `t` is reached, i.e. `dist[t]` is finite. When `t` is unreachable
(`dist[t] = ∞`) the formula does not apply — but neither is a frontier update wanted: no augmenting
path exists through the reached component, so it needs no reweighting at all. The connectivity
layer handles that case by splicing a connectivity arc and pinning only the target's side; see
[connectivity.md](connectivity.md) § "Potential update when t is unreachable". The infinity is the
signal that this is a connectivity concern, not a frontier one.
