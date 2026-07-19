# plans/

Algorithmic and implementation design notes. Product/interface/architecture design lives in
[../design.md](../design.md); the work tracker (with links into these notes) is
[../TODO.md](../TODO.md).

- [connectivity.md](connectivity.md) — connectivity policy for `fragile_mccf`: `HamCycle` vs.
  `Direct` × `Explicit`/`Implicit` (+ `None`); lazy loans; loan-count analysis; potential-update
  interaction.
- [ordinal_costs.md](ordinal_costs.md) — ordinal `(ordinal, real)` vs. CBOUND slope for
  connectivity-arc cost; precision / preprocessing / footprint tradeoffs.
- [potential_updates.md](potential_updates.md) — O(|frontier|) Johnson potential update via early
  termination; incremental `ReducedCost` companion.
- [capacity_ranges.md](capacity_ranges.md) — native `[lb, ub]` flow ranges in `fragile_mccf`;
  drops the synthetic-arc reduction for `lb ≤ 0`.
- [large_graph_optimization.md](large_graph_optimization.md) — lazy evaluation of edgeless roads:
  O(n) instead of O(|E|) on sparse-pin instances.
- [ssp.md](ssp.md) — successive shortest paths as an alternative to capacity scaling (esp. the
  incremental / web-app case).
- [webapp.md](webapp.md) — interactive Leaflet road-matching web app.
