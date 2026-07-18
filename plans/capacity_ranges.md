# Native `[lb, ub]` Capacity Ranges in `fragile_mccf`

## Summary

`fragile_mccf` currently assumes flow lower bound 0, so `build_flow_reduction` encodes
bidirectional and oneway roads with synthetic arcs and supply shifts. Adding an optional
`lb` map (default `lb[e] = 0`) lets the solver represent `[lb, ub]` ranges directly — with a
one-line change to the residual condition — eliminating the synthetic arcs, result
re-translation, and precision-hazardous supply offsets for the common `lb ≤ 0` case. Related:
the connectivity cycle is orthogonal to this ([connectivity.md](connectivity.md)).

---

## Motivation

Currently `build_flow_reduction` converts every bidirectional road into two synthetic arcs
(e_fwd, e_rev) and every oneway road into one arc shifted by `zmin`.  This requires:
1. Re-translation of results: `z = z_fwd − z_rev` (bidirectional) and `flow[e] − zmin[e]`
   (oneway).
2. The prohibitive cost on e_rev to discourage anti-parallel flow (handled correctly but
   adds arcs and cost evaluations).
3. Supply offsets for oneway roads with `zmin > 0`, polluting the supply map and
   introducing floating-point precision hazard from large offsets.
4. `lb = −∞` is impossible with the supply-shift approach — the shift would be infinite.

## Proposed change

Add an optional `lb` map (`Edge → double`, default `lb[e] = 0`) to `fragile_mccf` (and
`fragile_mccf_sparse`).  The only algorithmic change is:

```
// before
if (x >= D)  rgraph.add_edge(bwd, v, u);   // backward residual
// after
if (x - lb[e] >= D)  rgraph.add_edge(bwd, v, u);
```

(The forward check `x + D <= ub[e]` is the symmetric change, already using `capacity`.)

## Impact on `build_flow_reduction`

- Arcs with `lb[e] ≤ 0`: no supply offset, no `pwl_shift`, no re-translation of results.
  Covers bidirectional roads (`lb = −U`) and any oneway road whose minimum flow is ≤ 0.
- Arcs with `lb[e] > 0`: supply shift still required (initial flow `x=0` is infeasible);
  standard pre-flow + supply-adjustment applies.  Road type is immaterial — the lb sign
  is the only thing that matters.
- The Hamiltonian connectivity cycle arcs (`RobustInputGraph`) are unaffected and remain
  necessary to satisfy `fragile_mccf`'s strong-connectivity precondition (see
  [connectivity.md](connectivity.md), which may remove them independently).
- `lb = −∞` is representable (backward arc always available; forward arc only when
  `x + D ≤ ub`).  Not achievable via supply shift.

## Correctness

The capacity-scaling invariants are unchanged: linearized cost and reduced-cost formulas
are evaluated at the current flow `x`, and the residual availability conditions are the
only places `lb` appears.  The argument holds for any `lb[e] ≤ 0`; positive lower bounds
require the standard feasibility pre-flow and are out of scope here.
