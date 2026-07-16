# Ordinal Connectivity Costs vs. CBOUND

## Summary

Connectivity arcs (the Hamiltonian cycle in `RobustInputGraph`, or an implicit `s→t`
arc) must carry a cost that is used only as a last resort — dominating any path made
of real-cost arcs. Two ways to represent that dominating cost:

- **CBOUND (current):** a large *finite* real slope, `line(CBOUND)` where
  `CBOUND = Σ c(U)` over all edges (`cvxcostflow.py:242`).
- **Ordinal:** lexicographic `(ordinal, real)` pairs; connectivity arcs get `(1, 0.0)`,
  real arcs `(0, c)`.

Both are **correct**. They differ in floating-point precision, per-instance
preprocessing, and per-operation cost. The precision win of ordinal is
instance-dependent, so this is best left a **swappable `ConnectivityCostPolicy`**, not a
forced default. The value type threaded through the solver is a *function of* that
policy.

---

## Background: the dominating cost flows into potentials

The prohibitive cost is not confined to arc costs. It propagates through the whole
solver via the reduced-cost / potential machinery:

```
ReducedCost:  redcost = lincost + potential(j) − potential(i)
Dijkstra:     dist[i]  = Σ redcost along shortest path
update:       potential[i] −= dist[i]
```

So whatever type arc cost has, `lincost`, `redcost`, `dist`, and `potential` all inherit
it — the value type is **forced by the update chain**, not chosen independently. For
CBOUND that type is `double`; for ordinal it is `(int, double)`.

`(ℤ, ℝ)` under componentwise `±` and lexicographic order is a totally ordered abelian
group, so Dijkstra's non-negativity/optimality argument and the potential-update
invariant carry over unchanged — same theorem, richer value type.

---

## Correctness and precision

CBOUND is correct in exact arithmetic: `CBOUND = Σ c(U)` is a sound upper bound on the
cost of any feasible flow, so `line(CBOUND)` lexically dominates real-cost paths exactly
as intended. It is **not** a "hope it's large enough" fudge — it is a principled bound.

The cost is floating-point dynamic range. Folding a CBOUND-scale magnitude into the same
`double` as genuine small real costs consumes mantissa bits, eroding the precision left
for the real component. This bites at fine Δ (near ε), where the real costs being
compared are small relative to CBOUND. It is an **accuracy hazard, not a correctness
one** — and only materializes when CBOUND is large relative to the real costs in play.

Ordinal moves the dominance into a separate integer field, so the real component keeps
full `double` precision regardless of instance scale.

---

## Pros of ordinal

| | Benefit |
|---|---|
| **Exact dominance** | `(1, ·) > (0, ·)` independent of magnitude; no reliance on a computed bound. |
| **Full real precision** | Real component uncontaminated by a large slope — no dynamic-range erosion at fine Δ. |
| **No CBOUND preprocessing** | Skips the O(E) sum over `c(U)` (`cvxcostflow.py:242`) run per instance; the `(1, 0.0)` slope is a constant. |
| **Free diagnostic** | A nonzero ordinal potential flags a node currently reachable from `s` only across a connectivity arc (unreachable via real arcs alone). |

---

## Cons of ordinal

| | Cost | Masked? |
|---|---|---|
| **Comparison** | Up to ~2× per heap op (ordinal first, real only when equal — almost always). | **Mostly.** Heap sift-down is a latency-bound pointer chase; the extra load+compare µops hide in the slack of the next node's cache-latency load. The ordinal is cache-co-located with the real (`{int ord; double real}`, same line), and a lexicographic compare often compiles branchlessly. |
| **Memory footprint** | Every cost / potential / dist / heap-priority value grows 8 → 12 bytes (16 padded). ~1.5–2× on those arrays. | **No.** This is the honest downside. It also *undercuts* the masking above: bigger heap entries mean fewer per cache line and more lines touched per sift — i.e. the compare cost hides in slack, but the footprint cost *is* the slack getting bigger. |
| **Engineering** | Value type becomes an ordered-abelian-group pair (operator overloads) instead of a bare `double`. | N/A — one-time template cost, no runtime effect. |

---

## Instance dependence

The precision advantage only pays off when CBOUND's dynamic-range erosion actually bites:
large CBOUND, fine ε, ill-conditioned real costs. On well-scaled instances with a modest
CBOUND, ordinal buys **nothing** but the footprint/compare tax.

- **Ill-conditioned instances:** clear win (accuracy + no preprocessing).
- **Benign instances:** small net cost (footprint), no accuracy benefit.

This split is exactly why it should be a swappable policy rather than a forced default.

---

## Recommendation

- Keep `ConnectivityCostPolicy` a compile-time choice; the cost/potential **value type is
  a function of the policy** (`double` for CBOUND, `(int, double)` for ordinal), selected
  as a template type parameter — not a runtime branch — so CBOUND compiles to pure
  `double` arithmetic and pays zero ordinal overhead.
- Orthogonal to `ConnectivityPolicy` (Hamiltonian cycle vs. implicit `s→t` vs. none): any
  connectivity mechanism × any cost representation is valid.
- The compare-vs-footprint tradeoff is a constant factor that resists clean analysis.
  Since CBOUND and ordinal are two instantiations of one solver, swapping is a one-line
  change — **profile them competitively on real instances** rather than reasoning about
  cache lines.

See TODO.md § "Implicit connectivity + ordinal costs" for the connectivity-policy side of
this design (Hamiltonian cycle vs. implicit direct arc) and the removal of
`RobustInputGraph`.
