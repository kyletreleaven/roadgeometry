# Connectivity Policy for `fragile_mccf`

## Summary

`fragile_mccf` needs, at every augmentation step, an `s⇝t` path in the residual graph
while excess remains. Today this is guaranteed by `RobustInputGraph`, which adds a
Hamiltonian cycle over all n nodes — making *every* Δ-residual graph strongly connected.
That is a **stronger guarantee than the algorithm needs** (it needs `s⇝t` for the
current pair, not all-pairs) and it costs O(n) stored arcs plus O(n) processing per
Dijkstra.

The design space factors into **two axes** plus an escape hatch:

- **Topology** — *what* structure guarantees reachability: `HamCycle` vs. `Direct s→t`.
- **Materialization** — *how* it is represented: `Explicit` (arcs stored) vs. `Implicit`
  (synthesized inside Dijkstra).
- **`None`** — no structure added; the caller guarantees connectivity.

The key is that these two axes govern **different cost buckets** (see next section):
materialization affects only the *idle* footprint, while topology affects how flow is
*directed* — the algorithmically interesting difference. The *cost* of a connectivity arc
(how prohibitive it is) is a further **separate** policy — CBOUND vs. ordinal — analyzed in
[ordinal_costs.md](ordinal_costs.md). Any topology × materialization × cost-representation
is valid.

---

## Background: what connectivity is for

Stage 2 of `FragileMCCF` (`cvxcostflow.py:352-415`) repeatedly picks a surplus node `s`
(excess ≥ Δ) and a deficit node `t` (excess ≤ −Δ), runs Dijkstra from `s` on reduced
costs, reconstructs the `s→t` path by walking `upstream[]` back from `t`, and augments Δ
along it.

If no `s→t` path exists, `upstream[t]` is missing — the reconstruction fails and the
excess can never be cleared. So the precondition is precisely: **while surplus/deficit
remain, the chosen `s` can reach the chosen `t` in the residual graph.**

Strong connectivity (the Hamiltonian cycle) is *sufficient* but stronger than necessary.
`Direct s→t` supplies exactly the one reachability the current step requires.

---

## Two cost buckets: representation vs. flow direction

The two axes are worth separating because they price **different things**, and only one of
them touches what the algorithm actually does.

**Representation cost (materialization axis).** How much memory the topology's *idle*
arcs occupy. Crucially, **only zero flow can be implicit**: an "implicit" arc is free only
while it carries no flow. The moment a connectivity arc is *used* — a loan is taken — that
flow (the debt) must be stored somewhere, so it materializes regardless of policy. Implicit
storage is therefore not zero but "one entry per connectivity arc currently under load."
Materialization changes the idle footprint only; it cannot change the active footprint.

**Flow-direction cost (topology axis).** How the topology routes flow once connectivity is
invoked: loan granularity, debt magnitude, and the repayment work that debt forces at finer
Δ scales. This is intrinsic to `HamCycle` vs. `Direct` and independent of how the arcs are
stored. It is the bucket that changes the algorithm's *behavior*, not just its memory — and
the more interesting of the two.

---

## Axis 1 — topology: how flow is directed (the interesting axis)

| | `HamCycle` | `Direct s→t` |
|---|---|---|
| Guarantee | strong connectivity (any `s⇝t`) | just the current `s⇝t` |
| Loan granularity | **fine** — path uses real arcs where available, a cycle arc only where needed → smaller debt | **coarse** — full loan on one arc, bypasses available real capacity |
| Debt per disconnection event | spread over O(n/2) arcs (each gets Δ) | Δ on one arc |
| Repayment work at finer scales | more (proportional to debt) | O(n) less total debt |

The tradeoff is over-provisioning vs. targeting: `HamCycle` pays O(n) to guarantee more
than needed, but the surplus reachability is what lets it borrow in small pieces.
`Direct` borrows the whole amount at once but only where a step actually needs it — a win
when disconnection is **rare** (well-connected road networks), which is the expected case.
Whether the O(n)-less-debt property yields a formal asymptotic improvement is open.

---

## Axis 2 — materialization

| | `Explicit` | `Implicit` |
|---|---|---|
| Representation | idle connectivity arcs stored in the residual graph | synthesized on the fly inside Dijkstra relaxation |
| Dijkstra | generic (no policy awareness) | must know the policy (a relaxation hook) |
| Idle storage — cycle | O(n) arcs | O(n) still (virtual successors), unless tricks |
| Idle storage — direct | **O(n²) all-pairs** — dominated, never worth paying | **0** — synthesize known `s→t` |
| Bookkeeping | node ordering (`node_order_`, `node_index_`) for cycle | none for direct; cycle still needs a `next()` ordering |

The subtlety is that the *explicit* representation of `Direct` is **not** one arc — since
any `(s,t)` may arise and a static store can't predict which, materializing `Direct`
explicitly means the entire **all-pairs** set, O(n²). The Hamiltonian cycle is precisely
the trick that compresses that all-pairs guarantee to O(n) stored arcs (at the price of
multi-hop paths, which is also where its finer loan granularity comes from). `Direct` avoids
the blowup the other way: store nothing and synthesize the single needed arc at call time,
because `t` is known before each Dijkstra — O(0).

So the sensible cells collapse: `Direct` × `Explicit` is a non-option (O(n²) for no benefit);
`Direct` × `Implicit` is the sweet spot (O(0)). For `HamCycle`, `Implicit` saves the arc
*storage* but not the O(n) *relaxation* — each node still has a virtual cycle successor to
relax — unless combined with lazy relaxation (consider connectivity only if real arcs fail to
reach `t`), which is more intricate for a cycle than for a single direct arc.

---

## `None`

No connectivity structure, zero overhead. Correct **only if** the caller guarantees the
residual graph already provides `s⇝t` for every step (e.g. the input is and stays strongly
connected under residuals). Risky: if the guarantee is wrong, augmentation silently fails
(missing `upstream[t]`). Appropriate for power users who *know* their instance; never a
safe default.

---

## Cross product

| Policy | Idle storage | Per-Dijkstra | Loan | Reachability guaranteed |
|---|---|---|---|---|
| `None` | 0 | none | — | caller's responsibility |
| `HamCycle` × `Explicit` **(current)** | O(n) arcs + ordering | O(n) relax | fine | strong (all pairs) |
| `HamCycle` × `Implicit` | ordering only | O(n) relax (lazy: less) | fine | strong (all pairs) |
| `Direct` × `Explicit` | O(n²) all-pairs — **dominated** | O(1) | coarse | current `s⇝t` |
| `Direct` × `Implicit` | **0** | O(1) hook | coarse | current `s⇝t` |

Storage here is *idle* footprint only; an arc carrying flow is stored regardless of policy
(only zero flow can be implicit — see "Two cost buckets" above).

---

## Interaction: the potential-update reachability caveat

Stage 2 updates potentials with a **blanket** loop (`cvxcostflow.py:410-412`):

```python
# by connectivity, should touch *every* node
for i in network.nodes() : potential[i] -= dist[i]
```

This is a Johnson reweighting and only makes sense for nodes Dijkstra actually settled.
Under `HamCycle` every node is reachable from `s`, so every `dist[i]` is finite and the
blanket loop is correct — the code comment is literally relying on the cycle.

Under `Direct` / `None` the residual graph is not strongly connected, so unreached nodes
have infinite/absent `dist`. The loop must be guarded:

```python
for i in network.nodes():
    if dist[i] < inf:            # settled this round
        potential[i] -= dist[i]
    # else: leave potential[i] unchanged
```

This is the standard successive-shortest-paths treatment (cf. [ssp.md](ssp.md)). The
choice of Axis 1 therefore dictates whether line 412 needs the guard — an
**implementation coupling** to record, independent of the cost-representation question.
(The path reconstruction at `upstream[t]` likewise assumes `t` is reached: `Direct`
guarantees `t` specifically; `None` guarantees nothing.)

---

## Open questions & analysis

### Cycle vs. direct: two competing debt quantities

When connectivity is invoked, `HamCycle` and `Direct` create *different* debt. The TODO's
"O(n)-less debt" line captured only the **spread**; there is a countervailing **magnitude**:

| | debt spread (# arcs) | debt magnitude (real flow borrowed) |
|---|---|---|
| `Direct` | 1 arc — better | full `s→t`, ignores real capacity partway — worse |
| `HamCycle` | O(n/2) arcs — worse | only the residual gap, real arcs used elsewhere — better |

`Direct` loads all `Δ` onto one arc; `HamCycle` (routing over minimal cycle arcs) uses real arcs
as far as they reach and borrows only across the gap. Which dominates repayment work at finer Δ
is **unresolved** — likely `Direct` wins in practice on well-connected graphs (rare, small
events), but there is no proof either way, and it is the interesting cycle-vs-direct question.

### Escalating prohibitive loan (ordinal only)

A mechanism that gives `Direct` the granularity of `HamCycle` but *only over arcs disconnection
actually touched*: during Dijkstra, price a **new** loan (a connectivity arc not already under
load) at ordinal `(n+1, 0)`, where `n` = loans currently live; an **existing** loan keeps
`(1, 0)`. Committing an augment still uses `(1, 0)`. This enforces **reuse-before-create** —
Dijkstra freely chains real arcs and existing loans, and pays for a new loan only when even that
combined graph cannot reach `t`.

Consequence: the loan set grows lazily into a *demand-tailored cycle*. Early on `Direct` is
coarse (full-jump loans); as loans accumulate, paths string real arcs + existing loans + one new
loan across the gap — the same partial-real-usage granularity `HamCycle` had from the start, but
provisioned only where disconnection occurred. It **interpolates from coarse to cycle-like**.

### SSP risk from search ≠ commit cost — and why it is likely benign

The escalation searches at `(n+1, 0)` but commits at `(1, 0)`, so the potential update
(`cvxcostflow.py:412`) is derived against a different cost than the reduced-cost certificate —
the classic SSP trap (shortest path w.r.t. one cost, accounting at another). Three reasons it is
probably harmless, the last of which removes it outright:

- **Real optimum untouched.** The search/commit difference is `(n, 0)` — **purely ordinal, zero
  real component**. Real potentials, real reduced costs, and the real-cost optimum are provably
  unaffected; the risk is confined to the *ordinal* component of potentials (whether ordinal
  reduced costs stay ≥ 0 the next round).
- **Connectivity flow is transient.** Loans are repaid by `ε` (zero connectivity flow in the
  final answer), so the ordinal bookkeeping never surfaces in the result.
- **Fix — don't propagate until commit.** If the potential update absorbs only the *committed*
  `(1, 0)` cost of arcs actually augmented, the inflated search cost never enters persistent
  state and the trap cannot spring. Caveat: the *current* code does **not** defer — CBOUND
  propagates eagerly into potentials via the blanket update whenever a shortest path crosses a
  cycle arc (the only existing laziness is that the cost lands solely on nodes actually routed
  through connectivity). So commit-time-only propagation would be a new, deliberate structure —
  and the thing to verify is that it preserves ordinal reduced-cost ≥ 0.

### How many loans? — greedy is Θ(n²), not O(cycle)

What is ever **necessary** is small: a Hamiltonian cycle is necessary and sufficient for the
offline optimum (Eswaran–Tarjan: min additions to make a digraph strongly connected =
`max(#sources, #sinks) ≤ n`; n isolated nodes need exactly n). So **one cycle bounds what is
ever needed** — you never *need* two.

What greedy online **actually adds** is not so bounded. The rule — add `s→t` iff `t` is
currently unreachable from `s` — is **Θ(n²)** in the worst case, past *any* constant number of
cycles. The reason is that **directed reachability is far weaker than undirected connectivity**,
which is where the "two cycles suffice" intuition (spanning tree → bidirect → `2(n−1)`) breaks:
that intuition is undirected. Greedy can add an entire **transitive tournament** — all
`n(n−1)/2` forward arcs of a linear order — by inserting them **longest-span first**:

```
add i→j in order of decreasing span (j−i):
  1→n, 1→(n-1), 2→n, …, 1→2, 2→3, …, (n-1)→n
```

Each `i→j` is legal on insertion: every earlier arc has span > (j−i), and no forward path of
larger jumps can land exactly on `j`, so `j` is unreachable from `i` at that moment. That is
`n(n−1)/2` arcs — a DAG, still n SCCs — **before a single merge**. (Undirected, these all
collapse to "connected" after n−1 edges; directed, greedy keeps finding unreachable *ordered*
pairs.) Making it strongly connected then costs up to n−1 more, for ~n²/2 total.

This worst case assumes an *adversarial* demand order; real surplus/deficit sequences are
structured, so practice is likely far better.

**Why "SCCs collapse the problem" fails to bound it.** A tempting argument: each added arc may
form an SCC, after which that component collapses to a single node, so the process telescopes
(pairing off n/2 SCCs per phase, log n phases). It does not hold — an SCC forms only when an arc
points *into* a node that can already reach its tail, and the adversary never does that: forward
arcs of a linear order never close a cycle, so the tournament above reaches Θ(n²) arcs with **no
SCC ever formed**. No collapse, no telescoping. (Even the telescoping strategy itself sums to a
geometric `2n`, not `n log n` — so it is not the worst case in any case.)

**Escalation yields to the cycle in the worst case.** Repayment does *not* rescue this: the
worst case is the genuinely-disconnected regime (every pair forces a new loan), and there is no
real capacity to repay against, so all Θ(n²) loans stay outstanding. Escalation cannot help
either — in the tournament every target is unreachable even using all existing loans, so reuse
is never available and each new loan is forced regardless of its penalty. So:

- The **cycle is worst-case optimal**: exactly n arcs, and the worst case (heavy disconnection)
  is precisely the regime it was designed for — n up front vs. Θ(n²) accumulated lazily, of
  which only n are ever necessary. Structurally, the cycle wins because it *is* one SCC by
  construction and never spends time as a stretched-out DAG.
- The escalating / `Direct` approach is a strictly **average-case** optimization (rare
  disconnection → few loans, O(1) each) with **no worst-case guarantee**.

(This retracts an earlier conjecture that outstanding loans stay ≤ n with repayment — false in
the disconnected worst case.)

---

## Recommendation / defaults

- Keep `HamCycle` × `Explicit` as the **safe default** — current behavior, no algorithm
  change, blanket potential update stays valid.
- `Direct` × `Implicit` is the promising optimization for well-connected road networks:
  zero storage, O(1) per step, wins whenever disconnection is rare. Requires the guarded
  potential update and a Dijkstra that knows `t`.
- Expose topology and materialization as independent swappable template parameters,
  orthogonal to `ConnectivityCostPolicy` (see [ordinal_costs.md](ordinal_costs.md)).
- Constant factors (storage vs. relaxation vs. repayment) resist clean analysis — profile
  the instantiations competitively on real instances.

See TODO.md § "Implicit connectivity + ordinal costs" for the removal checklist
(`RobustInputGraph`, `RobustCapacity`, `RobustCost`, `robust_mccf`).
