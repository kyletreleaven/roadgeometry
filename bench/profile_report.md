# Matching Algorithm Profile Report

**Date:** 2026-04-05  
**Platform:** darwin (Apple Silicon assumed)  
**Python:** 3.13  
**Network:** 10×10 grid, unit-length edges (180 edges, 100 vertices)  
**Raw output:** `profile_report.txt`

---

## Timing Sweep

| n (points/side) | mean (s) | min (s) |
|---:|---:|---:|
| 50 | 0.073 | 0.072 |
| 100 | 0.101 | 0.091 |
| 200 | 0.139 | 0.134 |
| 500 | 0.197 | 0.193 |
| 1000 | 0.270 | 0.264 |

**Observation:** scaling is surprisingly flat — from n=50 to n=1000 (20×) runtime only grows ~3.7×.
This suggests overhead is dominated by graph structure (fixed at 10×10) rather than point count.
Worth re-running on larger or denser networks to get a clearer picture of asymptotic behavior.

---

## cProfile Summary (n=500, 0.51s total)

| Function | cumtime (s) | % total | calls |
|---|---:|---:|---:|
| `FragileMCCF` (cvxcostflow.py) | 0.383 | **75%** | 1 |
| `Dijkstra` (mygraph.py) | 0.097 | 19% | 124 |
| `LinearizeCost` (cvxcostflow.py) | 0.078 | 15% | 1375 |
| `ReducedCost` (cvxcostflow.py) | 0.072 | 14% | 419 |
| `costWrapper.__call__` / `bintrees.floor_item` | 0.063 | 12% | 18784 |
| NetworkX `reportviews` genexpr | 0.053 | 10% | 89038 |
| `priodict.__setitem__` + `smallest` | 0.054 | 11% | ~36k |
| `make_instance` (sampling) | 0.069 | 14% | — |

---

## Analysis

### 1. `MinConvexCostFlow` / `FragileMCCF` — 75% of runtime
The convex cost flow solver is the dominant hotspot by a wide margin. This is the
capacity-scaling algorithm in `cvxcostflow.py`. Everything below is a component of it.

### 2. `Dijkstra` on residual graph — 19%, 124 calls
Called repeatedly inside `FragileMCCF` for shortest-path augmentation. The implementation
is pure Python (`mygraph.py` + `priodict.py`). Clear C++ target: template on graph type
and priority queue type.

### 3. `LinearizeCost` + `ReducedCost` — 29% combined, ~1800 calls
Inner-loop operations of the capacity-scaling algorithm. Called per-edge per-phase.
`ReducedCost` involves `bintrees.floor_item` lookups on piecewise-linear cost functions.

### 4. `bintrees.floor_item` — 12%, 18784 calls
Already a C extension (`bintrees.RBTree`), but still significant. A C++ `std::map::lower_bound`
equivalent should be faster and integrates naturally with the template design.

### 5. NetworkX `reportviews` genexpr — 10%, 89038 calls
Surprising entry. NetworkX graph iteration overhead inside `mygraph.Dijkstra`. The
`mygraph` adjacency representation iterates via NetworkX views — replacing with a plain
dict-of-dicts or C++ adjacency list eliminates this entirely.

### 6. `priodict` — 11%, ~36k calls
Pure Python priority queue. `std::priority_queue` in C++ is an obvious replacement.

### 7. Sampling / `make_instance` — 14%
This is benchmark setup cost, not algorithm cost. Not a target for optimization.
The `roadmap_basic.get_road_data` call (1000 calls, 0.063s) is going through legacy code —
another reason to clean up the sampling path eventually.

---

## C++ Implementation Priority

| Priority | Target | Rationale |
|---|---|---|
| 1 | `FragileMCCF` + `Dijkstra` on residual graph | 75% of runtime; pure Python graph + heap |
| 2 | `ReducedCost` + `LinearizeCost` | 29% combined; tight inner loop of MCCF |
| 3 | Priority queue (`priodict`) | Pure Python heap, called 36k times |
| 4 | `mygraph` adjacency + NetworkX iteration | 10% from view overhead alone |
| 5 | `bintrees.RBTree` → `std::map` | Already C, but integrates with C++ template design |

---

## Scaling Law Analysis

Empirical log-log regression over n=100..10000 on a fixed 10×10 grid:

| n | t (s) | t/t₁₀₀ | O(n log n) | O(n) | O(n^0.43) |
|---:|---:|---:|---:|---:|---:|
| 100 | 0.096 | 1.00 | 1.00 | 1.0 | 1.00 |
| 200 | 0.147 | 1.53 | 2.30 | 2.0 | 1.34 |
| 500 | 0.190 | 1.98 | 6.75 | 5.0 | 1.99 |
| 1000 | 0.254 | 2.65 | 15.00 | 10.0 | 2.67 |
| 2000 | 0.322 | 3.35 | 33.01 | 20.0 | 3.60 |
| 5000 | 0.475 | 4.95 | 92.47 | 50.0 | 5.32 |
| 10000 | 0.806 | 8.40 | 200.00 | 100.0 | 7.15 |

**Empirical exponent: n^0.43** — sub-linear, fitting the data well.

### Interpretation

Runtime has the structure:

```
T(n) ≈ C_flow(n) + C_sort × n × log(n)
```

- `C_sort × n × log(n)` — inserting 2n points into the sorted map (`sort_points`). O(n log n)
  with a small constant. Will eventually dominate at very large n on a fixed graph, but has
  not crossed over yet at n=10000.
- `C_flow(n)` — the flow solver work. Grows sub-linearly in n on a fixed graph because the
  network saturates at high point density: many points co-locate on the same roads and are
  pre-matched before the flow solver runs, shrinking the effective problem. Dijkstra call
  counts confirm this: 82 at n=200, 124 at n=500 (ratio 1.51 vs n-ratio 2.5 → ~n^0.45).

In the practical regime (small to mid-sized n on a fixed network), **`C_flow(n)` dominates**.
The sort term is a red herring for optimization — it will get a free improvement from
replacing `bintrees.RBTree` with a C++ stdlib sorted container, but the real wins are in
drawing down the flow solver terms.

---

## Notes and Caveats

- `TRAVERSE2` / `TRAVERSE3` (matching construction, Phase III) does not appear in the top 30
  — essentially free at this problem size.
- Sampling overhead (~14% at n=500) is benchmark setup cost, not algorithm cost. The legacy
  `roadmap_basic` path used for sampling should be replaced regardless.
- True O(n log n) asymptotic behavior (sort-dominated) would only emerge at very large n on
  a fixed graph — not the regime we care about for optimization.

---

## cProfile — After C++ FragileMCCF + PWL (2026-04-12, n=100, 0.046s total)

**Solver:** `cpp_fragile` (`CppMinConvexCostFlow` → `_cpp.fragile_mccf`)  
**Changes since prior report:**
- `FragileMCCF` replaced by C++ capacity-scaling SSP
- `bintrees.floor_item` cost evaluation replaced by `PiecewiseLinear` (C++ bisect, no Python callback)
- `CppPiecewiseLinear` objects passed directly to the binding — no `std::function` wrapper for normal edges

```
         157128 function calls (157126 primitive calls) in 0.046 seconds

   Ordered by: cumulative time

   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
        1    0.000    0.000    0.046    0.046 profile_matching.py:68(run_once)
        1    0.000    0.000    0.029    0.029 bm.py:110(compute_optimal_results)
        1    0.000    0.000    0.029    0.029 bm.py:127(run)
        1    0.000    0.000    0.026    0.026 bm.py:140(_compute_optimal_flow)
        1    0.000    0.000    0.018    0.018 bm.py:405(compute_optimal_flow)
```
> `compute_optimal_flow` — the top-level algorithm entry point — takes 18ms of the 46ms total.
> The remaining 28ms is split between `make_instance` (setup) and miscellaneous overhead.

```
        1    0.000    0.000    0.017    0.017 profile_matching.py:58(make_instance)
      200    0.000    0.000    0.014    0.000 probability.py:99(sample)
      200    0.002    0.000    0.014    0.000 roadmap_basic.py:79(get_road_data)
    18741    0.008    0.000    0.012    0.000 networkx/reportviews.py:958(<genexpr>)
```
> `make_instance` + sampling accounts for 17ms — **benchmark setup cost, not algorithm cost**.
> The 18k NetworkX `reportviews` genexpr calls happen entirely inside `get_road_data` during
> point sampling, not inside the flow solver.

```
        1    0.000    0.000    0.008    0.008 cvxcostflow.py:508(CppMinConvexCostFlow)
     1832    0.005    0.000    0.007    0.000 bintrees/rbtree.py:123(insert)
        1    0.000    0.000    0.007    0.007 cvxcostflow.py:467(cpp_fragile_mccf)
      180    0.001    0.000    0.007    0.000 bm.py:497(OBJECTIVE)
        1    0.006    0.006    0.006    0.006 {built-in method _cpp.fragile_mccf}
```
> The C++ solver itself (`built-in method _cpp.fragile_mccf`) is **6ms** — down from ~28ms before,
> a ~5× speedup. It no longer dominates.
>
> `bintrees/rbtree.py:insert` — 1832 calls, 7ms — is now the **leading algorithm cost**.
> This is `bm.py:OBJECTIVE` building one RBTree per road (180 roads × ~10 inserts each).
> These trees are the pre-processing step that constructs PWL cost functions before the solver runs.
> Note that `bintrees.floor_item` no longer appears in the profile at all: cost evaluation now
> happens entirely in C++ (`PiecewiseLinear::operator()`), with no Python callbacks.

```
        1    0.000    0.000    0.004    0.004 bm.py:223(compute_segments2)
        1    0.000    0.000    0.004    0.004 bm.py:246(sort_points)
      200    0.000    0.000    0.004    0.000 bm.py:262(ensure_key)
      180    0.001    0.000    0.003    0.000 bm.py:286(MEASURE)
        1    0.000    0.000    0.003    0.003 bm.py:174(_compute_matching)
      580    0.000    0.000    0.003    0.000 bintrees/abctree.py:371(set_default)
     2485    0.001    0.000    0.003    0.000 bintrees/abctree.py:819(_iter_items_forward)
    29126    0.002    0.000    0.002    0.000 {method 'items' of 'dict' objects}
        1    0.001    0.001    0.002    0.002 bm.py:737(TRAVERSE2)
```
> The remaining bintrees calls (`set_default`, `_iter_items_forward`) are from `MEASURE` and
> `compute_matching_for_acyclic_flow` — also pre/post-processing, not the solver.
> `TRAVERSE2` (matching construction, Phase III) is essentially free at 1ms.

### Summary

| Phase | Time | Previously |
|---|---:|---:|
| Benchmark setup (`make_instance`) | 17ms | ~14% |
| `OBJECTIVE` RBTree construction | 7ms | *(hidden inside FragileMCCF cost callbacks)* |
| C++ `fragile_mccf` solver | 6ms | ~28ms (pure Python FragileMCCF) |
| Everything else (sort, matching, marshal) | ~16ms | — |

The solver is no longer the bottleneck. Next target: `OBJECTIVE`/`bintrees.insert` — the RBTree
construction that builds PWL cost functions. Since the tree is built once and then converted to
`PiecewiseLinear` for the solver, it could be replaced with a plain sorted list or built directly
as a C++ object.
