# Matching Algorithm Profile Report

**Date:** 2026-04-12  
**Platform:** darwin (Apple Silicon assumed)  
**Python:** 3.13  
**Network:** 10×10 grid, unit-length edges (180 edges, 100 vertices)  
**Solver:** `cpp_fragile` (`CppMinConvexCostFlow` → `_cpp.fragile_mccf`)

---

## cProfile (n=100, 0.031s total)

```
         82446 function calls (82444 primitive calls) in 0.031 seconds

   Ordered by: cumulative time

   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
        1    0.000    0.000    0.031    0.031 profile_matching.py:68(run_once)
        1    0.000    0.000    0.030    0.030 bm.py:110(compute_optimal_results)
        1    0.000    0.000    0.027    0.027 bm.py:140(_compute_optimal_flow)
        1    0.000    0.000    0.018    0.018 bm.py:405(compute_optimal_flow)
```
> `make_instance` takes 2ms — sampling is essentially free after vectorization.
> All remaining cost is algorithm cost.

```
        1    0.000    0.000    0.008    0.008 cvxcostflow.py:508(CppMinConvexCostFlow)
      180    0.001    0.000    0.007    0.000 bm.py:497(OBJECTIVE)
        1    0.000    0.000    0.006    0.006 cvxcostflow.py:467(cpp_fragile_mccf)
     1646    0.004    0.000    0.006    0.000 bintrees/rbtree.py:123(insert)          # OBJECTIVE
        1    0.005    0.005    0.005    0.005 {built-in method _cpp.fragile_mccf}
```
> `bintrees.insert` (6ms, 1646 calls) and the C++ solver (5ms) are neck-and-neck.
> `OBJECTIVE` builds one RBTree per road (180 roads) — these become the PWL cost functions.

```
        1    0.000    0.000    0.005    0.005 bm.py:223(compute_segments2)
        1    0.000    0.000    0.004    0.004 bm.py:246(sort_points)
      580    0.000    0.000    0.003    0.000 bintrees/abctree.py:371(set_default)    # MEASURE
      180    0.001    0.000    0.003    0.000 bm.py:286(MEASURE)
     2473    0.001    0.000    0.003    0.000 bintrees/abctree.py:819(_iter_items_forward)
        1    0.000    0.000    0.003    0.003 bm.py:700(compute_matching_for_acyclic_flow)
     2473    0.002    0.000    0.002    0.000 bintrees/abctree.py:829(_iter_items)
      580    0.000    0.000    0.002    0.000 bintrees/abctree.py:317(__setitem__)     # MEASURE
        1    0.001    0.001    0.002    0.002 bm.py:737(TRAVERSE2)
        1    0.000    0.000    0.002    0.002 profile_matching.py:58(make_instance)
```
> Every remaining Python algorithm cost is bintrees: construction (`insert`, `set_default`,
> `__setitem__`) and iteration (`_iter_items_forward`, `_iter_items`).
> `OBJECTIVE` builds cost function trees; `MEASURE` builds measure trees.
> Both are pre-processing — the data is written once, then read sequentially.

### Summary

| Phase | Time |
|---|---:|
| Benchmark setup (`make_instance`) | 2ms |
| `OBJECTIVE` + `MEASURE` bintrees construction | ~9ms |
| bintrees iteration (`_pwl_from_objective`, matching) | ~3ms |
| C++ `fragile_mccf` solver | 5ms |
| Everything else (sort, marshal, topograph) | ~12ms |

The biggest opportunity is `OBJECTIVE`: `sort_points` already produces sorted data per road,
so `OBJECTIVE` could build the `PiecewiseLinear` directly from that output rather than first
inserting into an RBTree and then iterating it. The intermediate RBTree construction is pure
overhead.

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

Runtime has the structure `T(n) ≈ C_flow(n) + C_sort × n × log(n)`:

- `C_sort × n × log(n)` — inserting 2n points into the sorted map (`sort_points`). O(n log n)
  is the right complexity and the implementation is not a concern. Will eventually dominate at
  very large n on a fixed graph, but has not crossed over at n=10000.
- `C_flow(n)` — grows sub-linearly because the network saturates at high point density:
  many points co-locate on the same roads and are pre-matched, shrinking the effective problem.
  Dijkstra call counts confirm this: 82 at n=200, 124 at n=500 (ratio 1.51 vs n-ratio 2.5).

In the practical regime, **`C_flow(n)` dominates**. Scaling numbers above are from the pure
Python solver and will shift downward with the C++ backend — worth re-running.
