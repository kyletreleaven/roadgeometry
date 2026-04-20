# Matching Algorithm Profile Report

**Date:** 2026-04-19  
**Platform:** darwin (Apple Silicon assumed)  
**Python:** 3.13  
**Network:** 10×10 grid, unit-length edges (180 edges, 100 vertices)  
**Solver:** `cpp_fragile` (`CppMinConvexCostFlow` → `_cpp.fragile_mccf`)

---

## Timing sweep (3 repeats each)

| n | py | cpp_dijkstra | cpp_fragile |
|--:|--:|--:|--:|
| 10 | 0.0285 | 0.0318 | 0.0097 |
| 25 | 0.0418 | 0.0463 | 0.0109 |
| 50 | 0.0966 | 0.0982 | 0.0276 |
| 100 | 0.1261 | 0.1186 | 0.0122 |
| 200 | 0.1096 | 0.0973 | 0.0158 |

cpp_fragile at n=100: **12ms** (was 16ms — `sort_points2` batch sort replaces bintrees, 28× faster at n=10000).

---

## cProfile (n=10000, cpp_fragile, 0.43s total)

```
         1250769 function calls in 0.425 seconds

   Ordered by: cumulative time

   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
        1    0.000    0.000    0.246    0.246 bm.py:175(_compute_matching)
        1    0.000    0.000    0.246    0.246 bm.py:779(compute_matching_for_acyclic_flow)
        1    0.045    0.045    0.152    0.152 bm.py:816(TRAVERSE2)
        1    0.001    0.001    0.165    0.165 bm.py:141(_compute_optimal_flow)
        1    0.043    0.043    0.094    0.094 bm.py:262(compute_segments2)
        1    0.016    0.016    0.094    0.094 bm.py:711(create_topograph)
    20180    0.006    0.000    0.074    0.000 bm.py:717(add_edge)
    18842    0.059    0.000    0.068    0.000 networkx/classes/digraph.py:677(add_edge)
```
> `TRAVERSE2` + networkx topograph dominates at 246ms.
> `compute_segments2` groupby is 94ms (43ms Python loop + BiPartite creation).

```
        1    0.000    0.000    0.021    0.021 bm.py:248(sort_points2)   # was 567ms with bintrees
      180    0.009    0.000    0.022    0.022 bm.py:330(MEASURE)
    20000    0.012    0.000    0.013    0.000 bm.py:205(create_with)    # BiPartite creation
      180    0.008    0.000    0.012    0.012 bm.py:313(PREMATCH)
        1    0.015    0.015    0.016    0.016 {built-in method _cpp.fragile_mccf}
```
> `sort_points2` dropped from 567ms to 21ms — 27× improvement from batch sort.
> C++ solver (16ms) is now cheaper than MEASURE (22ms) and PREMATCH (12ms).

### Summary (n=10000)

| Phase | Time |
|---|---:|
| `sort_points2` (batch sort) | 21ms |
| `compute_segments2` groupby + BiPartite creation | 94ms |
| PREMATCH + SURPLUS + MEASURE + OBJECTIVE | ~60ms |
| C++ `fragile_mccf` solver | ~16ms |
| `TRAVERSE2` + networkx topograph | ~246ms |

**On flow computation alone:** `_compute_optimal_flow` takes 165ms, of which the C++ solver
is only 16ms. The remaining ~150ms is entirely instance translation — `compute_segments2` +
PREMATCH + MEASURE + OBJECTIVE. The solver is essentially free; Python overhead building its
input is the whole cost.

**Next bottleneck (end-to-end):** `TRAVERSE2` + networkx topograph (246ms) — pure Python/networkx overhead.
`compute_segments2` groupby (94ms) will be eliminated by C++ port of sort+segment.

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
(Scaling law measured before bintrees/sort_points2 optimisations; re-run pending.)
