# Matching Algorithm Profile Report

**Date:** 2026-04-18  
**Platform:** darwin (Apple Silicon assumed)  
**Python:** 3.13  
**Network:** 10×10 grid, unit-length edges (180 edges, 100 vertices)  
**Solver:** `cpp_fragile` (`CppMinConvexCostFlow` → `_cpp.fragile_mccf`)

---

## Timing sweep (3 repeats each)

| n | py | cpp_dijkstra | cpp_fragile |
|--:|--:|--:|--:|
| 10 | 0.0248 | 0.0248 | 0.0088 |
| 25 | 0.0365 | 0.0386 | 0.0108 |
| 50 | 0.0557 | 0.0508 | 0.0129 |
| 100 | 0.0740 | 0.0747 | 0.0159 |
| 200 | 0.1072 | 0.0965 | 0.0211 |

cpp_fragile at n=100: **16ms** (was 31ms — 2× improvement from eliminating bintrees in MEASURE and OBJECTIVE).

---

## cProfile (n=100, 0.033s with cProfile overhead)

```
         95976 function calls (95614 primitive calls) in 0.033 seconds

   Ordered by: cumulative time

   ncalls  tottime  percall  cumtime  percall filename:lineno(function)
        1    0.000    0.000    0.033    0.033 profile_matching.py:68(run_once)
        1    0.000    0.000    0.031    0.031 bm.py:111(compute_optimal_results)
        1    0.000    0.000    0.028    0.028 bm.py:141(_compute_optimal_flow)
        1    0.000    0.000    0.023    0.023 bm.py:415(compute_optimal_flow)
        1    0.000    0.000    0.016    0.016 cvxcostflow.py:508(CppMinConvexCostFlow)
        1    0.000    0.000    0.015    0.015 cvxcostflow.py:467(cpp_fragile_mccf)
        1    0.008    0.008    0.014    0.014 {built-in method _cpp.fragile_mccf}
```
> C++ solver accounts for 8ms tottime + 6ms Python callback overhead = 14ms cumtime.

```
     6892    0.004    0.000    0.006    0.000 pwl.py:69(IntPWL.__call__)    # NEW bottleneck
      180    0.000    0.000    0.004    0.000 bm.py:502(OBJECTIVE_FUNC)
      180    0.001    0.000    0.004    0.000 bm.py:507(OBJECTIVE)
      181    0.001    0.000    0.001    0.000 {built-in method __build_class__}  # inline subclass
```
> `IntPWL.__call__` is called 6892 times (4ms) — the C++ solver calls back into Python for every
> cost evaluation. This is the main remaining Python bottleneck.
> `__build_class__` fires 181 times: the `_IntPWLWithLines` subclass is re-defined inside
> `OBJECTIVE` once per road (180 roads). Easy fix: hoist it out.

```
      400    0.001    0.000    0.002    0.000 bintrees/rbtree.py:123(insert)   # sort_points only
      200    0.000    0.000    0.002    0.000 bintrees/abctree.py:371(set_default)
      200    0.000    0.000    0.001    0.000 bintrees/abctree.py:317(__setitem__)
```
> bintrees is down to 400 inserts (was 1646) — only `sort_points` remains.

```
        1    0.000    0.000    0.004    0.004 bm.py:245(compute_segments2)
        1    0.000    0.000    0.003    0.003 bm.py:268(sort_points)
        1    0.000    0.000    0.003    0.003 bm.py:175(_compute_matching)
        1    0.001    0.001    0.002    0.002 bm.py:792(TRAVERSE2)
        1    0.000    0.000    0.001    0.001 profile_matching.py:58(make_instance)
```
> Everything else is small. `make_instance` is 1ms — sampling is free.

### Summary

| Phase | Time |
|---|---:|
| Benchmark setup (`make_instance`) | 1ms |
| `OBJECTIVE` + `MEASURE` (DoubleEndedVector + IntPWL) | ~4ms |
| `IntPWL.__call__` callbacks from C++ solver | ~6ms |
| C++ `fragile_mccf` solver (pure C++ time) | ~8ms |
| Everything else (sort, marshal, topograph, matching) | ~5ms |

**Next opportunity:** wire `IntPWL` directly to C++ as a `CppPiecewiseLinear` so the solver
evaluates cost in C++ without Python callbacks. This would eliminate the 6ms callback overhead.
Also hoist `_IntPWLWithLines` out of `OBJECTIVE` to avoid 180 `__build_class__` calls.

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
(Scaling law measured before bintrees removal; re-run pending.)
