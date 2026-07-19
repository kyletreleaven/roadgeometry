# roadgeometry — Architecture & Distribution Design

## Summary

Product/interface design for shipping roadgeometry as a pluggable, C++-backed library accessible
to Python users of all levels, C purists, and C++ power developers. This doc is the **architecture
and distribution surface**; the **algorithmic substance** of the solver — connectivity/cost
policies, potential-update strategy, capacity ranges, SSP vs. capacity scaling — lives in
[plans/](plans/) and becomes the behavior/template parameters of the C++ solver (Step 3).
Implementation status and the step checklist live in [TODO.md](TODO.md).

Naming/placement of this doc is provisional (may become `architecture.md` / `docs/`), and it can
grow to hold other product/interface decisions beyond the C++ backend.

---

## Goal

Refactor the Python code to support pluggable implementations of core algorithms and data
structures, re-backed by C++ by default, while remaining accessible to Python users of all
levels, C purists, and C++ power developers.

## Architecture: four layers

```
Layer 4: Python public API          — "just works", no configuration needed
Layer 3: Python Protocol interfaces — Python power users inject custom impls
Layer 2: C interface (extern "C")   — C purists, FFI, other language bindings
Layer 1: C++ header-only templates  — C++ power users, maximum composability
```

pybind11 binds **directly** to C++ templates (not via the C layer) to avoid double-wrapping.
The C interface is a sibling compiled artifact, not a dependency of the Python bindings.

## Plugin seams (Protocol interfaces)

One primary seam at the Python level:

| Protocol | Python impl | C++ template |
|---|---|---|
| `FlowSolver` | `MinConvexCostFlow` | `ConvexCostFlowSolver<Graph, PriorityQueue>` |

Two variants of `FlowSolver`:
- **Oracle-style** — black box callable, any solver can satisfy it
- **Piecewise-linear-aware** — receives cost structure directly, can exploit convexity

Priority queue and augmentation graph are template parameters of the C++ solver. Not
exposed at the Python protocol level initially, but pybind11 bindings could expose named
instantiations to Python power users post-1.0. (SSP is an alternative `FlowSolver` — see
[plans/ssp.md](plans/ssp.md); connectivity and cost policies are further sub-seams of the
solver — see [plans/connectivity.md](plans/connectivity.md) and
[plans/ordinal_costs.md](plans/ordinal_costs.md).)

## C++ template library (Layer 1)
- Header-only, self-contained, no compilation required to use as a C++ library
- Distributed as a CMake `FetchContent`-compatible tarball / GitHub release
- Supports `find_package(roadgeometry)` via installed `CMakeLists.txt`
- C++ standard: C++17 minimum; C++20 concepts for constraint expressions if feasible
- Use templates (not pointer-to-implementation / virtual dispatch) for zero-overhead composability

## C interface (Layer 2)
- `extern "C"` shared library (`libroadgeometry.so/.dylib/.dll`)
- Opaque handles + free functions (e.g. `rg_matcher_create`, `rg_matcher_solve`, `rg_matcher_destroy`)
- Default instantiation exposed out of the box; function-pointer vtable hooks for customization (post-1.0)
- Stable ABI — enables bindings from Rust (bindgen), Julia (ccall), R, Go (cgo)
- Built as a CMake target alongside the pybind11 module (essentially free, same compilation)

## Python bindings (Layer 3)
- pybind11 for binding C++ template instantiations
- Python Protocol types (`DistanceOracle`, `FlowSolver`, `MatchingSolver`) accept either
  C++-backed objects or pure-Python objects satisfying the same structural interface
- Auto-detection with graceful fallback:
  ```python
  try:
      from setiptah.roadgeometry._cpp import DijkstraOracle, RoadnetMatcher
  except ImportError:
      from setiptah.roadgeometry._python import DijkstraOracle, RoadnetMatcher
  ```

## Python public API (Layer 4)
- Existing call signatures preserved — no breaking changes for existing users
- C++ backend selected by default when available; pure-Python always a valid fallback

## Distribution
| Artifact | Format | Channel |
|---|---|---|
| C++ header-only library | tarball / CMake FetchContent | GitHub releases, eventually Conan/vcpkg |
| C shared library | compiled binary + header | same as above |
| Python package | wheel | PyPI (`pip install`) |

PyPI / Python wheel is the 1.0 priority. Conan/vcpkg packaging deferred until demand.

## Build system
- `scikit-build-core` + `CMakeLists.txt`
- C library and pybind11 module as separate CMake targets, built together
- C++ coverage from Python test suite: compile with `--coverage`, run `pytest`, collect with `lcov`
- Same Python test suite validates both pure-Python and C++ backends

## Design decisions

- **`FlowSolver` is the primary plugin seam** — not `DistanceOracle` or `MatchingSolver`.
  `RoadnetMetric.distance()` is not in the hot path (only used for post-hoc cost verification).
  `MatchingSolver` as a protocol adds abstraction without motivation — the interesting variation
  is in the solver, not the overall algorithm structure.

- **Two `FlowSolver` variants** — oracle-style (black box callable, any solver) and
  piecewise-linear-aware (receives `RBTree` of `LineData` directly, can exploit convex
  structure). Current `MinConvexCostFlow` is the canonical piecewise-linear implementation.

- **Priority queue and augmentation graph are sub-seams of `FlowSolver`** — not exposed at
  the Python protocol level. In C++ they become template parameters of
  `ConvexCostFlowSolver<Graph, PriorityQueue>`, visible to C++ power users but invisible
  at the Python boundary.

- **Python always normalizes to `<int, int>` before dispatch** — `IntRoadnet.normalize()`
  is O(n) and strictly dominated by the O(n log n) solve; amortized to zero for repeated
  solves on the same network. The pybind11 binding exposes only the `<int, int>`
  instantiation; no runtime dispatch shim needed.

- **C++ template stays fully general** — `ConvexCostFlowSolver<RoadId, VertId, Graph,
  PriorityQueue>` works with any ID types. C++ devs with non-integer IDs use the template
  directly without normalizing. The `<int, int>` instantiation is the canonical fast default
  (packed arrays, cache-friendly) but not the only valid one.

- **No normalization utilities in the C/C++ library** — C++ devs write their own
  `std::unordered_map<T, int>` index if they want it. Python normalization machinery
  (`IntRoadnet.normalize()`) covers the primary audience. Revisit post-1.0 if there is
  demand from C users.
