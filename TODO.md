# TODO

---

## Housekeeping (pre-merge)

- [ ] Purge `polyglot-monorepo.md` and `orchestrator-design.md` from `mccf-sparse` history
  before promoting to `main`. They were moved to their own repo and deleted in commit `f37c29b`,
  but remain in earlier history — rewrite with `git filter-repo` (or equivalent) to evict.

---

## C++ backend & multi-language distribution

Design: [design.md](design.md) — four-layer architecture, `FlowSolver` plugin seam, C/C++/Python
interface surfaces, distribution, and design decisions.

### Implementation order
- [x] **Step 0: Profile** — `FragileMCCF` dominates (75%); `Dijkstra` loop (19%), `ReducedCost` +
  `LinearizeCost` (29% combined), `priodict` (11%), `bintrees.floor_item` (12%). `TRAVERSE2/3`
  essentially free at practical n. See `bench/profile_report.md`.
- [x] **Step 1: Python refactor** — `FlowSolver` and `ConvexFlowSolver` protocols in
  `matching/protocol.py`; `flow_solver` injected into `RoadnetMatchingProblem` and
  `compute_optimal_flow`; module-level `default_flow_solver` as single swap point.
  All 24 tests pass.
- [x] **Step 2: C++ core data structures** — priority queue with decrease-key via lazy deletion; `PriorityDict` binding as drop-in for `priorityDictionary`
  - [x] `cpp/` source tree and `roadgeometry::roadgeometry` CMake interface target
  - [x] `setiptah-roadgeometry-cpp` package with scikit-build-core + pybind11
  - [x] `PriorityQueue<Key, Priority>` header-only template (`cpp/include/roadgeometry/priority_queue.hpp`)
  - [x] pybind11 binding: `_cpp.PriorityQueue` (int keys, double priorities)
  - [x] Test suite: parity tests vs `priorityDictionary`, Dijkstra parity, unit tests
  - [x] C++ coverage via gcovr in `nox test` session
  - [x] Dijkstra using `PriorityQueue` bound and wired as default in `mygraph.Dijkstra`
- [x] **Step 2.5: C++ Dijkstra in `FragileMCCF`** — flat int-indexed arrays, node normalization
  pre-computed once per solve, `_normalize_graph`/`_denormalize_dijkstra` helpers;
  `bench` session now installs cpp packages. Profile hotspots shift to `LinearizeCost` +
  `ReducedCost` (combined ~37%) and `bintrees.floor_item` (13%); Dijkstra no longer dominates.
- [ ] **Step 3: `ConvexCostFlowSolver<Graph, PriorityQueue>`** — C++ template, bound via
  pybind11 as default `<int,int>` instantiation, validated against Python impl with existing tests
  - [ ] Sorted container (`std::map`) replacing `bintrees.RBTree`
  - [ ] `IntRoadnet` mirror and augmentation graph
- [ ] **Step 4: wire into matching** — Python `RoadnetMatchingProblem` uses C++ solver by
  default when available; pure-Python fallback
- [ ] **Step 5: C interface** — `libroadgeometry` with opaque handles, built alongside pybind11 module
- [ ] **Step 6: packaging** — `scikit-build-core` build, PyPI wheel, CMake install rules for headers + C lib
- [ ] **Examples & tests per language** — at minimum one usage example and one correctness/
  performance test in each target language:
  - Python: already covered by existing test suite + `bench/profile_matching.py`
  - C++: standalone example using the header-only library; demonstrates default instantiation
    and at least one custom template composition
  - C: example using the `libroadgeometry` opaque handle API
- [ ] **Post-1.0: C interface customization** — function-pointer vtables for swapping solver at C level
- [ ] **Post-1.0: package managers** — Conan and/or vcpkg recipes

---

## Native `[lb, ub]` capacity ranges in `fragile_mccf`

Design: [plans/capacity_ranges.md](plans/capacity_ranges.md).

- [ ] Add optional `lb` map (default `lb[e]=0`) to `fragile_mccf` / `fragile_mccf_sparse`;
  residual condition becomes `x - lb[e] >= D`.
- [ ] Simplify `build_flow_reduction` for `lb ≤ 0` arcs (drop synthetic arcs / supply shifts /
  result re-translation for bidirectional + non-positive-min oneway roads).

---

## Implicit connectivity + ordinal costs (replace `RobustInputGraph`)

Design: [plans/connectivity.md](plans/connectivity.md) — connectivity topology × materialization
policies (`HamCycle`/`Direct`/`None` × `Explicit`/`Implicit`), lazy loans, loan-count analysis,
potential-update interaction; and [plans/ordinal_costs.md](plans/ordinal_costs.md) — ordinal vs.
CBOUND cost representation. Defaults stay `HamCycle` × `Explicit` + CBOUND.

Removal checklist once a non-`HamCycle` policy lands:
- [ ] Expose `ConnectivityPolicy` and `ConnectivityCostPolicy` as swappable params (option
  matrix in the design docs).
- [ ] Remove / thin `RobustInputGraph`, `RobustCapacity`, `RobustCost`, `robust_mccf` (reduce to a
  wrapper that passes `t` to the solver).
- [ ] Drop node-ordering bookkeeping (`node_order_`, `node_index_`); residual graph shrinks by n arcs.
- [ ] Under `Direct`/`None`: guard the potential update to settled nodes; handle the disconnected
  loan-splice (see connectivity.md).

---

## Testing infrastructure

Design: [testing.md](testing.md). Land before Solver efficiency — its changes are the
answer-preserving ones the differential + certificate harness guards.

- [ ] Native C++ test target (doctest + CTest) in `cpp/CMakeLists.txt`, buildable without the
  pybind11 module.
- [ ] Framework-free test machinery: seeded instance generator, optimality-certificate oracle,
  parity harness, flow/cost comparators (return plain results; no framework dependency).
- [ ] Portable `for_each_policy<Test>()` typelist helper for the policy matrix.
- [ ] Seed suite: one certificate check + one policy-parity test.
- [ ] `nox`/Make target for the fast `cmake --build && ctest` loop; keep pytest as acceptance.

---

## Solver efficiency

Design: [plans/potential_updates.md](plans/potential_updates.md),
[plans/large_graph_optimization.md](plans/large_graph_optimization.md).

- [ ] Incremental `ReducedCost` — recompute only arcs incident to moved potentials (profiled
  hotspot; the evidence-backed win).
- [ ] Lazy edgeless roads — O(n) instead of O(|E|) on sparse-pin instances.
- [ ] Early-termination + frontier potential update — needs the `dist[t]` clamp; minor on its own,
  pairs with incremental `ReducedCost`.

---

## Alternative solver: SSP

Design: [plans/ssp.md](plans/ssp.md).

- [ ] Successive shortest paths as an alternative `FlowSolver`, for the incremental / web-app path.
