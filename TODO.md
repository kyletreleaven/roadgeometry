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
  - [ ] Convert `fragile_mccf_state` to consume an `Instance` (single arg) instead of the
    unpacked `network/capacity/supply/cost/U/epsilon/lb` params — the accessors (`network()`,
    `cost(e)`, `ub(e)`, `lb(e)`, `supply(n)`) already model this. This is the unification seam:
    once dense is Instance-based, dense and sparse differ only by residual-maintenance strategy.
    Keep the current unpacked wrapper as a shim so existing callers/binding are untouched.
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

- [x] Native C++ test target (doctest + CTest) in `cpp/CMakeLists.txt`, buildable without the
  pybind11 module (`cpp/tests/`, vendored doctest, guarded by `ROADGEOMETRY_BUILD_TESTS`).
- [x] Fast-loop script `scripts/cpptest.sh` (configure→build→ctest). `nox` wrapper optional.
- [ ] Framework-free machinery: feasibility oracle + cost comparator done (`cpp/tests/support.hpp`);
  still need a seeded instance generator, a full optimality-certificate oracle (potentials /
  negative-cycle), and a parity harness.
- [ ] Portable `for_each_policy<Test>()` typelist helper for the policy matrix.
- [ ] Grow the suite: certificate check + policy-parity test (have: feasibility+optimum and
  `lb < 0` reverse-flow behavior).

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
