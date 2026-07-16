# TODO

---

## C++ backend & multi-language distribution

### Goal
Refactor the Python code to support pluggable implementations of core algorithms and data
structures, re-backed by C++ by default, while remaining accessible to Python users of all
levels, C purists, and C++ power developers.

### Architecture: four layers

```
Layer 4: Python public API          — "just works", no configuration needed
Layer 3: Python Protocol interfaces — Python power users inject custom impls
Layer 2: C interface (extern "C")   — C purists, FFI, other language bindings
Layer 1: C++ header-only templates  — C++ power users, maximum composability
```

pybind11 binds **directly** to C++ templates (not via the C layer) to avoid double-wrapping.
The C interface is a sibling compiled artifact, not a dependency of the Python bindings.

### Plugin seams (Protocol interfaces)

One primary seam at the Python level:

| Protocol | Python impl | C++ template |
|---|---|---|
| `FlowSolver` | `MinConvexCostFlow` | `ConvexCostFlowSolver<Graph, PriorityQueue>` |

Two variants of `FlowSolver`:
- **Oracle-style** — black box callable, any solver can satisfy it
- **Piecewise-linear-aware** — receives cost structure directly, can exploit convexity

Priority queue and augmentation graph are template parameters of the C++ solver. Not
exposed at the Python protocol level initially, but pybind11 bindings could expose named
instantiations to Python power users post-1.0.

### C++ template library (Layer 1)
- Header-only, self-contained, no compilation required to use as a C++ library
- Distributed as a CMake `FetchContent`-compatible tarball / GitHub release
- Supports `find_package(roadgeometry)` via installed `CMakeLists.txt`
- C++ standard: C++17 minimum; C++20 concepts for constraint expressions if feasible
- Use templates (not pointer-to-implementation / virtual dispatch) for zero-overhead composability

### C interface (Layer 2)
- `extern "C"` shared library (`libroadgeometry.so/.dylib/.dll`)
- Opaque handles + free functions (e.g. `rg_matcher_create`, `rg_matcher_solve`, `rg_matcher_destroy`)
- Default instantiation exposed out of the box; function-pointer vtable hooks for customization (post-1.0)
- Stable ABI — enables bindings from Rust (bindgen), Julia (ccall), R, Go (cgo)
- Built as a CMake target alongside the pybind11 module (essentially free, same compilation)

### Python bindings (Layer 3)
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

### Python public API (Layer 4)
- Existing call signatures preserved — no breaking changes for existing users
- C++ backend selected by default when available; pure-Python always a valid fallback

### Distribution
| Artifact | Format | Channel |
|---|---|---|
| C++ header-only library | tarball / CMake FetchContent | GitHub releases, eventually Conan/vcpkg |
| C shared library | compiled binary + header | same as above |
| Python package | wheel | PyPI (`pip install`) |

PyPI / Python wheel is the 1.0 priority. Conan/vcpkg packaging deferred until demand.

### Build system
- `scikit-build-core` + `CMakeLists.txt`
- C library and pybind11 module as separate CMake targets, built together
- C++ coverage from Python test suite: compile with `--coverage`, run `pytest`, collect with `lcov`
- Same Python test suite validates both pure-Python and C++ backends

### Design decisions

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

### Motivation
Currently `build_flow_reduction` converts every bidirectional road into two synthetic arcs
(e_fwd, e_rev) and every oneway road into one arc shifted by `zmin`.  This requires:
1. Re-translation of results: `z = z_fwd − z_rev` (bidirectional) and `flow[e] − zmin[e]`
   (oneway).
2. The prohibitive cost on e_rev to discourage anti-parallel flow (handled correctly but
   adds arcs and cost evaluations).
3. Supply offsets for oneway roads with `zmin > 0`, polluting the supply map and
   introducing floating-point precision hazard from large offsets.
4. `lb = −∞` is impossible with the supply-shift approach — the shift would be infinite.

### Proposed change
Add an optional `lb` map (`Edge → double`, default `lb[e] = 0`) to `fragile_mccf` (and
`fragile_mccf_sparse`).  The only algorithmic change is:

```
// before
if (x >= D)  rgraph.add_edge(bwd, v, u);   // backward residual
// after
if (x - lb[e] >= D)  rgraph.add_edge(bwd, v, u);
```

(The forward check `x + D <= ub[e]` is the symmetric change, already using `capacity`.)

### Impact on `build_flow_reduction`
- Arcs with `lb[e] ≤ 0`: no supply offset, no `pwl_shift`, no re-translation of results.
  Covers bidirectional roads (`lb = −U`) and any oneway road whose minimum flow is ≤ 0.
- Arcs with `lb[e] > 0`: supply shift still required (initial flow `x=0` is infeasible);
  standard pre-flow + supply-adjustment applies.  Road type is immaterial — the lb sign
  is the only thing that matters.
- The Hamiltonian connectivity cycle arcs (`RobustInputGraph`) are unaffected and remain
  necessary to satisfy `fragile_mccf`'s strong-connectivity precondition.
- `lb = −∞` is representable (backward arc always available; forward arc only when
  `x + D ≤ ub`).  Not achievable via supply shift.

### Correctness
The capacity-scaling invariants are unchanged: linearized cost and reduced-cost formulas
are evaluated at the current flow `x`, and the residual availability conditions are the
only places `lb` appears.  The argument holds for any `lb[e] ≤ 0`; positive lower bounds
require the standard feasibility pre-flow and are out of scope here.

---

## Implicit connectivity + ordinal costs (replace `RobustInputGraph`)

### Motivation
`RobustInputGraph` adds a Hamiltonian cycle over all n nodes to guarantee strong
connectivity of every Delta-residual graph.  This requires:
1. Materializing a specific node ordering (`node_order_`, `node_index_`) in memory.
2. Adding n explicit cycle arcs to the residual graph, each processed in every Dijkstra
   and Stage 1 pass.
3. Computing a CBOUND slope (sum of `cost(U)` over all edges) to make cycle-edge cost
   prohibitive — a per-instance O(E) preprocessing pass, and (though a sound bound) a
   dynamic-range/precision cost at fine Δ.  See [plans/ordinal_costs.md](plans/ordinal_costs.md).

### Proposed change
Replace `RobustInputGraph` with implicit connectivity inside Dijkstra:

- **Ordinal costs**: represent connectivity-arc cost as a lexicographic `(ordinal, real)`
  pair — `(1, 0.0)` dominates any pure-real path `(0, *)`, eliminating CBOUND.  The pair type
  is forced through `lincost`/`redcost`/`dist`/`potential` by the reduced-cost update chain.
  Full pros/cons vs. the CBOUND slope — correctness, precision/dynamic-range, preprocessing,
  compare vs. footprint cost, instance-dependence — are analyzed in
  [plans/ordinal_costs.md](plans/ordinal_costs.md).
- **Implicit direct arc s→t**: since t is known before each Dijkstra call, treat s as
  having one implicit arc to t with cost `(0, 1)`.  No arcs stored, no specific cycle
  chosen.  When taken, flow on this arc is tracked (it carries a real "loan" that gets
  repaid at finer scales via real backward arcs).

### Tradeoffs vs. Hamiltonian cycle
- **When disconnection is rare** (e.g. well-connected road networks): direct s→t wins.
  The Hamiltonian cycle traverses O(n/2) arcs on average per disconnection event (flow
  updates + residual updates + reduced-cost recomputes for each); direct s→t costs O(1).
- **Loan granularity**: the cycle can use real arcs for part of the path and a cycle arc
  only where needed, so the loan is smaller.  Direct s→t always takes the full loan,
  bypassing available real capacity.  More flow on connectivity arcs means more repayment
  work at finer scales — but proportional to how often disconnection occurs, which is rare.
- **Space**: Hamiltonian cycle has fixed O(n) arc state; implicit approach accumulates
  one flow entry per distinct (s, t) pair that needed connectivity — hopefully sparse, but
  O(n log U/ε) in the worst case.
- **Dijkstra always prefers direct s→t** over "real arcs partway + implicit arc" since
  the implicit arc has zero real cost and d_real(s, s) = 0.  No partial real-arc usage on
  the connectivity path.
- **Asymptotic dominance**: per disconnection event, the Hamiltonian cycle accumulates
  debt on O(n/2) arcs (each gets Δ flow), whereas direct s→t accumulates Δ on one arc —
  O(n) times less total debt, which directly bounds repayment work at finer scales.
  Whether this translates to a formal asymptotic improvement needs further investigation.

### Design: swappable policies
Both choices should be independently swappable template parameters, consistent with the
existing architecture's philosophy of composable parts with good defaults:

| Policy | Options |
|---|---|
| `ConnectivityPolicy` | `HamiltonianCycle` (current default), `ImplicitDirect` (s→t per step), `None` (caller guarantees strong connectivity) |
| `ConnectivityCostPolicy` | `FiniteSlope` / CBOUND (current default), `Ordinal` (lexicographic `(ordinal, real)`), `Custom` (user-supplied slope or function) |

Power users can compose freely; `None` + any cost policy is valid for well-connected
instances and pays zero connectivity overhead.

### Impact
- `RobustInputGraph`, `RobustCapacity`, `RobustCost`, and `robust_mccf` can be removed
  entirely (or reduced to a thin wrapper that passes t to the solver).
- Residual graph shrinks by n arcs; no node-ordering bookkeeping.
- Dijkstra needs to know t upfront — already the case (s and t are chosen before each
  augmentation step).
- Stage 1 (saturate negative-redcost arcs) is unaffected: ordinal cost ≥ 0 always, so
  connectivity arcs are never pushed.
