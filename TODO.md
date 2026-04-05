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

Three composable abstractions:

| Protocol | Python impl | C++ template |
|---|---|---|
| `DistanceOracle` | `RoadnetMetric` | `DijkstraOracle<RoadId, VertId>` |
| `FlowSolver` | `MinConvexCostFlow` | `ConvexCostFlowSolver<CostFn>` |
| `MatchingSolver` | `RoadnetMatchingProblem` | `RoadnetMatcher<DistOracle, FlowSolver>` |

`RoadnetMatcher` is parameterized on the other two — C++ users compose freely:

```cpp
// default
using FastMatcher = RoadnetMatcher<DijkstraOracle<int,int>, ConvexCostFlowSolver<PiecewiseLinear>>;
// custom distance oracle
using CustomMatcher = RoadnetMatcher<LookupTableOracle, ConvexCostFlowSolver<PiecewiseLinear>>;
```

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

### Implementation order
- [ ] **Step 0: Profile** — instrument Python matching code on realistic input to confirm hot paths
  before writing any C++. Candidates: `MinConvexCostFlow`, `TRAVERSE3`, `RoadnetMetric.distance()`
- [ ] **Step 1: Python refactor** — define Protocol interfaces, restructure existing code behind
  seams, verify existing tests still pass. No C++ yet.
- [ ] **Step 2: C++ core data structures** — priority queue, sorted container (replacing
  `bintrees.RBTree`), `IntRoadnet` mirror
- [ ] **Step 3: `DijkstraOracle<RoadId, VertId>`** — bind via pybind11, validate against Python
  impl with existing tests
- [ ] **Step 4: `ConvexCostFlowSolver<CostFn>`** — bind via pybind11, validate
- [ ] **Step 5: `RoadnetMatcher<DistOracle, FlowSolver>`** — compose above, full end-to-end binding
- [ ] **Step 6: C interface** — `libroadgeometry` with opaque handles, built alongside pybind11 module
- [ ] **Step 7: packaging** — `scikit-build-core` build, PyPI wheel, CMake install rules for headers + C lib
- [ ] **Post-1.0: C interface customization** — function-pointer vtables for swapping oracle/solver at C level
- [ ] **Post-1.0: package managers** — Conan and/or vcpkg recipes
