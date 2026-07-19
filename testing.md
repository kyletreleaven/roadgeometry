# roadgeometry — Testing Strategy

## Summary

As the interesting algorithmic variation moves into C++ template seams (connectivity policy, cost
policy, potential-update strategy, priority queue, graph), the current pytest-through-pybind11 path
is both **slow** (every test pays the full module compile) and **seam-blind** (only the `<int,int>`
instantiation is exposed to Python — see [design.md](design.md)). The fix is a **native C++ test
target over the header-only core** plus a methodology that is cheap to write and naturally covers
the seams. Tracked in [TODO.md](TODO.md) § Testing infrastructure.

---

## The problem (grounded)

There is no native C++ test target today; every test routes through pytest → the pybind11 module
(`cpp/include/roadgeometry/` is a rich header-only core, but only reachable via the binding). Two
costs follow:

1. **Compile-before-any-test** — the whole pybind11 module builds before a single assertion runs.
2. **Seam-blindness** — Python sees one exposed instantiation, so template-parameter policies
   cannot be tested from Python *by design*.

## Approach: native C++ tests over the header core

Because the core is header-only, a native test binary just `#include`s the headers plus a light
framework — no pybind11, no Python — so it compiles fast and can instantiate *any* template with
*any* policy. Wire it through **CTest** in the existing `cpp/CMakeLists.txt`; the inner loop
becomes `cmake --build build && ctest`.

## Methodology — cheap and seam-covering

Most planned improvements are **answer-preserving** (incremental `ReducedCost`, frontier potential
update, ordinal↔CBOUND, `Direct`↔`HamCycle` on connected instances, lazy edgeless). Lean on that
instead of hand-writing expected outputs:

- **Differential / parity.** Run two policy instantiations on the same seeded-random instance
  corpus; assert identical flow and cost. This is exactly the test for "this refactor must not
  change the result," and it exercises the new seams for free.
- **Optimality-certificate invariants.** A self-contained oracle needing no reference solver:
  feasibility (flow conservation + capacity bounds) plus reduced-cost ≥ 0 on the residual graph.
  The solver already computes potentials, so this is nearly free and catches correctness bugs
  directly.
- **Golden micro-fixtures.** A handful of tiny hand-built graphs with known optima, as readable
  regression anchors.
- Back these with a small **seeded random instance generator** (graphs, supplies, convex costs).

## Layers

| Layer | Scope | Cadence |
|---|---|---|
| Native C++ (doctest + CTest) | the seams; certificate + parity; fast | primary dev loop |
| pytest / pybind11 | the binding, the Python API, Python↔C++ parity | acceptance, run less often |

Speed the acceptance layer with `ccache` if the module compile still bites; keep C++ coverage
(gcovr) where it already runs.

## Framework choice

Default to **doctest** (lightest `#include`, CTest-friendly). Honest limitations, in order of how
much they bite here:

1. **Weak parameterized/typed tests** — no `TYPED_TEST`/`TEST_P` equivalent, so the policy matrix
   is a hand-rolled templated helper in a loop, and failures report the loop site, not the policy.
   This is the one real downgrade for a matrix-heavy suite.
2. **Spartan matchers/fixtures** — no matcher library, only `SUBCASE` setup; hand-write flow/cost
   comparison helpers and instance builders (we would anyway).
3. **No mocking, no death tests** — mocking is irrelevant (pure algorithms); the lack of
   `EXPECT_DEATH` means precondition-abort behavior (e.g. `None` on a disconnected instance) is
   awkward to assert.
4. **Compile-speed edge is marginal** — heavy template instantiations dominate each test TU, so
   framework choice is a small slice. The real win is **native-vs-pybind11**, not doctest-vs-others.

If the policy-combination matrix becomes the centerpiece, reconsider **GoogleTest** (`TYPED_TEST`
earns its heavier compile). Middle path: doctest + a `run_all_policies(...)` template helper,
accepting coarser failure names to keep the light compile.

### Avoiding lock-in (a precondition for starting with doctest)

Starting with doctest is only acceptable because a later upgrade stays cheap — but that cheapness
is a *discipline*, not automatic:

- **All machinery lives in framework-free headers.** The instance generator, the
  optimality-certificate oracle, the parity harness, and the flow/cost comparators depend on *no*
  test framework and return plain `bool`/result structs. The framework file is a thin shell:
  `CHECK(check_certificate(solver, inst).ok)`. Migrating frameworks rewrites the shell, not the
  machinery — which is the bulk of the code.
- **CTest is the stable outer layer** — the build/CI interface (`ctest -j`) is unchanged by any
  framework swap.
- **A portable `for_each_policy<Test>()` typelist helper** drives the policy matrix instead of
  framework-specific parameterization. It both blunts doctest's one real weakness and survives a
  migration.
- **Guardrail:** do not carry test logic in `SUBCASE` nesting or framework matchers (they do not
  port to gtest). Keep the framework's role to thin `CHECK`/`REQUIRE` wrapping only.

With this, doctest → Catch2 is near-mechanical (shared API lineage) and doctest → GoogleTest is a
bounded, shell-only rewrite.

## Build ergonomics

- Native tests as a **separate CMake target**, buildable/runnable without the pybind11 module.
- A `nox` session or Makefile target for the fast loop; `ctest -j` for parallelism across cases.
- Land this **before** the Solver-efficiency work — those changes are precisely the
  answer-preserving ones the differential + certificate harness is built to guard.
