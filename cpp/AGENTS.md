# cpp/ — Coding Discipline for the C++ Kernel

This subtree is a **header-only, zero-overhead template kernel** (min-cost convex-cost flow +
matching). Architecture and layers are in [/design.md](../design.md); test strategy in
[/testing.md](../testing.md); algorithm design in [/plans/](../plans/). This file is the *coding
discipline* — read it before adding or changing template/concept structure here.

## Stance: concepts over virtual dispatch

Composability is via **template parameters + concepts**, not runtime polymorphism. Structural
choices are resolved at compile time so hot loops (Dijkstra, residual updates, reduced cost) stay
fully specialized — no vtable, no runtime policy branch.

## Three guardrails (when to template, when not)

1. **Demand-driven extraction.** Template a choice only when there is *motivated, plural*
   variation. Keep it concrete until a second real implementation exists — extracting a concept
   from working code later is mechanical; un-exposing an over-eager seam is a breaking change.
   (E.g. extract `ConnectivityPolicy` when the *second* policy lands, not speculatively.)

2. **Separate DATA from MACHINERY, and bundle each.**
   - *Problem data* → one `FlowInstance` concept (network + cost + ub + lb + supply, via
     accessors). Ship a `map_backed_instance(...)` adapter so callers with plain maps stay
     one-liners; power users (e.g. `build_flow_reduction`) implement `FlowInstance` directly for
     fused / lazy / computed representations that separate map params cannot express.
   - *Solver machinery* → traits (`PriorityQueue`, `ResidualGraph`, connectivity / cost-repr
     policies). Do not conflate the two: "what problem" vs. "how to solve it."

3. **Mind the instantiation / test matrix.** Concept-heavy ⇒ more instantiations ⇒ slower
   compiles and a bigger differential-test *product*. Keep the *tested* matrix bounded (the
   `for_each_policy` helper in [/testing.md](../testing.md)) — don't instantiate combinations you
   don't test.

## Concept mechanics (the footguns)

- **Refine by composing NAMED concepts:** `Refined = Base<T> && requires(...) { ... };`.
  Subsumption — which gives "more-constrained overload / partial-spec wins," the compile-time
  analog of most-derived dispatch — only works on *named atomic constraints*. Two independently
  spelled `requires{}` blocks do **not** subsume each other even when one logically implies the
  other. If you re-spell shared requirements instead of `&&`-ing the base concept, you silently
  lose specialization ordering.
- **Concepts are always templates; multi-param is fine** (`EdgeMap<M, E>`). Used as a constraint,
  the constrained type *prepends*: `template <EdgeMap<Edge> M>` means `EdgeMap<M, Edge>`.
- **Orthogonal policy concepts form a partial order.** A type modeling two independent concepts
  makes overloads *ambiguous* (hard error), not auto-resolved. Fix by naming the join
  (`template <A_and_B T>` where `A_and_B = A<T> && B<T>`).

## Conventions in this tree

- `EdgeMap<M, E>` — map-like (`.find()` / `.end()`); the basis for bound maps (lb, ub) and cost
  maps. lb and ub are symmetric — type them the same way.
- Solver entry points are read-only: `const&` throughout; instance and traits are never mutated.
- Adding a structural choice: make it work **concretely + a native test first**; extract a concept
  only when a second implementation motivates it, and **refine from the existing base concept** so
  subsumption stays intact.

## Tests (see [/testing.md](../testing.md))

- **TDD**: red (often a *compile failure* for a missing seam) → green → refactor.
- Native **doctest + CTest** via `scripts/cpptest.sh` — fast loop, no pybind11, no Python.
- Keep test machinery **framework-free** (`cpp/tests/support.hpp`): generators, the optimality
  certificate oracle, and comparators return plain values; the framework layer is a thin shell of
  `CHECK(...)`. That discipline is what keeps a framework swap cheap.
- Prefer **differential/parity** tests (answer-preserving policy swaps) and **optimality-certificate
  invariants** over hand-written expected outputs.
