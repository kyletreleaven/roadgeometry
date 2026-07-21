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

## Organizing concepts, traits, models, algorithms

Keep four roles distinct — conflating them is where the tree rots:

| Role | Is | Lives in |
|---|---|---|
| **Concept** | a contract (required ops) | `concepts.hpp` |
| **Model** | a concrete type satisfying a concept | `traits.hpp` (defaults), or `models.hpp` once they multiply |
| **Traits** | a bridge: associated types + default behaviour, per-type specializable | `traits.hpp` |
| **Algorithm** | generic code constrained by concepts | its own header (`solver.hpp`, `certificate.hpp`) |

- **Organize by role, not feature.** Don't colocate a concept with its model in one `foo.hpp`;
  put concepts together so refinements `&&`-compose and subsumption works.
- **Concepts at the altitude of their generality.** Cross-cutting contracts (`InputGraph`,
  `EdgeMap`) → top-level `concepts.hpp`; domain-specific ones (`Instance`) → the module's
  `mccf/concepts.hpp`.
- **`traits.hpp` = bridges + default models/policies** for a module — a "defaults + plumbing"
  file. But a model (*satisfies* a concept) is not a trait (*bridges to* one); split models into
  `models.hpp` the moment they multiply, so `traits.hpp` doesn't become a junk drawer.
- **Prefer named associated types over inline `decltype`.** Expose `using network_type = G;` and
  constrain `InputGraph<typename I::network_type>`, not `remove_cvref_t<decltype(inst.network())>`.
  Member typedefs for types you own; a traits class only to retrofit types you don't.
- **Per-problem subdirectory** (`mccf/`) once a problem outgrows a header or two.
- **Customization points:** member-accessor concepts (`requires { inst.cost(e, x); }`) are the
  default. CPOs / `tag_invoke` are for retrofitting external types you can't modify — reach for
  them only when that need is real.

This is guardrail 2 (data vs. machinery) in file form: concepts *constrain data* (`Instance`,
`EdgeMap`); traits *provide machinery* (`PriorityQueue`, `ResidualGraph`, policies).

## Special cases → optimization: refinement vs. trait

A special case always carries more invariants (less freedom), and more known structure means more
optimization headroom — every fast path is "I know something extra about this input, so I can drop
work." The only design question is *how to encode the specialness at the type level*, and it forks:

- **Capability invariant** — the special case *can do more* (richer interface). Encode as a
  **concept refinement**; dispatch by subsumption. Direction aligns: the more-special concept is
  the more-capable one. (`random_access_iterator` refines `forward_iterator`; the O(1) `advance`
  is chosen for it.)
- **Value / structural invariant** — the special case *constrains values or shape* and thereby
  *needs less* (same or sparser interface). Encode as a **trait/tag** + `if constexpr`; dispatch on
  the trait. Direction inverts or is orthogonal. (`lb ≡ 0`, "is sorted", "is acyclic",
  trivially-copyable, symmetric.)

**The tell** — can you write a *syntactic* requirement that captures the specialness?
- Yes (`it + n` compiles) → capability → **refinement**.
- No (`lb ≡ 0`, sortedness, acyclicity are unprovable from the interface) → value/structural →
  **trait/tag**.

**The trap:** forcing a value-specialization into a refinement *inverts generality*. The special
case (zero-lb) is a semantic *subset* of the general problem, but omitting the accessor makes its
concept the *less*-refined one — so "is-a" ends up backwards (the square-rectangle inversion). Keep
the general concept general (`Instance` keeps `lb`); mark the trivial case with a trait
(`has_lower_bounds<I> == false`) and `if constexpr` past the work.

Most efficiency-bearing special cases are value/structural, so **trait/tag is the default and
refinement is the exception** — which is why the STL ships both (`iterator_category` traits *and*
capability concepts).

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
