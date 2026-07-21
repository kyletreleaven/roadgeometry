# cpp/ — Rationale & Modern-C++ Lessons

The *why* behind the decisions in [AGENTS.md](AGENTS.md). AGENTS.md is loaded into an agent's
context on every task, so it states the rules tersely; this file holds the reasoning — for a human
learning the codebase, and for an agent that has to resolve intent on a case the rules don't
cover. AGENTS.md rules link here by section.

## Associated types: derive, don't duplicate

**Decision:** derive a composed concept's associated types through the source with a helper alias
(`edge_t<I> = typename I::network_type::edge_type`); do **not** give the outer type its own
`edge_type` member and `same_as`-tie it to the graph's.

Why derive-through is the more canonical form (cf. `std::ranges::range_value_t`, which derives a
range's value type through `iterator_t<R>` rather than making `range` re-declare it):

- **Divergence is impossible, not merely detected.** The type is written once (on the graph) and
  everything derives it, so there's no second declaration to get wrong. A `same_as` guard would be
  compensating for an error class this form can't even express — and canonical designs eliminate
  error classes structurally rather than checking for them.
- **Non-intrusive / retrofittable.** A free alias works on types you don't own or can't add member
  typedefs to — this is exactly why `std::iterator_traits` exists (raw pointers can't carry member
  typedefs). Member associated types require the model author's cooperation; aliases don't.
- **Uniform vocabulary.** One name normalizes across models that spell the type differently — or
  not at all (a container's `value_type`, a C array's nothing, a view). The library owns the name
  and the derivation, so a change lands in one place.
- **Honest ownership (has-a).** The edge type belongs to the graph; the instance has-a graph. Derive
  borrows it; a member + `same_as` asserts a co-equal copy agrees, subtly implying the instance has
  its own edge type.
- **Cleaner generic code.** `edge_t<I>` avoids the `typename I::edge_type` disambiguator noise in
  the generic contexts where you mostly spell it.

**When the fallback (member types + a supplying mixin, Boost.Iterator `iterator_facade` style) is
justified:** you specifically want the `I::edge_type` spelling and IDE autocomplete discoverability,
*and* the seam is implemented only by representations you control. For a seam implemented by
representations you don't control (the point of the `Instance` seam), prefer derive-through.

## Special cases → optimization: refinement vs. trait

**Decision:** a special case enables optimization; encode it as a **concept refinement** iff the
specialness is a capability the interface can syntactically check, else a **trait/tag +
`if constexpr`**.

A special case always carries more invariants (less freedom), and more known structure means more
optimization headroom — every fast path is "I know something extra about this input, so I can drop
work." The only question is *how to encode the specialness at the type level*, and it forks:

- **Capability invariant** — the special case *can do more* (richer interface). Encode as a
  **concept refinement**; dispatch by subsumption. Direction aligns: the more-special concept is
  the more-capable one. (`random_access_iterator` refines `forward_iterator`; O(1) `advance` is
  chosen for it.)
- **Value / structural invariant** — the special case *constrains values or shape* and thereby
  *needs less* (same or sparser interface). Encode as a **trait/tag** + `if constexpr`. Direction
  inverts or is orthogonal. (`lb ≡ 0`, "is sorted", "is acyclic", trivially-copyable, symmetric.)

**The tell** — can you write a *syntactic* requirement that captures the specialness? Yes (`it + n`
compiles) → capability → refinement. No (`lb ≡ 0`, sortedness, acyclicity are unprovable from the
interface) → value/structural → trait/tag.

**The trap:** forcing a value-specialization into a refinement *inverts generality*. The special
case (zero-lb) is a semantic *subset* of the general problem, but omitting the accessor makes its
concept the *less*-refined one — so "is-a" ends up backwards (the square-rectangle inversion). Keep
the general concept general (`Instance` keeps `lb`); mark the trivial case with a trait
(`has_lower_bounds<I> == false`) and `if constexpr` past the work.

Most efficiency-bearing special cases are value/structural, so trait/tag is the default and
refinement the exception — which is why the STL ships both (`iterator_category` traits *and*
capability concepts).

## Companion mixins: kernel spec → full interface

**Decision:** ship companion mixins so an implementor writes only the kernel and inherits the rest;
keep mixins orthogonal and value defaults dumb; tags are members.

A concept has a **required (core)** interface and a **provided (derived)** one. Let an implementor
write only the *kernel* (the core, plus what's genuinely special) and inherit the rest from a
**mixin** — the C++ analog of Rust trait defaults / Python `collections.abc` mixin methods. C++
concepts can't carry defaults, so the concept↔mixin pairing is manual; name it predictably and
document core-vs-provided so it's discoverable even though the language won't surface it.

Two flavours:
- **Defaults base** (`<Concept>Defaults`) — pure derivations valid for *every* model (e.g. fill
  optional accessors with trivial values). Always safe.
- **Specialization mixin** (`ZeroLowerBounds`, `Uncapacitated`) — bundles a special-case *value +
  its tag* (`static constexpr bool has_lower_bounds = false; double lb(Edge) const { return 0; }`),
  so one inheritance yields the free accessor *and* the `if constexpr` marker.

Two disciplines, because C++ multiple inheritance is clunkier than Rust/Python here:
- **Keep mixins orthogonal** — each owns a *disjoint* set of accessors; overlapping defaults → MI
  ambiguity (hard error or a silent wrong pick). At most one defaults base.
- **Keep value defaults dumb** — trivial, obviously-correct values only (`lb = 0`, `ub = ∞`); never
  clever/derived values that could silently mask a model's bug.

Tags are **members** (mixin-injectable); use an external `<trait><I>` template only to retrofit a
type you don't own (no mixin there — specialize the trait instead).
