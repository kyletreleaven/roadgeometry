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
