#pragma once
#include <limits>
#include <type_traits>
#include <utility>

#include "roadgeometry/mccf/concepts.hpp"
#include "roadgeometry/mccf/detail.hpp"   // map_get

namespace roadgeometry::mccf {

// ---------------------------------------------------------------------------
// has_lower_bounds<I> — value-specialization tag (see AGENTS.md).
//
// Reads the model's `has_lower_bounds` member if present, else true: a general
// Instance carries lower bounds. A member (not an external trait) so a mixin can
// inject it. The solver/checker branch on has_lower_bounds_v<I> via `if constexpr`
// to skip the lb() call, the subtraction, and lb storage on the lb ≡ 0 fast path.
// ---------------------------------------------------------------------------
// Detects the `has_lower_bounds` tag member (named, so no inline `requires requires`).
template <class I>
concept has_lower_bounds_tag = requires { I::has_lower_bounds; };

template <class I>
struct lower_bounds_trait : std::true_type {};
template <has_lower_bounds_tag I>
struct lower_bounds_trait<I> : std::bool_constant<I::has_lower_bounds> {};

template <class I>
inline constexpr bool has_lower_bounds_v = lower_bounds_trait<I>::value;

// ---------------------------------------------------------------------------
// ZeroLowerBounds<Edge> — specialization mixin (value + tag).
//
// Inherit to get a trivial lb() = 0 for free and mark the lb ≡ 0 fast path.
// Keep such mixins ORTHOGONAL (each owns disjoint accessors) to avoid MI
// ambiguity, and keep the default value dumb.
// ---------------------------------------------------------------------------
template <class Edge>
struct ZeroLowerBounds {
    static constexpr bool has_lower_bounds = false;
    double lb(Edge) const { return 0.0; }
};

// ---------------------------------------------------------------------------
// MapBackedInstance — the default Instance model, over plain maps.
//
// Owns its arguments (moved in), so passing temporaries is safe. For large
// data, model Instance directly with a view-based type instead of copying.
//   Cost:   map edge -> callable double(double)   (missing => zero cost)
//   UB/LB:  EdgeMap                               (missing ub => +inf, lb => 0)
//   Supply: map node -> double                    (missing => 0)
// ---------------------------------------------------------------------------
template <InputGraph G, class Cost,
          KeyMap<typename G::edge_type> UB,
          KeyMap<typename G::edge_type> LB,
          class Supply>
class MapBackedInstance {
public:
    using network_type = G;
    using node_type    = typename G::node_type;
    using edge_type    = typename G::edge_type;

    MapBackedInstance(G g, Cost cost, UB ub, LB lb, Supply supply)
        : g_(std::move(g)), cost_(std::move(cost)), ub_(std::move(ub)),
          lb_(std::move(lb)), supply_(std::move(supply)) {}

    const G& network() const { return g_; }

    // The cost map's per-edge fn type, preserved (not erased to std::function).
    using cost_fn_type = std::remove_cvref_t<
        decltype(std::declval<const Cost&>().find(std::declval<edge_type>())->second)>;

    // cost(e) returns the edge's cost fn BY CONST REFERENCE — a raw borrow into the
    // owned cost map. Zero abstraction cost, no copy (safe for move-only fns).
    //
    // The cost map must be TOTAL — every edge present. There is deliberately NO
    // missing-edge fallback: "missing => zero" is a footgun (it is the *opposite* of a
    // real absent-edge cost, e.g. an empty road's length*|f|) and was never used — the
    // matching reduction fills every edge explicitly. A cost source that *synthesizes*
    // a real cost for absent edges is a different Instance/cost type returning its
    // handle by value (a genuine storage/runtime tradeoff), not this borrow.
    const cost_fn_type& cost(edge_type e) const { return cost_.find(e)->second; }
    double ub(edge_type e) const {
        return detail::map_get(ub_, e, std::numeric_limits<double>::infinity());
    }
    double lb(edge_type e)     const { return detail::map_get(lb_, e, 0.0); }
    double supply(node_type n) const { return detail::map_get(supply_, n, 0.0); }

private:
    G g_;
    Cost cost_;
    UB ub_;
    LB lb_;
    Supply supply_;
};

template <InputGraph G, class Cost, class UB, class LB, class Supply>
MapBackedInstance<G, Cost, UB, LB, Supply>
map_backed_instance(G g, Cost cost, UB ub, LB lb, Supply supply) {
    return MapBackedInstance<G, Cost, UB, LB, Supply>(
        std::move(g), std::move(cost), std::move(ub),
        std::move(lb), std::move(supply));
}

} // namespace roadgeometry::mccf
