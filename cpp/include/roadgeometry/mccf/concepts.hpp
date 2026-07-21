#pragma once
#include <concepts>

#include "roadgeometry/concepts.hpp"

namespace roadgeometry::mccf {

// ---------------------------------------------------------------------------
// Instance — a min convex-cost flow problem.
//
// Exposes the problem data through accessors so the *representation* is the
// modeller's choice (plain maps, fused per-edge structs, computed/constant
// fields, a view over external data):
//
//   network()     an InputGraph (network_type)
//   cost(e, x)    convex per-edge cost evaluated at flow value x
//   ub(e), lb(e)  per-edge upper / lower flow bounds
//   supply(n)     net supply at a node (conservative: sums to 0)
//
// network_type is the single source of truth for the node/edge types — the
// accessors are keyed on the graph's own types, so a model can't (and needn't)
// declare a divergent edge/node type. Use the node_t / edge_t shorthands below.
// map_backed_instance(...) in traits.hpp is the default adapter over plain maps.
// ---------------------------------------------------------------------------
template <typename I>
concept Instance = requires(const I& inst,
                            typename I::network_type::node_type n,
                            typename I::network_type::edge_type e,
                            double x) {
    typename I::network_type;
    requires InputGraph<typename I::network_type>;

    { inst.network()  } -> std::convertible_to<const typename I::network_type&>;
    { inst.cost(e, x) } -> std::convertible_to<double>;
    { inst.ub(e)      } -> std::convertible_to<double>;
    { inst.lb(e)      } -> std::convertible_to<double>;
    { inst.supply(n)  } -> std::convertible_to<double>;
};

// Associated-type shorthands (STL-style, cf. std::ranges::range_value_t): derived
// through network_type, so always available, uniform, and impossible to diverge.
// In generic code these also avoid the `typename I::...` noise.
template <Instance I> using node_t = typename I::network_type::node_type;
template <Instance I> using edge_t = typename I::network_type::edge_type;

} // namespace roadgeometry::mccf
