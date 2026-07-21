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
// Associated types: network_type (an InputGraph), plus node_type / edge_type
// (its nodes / edges). map_backed_instance(...) in traits.hpp is the default
// adapter over plain maps.
// ---------------------------------------------------------------------------
template <typename I>
concept Instance = requires(const I& inst,
                            typename I::node_type n,
                            typename I::edge_type e,
                            double x) {
    typename I::node_type;
    typename I::edge_type;
    typename I::network_type;
    requires InputGraph<typename I::network_type>;

    { inst.network()  } -> std::convertible_to<const typename I::network_type&>;
    { inst.cost(e, x) } -> std::convertible_to<double>;
    { inst.ub(e)      } -> std::convertible_to<double>;
    { inst.lb(e)      } -> std::convertible_to<double>;
    { inst.supply(n)  } -> std::convertible_to<double>;
};

} // namespace roadgeometry::mccf
