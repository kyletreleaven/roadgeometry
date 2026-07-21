#pragma once
#include <concepts>
#include <ranges>
#include <utility>

namespace roadgeometry {

// ---------------------------------------------------------------------------
// InputGraph
//
// A static directed graph over which a flow problem is defined.
// The algorithm reads it but never mutates it.
// ---------------------------------------------------------------------------
template <typename G>
concept InputGraph = requires(const G g,
                               typename G::node_type u,
                               typename G::edge_type e) {
    typename G::node_type;
    typename G::edge_type;

    // Iteration
    { g.nodes() } -> std::ranges::range;
    { g.edges() } -> std::ranges::range;

    // Adjacency
    { g.out_edges(u) } -> std::ranges::range;
    { g.in_edges(u)  } -> std::ranges::range;

    // Topology
    { g.endpoints(e) } -> std::same_as<std::pair<typename G::node_type,
                                                  typename G::node_type>>;
};


// ---------------------------------------------------------------------------
// ResidualGraph
//
// A mutable directed graph representing the residual network.
// Arcs are keyed as (input_edge, direction) — at most two per input edge —
// so implementations can exploit that structure (e.g. two bits per edge)
// rather than maintaining a full general adjacency structure.
// ---------------------------------------------------------------------------
template <typename G>
concept ResidualGraph = requires(G g,
                                  typename G::node_type u,
                                  typename G::node_type v,
                                  typename G::edge_type e) {
    typename G::node_type;
    typename G::edge_type;

    // Mutation
    { g.add_edge(e, u, v) } -> std::same_as<void>;
    { g.remove_edge(e)    } -> std::same_as<void>;

    // Query
    { g.has_edge(e)   } -> std::convertible_to<bool>;
    { g.endpoints(e)  } -> std::same_as<std::pair<typename G::node_type,
                                                   typename G::node_type>>;
    { g.out_edges(u)  } -> std::ranges::range;  // needed by Dijkstra
    { g.edges()       } -> std::ranges::range;  // needed by Stage 1 iteration
};


// ---------------------------------------------------------------------------
// EdgeMap
//
// A read-only map from edges to scalars (e.g. lower/upper flow bounds). Any type
// with find()/end() qualifies — std::unordered_map, std::map, a flat adapter, or
// a constant-valued view — so lb and ub are never nailed to one container.
// ---------------------------------------------------------------------------
template <typename M, typename E>
concept EdgeMap = requires(const M& m, E e) {
    m.find(e);
    m.end();
};


// ---------------------------------------------------------------------------
// residual_graph_traits
//
// Associates a default ResidualGraph implementation with an InputGraph type.
// Specialize this to opt a custom InputGraph into a faster residual
// representation without changing any call sites.
//
// Forward-declares HashMapResidualGraph so the default type alias compiles
// before residual_graph.hpp is included.
// ---------------------------------------------------------------------------
template <typename Node, typename Edge> class HashMapResidualGraph;

template <InputGraph G>
struct residual_graph_traits {
    using type = HashMapResidualGraph<typename G::node_type, typename G::edge_type>;
};

} // namespace roadgeometry
