#pragma once
#include <concepts>
#include <ranges>
#include <type_traits>
#include <utility>

namespace roadgeometry {

// ---------------------------------------------------------------------------
// range_of<R, T> — a range whose elements are convertible to T.
//
// Lets a compound requirement read `{ g.nodes() } -> range_of<node_type>` instead
// of spelling out range_value_t/remove_cvref_t inline. remove_cvref_t normalizes
// R because the `-> concept` form passes decltype of the expression (which may be
// a value or a reference depending on how the accessor returns).
// ---------------------------------------------------------------------------
template <typename R, typename T>
concept range_of =
    std::ranges::range<std::remove_cvref_t<R>> &&
    std::convertible_to<std::ranges::range_value_t<std::remove_cvref_t<R>>, T>;

// ---------------------------------------------------------------------------
// InputGraph
//
// A static directed graph over which a flow problem is defined.
// The algorithm reads it but never mutates it.
//
// This concept fixes only the graph's *structure*. Requirements on node_type /
// edge_type as usable *values* — hashable, equality-comparable, regular,
// default-constructible, needed because the solvers key map-based state on them —
// are imposed by the algorithms that consume an InputGraph, NOT here, so "being a
// graph" stays decoupled from any one solver's storage choices.
//
// Semantic contract (not syntactically checkable, but part of the interface all
// the same):
//   * the graph is immutable for the duration of a solve;
//   * nodes() / edges() enumerate every node / edge exactly once;
//   * adjacency agrees with topology: e in out_edges(u) => endpoints(e).first == u,
//     and e in in_edges(u) => endpoints(e).second == u.
// ---------------------------------------------------------------------------
template <typename G>
concept InputGraph = requires(const G& g,
                               typename G::node_type u,
                               typename G::edge_type e) {
    typename G::node_type;
    typename G::edge_type;

    // Iteration — each range yields the graph's own node / edge type.
    { g.nodes() } -> range_of<typename G::node_type>;
    { g.edges() } -> range_of<typename G::edge_type>;

    // Adjacency — each yields edges incident to u.
    { g.out_edges(u) } -> range_of<typename G::edge_type>;
    { g.in_edges(u)  } -> range_of<typename G::edge_type>;

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
// KeyMap
//
// A read-only lookup keyed by K (find()/end()): edge->scalar bound maps, flow
// maps, node->potential maps. Any type with find()/end() qualifies —
// std::unordered_map, std::map, a flat adapter, a constant-valued view — so no
// map is nailed to one container.
// ---------------------------------------------------------------------------
template <typename M, typename K>
concept KeyMap = requires(const M& m, K k) {
    m.find(k);
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
