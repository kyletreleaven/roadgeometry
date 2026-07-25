#pragma once
#include <concepts>
#include <cstddef>
#include <functional>
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
// Hashable<T> — usable as a std::unordered_map key via std::hash.
//
// The standard has no `hashable` concept (only regular / equality_comparable /
// etc.), so we spell it out: std::hash<T> is well-formed and size_t-ish. This
// also rejects types whose std::hash specialization is the disabled one. The
// flow solvers key residual / flow / potential / excess state on node and edge
// types, so those must be hashable.
// ---------------------------------------------------------------------------
template <typename T>
concept Hashable = requires(const T& t) {
    { std::hash<T>{}(t) } -> std::convertible_to<std::size_t>;
};

// ---------------------------------------------------------------------------
// HashKey<T> — a value usable as an unordered_map key: hashable, equality-
// comparable, and copyable. (Exactly "regular minus default_initializable", plus
// Hashable.)
//
// Deliberately NOT default-constructible: that isn't part of being a graph key,
// it's an *algorithm* need (e.g. the dense solver's `Node s{}` sentinel), so it
// rides on the solver template, not here.
// ---------------------------------------------------------------------------
template <typename T>
concept HashKey = Hashable<T> && std::equality_comparable<T> && std::copyable<T>;

// ---------------------------------------------------------------------------
// InputGraph
//
// A static directed graph over which a flow problem is defined.
// The algorithm reads it but never mutates it.
//
// node_type / edge_type must be usable as unordered_map keys (HashKey below),
// because every consumer in this codebase keys map-based state (residual / flow /
// potential / excess) on nodes and edges. This lives here, not on the algorithms,
// because there is no realistic graph whose identifiers aren't hashable / comparable
// / copyable — so it's part of what "graph" means here. (Default-constructibility is
// deliberately NOT required: it isn't part of being a graph key, and the solvers
// avoid it — e.g. std::optional instead of a `Node s{}` sentinel.)
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

    // Usable as unordered_map keys (see HashKey).
    requires HashKey<typename G::node_type>;
    requires HashKey<typename G::edge_type>;

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
