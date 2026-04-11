#pragma once
#include <utility>

#include "concepts.hpp"
#include "input_graph.hpp"

namespace roadgeometry {

// ---------------------------------------------------------------------------
// HashMapResidualGraph<Node, Edge>
//
// General-purpose ResidualGraph backed by a single HashMapGraph.
// Residual arcs are keyed as (input_edge, direction) where direction is
// +1 (forward) or -1 (backward).
// ---------------------------------------------------------------------------
template <typename Node, typename Edge>
class HashMapResidualGraph {
public:
    using node_type = Node;
    using edge_type = std::pair<Edge, int>;

private:
    using Arc = edge_type;
    HashMapGraph<Node, Arc> g_;

public:
    void add_edge(Arc e, Node u, Node v) { g_.add_edge(e, u, v); }
    void remove_edge(Arc e)              { g_.remove_edge(e); }
    bool has_edge(Arc e)   const         { return g_.has_edge(e); }

    std::pair<Node, Node> endpoints(Arc e) const { return g_.endpoints(e); }

    const auto& out_edges(Node u) const { return g_.out_edges(u); }
    const auto& in_edges(Node u)  const { return g_.in_edges(u); }
    auto        nodes()           const { return g_.nodes(); }
    auto        edges()           const { return g_.edges(); }

    void add_node(Node u) { g_.add_node(u); }
};


// ---------------------------------------------------------------------------
// Default trait: any InputGraph gets HashMapResidualGraph.
// Specialize for a specific InputGraph to opt into a faster residual.
// ---------------------------------------------------------------------------
template <InputGraph G>
struct residual_graph_traits<G> {
    using type = HashMapResidualGraph<typename G::node_type, typename G::edge_type>;
};

} // namespace roadgeometry
