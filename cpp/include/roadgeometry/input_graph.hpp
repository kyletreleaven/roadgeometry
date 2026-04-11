#pragma once
#include <ranges>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "concepts.hpp"

namespace roadgeometry {

// ---------------------------------------------------------------------------
// HashMapGraph<Node, Edge>
//
// A directed graph backed by hash maps.  Satisfies both InputGraph (read
// access for the algorithm) and ResidualGraph (mutation during the solve).
//
// Node and Edge types must be hashable.
// ---------------------------------------------------------------------------
template <typename Node, typename Edge>
class HashMapGraph {
public:
    using node_type = Node;
    using edge_type = Edge;

    // -- Mutation -----------------------------------------------------------

    void add_node(Node u) {
        out_.emplace(u, std::unordered_set<Edge>{});
        in_.emplace(u,  std::unordered_set<Edge>{});
    }

    void add_edge(Edge e, Node u, Node v) {
        if (endpoints_.count(e))
            throw std::invalid_argument("duplicate edge");
        add_node(u);
        add_node(v);
        endpoints_[e] = {u, v};
        out_[u].insert(e);
        in_[v].insert(e);
    }

    void remove_edge(Edge e) {
        auto [u, v] = endpoints_.at(e);
        out_[u].erase(e);
        in_[v].erase(e);
        endpoints_.erase(e);
    }

    // -- Query --------------------------------------------------------------

    bool has_edge(Edge e) const { return endpoints_.count(e); }

    std::pair<Node, Node> endpoints(Edge e) const { return endpoints_.at(e); }

    auto nodes()          const { return std::views::keys(out_); }
    auto edges()          const { return std::views::keys(endpoints_); }
    const auto& out_edges(Node u) const { return out_.at(u); }
    const auto& in_edges(Node u)  const { return in_.at(u); }

private:
    std::unordered_map<Edge, std::pair<Node, Node>>    endpoints_;
    std::unordered_map<Node, std::unordered_set<Edge>> out_;
    std::unordered_map<Node, std::unordered_set<Edge>> in_;
};

static_assert(InputGraph<HashMapGraph<int, int>>);

} // namespace roadgeometry
