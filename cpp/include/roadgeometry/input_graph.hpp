#pragma once
#include <functional>
#include <ranges>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "concepts.hpp"

namespace roadgeometry {

// ---------------------------------------------------------------------------
// HashMapGraph<Node, Edge, Hash>
//
// A directed graph backed by hash maps.  Satisfies both InputGraph (read
// access for the algorithm) and ResidualGraph (mutation during the solve).
//
// Node must be hashable via std::hash<Node>.
// Edge must be hashable via Hash (defaults to std::hash<Edge>), allowing
// callers to supply a custom hasher when Edge is e.g. std::pair<…>.
// ---------------------------------------------------------------------------
template <typename Node, typename Edge, typename Hash = std::hash<Edge>>
class HashMapGraph {
public:
    using node_type = Node;
    using edge_type = Edge;

    // -- Mutation -----------------------------------------------------------

    void add_node(Node u) {
        out_.emplace(u, std::unordered_set<Edge, Hash>{});
        in_.emplace(u,  std::unordered_set<Edge, Hash>{});
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
    std::unordered_map<Edge, std::pair<Node, Node>, Hash>    endpoints_;
    std::unordered_map<Node, std::unordered_set<Edge, Hash>> out_;
    std::unordered_map<Node, std::unordered_set<Edge, Hash>> in_;
};

static_assert(InputGraph<HashMapGraph<int, int>>);

// ---------------------------------------------------------------------------
// normalize_road_network
//
// Assigns a stable int index to every Road and every Vertex appearing in
// endpoints.  Roads are indexed in iteration order of endpoints; each
// distinct Vertex is assigned the next available int on first encounter.
//
// Returns {road_to_int, int_to_road, vert_to_int}.
// int_to_road[i] == road  iff  road_to_int[road] == i.
// ---------------------------------------------------------------------------
template <typename Road, typename Vertex>
inline std::tuple<
    std::unordered_map<Road,int>,
    std::vector<Road>,
    std::unordered_map<Vertex,int>
>
normalize_road_network(
    const std::unordered_map<Road, std::pair<Vertex,Vertex>>& endpoints
) {
    std::unordered_map<Road,   int> road_to_int;
    std::vector<Road>               int_to_road;
    std::unordered_map<Vertex, int> vert_to_int;
    for (auto& [road, uv] : endpoints) {
        if (road_to_int.emplace(road, (int)int_to_road.size()).second)
            int_to_road.push_back(road);
        vert_to_int.emplace(uv.first,  (int)vert_to_int.size());
        vert_to_int.emplace(uv.second, (int)vert_to_int.size());
    }
    return {std::move(road_to_int), std::move(int_to_road), std::move(vert_to_int)};
}

} // namespace roadgeometry
