#pragma once

#include <unordered_map>
#include <utility>
#include <vector>
#include "roadgeometry/priority_queue.hpp"

namespace roadgeometry {

/**
 * Dijkstra's shortest-path algorithm.
 *
 * @tparam Graph  Graph type. Must provide:
 *                  - `out_edges(node)` → iterable of edge ids
 *                  - `endpoints(edge)` → std::pair<node, node> (tail, head)
 *                  - typedefs `node_type` and `edge_type`
 *
 * @param graph   The graph to search.
 * @param cost    Callable or subscriptable: cost(edge) → double.
 * @param source  Source node.
 * @return Pair of (distance map, upstream edge map).
 *         upstream[v] = edge that leads to v on the shortest path from source.
 *         Nodes unreachable from source are absent from both maps.
 */
template<typename Graph, typename Cost>
std::pair<
    std::unordered_map<typename Graph::node_type, double>,
    std::unordered_map<typename Graph::node_type, typename Graph::edge_type>
>
dijkstra(const Graph& graph, const Cost& cost, typename Graph::node_type source) {
    using Node = typename Graph::node_type;
    using Edge = typename Graph::edge_type;

    std::unordered_map<Node, double> dist;
    std::unordered_map<Node, Edge>   upstream;
    std::unordered_map<Node, double> tentative;

    PriorityQueue<Node> pq;
    pq.push(source, 0.0);
    tentative[source] = 0.0;

    while (!pq.empty()) {
        Node i  = pq.pop_min();
        double di = tentative[i];
        dist[i] = di;

        for (const Edge& e : graph.out_edges(i)) {
            auto [tail, j] = graph.endpoints(e);
            if (dist.count(j)) continue;

            double dj = di + cost[e];
            auto it = tentative.find(j);
            if (it == tentative.end() || dj < it->second) {
                tentative[j] = dj;
                pq.push(j, dj);
                upstream[j] = e;
            }
        }
    }

    return {dist, upstream};
}

/**
 * Simple flat-array graph for use with normalized (int) node and edge ids.
 *
 * out_edges[node]  = list of edge ids leaving that node
 * endpoints[edge]  = (tail_node, head_node)
 */
struct IntGraph {
    using node_type = int;
    using edge_type = int;

    std::vector<std::vector<int>>        out_edges_;
    std::vector<std::pair<int, int>>     endpoints_;

    const std::vector<int>& out_edges(int node) const { return out_edges_[node]; }
    std::pair<int, int>     endpoints(int edge) const { return endpoints_[edge]; }
};

}  // namespace roadgeometry
