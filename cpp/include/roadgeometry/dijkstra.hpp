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


// TODO: The two overloads below could be unified into a single template:
//
//   template<typename OutEdges, typename Endpoints, typename Cost>
//   dijkstra(const OutEdges& out_edges, const Endpoints& endpoints,
//            const Cost& cost, <source-type> source);
//
// Both implementations share the same algorithm body. The only type bound
// required is operator[] on out_edges, endpoints, and cost. The Graph struct
// abstraction is unnecessary — it can be "rolled out" into these three
// subscriptable parameters directly. Node/edge types would be deduced via
// decltype(out_edges[source]) etc.

/**
 * Dijkstra optimized for dense integer node/edge ids.
 *
 * Takes flat array inputs and returns parallel output vectors, avoiding hash
 * map overhead. Suitable for normalized graphs where nodes are 0..n-1 and
 * edges are 0..m-1.
 *
 * @param out_edges  out_edges[node] = list of edge ids leaving that node
 * @param endpoints  endpoints[edge] = (tail, head)
 * @param cost       cost[edge]
 * @param source     source node id
 * @return (dist, upstream) as parallel vectors:
 *         dist[i]     = shortest distance to node i (inf if unreachable)
 *         upstream[i] = edge id on shortest path to i (-1 if source/unreachable)
 */
inline std::pair<std::vector<double>, std::vector<int>>
dijkstra(
    const std::vector<std::vector<int>>&    out_edges,
    const std::vector<std::pair<int, int>>& endpoints,
    const std::vector<double>&              cost,
    int                                      source
) {
    const int    n   = static_cast<int>(out_edges.size());
    const double inf = std::numeric_limits<double>::infinity();

    std::vector<double> dist(n, inf);
    std::vector<int>    upstream(n, -1);
    std::vector<double> tentative(n, inf);

    PriorityQueue<int> pq;
    pq.push(source, 0.0);
    tentative[source] = 0.0;

    while (!pq.empty()) {
        int i  = pq.pop_min();
        dist[i] = tentative[i];

        for (int e : out_edges[i]) {
            int j = endpoints[e].second;
            if (dist[j] < inf) continue;

            double dj = dist[i] + cost[e];
            if (dj < tentative[j]) {
                tentative[j] = dj;
                pq.push(j, dj);
                upstream[j] = e;
            }
        }
    }

    return {dist, upstream};
}

}  // namespace roadgeometry
