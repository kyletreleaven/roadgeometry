#pragma once
#include <stdexcept>
#include <vector>

namespace roadgeometry {

// ---------------------------------------------------------------------------
// IntGraph
//
// A directed graph with int nodes and int edges, backed by flat arrays.
// Nodes and edges are assigned sequential ids starting from 0.
// Suitable for compact, cache-friendly traversal where the full node/edge
// set is known or built incrementally.
// ---------------------------------------------------------------------------
struct IntGraph {
    int num_nodes = 0;
    int num_edges = 0;
    std::vector<int> edge_src, edge_dst;        // indexed by edge_id
    std::vector<std::vector<int>> out_edges;    // out_edges[node] = list of edge_ids
    std::vector<std::vector<int>> in_edges;     // in_edges[node]  = list of edge_ids

    int add_node() {
        out_edges.emplace_back();
        in_edges.emplace_back();
        return num_nodes++;
    }

    int add_edge(int u, int v) {
        int e = num_edges++;
        edge_src.push_back(u);
        edge_dst.push_back(v);
        out_edges[u].push_back(e);
        in_edges[v].push_back(e);
        return e;
    }
};

// ---------------------------------------------------------------------------
// topological_sort
//
// Kahn's algorithm.  Returns nodes in topological order.
// Throws std::runtime_error if the graph contains a cycle.
// ---------------------------------------------------------------------------
inline std::vector<int> topological_sort(const IntGraph& g) {
    std::vector<int> in_degree(g.num_nodes, 0);
    for (int e = 0; e < g.num_edges; ++e)
        ++in_degree[g.edge_dst[e]];

    std::vector<int> queue;
    for (int u = 0; u < g.num_nodes; ++u)
        if (in_degree[u] == 0) queue.push_back(u);

    std::vector<int> order;
    order.reserve(g.num_nodes);
    for (int qi = 0; qi < (int)queue.size(); ++qi) {
        int u = queue[qi];
        order.push_back(u);
        for (int e : g.out_edges[u]) {
            int v = g.edge_dst[e];
            if (--in_degree[v] == 0) queue.push_back(v);
        }
    }

    if ((int)order.size() != g.num_nodes)
        throw std::runtime_error("topological_sort: graph contains a cycle");

    return order;
}

} // namespace roadgeometry
