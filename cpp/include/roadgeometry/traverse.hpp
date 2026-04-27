#pragma once
#include <unordered_map>
#include <utility>
#include <vector>
#include "dag.hpp"
#include "segment.hpp"

namespace roadgeometry {

// ---------------------------------------------------------------------------
// Topograph
//
// A DAG representation of the road network annotated with point flow.
// Used as the intermediate between flow computation and matching.
//
// Nodes are either vertex nodes (one per road network vertex) or group nodes
// (one per segment group, plus stub nodes for roads with no points).
// Edges carry the flow amount (weight) and the road-length contribution
// (length) of the segment between consecutive groups.
//
// node_queue[n] is null for vertex and stub nodes, non-null for group nodes.
// The pointed-to YGroup is owned by the segments passed to create_topograph,
// which must outlive the Topograph.
// ---------------------------------------------------------------------------
struct Topograph {
    IntGraph                      graph;
    std::vector<int>              edge_weight;  // flow amount, indexed by edge_id
    std::vector<double>           edge_length;  // road-length contribution, indexed by edge_id
    std::vector<const YGroup*>    node_queue;   // null for vertex/stub nodes
};

// ---------------------------------------------------------------------------
// create_topograph<Road, Vertex>
//
// Builds a Topograph from pre-matched segments and an integer flow.
//
// For each road (u → v) with flow h:
//   h > 0: edges run u → groups → v  (flow moves from u to v)
//   h < 0: edges run v → groups → u  (flow moves from v to u, weight = |h|)
//   h = 0: road contributes no edges
//
// Roads with no segment groups get a stub node between their endpoints so
// that each road's edges remain distinct in the DAG (two roads can share
// the same endpoint pair).
//
// INPUTS
// ------
//   segments  : RoadSegments<Road> — pre-matched segment groups per road
//   flow      : Road → int         — integer flow per road (may be negative)
//   endpoints : Road → (Vertex u, Vertex v)
//   lengths   : Road → double
//
// OUTPUT
//   Topograph with vertex nodes 0..V-1 (in order of first appearance in
//   endpoints) followed by group/stub nodes.
// ---------------------------------------------------------------------------
template <typename Road, typename Vertex>
Topograph create_topograph(
    const RoadSegments<Road>&                                  segments,
    const std::unordered_map<Road, int>&                       flow,
    const std::unordered_map<Road, std::pair<Vertex,Vertex>>&  endpoints,
    const std::unordered_map<Road, double>&                    lengths
) {
    Topograph topo;
    auto& g = topo.graph;

    // Assign a node id to each network vertex, in order of first appearance.
    std::unordered_map<Vertex, int> vertex_node;
    for (auto& [road, uv] : endpoints) {
        if (!vertex_node.count(uv.first))  { vertex_node[uv.first]  = g.add_node(); topo.node_queue.push_back(nullptr); }
        if (!vertex_node.count(uv.second)) { vertex_node[uv.second] = g.add_node(); topo.node_queue.push_back(nullptr); }
    }

    // Add a directed edge in the direction of h.  h == 0 means no edge.
    auto add_edge = [&](int from, int to, int h, double length) {
        if (h == 0) return;
        (h > 0) ? g.add_edge(from, to) : g.add_edge(to, from);
        topo.edge_weight.push_back(std::abs(h));
        topo.edge_length.push_back(length);
    };

    static const std::vector<YGroup> empty_seg;

    for (auto& [road, uv] : endpoints) {
        int u_node = vertex_node.at(uv.first);
        int v_node = vertex_node.at(uv.second);

        auto flow_it = flow.find(road);
        int h = (flow_it != flow.end()) ? flow_it->second : 0;

        auto seg_it = segments.find(road);
        const auto& seg = (seg_it != segments.end()) ? seg_it->second : empty_seg;

        int    prev_node = u_node;
        double prev_y    = 0.0;

        for (const auto& grp : seg) {
            int curr_node = g.add_node();
            topo.node_queue.push_back(&grp);
            add_edge(prev_node, curr_node, h, grp.y - prev_y);
            h        += (int)grp.supply.size() - (int)grp.demand.size();
            prev_node = curr_node;
            prev_y    = grp.y;
        }

        // For pointless roads, IntGraph supports parallel edges so no stub needed.
        add_edge(prev_node, v_node, h, lengths.at(road) - prev_y);
    }

    return topo;
}

// ---------------------------------------------------------------------------
// traverse
//
// Given a Topograph, computes a minimum-cost matching by walking the DAG in
// topological order and greedily pairing supply and demand indices.
//
// At each node n (in topological order):
//   1. If node_queue[n] is non-null, extend L with its supply indices.
//   2. If node_queue[n] is non-null, for each demand index j: pop front of L, emit (i, j).
//   3. For each out-edge e of n: move the first edge_weight[e] elements of L
//      to the target node's list, accumulate edge_weight[e] * edge_length[e] into cost.
//
// Returns total cost.  Matched pairs are pushed into `out` via push_back.
// ---------------------------------------------------------------------------
template <typename Out>
double traverse(const Topograph& topo, Out& out) {
    auto order = topological_sort(topo.graph);

    // TODO: If nodes are ordered by (topo-level of source vertex, road_id,
    // position within road), each road is processed atomically and the active
    // list count is bounded by vertex set size + 1.  In that case a sparser
    // storage mechanism (e.g. unordered_map) would be preferable to this dense
    // vector — so the storage type should be swappable, e.g. via a trait or
    // template parameter.
    std::vector<std::deque<int>> lists(topo.graph.num_nodes);
    double cost = 0.0;

    for (int n : order) {
        auto& L = lists[n];

        const YGroup* q = topo.node_queue[n];
        if (q != nullptr) {
            for (int i : q->supply) L.push_back(i);
            for (int j : q->demand) {
                int i = L.front(); L.pop_front();
                out.push_back({i, j});
            }
        }

        for (int e : topo.graph.out_edges[n]) {
            int    w = topo.edge_weight[e];
            int    v = topo.graph.edge_dst[e];
            auto& Lv = lists[v];
            for (int k = 0; k < w; ++k) {
                Lv.push_back(L.front());
                L.pop_front();
            }
            cost += static_cast<double>(w) * topo.edge_length[e];
        }
    }

    return cost;
}

} // namespace roadgeometry
